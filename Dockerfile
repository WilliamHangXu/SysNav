# syntax=docker/dockerfile:1
#
# SysNav (VLM_ROS AlphaZ) — full build, GPU, headless, AWS-ready.
#
# Base: CUDA 12.6.3 devel (matches host: nvcc 12.6, torch 2.9.0+cu126) on
# Ubuntu 24.04, which is the ROS 2 Jazzy platform. `devel` (not runtime) is
# required because we compile CUDA extensions at build time (detectron2,
# pytorch3d) and several CUDA-aware ROS packages.
#
# Layer order is chosen for cache reuse: rarely-changing system + ROS + native
# deps + python deps come first; application source (which changes often) and
# the colcon build come last.
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
SHELL ["/bin/bash", "-c"]

# ----------------------------------------------------------------------------
# 1) Locale + ROS 2 Jazzy apt repo
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg2 lsb-release software-properties-common ca-certificates && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# 2) System + ROS packages, build toolchain, and native-dep build deps
#    (desktop-full is required to BUILD the rviz/overlay plugins even though we
#     run headless. Boost/TBB/Eigen/glog/SuiteSparse/fmt are for the SLAM deps.)
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop-full \
        ros-jazzy-pcl-ros \
        ros-jazzy-backward-ros \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential cmake ninja-build git pkg-config \
        libpcl-dev nlohmann-json3-dev \
        libeigen3-dev libboost-all-dev libtbb-dev libfmt-dev \
        libgoogle-glog-dev libgflags-dev libsuitesparse-dev libatlas-base-dev \
        libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 && \
    rm -rf /var/lib/apt/lists/*

# AWS CLI is only needed for the S3 bag pull/push in entrypoint.sh, which is
# guarded by BAG_S3_URI / OUTPUT_S3_URI and skipped on local runs. Ubuntu 24.04
# has no `awscli` apt package, so install CLI v2 from AWS only when deploying:
#   RUN curl -sSL "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o /tmp/awscliv2.zip && \
#       apt-get update && apt-get install -y --no-install-recommends unzip && \
#       unzip -q /tmp/awscliv2.zip -d /tmp && /tmp/aws/install && \
#       rm -rf /tmp/aws /tmp/awscliv2.zip && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# 3) SLAM / lidar native dependencies, compiled & installed system-wide.
#    Copied individually so this expensive layer (esp. GTSAM/Ceres) is cached
#    independently of application source changes.
#    MARCH_NATIVE=OFF on GTSAM keeps the binary portable across the build host
#    and the AWS GPU instance CPU.
# ----------------------------------------------------------------------------
WORKDIR /deps

COPY src/slam/dependency/Sophus ./Sophus
RUN rm -rf Sophus/build && \
    cmake -S Sophus -B Sophus/build -DBUILD_TESTS=OFF && \
    cmake --build Sophus/build -j"$(nproc)" && cmake --install Sophus/build

COPY src/slam/dependency/ceres-solver ./ceres-solver
RUN rm -rf ceres-solver/build && \
    cmake -S ceres-solver -B ceres-solver/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build ceres-solver/build -j"$(nproc)" && cmake --install ceres-solver/build

COPY src/slam/dependency/gtsam ./gtsam
RUN rm -rf gtsam/build && \
    cmake -S gtsam -B gtsam/build \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build gtsam/build -j"$(nproc)" && cmake --install gtsam/build

COPY src/utilities/livox_ros_driver2/Livox-SDK2 ./Livox-SDK2
RUN rm -rf Livox-SDK2/build && \
    cmake -S Livox-SDK2 -B Livox-SDK2/build && \
    cmake --build Livox-SDK2/build -j"$(nproc)" && cmake --install Livox-SDK2/build && \
    ldconfig

# ----------------------------------------------------------------------------
# 4) Python / ML stack. Ubuntu 24.04 is PEP-668 externally-managed and ROS uses
#    the system interpreter, so packages go into system site-packages with
#    --break-system-packages (matches README). torch is pinned to the cu126
#    wheels to match the host. FORCE_CUDA + arch list let detectron2/pytorch3d
#    build their CUDA kernels without a GPU present during `docker build`.
#    Arch list covers common AWS GPUs: T4(7.5) A100(8.0) A10G(8.6) L4(8.9) H100(9.0).
# ----------------------------------------------------------------------------
ENV CUDA_HOME=/usr/local/cuda
ENV FORCE_CUDA=1
ENV TORCH_CUDA_ARCH_LIST="7.5;8.0;8.6;8.9;9.0+PTX"
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN pip install --no-cache-dir torch==2.9.0 torchvision==0.24.0 \
        --index-url https://download.pytorch.org/whl/cu126

# psutil ships as a Debian package (python3-psutil, via colcon/desktop-full) with
# no pip RECORD, so pip can't uninstall it when a requirement needs a different
# version. Re-install it as a pip-managed package first (--ignore-installed
# scopes this to psutil so it does NOT touch the cu126 torch wheel above).
RUN pip install --no-cache-dir --ignore-installed psutil blinker

# Pin TensorRT to the host-validated version BEFORE the requirements step.
# requirement.txt lists `tensorrt` unpinned, which otherwise resolves to the
# newest release (11.x, cu13) — a large, untested download. Pre-installing
# 10.16.1.11 satisfies the unpinned line so pip leaves it alone. (This still
# pulls ~1-2 GB of TRT libs, so expect a few minutes here.)
RUN pip install --no-cache-dir tensorrt==10.16.1.11

COPY requirement.txt /tmp/requirement.txt
RUN pip install --no-cache-dir -r /tmp/requirement.txt

# CUDA-compiled vision packages (build from source, need torch + nvcc above)
RUN pip install --no-cache-dir 'git+https://github.com/facebookresearch/detectron2.git' && \
    pip install --no-cache-dir --no-build-isolation \
        'git+https://github.com/facebookresearch/pytorch3d.git' && \
    pip install --no-cache-dir 'git+https://github.com/ultralytics/CLIP.git' && \
    python3 -m spacy download en_core_web_sm

# ----------------------------------------------------------------------------
# 5) Application source + workspace build
# ----------------------------------------------------------------------------
WORKDIR /app
COPY . /app

# sam2 (editable install from vendored source)
RUN pip install --no-cache-dir -e src/semantic_mapping/semantic_mapping/external/sam2

# ----------------------------------------------------------------------------
# 6) Bake GPU-FREE model artifacts into the image.
#    - mobileclip2_b.ts and the YOLO .pt/.onnx/.engine files ship in the repo and
#      are already baked by the COPY above.
#    - SAM2 checkpoints are a plain download (no GPU), so fetch them here.
#    The YOLO TensorRT *engine* export (set_yolo_e.py / set_yolo_world.py) is NOT
#    run here: `docker build` has no GPU, and TRT engines are GPU-architecture
#    specific. That export happens at container startup (see docker/entrypoint.sh),
#    on the actual GPU, and is skipped when a valid engine is already present.
# ----------------------------------------------------------------------------
RUN cd src/semantic_mapping/semantic_mapping/external/sam2/checkpoints && ./download_ckpts.sh

# ----------------------------------------------------------------------------
# 7) Full colcon build (SLAM + lidar driver included).
# ----------------------------------------------------------------------------
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ----------------------------------------------------------------------------
# 8) Entrypoint: source overlays -> pull bag from S3 -> run headless system
#    -> push outputs to S3.
# ----------------------------------------------------------------------------
RUN chmod +x /app/docker/entrypoint.sh /app/docker/run_system.sh

ENV BAG_LOCAL=/data/bag
ENTRYPOINT ["/app/docker/entrypoint.sh"]
