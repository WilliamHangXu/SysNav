# syntax=docker/dockerfile:1
#
# SysNav unified DEV container — ROS 1 Noetic <-> ROS 2 Jazzy bridge + the whole
# scene-graph pipeline in ONE image, GPU, for live-robot (and bag) development.
#
# DESIGN: bake the stable floor, MOUNT the volatile workspace.
#   Baked here  : Ubuntu+CUDA, ROS 2 Jazzy (desktop-full), ROS 1 Noetic + the
#                 compiled ros1_bridge (grafted from the upstream builder), native
#                 SLAM deps (Sophus/Ceres/GTSAM/Livox), the Python/ML stack.
#   NOT baked   : the application source, the colcon build, the model weights.
#                 `src/` (which already contains the SAM2/CLIP/YOLO weights) is
#                 bind-mounted at runtime and `colcon build --symlink-install` runs
#                 IN-container into a named volume -- so constant pipeline dev needs
#                 no image rebuild. See docker/supervisor.sh + docker/run.sh.
#
# Base: CUDA 12.6.3 devel (matches host: nvcc 12.6, torch 2.9.0+cu126) on Ubuntu
# 24.04 = the ROS 2 Jazzy platform AND the builder's platform (noble), so the
# grafted Noetic/bridge binaries are ABI-compatible (validated: step-1 graft spike).
# `devel` (not runtime) is required because we compile CUDA extensions at build
# time (detectron2, pytorch3d) and several CUDA-aware ROS packages.
#
# Build (two images, once):
#   1. the upstream bridge compiler (see docker/ros1_bridge/README.md):
#        docker build . -t ros-jazzy-ros1-bridge-builder:latest   # in that repo
#   2. this image (context = repo root):
#        docker build -f docker/Dockerfile -t sysnav:latest .
#
# Layer order is chosen for cache reuse: rarely-changing system + ROS + native
# deps + python deps; the bridge graft is last (small, only invalidates if the
# builder image changes).

# The compiled bridge + noble-native Noetic come from the upstream builder image.
ARG BRIDGE_IMAGE=ros-jazzy-ros1-bridge-builder:latest
FROM ${BRIDGE_IMAGE} AS bridge

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
#    (desktop-full is required to BUILD the rviz/overlay plugins and to RUN RViz
#     in the container. Boost/TBB/Eigen/glog/SuiteSparse/fmt are for the SLAM deps.
#     python3-catkin-pkg: catkin_make against the grafted Noetic needs the
#     catkin_find_pkg script (the bridge graft bundles only -modules); used to
#     build ROS 1 packages like scene_graph in-container.)
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop-full \
        ros-jazzy-pcl-ros \
        ros-jazzy-backward-ros \
        python3-colcon-common-extensions \
        python3-catkin-pkg \
        python3-pip \
        build-essential cmake ninja-build git pkg-config \
        libpcl-dev nlohmann-json3-dev \
        libeigen3-dev libboost-all-dev libtbb-dev libfmt-dev \
        libgoogle-glog-dev libgflags-dev libsuitesparse-dev libatlas-base-dev \
        libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# 3) SLAM / lidar native dependencies, compiled & installed system-wide.
#    Copied individually so this expensive layer (esp. GTSAM/Ceres) is cached
#    independently of application source changes. These are vendored third-party
#    deps that rarely change, so baking them (vs mounting) is correct.
#    MARCH_NATIVE=OFF on GTSAM keeps the binary portable across build/run CPUs.
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
#    Arch list spans Turing..Hopper so the image is portable across dev GPUs.
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

# Same Debian-RECORD problem as psutil/blinker above, but for the google-auth
# chain pulled by google-generativeai/openai: PyJWT (python3-jwt) and cryptography
# ship as Debian packages with no pip RECORD, so pip can't replace them when a
# requirement.txt dependency needs a different version ("Cannot uninstall PyJWT
# ..., RECORD file not found"). Pre-install them as pip-managed so the -r step can.
RUN pip install --no-cache-dir --ignore-installed pyjwt cryptography

COPY requirement.txt /tmp/requirement.txt
RUN pip install --no-cache-dir -r /tmp/requirement.txt

# CUDA-compiled vision packages (build from source, need torch + nvcc above)
RUN pip install --no-cache-dir 'git+https://github.com/facebookresearch/detectron2.git' && \
    pip install --no-cache-dir --no-build-isolation \
        'git+https://github.com/facebookresearch/pytorch3d.git' && \
    pip install --no-cache-dir 'git+https://github.com/ultralytics/CLIP.git' && \
    python3 -m spacy download en_core_web_sm

# ----------------------------------------------------------------------------
# 5) ROS 1 Noetic + compiled ros1_bridge GRAFT (validated by the step-1 spike).
#    The builder image is noble-native, so its /opt/ros/noetic + bridge install
#    are ABI-compatible here. The bridge install BUNDLES its own ROS 1 libs
#    (log4cxx, apr, roscpp, rosconsole, ...) so only the items below are needed:
#      - example-interfaces/map-msgs/turtlesim: the bridge binary links these as
#        NEEDED message-typesupport libs (it was compiled with mappings for every
#        pairable type incl. demo msgs); mandatory even though we never bridge them.
#      - python3-defusedxml/rospkg/netifaces: Noetic python runtime for roscore +
#        `rosparam load` of the bridge allowlist (roscore dies without defusedxml).
#      - python3-pycryptodome/gnupg: rosbag CLI hard-imports Cryptodome + gnupg at
#        module load; without them `rosbag play/info` dies on import (bag mode).
#      - libgpgme11t64: rosbag's default encryption plugin dlopens libgpgme.so.11;
#        `rosbag play` FATALs at startup without it.
# ----------------------------------------------------------------------------
COPY --from=bridge /opt/ros/noetic /opt/ros/noetic
COPY --from=bridge /ros-jazzy-ros1-bridge /ros-jazzy-ros1-bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-example-interfaces ros-jazzy-map-msgs ros-jazzy-turtlesim \
        python3-defusedxml python3-rospkg python3-netifaces \
        python3-pycryptodome python3-gnupg libgpgme11t64 && \
    rm -rf /var/lib/apt/lists/*
# rosbag's C++ play/record binaries link liblog4cxx.so.15 (+ its apr/aprutil
# deps), which the graft only bundles inside the bridge install dir (on
# LD_LIBRARY_PATH only when the bridge env is sourced). Link them onto Noetic's
# lib path so plain `source noetic` works; ldd of rosbag/play then fully resolves.
RUN for lib in liblog4cxx.so.15 libapr-1.so.0 libaprutil-1.so.0; do \
        ln -s /ros-jazzy-ros1-bridge/install/ros1_bridge/lib/$lib \
              /opt/ros/noetic/lib/$lib; \
    done

# ----------------------------------------------------------------------------
# 6) Bake the sam2 editable install. sam2 is vendored/stable (you develop the
#    pipeline, not sam2), so install it ONCE here instead of on every container
#    start. The editable metadata is written to site-packages (an IMAGE layer),
#    which a runtime `pip install -e` could not do persistently -- it landed in
#    the container's writable layer and was discarded by `docker run --rm`, so
#    sam2 re-installed every run. We COPY only the sam2 package source (its big
#    checkpoints are .dockerignore'd and arrive via the src mount) to the SAME
#    path it is mounted at runtime, and `pip install -e` it there: the baked
#    egg-link then resolves to your live, mounted sam2 source at runtime.
# ----------------------------------------------------------------------------
COPY src/semantic_mapping/semantic_mapping/external/sam2 \
     /app/src/semantic_mapping/semantic_mapping/external/sam2
RUN pip install --no-cache-dir -e \
        /app/src/semantic_mapping/semantic_mapping/external/sam2

# ----------------------------------------------------------------------------
# 7) Runtime layout. Apart from the sam2 install above, NO source is baked:
#    `src/` is bind-mounted to /app/src and the colcon build lands in a named
#    volume (/app/build, /app/install, /app/log) at runtime. The launch scripts
#    (docker/) are mounted too, so edits to the supervisor / run.sh apply without
#    rebuilding. See docker/run.sh.
# ----------------------------------------------------------------------------
WORKDIR /app
ENTRYPOINT ["/app/docker/supervisor.sh"]
