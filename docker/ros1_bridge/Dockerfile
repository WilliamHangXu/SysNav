# syntax=docker/dockerfile:1
#
# Runtime bridge image: ROS 1 Noetic <-> ROS 2 Jazzy `dynamic_bridge` for the
# SysNav scene-graph pipeline.
#
# WHY this is built ON TOP of an upstream image instead of from scratch:
# bridging Noetic (Ubuntu 20.04) and Jazzy (Ubuntu 24.04) means compiling
# ros1_bridge against two distros that target different Ubuntu releases. The
# proven upstream builder (TommyChangUMD/ros-jazzy-ros1-bridge-builder) compiles
# ros1_bridge against Jazzy + a Focal-Noetic copy and handles the cross-Ubuntu
# linker gotchas (e.g. the lttng/tracetools `-llttng-ust-common` failure) for us.
# We add only a thin entrypoint so the container RUNS the bridge instead of the
# upstream default (which just emits a precompiled tarball to stdout).
#
# Our boundary is all-standard message types (PointCloud2, Odometry, Image,
# CameraInfo, TFMessage, Clock), so the bridge's built-in mappings cover
# everything — no custom-message rebuild needed.
#
# ── PREREQUISITE: build the upstream builder image once ───────────────────────
#   git clone --recurse-submodules \
#     https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder.git
#   cd ros-jazzy-ros1-bridge-builder
#   docker build . -t ros-jazzy-ros1-bridge-builder:latest
#
# ── Then build this image ─────────────────────────────────────────────────────
#   docker build -t sysnav-ros1-bridge:latest docker/ros1_bridge
#
ARG BUILDER_IMAGE=ros-jazzy-ros1-bridge-builder:latest
FROM ${BUILDER_IMAGE}

COPY start_bridge.sh /usr/local/bin/start_bridge.sh
COPY entrypoint.sh /usr/local/bin/bridge-entrypoint.sh
RUN chmod +x /usr/local/bin/bridge-entrypoint.sh /usr/local/bin/start_bridge.sh

# Must match the host pipeline's ROS_DOMAIN_ID (override at run time).
ENV ROS_DOMAIN_ID=0
# Default ROS 1 master = same host (host networking). Override for the live robot.
ENV ROS_MASTER_URI=http://localhost:11311

ENTRYPOINT ["/usr/local/bin/bridge-entrypoint.sh"]
CMD ["bridge"]
