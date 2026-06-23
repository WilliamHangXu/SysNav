#!/usr/bin/env bash
# Host launcher for the unified SysNav dev container -- one command, a few knobs.
# Bakes the stable floor (image); MOUNTS the volatile workspace (src/ + the colcon
# build in a named volume), so editing the pipeline needs no image rebuild.
#
# Usage (knobs are env vars):
#   MODE=live ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh
#   MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh
#   MODE=bag        BAG=/home/all/AlphaZ/bags/multifloor_test_slam     docker/run.sh
#   docker/run.sh shell          # debug shell in the container (workspace sourced)
#   BUILD=1 ... docker/run.sh     # force a colcon rebuild (e.g. after C++ changes)
#
# Knobs: IMAGE MODE ROS_DOMAIN_ID RVIZ ROBOT_IP LAPTOP_IP BAG BUILD VOLUME
#   RVIZ=1 (default) forwards X; you may need `xhost +local:root` once on the host.
#   Cloud-VLM keys (GEMINI_API_KEY / DASHSCOPE_API_KEY / VLM_PROVIDER) pass through
#   from your environment if set.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

IMAGE="${IMAGE:-sysnav:latest}"
MODE="${MODE:-live}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RVIZ="${RVIZ:-1}"
BUILD="${BUILD:-0}"
VOLUME="${VOLUME:-sysnav-build}"   # named volume holding /app/{build,install,log}

# Bind targets for artifacts must exist as the host user (else docker makes them root).
mkdir -p "$REPO/output" "$REPO/runlogs"

run=( docker run --rm -it
  --gpus all --network host --ipc host
  -e MODE="$MODE" -e RVIZ="$RVIZ" -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e BUILD="$BUILD"
  -e FORCE_ENGINE_REBUILD="${FORCE_ENGINE_REBUILD:-0}"
  -v "$VOLUME:/app"                       # build/install/log persist here
  -v "$REPO/src:/app/src"                 # the volatile workspace (mounted, not baked)
  -v "$REPO/docker:/app/docker:ro"        # supervisor + scripts (edit-without-rebuild)
  -v "$REPO/output:/app/output"
  -v "$REPO/runlogs:/app/runlogs" )

# Cloud-VLM credentials / provider knobs (only if present in the environment).
for k in GEMINI_API_KEY DASHSCOPE_API_KEY VLM_PROVIDER QWEN_MODEL QWEN_MODEL_LITE; do
  [ -n "${!k:-}" ] && run+=( -e "$k=${!k}" )
done

# RViz needs the host X server AND a GL stack. The CUDA base only requests the
# `compute,utility` driver capabilities, so the container has no OpenGL -- RViz
# falls back to Mesa, fails to reach a DRM device, and dies ("failed to load
# driver: iris"). NVIDIA_DRIVER_CAPABILITIES=all makes the toolkit inject the
# NVIDIA GL/GLX libs (RViz renders on the dGPU); mounting /dev/dri also lets the
# Mesa/iGPU path work as a fallback. QT_X11_NO_MITSHM avoids a Qt/X SHM crash.
if [ "$RVIZ" = "1" ]; then
  run+=( -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix
         -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all
         -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root )
  [ -d /dev/dri ] && run+=( --device /dev/dri )
fi

case "$MODE" in
  live)
    : "${ROBOT_IP:?live mode needs ROBOT_IP=<robot ip on its subnet, e.g. 192.168.123.18>}"
    : "${LAPTOP_IP:?live mode needs LAPTOP_IP=<your ip on the robot subnet, e.g. 192.168.123.190>}"
    run+=( -e ROS_MASTER_URI="http://$ROBOT_IP:11311" -e ROS_IP="$LAPTOP_IP" )
    ;;
  bag|bag-direct)
    : "${BAG:?$MODE mode needs BAG=<bag directory>}"
    [ -d "$BAG" ] || { echo "BAG '$BAG' is not a directory" >&2; exit 1; }
    run+=( -e BAG_PATH=/app/bag -v "$BAG:/app/bag:ro" )
    ;;
  *) echo "unknown MODE=$MODE (use live | bag | bag-direct)" >&2; exit 2 ;;
esac

# Pass-through extra args (e.g. `shell`) to the supervisor entrypoint.
run+=( "$IMAGE" "$@" )

echo "+ ${run[*]}"
exec "${run[@]}"
