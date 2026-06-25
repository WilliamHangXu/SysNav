#!/usr/bin/env bash
# Host launcher for the unified SysNav dev container -- one command, a few knobs.
# Bakes the stable floor (image); MOUNTS the volatile workspace (src/ + the colcon
# build in a named volume), so editing the pipeline needs no image rebuild.
#
# Usage (knobs are env vars):
#   MODE=live ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh
#   MODE=demo ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh  # robot-gated live
#   MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh
#   MODE=bag        BAG=/home/all/AlphaZ/bags/multifloor_test_slam     docker/run.sh
#   docker/run.sh shell          # debug shell in the container (workspace sourced)
#   BUILD=1 ... docker/run.sh     # force a colcon rebuild (e.g. after C++ changes)
#
# Knobs: IMAGE MODE ROS_DOMAIN_ID RVIZ OBJECTS ROBOT_IP LAPTOP_IP BAG BUILD VOLUME NAME
#        START_OFFSET DURATION  (bag/bag-direct: seconds to skip / play; default = whole bag)
#        HOLD  (bag/bag-direct/demo: default 1 keeps the stack incl. RViz up after the
#              bag ends / the robot's demo run finishes; HOLD=0 auto-exits. Ctrl-C quits)
#        RECORD  (live/demo: RECORD=1 records a ROS 2 bag of the pipeline inputs to
#              output/recordings/<ts>/, replayable in bag-direct; off by default)
#   The container is named NAME (default 'sysnav'), so a second terminal can attach:
#     docker exec -it sysnav bash      # ROS-ready (~/.bashrc sources ROS 2 + workspace)
#   Set NAME=... to run more than one container at once.
#   bag/bag-direct confine ROS 2 discovery to this host (ROS_AUTOMATIC_DISCOVERY_RANGE
#   =LOCALHOST) so two laptops on the same WiFi don't cross-wire; override with
#   ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET if you ever need cross-host ROS 2 there.
#   RVIZ=1 (default) forwards X; you may need `xhost +local:root` once on the host.
#   OBJECTS=1 adds object detection+mapping; default 0 = rooms + navgraph only.
#   Cloud-VLM keys (GEMINI_API_KEY / DASHSCOPE_API_KEY / VLM_PROVIDER) pass through
#   from your environment if set.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

IMAGE="${IMAGE:-sysnav:latest}"
MODE="${MODE:-live}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RVIZ="${RVIZ:-1}"
OBJECTS="${OBJECTS:-0}"
BUILD="${BUILD:-0}"
VOLUME="${VOLUME:-sysnav-build}"   # named volume holding /app/{build,install,log}

# Bind targets for artifacts must exist as the host user (else docker makes them root).
# output/recordings is pre-created too so RECORD=1 bags land host-owned, not root.
mkdir -p "$REPO/output" "$REPO/output/recordings" "$REPO/runlogs"

run=( docker run --rm -it
  --name "${NAME:-sysnav}"                 # so `docker exec -it sysnav bash` is predictable
  --gpus all --network host --ipc host
  -e MODE="$MODE" -e RVIZ="$RVIZ" -e OBJECTS="$OBJECTS" -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e BUILD="$BUILD"
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

# Optional demo-mode (MODE=demo) overrides; the supervisor bakes sane defaults.
for k in REQ_TOPIC RESP_TOPIC ACK_TIMEOUT; do
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
  live|demo)
    # demo == live wiring (robot is the ROS 1 master), but the in-container
    # supervisor gates the pipeline on a robot request instead of auto-starting.
    : "${ROBOT_IP:?$MODE mode needs ROBOT_IP=<robot ip on its subnet, e.g. 192.168.123.18>}"
    : "${LAPTOP_IP:?$MODE mode needs LAPTOP_IP=<your ip on the robot subnet, e.g. 192.168.123.190>}"
    # HOLD is demo-only here: default 1 keeps the stack (incl. RViz) up after the
    # robot's run finishes (HOLD=0 auto-exits). Harmless in live (its supervisor
    # ignores it -- live already blocks on the pipeline until Ctrl-C).
    # RECORD=1 records a ROS 2 bag of the pipeline inputs (off by default); see
    # the supervisor header. Records the in-container ROS 2 side, so no WiFi cost.
    run+=( -e ROS_MASTER_URI="http://$ROBOT_IP:11311" -e ROS_IP="$LAPTOP_IP"
           -e HOLD="${HOLD:-1}" -e RECORD="${RECORD:-0}" )
    ;;
  bag|bag-direct)
    : "${BAG:?$MODE mode needs BAG=<bag directory>}"
    [ -d "$BAG" ] || { echo "BAG '$BAG' is not a directory" >&2; exit 1; }
    # START_OFFSET / DURATION (seconds) trim playback; empty = whole bag.
    # HOLD (default 1) keeps the stack (incl. RViz) up after the bag finishes;
    # HOLD=0 auto-exits when the bag ends (scripted/batch runs).
    # Confine ROS 2 discovery to this host: bag modes are a self-contained ROS 2
    # graph (bag -> pipeline, no robot), so they never need to talk off-box. With
    # --network host + the default ROS_DOMAIN_ID, two people on the same LAN/WiFi
    # would otherwise auto-discover each other and cross-wire /clock, /tf, node
    # names and /rosbag2_player -- breaking both runs. LOCALHOST keeps each laptop
    # isolated. (live/demo reach the robot over ROS 1, not ROS 2, so this is moot
    # there -- left unset to preserve their default subnet discovery.)
    run+=( -e BAG_PATH=/app/bag -v "$BAG:/app/bag:ro"
           -e START_OFFSET="${START_OFFSET:-}" -e DURATION="${DURATION:-}"
           -e HOLD="${HOLD:-1}"
           -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" )
    ;;
  *) echo "unknown MODE=$MODE (use live | demo | bag | bag-direct)" >&2; exit 2 ;;
esac

# Pass-through extra args (e.g. `shell`) to the supervisor entrypoint.
run+=( "$IMAGE" "$@" )

echo "+ ${run[*]}"

# RViz (run as root in the container) can't open the host X server until it's
# authorized -- otherwise it aborts with "Authorization required ... could not
# connect to display :0" and the pipeline runs on, headless. Grant local root
# access for the lifetime of this run and revoke on exit, so we don't leave the
# host X server loosened. Skip silently if xhost isn't installed / no X.
if [ "$RVIZ" = "1" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
  trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT
  "${run[@]}"
else
  exec "${run[@]}"
fi
