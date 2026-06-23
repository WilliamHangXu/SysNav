#!/usr/bin/env bash
# In-container orchestration for the unified SysNav dev container: provision the
# (mounted) workspace on first run, then start the bridge + the whole scene-graph
# pipeline. The container equivalent of the tmuxp runners, headless-friendly.
#
# Source is NOT baked: `src/` is bind-mounted at /app/src and the colcon build
# lands in a named volume (/app/build,/install,/log), so day-to-day pipeline edits
# need no image rebuild. `docker/` is mounted too, so editing THIS script applies
# on the next run.
#
# Driven by env (set by docker/run.sh):
#   MODE            live | bag | bag-direct          (default live)
#   RVIZ            1|0   launch RViz (needs X)       (default 1)
#   ROS_DOMAIN_ID                                     (default 0)
#   BUILD           1 forces a colcon rebuild
#   FORCE_ENGINE_REBUILD  1 re-exports the TRT engines
#   AUTOPLAY_DELAY  seconds before the pipeline/bag resume (default 12)
#   live: ROS_MASTER_URI (robot), ROS_IP (laptop)
#   bag / bag-direct: BAG_PATH (mounted bag dir)
#   GEMINI_API_KEY / DASHSCOPE_API_KEY ... passed through for the cloud VLM
#
# `supervisor.sh shell` drops into an interactive shell (workspace sourced) for
# the dev inner loop (edit src -> colcon build -> ros2 launch by hand).
set -uo pipefail

MODE="${MODE:-live}"
RVIZ="${RVIZ:-1}"
: "${ROS_DOMAIN_ID:=0}"; export ROS_DOMAIN_ID
: "${AUTOPLAY_DELAY:=12}"
APP=/app
JAZZY_SETUP=/opt/ros/jazzy/setup.bash
NOETIC_SETUP=/opt/ros/noetic/setup.bash
export ROBOT_CONFIG_FILE="${ROBOT_CONFIG_FILE:-$APP/src/exploration_planner/tare_planner/config/robot.yaml}"
export BRIDGE_TOPICS_FILE="${BRIDGE_TOPICS_FILE:-$APP/docker/ros1_bridge/bridge_topics.yaml}"
START_BRIDGE="$APP/docker/ros1_bridge/start_bridge.sh"

cd "$APP"

# ---------------------------------------------------------------------------
# Provision the mounted workspace (idempotent; the slow bits run only once).
# ---------------------------------------------------------------------------
set +u; source "$JAZZY_SETUP"; set -u

SAM2_DIR="$APP/src/semantic_mapping/semantic_mapping/external/sam2"
# sam2 is baked editable in the image (egg-link in site-packages -> the mounted
# sam2 source), so this normally passes instantly. Self-heal only if it's gone.
# find_spec avoids importing torch (which `import sam2` would -- several seconds).
if ! python3 -c "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('sam2') else 1)" 2>/dev/null; then
  echo "[supervisor] sam2 not importable; installing editable from mounted src ..."
  pip install --no-cache-dir -e "$SAM2_DIR"
fi

if [ "${BUILD:-0}" = "1" ] || [ ! -f "$APP/install/setup.bash" ]; then
  echo "[supervisor] colcon build --symlink-install (first run is slow; cached in the named volume) ..."
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
set +u; source "$APP/install/setup.bash"; set -u

# `supervisor.sh shell` -> interactive shell with the workspace sourced.
[ "${1:-}" = "shell" ] && exec bash

# YOLO TensorRT engines are GPU-architecture specific and need a live GPU, so they
# cannot be baked at image-build time -- export here, once, on the real GPU.
EXT="$APP/src/semantic_mapping/semantic_mapping/external"
E1="$EXT/yoloe-26x-seg.engine"; E2="$EXT/yolov8x-worldv2_cus.engine"
[ "${FORCE_ENGINE_REBUILD:-0}" = "1" ] && rm -f "$E1" "$E2"
if [ ! -f "$E1" ] || [ ! -f "$E2" ]; then
  echo "[supervisor] exporting YOLO TensorRT engines on the GPU (one-time) ..."
  ( cd "$APP" && python3 set_yolo_e.py && python3 set_yolo_world.py ) \
    || echo "[supervisor] WARN: engine export failed -- check GPU / torch."
fi

# ---------------------------------------------------------------------------
# Process supervision: background each piece, tear everything down on exit.
# ---------------------------------------------------------------------------
LOG_DIR="$APP/runlogs/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"
echo "[supervisor] MODE=$MODE  RVIZ=$RVIZ  logs -> $LOG_DIR"
PIDS=(); LAST_PID=""; PRIMARY=""

launch() {  # launch <name> <cmd...>
  local n="$1"; shift
  ( exec "$@" ) >"$LOG_DIR/$n.log" 2>&1 &
  LAST_PID=$!; PIDS+=("$LAST_PID")
  echo "[supervisor] started $n (pid $LAST_PID) -> $LOG_DIR/$n.log"
}
cleanup() {
  echo "[supervisor] tearing down (${#PIDS[@]} processes)"
  for p in "${PIDS[@]}"; do kill -INT  "$p" 2>/dev/null || true; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

start_pipeline() {  # start_pipeline <use_sim_time>
  launch pipeline bash -c \
    "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
     exec ros2 launch tare_planner scene_graph.launch use_sim_time:=$1 rviz:=$RVIZ"
}

case "$MODE" in
  live)
    # Robot is the ROS 1 master; bridge talks to it, pipeline runs on wall clock.
    : "${ROS_MASTER_URI:?live mode needs ROS_MASTER_URI=http://<robot-ip>:11311}"
    launch bridge bash "$START_BRIDGE"
    sleep 3                       # let the eager bridges come up
    start_pipeline false; PRIMARY=$LAST_PID
    ;;

  bag)
    # ROS 1 bag through the bridge: in-container roscore + rosbag --clock.
    : "${BAG_PATH:?bag mode needs BAG_PATH (a ROS 1 .bag directory)}"
    export ROS_MASTER_URI=http://localhost:11311 ROS_IP=127.0.0.1
    launch roscore bash -c "source $NOETIC_SETUP; exec roscore"
    for i in $(seq 1 20); do
      bash -c "source $NOETIC_SETUP; rosparam list" >/dev/null 2>&1 && break; sleep 1
    done
    launch bridge bash "$START_BRIDGE"
    launch bag bash -c "source $NOETIC_SETUP; exec rosbag play --clock $BAG_PATH/*.bag"
    BAG_PID=$LAST_PID
    # /tf_static survives via the bridge's transient_local QoS even if the pipeline
    # starts a few seconds in; the delay just primes /clock so TARE doesn't see the
    # 0 -> bag-time jump.
    echo "[supervisor] priming /clock for ${AUTOPLAY_DELAY}s before the pipeline ..."
    sleep "$AUTOPLAY_DELAY"
    start_pipeline true
    PRIMARY=$BAG_PID              # exit when the bag finishes
    ;;

  bag-direct)
    # ROS 2 bag straight into the pipeline (no bridge) -- the fast dev/test path.
    # Start PAUSED so /clock is alive at bag-start before the planner boots, then
    # resume once the stack is up (mirrors the tmuxp runner).
    : "${BAG_PATH:?bag-direct mode needs BAG_PATH (a ROS 2 bag directory)}"
    launch bag bash -c \
      "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
       exec ros2 bag play '$BAG_PATH' --clock --start-paused --disable-keyboard-controls < /dev/null"
    BAG_PID=$LAST_PID
    start_pipeline true
    echo "[supervisor] waiting ${AUTOPLAY_DELAY}s for the stack, then resuming the bag ..."
    sleep "$AUTOPLAY_DELAY"
    bash -c "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
             ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'" || true
    PRIMARY=$BAG_PID              # exit when the bag finishes
    ;;

  *) echo "[supervisor] unknown MODE=$MODE (use live | bag | bag-direct)" >&2; exit 2 ;;
esac

# Block on the primary process; the EXIT trap tears the rest down.
wait "$PRIMARY" 2>/dev/null || true
echo "[supervisor] primary process exited; shutting down."
