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
#   MODE            live | demo | bag | bag-direct   (default live)
#   RVIZ            1|0   launch RViz (needs X)       (default 1)
#   OBJECTS         1|0   run object detection+mapping (default 0 = rooms +
#                         navgraph only, also skips the GPU YOLO engine export);
#                         1 = full pipeline with objects
#   ROS_DOMAIN_ID                                     (default 0)
#   BUILD           1 forces a colcon rebuild
#   FORCE_ENGINE_REBUILD  1 re-exports the TRT engines
#   AUTOPLAY_DELAY  seconds before the pipeline/bag resume (default 12)
#   live: ROS_MASTER_URI (robot), ROS_IP (laptop)
#   demo: like live, but the pipeline is gated -- waits for "start" on
#         /scene_graph_generator/request before launching; optional
#         REQ_TOPIC / RESP_TOPIC / ACK_TIMEOUT overrides. HOLD (default 1) keeps the
#         stack (incl. RViz) up after the robot's run finishes; HOLD=0 auto-exits.
#   live / demo: RECORD=1 records a ROS 2 bag of the pipeline INPUTS (exactly what
#         the pipeline receives -- the bridged sensor/TF topics) to
#         output/recordings/<ts>/, replayable in bag-direct. Off by default; records
#         from the in-container ROS 2 side so it adds no WiFi load. demo records only
#         the run window (start->done); live records for the container's lifetime.
#   bag / bag-direct: BAG_PATH (mounted bag dir);
#                     START_OFFSET / DURATION (seconds) trim playback (empty = whole
#                     bag). A non-zero START_OFFSET also starts a /tf_static primer,
#                     since the offset skips the bag's latched static-TF tree.
#                     HOLD (default 1) keeps the stack (incl. RViz) up after the bag
#                     finishes, for inspection (Ctrl-C to quit); HOLD=0 auto-exits.
#   GEMINI_API_KEY / DASHSCOPE_API_KEY ... passed through for the cloud VLM
#
# `supervisor.sh shell` drops into an interactive shell (workspace sourced) for
# the dev inner loop (edit src -> colcon build -> ros2 launch by hand).
set -uo pipefail

MODE="${MODE:-live}"
RVIZ="${RVIZ:-1}"
OBJECTS="${OBJECTS:-0}"
RECORD="${RECORD:-0}"   # live/demo: 1 records a ROS 2 bag of the pipeline inputs
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

# Make `docker exec -it <container> bash` immediately ROS-ready: a plain interactive
# shell sources ~/.bashrc, so drop the ROS 2 + workspace sourcing there. Lets you
# pop a second terminal into a running live/demo (or bag) container and `ros2 topic
# hz ...` with no manual setup. Idempotent (marker-guarded); ROS 2 only -- source
# /opt/ros/noetic/setup.bash by hand if you need ROS 1 (`rostopic`) in that shell.
if ! grep -q 'SYSNAV-EXEC-SHELL' /root/.bashrc 2>/dev/null; then
  {
    echo ''
    echo '# >>> SYSNAV-EXEC-SHELL (added by supervisor.sh) >>>'
    echo "source $JAZZY_SETUP"
    echo "[ -f $APP/install/setup.bash ] && source $APP/install/setup.bash"
    echo '# <<< SYSNAV-EXEC-SHELL <<<'
  } >> /root/.bashrc
fi

# `supervisor.sh shell` -> interactive shell with the workspace sourced.
[ "${1:-}" = "shell" ] && exec bash

# YOLO TensorRT engines are GPU-architecture specific and need a live GPU, so they
# cannot be baked at image-build time -- export here, once, on the real GPU. Skipped
# when OBJECTS=0: the object branch (which is the only consumer) isn't launched.
if [ "$OBJECTS" = "1" ]; then
  EXT="$APP/src/semantic_mapping/semantic_mapping/external"
  E1="$EXT/yoloe-26x-seg.engine"; E2="$EXT/yolov8x-worldv2_cus.engine"
  [ "${FORCE_ENGINE_REBUILD:-0}" = "1" ] && rm -f "$E1" "$E2"
  if [ ! -f "$E1" ] || [ ! -f "$E2" ]; then
    echo "[supervisor] exporting YOLO TensorRT engines on the GPU (one-time) ..."
    ( cd "$APP" && python3 set_yolo_e.py && python3 set_yolo_world.py ) \
      || echo "[supervisor] WARN: engine export failed -- check GPU / torch."
  fi
fi

# ---------------------------------------------------------------------------
# Process supervision: background each piece, tear everything down on exit.
# ---------------------------------------------------------------------------
RUN_TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$APP/runlogs/$RUN_TS"; mkdir -p "$LOG_DIR"
echo "[supervisor] MODE=$MODE  RVIZ=$RVIZ  OBJECTS=$OBJECTS  RECORD=$RECORD  logs -> $LOG_DIR"
PIDS=(); LAST_PID=""; PRIMARY=""; REC_PID=""

launch() {  # launch <name> <cmd...>
  local n="$1"; shift
  ( exec "$@" ) >"$LOG_DIR/$n.log" 2>&1 &
  LAST_PID=$!; PIDS+=("$LAST_PID")
  echo "[supervisor] started $n (pid $LAST_PID) -> $LOG_DIR/$n.log"
}
cleanup() {
  echo "[supervisor] tearing down (${#PIDS[@]} processes)"
  # Finalize the bag FIRST: SIGINT the recorder and give rosbag2 time to flush +
  # close its storage file, else the general KILL below can truncate/un-index the
  # bag. (In demo, stop_recorder already did this and cleared REC_PID -> skipped.)
  if [ -n "$REC_PID" ]; then
    echo "[supervisor] finalizing bag recorder (pid $REC_PID) ..."
    kill -INT "$REC_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
  fi
  for p in "${PIDS[@]}"; do kill -INT  "$p" 2>/dev/null || true; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

start_pipeline() {  # start_pipeline <use_sim_time>
  launch pipeline bash -c \
    "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
     exec ros2 launch tare_planner scene_graph.launch use_sim_time:=$1 rviz:=$RVIZ objects:=$OBJECTS"
}

# The bag-record input set = the bridged pipeline inputs, rendered for the robot
# namespace, minus /clock (regenerated by `ros2 bag play --clock` on replay) and
# the demo control channel. Reuses bridge_topics.yaml so the record set always
# tracks the bridge allowlist -- "record exactly what the pipeline receives".
record_topics() {
  local ns
  [ -f "$BRIDGE_TOPICS_FILE" ] || return 0
  ns="$(grep -E '^[[:space:]]*robot_namespace:' "$ROBOT_CONFIG_FILE" 2>/dev/null \
        | head -1 | sed -E 's/.*robot_namespace:[[:space:]]*//; s/#.*$//; s/[[:space:]]*$//')"
  [ -n "$ns" ] || return 0
  grep -E '^[[:space:]]*-[[:space:]]*topic:' "$BRIDGE_TOPICS_FILE" \
    | sed -E 's/.*topic:[[:space:]]*//; s/#.*$//; s/[[:space:]]*$//' \
    | sed "s|__NS__|$ns|g" \
    | grep -vE '^/clock$|^/scene_graph_generator/'
}

start_recorder() {  # start_recorder -- records the pipeline inputs to a ROS 2 bag
  # Records the in-container ROS 2 side (the already-bridged inputs), so it adds
  # ZERO WiFi load -- the data is on this host already. Replayable in bag-direct.
  local out="$APP/output/recordings/$RUN_TS" topics
  topics="$(record_topics | tr '\n' ' ')"
  if [ -z "${topics// }" ]; then
    echo "[supervisor] RECORD=1 but no input topics resolved from $BRIDGE_TOPICS_FILE; skipping recorder." >&2
    return 0
  fi
  mkdir -p "$(dirname "$out")"   # ros2 bag record creates the leaf; parent must exist
  echo "[supervisor] RECORD=1: ros2 bag record -> $out"
  echo "[supervisor]   topics: $topics"
  launch recorder bash -c \
    "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
     exec ros2 bag record -o '$out' $topics"
  REC_PID=$LAST_PID
}

stop_recorder() {  # stop_recorder -- SIGINT + wait so rosbag2 finalizes the bag
  [ -n "$REC_PID" ] || return 0
  echo "[supervisor] stopping bag recorder (pid $REC_PID) to finalize the bag ..."
  kill -INT "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true   # block until the storage file is closed
  REC_PID=""
}

run_helper() {  # run_helper <demo_control.py args...> -- foreground; returns its exit code
  # The demo-mode control helper (rclpy) blocks on robot requests; sourced env
  # mirrors start_pipeline. Args after the function name pass through verbatim.
  bash -c "set +u; source '$JAZZY_SETUP'; source '$APP/install/setup.bash'; \
           exec python3 '$APP/docker/demo_control.py' \"\$@\"" demo_control "$@"
}

case "$MODE" in
  live)
    # Robot is the ROS 1 master; bridge talks to it, pipeline runs on wall clock.
    : "${ROS_MASTER_URI:?live mode needs ROS_MASTER_URI=http://<robot-ip>:11311}"
    launch bridge bash "$START_BRIDGE"
    sleep 3                       # let the eager bridges come up
    start_pipeline false; PRIMARY=$LAST_PID
    # RECORD=1: capture the pipeline inputs for the container's lifetime (the bag
    # is finalized by cleanup's SIGINT on Ctrl-C / pipeline exit).
    [ "$RECORD" = "1" ] && start_recorder
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
    # START_OFFSET / DURATION map to rosbag play's --start / --duration (seconds).
    R1_OPTS=""
    [ -n "${START_OFFSET:-}" ] && R1_OPTS="$R1_OPTS --start $START_OFFSET"
    [ -n "${DURATION:-}" ]     && R1_OPTS="$R1_OPTS --duration $DURATION"
    launch bag bash -c "source $NOETIC_SETUP; exec rosbag play --clock $R1_OPTS $BAG_PATH/*.bag"
    BAG_PID=$LAST_PID
    # A non-zero --start skips the bag's latched /tf_static (the robot tf tree the
    # camera<-base calibration needs), so re-emit it with a second looping player
    # (no --clock). With offset 0 the main player already carries it.
    if [ "${START_OFFSET:-0}" != "0" ]; then
      launch tf_static_primer bash -c \
        "source $NOETIC_SETUP; exec rosbag play --loop $BAG_PATH/*.bag --topics /tf_static"
    fi
    # /tf_static survives via the bridge's transient_local QoS even if the pipeline
    # starts a few seconds in; the delay just primes /clock so TARE doesn't see the
    # 0 -> bag-time jump.
    echo "[supervisor] priming /clock for ${AUTOPLAY_DELAY}s before the pipeline ..."
    sleep "$AUTOPLAY_DELAY"
    start_pipeline true; PIPE_PID=$LAST_PID
    # HOLD (default 1) blocks on the pipeline so the stack (incl. RViz) stays up for
    # inspection after playback; HOLD=0 exits when the bag finishes (scripted runs).
    PRIMARY=$BAG_PID
    if [ "${HOLD:-1}" = "1" ]; then
      PRIMARY=$PIPE_PID
      echo "[supervisor] HOLD=1: stack (incl. RViz) stays up after the bag ends -- Ctrl-C to quit."
    fi
    ;;

  bag-direct)
    # ROS 2 bag straight into the pipeline (no bridge) -- the fast dev/test path.
    # Start PAUSED so /clock is alive at bag-start before the planner boots, then
    # resume once the stack is up (mirrors the tmuxp runner).
    : "${BAG_PATH:?bag-direct mode needs BAG_PATH (a ROS 2 bag directory)}"
    # START_OFFSET / DURATION map to ros2 bag play --start-offset / --playback-duration.
    R2_OPTS=""
    [ -n "${START_OFFSET:-}" ] && R2_OPTS="$R2_OPTS --start-offset $START_OFFSET"
    [ -n "${DURATION:-}" ]     && R2_OPTS="$R2_OPTS --playback-duration $DURATION"
    launch bag bash -c \
      "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
       exec ros2 bag play '$BAG_PATH' $R2_OPTS --clock --start-paused --disable-keyboard-controls < /dev/null"
    BAG_PID=$LAST_PID
    start_pipeline true; PIPE_PID=$LAST_PID
    echo "[supervisor] waiting ${AUTOPLAY_DELAY}s for the stack, then resuming the bag ..."
    sleep "$AUTOPLAY_DELAY"
    bash -c "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
             ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'" || true
    # A non-zero --start-offset skips the bag's latched /tf_static (the robot tf tree
    # incl. base->front_cam->front_cam_ar that camera<-base calibration needs). Re-emit
    # it with a second player looping just /tf_static from bag start (no --clock).
    # Started AFTER the resume so only the main player owns /rosbag2_player/resume
    # (no service-name collision). Mirrors the tmuxp runner.
    if [ "${START_OFFSET:-0}" != "0" ]; then
      launch tf_static_primer bash -c \
        "set +u; source $JAZZY_SETUP; source $APP/install/setup.bash; \
         exec ros2 bag play '$BAG_PATH' --topics /tf_static --loop --disable-keyboard-controls < /dev/null"
    fi
    # HOLD (default 1) blocks on the pipeline so the stack (incl. RViz) stays up for
    # inspection after playback; HOLD=0 exits when the bag finishes (scripted runs).
    PRIMARY=$BAG_PID
    if [ "${HOLD:-1}" = "1" ]; then
      PRIMARY=$PIPE_PID
      echo "[supervisor] HOLD=1: stack (incl. RViz) stays up after the bag ends -- Ctrl-C to quit."
    fi
    ;;

  demo)
    # Live robot, but the pipeline is GATED on the robot's request instead of
    # starting immediately. The robot drives the run over the String control
    # channel /scene_graph_generator/{request,response}: "start" launches the
    # pipeline, "complete" saves + streams the scene-graph JSON back until
    # "received", "cancel" saves locally and tears down. See docker/demo_control.py.
    : "${ROS_MASTER_URI:?demo mode needs ROS_MASTER_URI=http://<robot-ip>:11311}"
    REQ_TOPIC="${REQ_TOPIC:-/scene_graph_generator/request}"
    RESP_TOPIC="${RESP_TOPIC:-/scene_graph_generator/response}"
    ACK_TIMEOUT="${ACK_TIMEOUT:-300}"
    launch bridge bash "$START_BRIDGE"
    sleep 3                       # let the eager bridges come up
    echo "[supervisor] demo: pipeline DOWN; waiting for 'start' on $REQ_TOPIC ..."
    # Gate: exit 0 = start (launch), exit 3 = cancel-before-start (nothing to save).
    if run_helper await --topic "$REQ_TOPIC" --keyword start --cancel-keyword cancel; then
      start_pipeline false; PIPE_PID=$LAST_PID   # live pipeline (wall clock); in PIDS
      # RECORD=1: record only the RUN WINDOW -- start here (on the robot's "start")
      # and stop when serve returns below, so the bag spans exactly the run.
      [ "$RECORD" = "1" ] && start_recorder
      run_helper serve \
        --req "$REQ_TOPIC" --resp "$RESP_TOPIC" \
        --save-topic /keyboard_input --save-keyword ssg \
        --output-root "$APP/output/scene_graph" \
        --interval 5 --ack received --cancel cancel \
        --file-timeout 15 --ack-timeout "$ACK_TIMEOUT" || true
      # Run window over (JSON delivered + acked, or cancelled). Close the bag now --
      # before any HOLD idle -- so it spans the run, not the inspection period.
      stop_recorder
      # HOLD (default 1) then blocks on the still-running pipeline so RViz stays up
      # for inspection -- the live feed keeps updating (wall clock, robot still
      # publishing), Ctrl-C quits. HOLD=0 tears down instead.
      if [ "${HOLD:-1}" = "1" ]; then
        PRIMARY=$PIPE_PID
        echo "[supervisor] HOLD=1: stack (incl. RViz) stays up after the demo run -- Ctrl-C to quit."
      fi
    else
      echo "[supervisor] demo: cancelled before start; shutting down."
    fi
    # If a pipeline started and HOLD=1 (the default), PRIMARY is the pipeline, so the
    # final `wait` holds the stack up. Otherwise PRIMARY stays empty and we fall
    # through to the EXIT trap, which tears down the bridge + pipeline (container
    # exits) -- the HOLD=0 path, or cancel-before-start where nothing launched.
    ;;

  *) echo "[supervisor] unknown MODE=$MODE (use live | demo | bag | bag-direct)" >&2; exit 2 ;;
esac

# Block on the primary process; the EXIT trap tears the rest down.
wait "$PRIMARY" 2>/dev/null || true
echo "[supervisor] primary process exited; shutting down."
