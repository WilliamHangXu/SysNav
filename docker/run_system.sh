#!/usr/bin/env bash
#
# Headless, single-process supervisor — the container equivalent of
# vlm_ros_alphaz.tmuxp.yaml.  No tmux, no RViz, no manual SPACE-to-start.
#
# The bag is started FIRST but PAUSED (--start-paused), so /clock is alive at
# the bag's real start time before the planner boots. This avoids the 0 ->
# bag-time clock jump that crashes TARE. Each node then launches in the
# background (a crash in one does NOT abort the run); once the stack is up the
# bag is auto-resumed via the rosbag2 /resume service (the container has no tty
# to press SPACE). When the bag finishes, every child is torn down and we exit.
#
# Input:
#   BAG_PATH   path to the local ros2 bag directory (set by entrypoint.sh)
#   AUTOPLAY_DELAY   seconds to wait, after the paused bag + stack start, before
#                    resuming playback (default 15; matches the tmuxp).
#
set -uo pipefail

: "${BAG_PATH:?BAG_PATH must be set (local ros2 bag directory)}"
: "${AUTOPLAY_DELAY:=15}"

# ROS setup scripts are not `set -u`-safe (reference unset AMENT_* vars), so
# relax nounset just around sourcing the overlays.
set +u
source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash
set -u
cd /app

LOG_DIR="/app/runlogs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"
echo "[run_system] logs -> ${LOG_DIR}"

PIDS=()

# launch <name> <delay_s> <command...>
launch() {
  local name="$1"; local delay="$2"; shift 2
  (
    sleep "${delay}"
    echo "[run_system] starting: ${name}"
    exec "$@"
  ) >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  echo "[run_system] tearing down (${#PIDS[@]} processes)"
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# --- bag: start PAUSED first so /clock is alive before the planner boots -----
# --disable-keyboard-controls + stdin from /dev/null keep the backgrounded
# player from reading the tty (otherwise SIGTTIN suspends it and its /resume
# service never responds).
echo "[run_system] starting bag PAUSED: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock --start-paused --disable-keyboard-controls \
    < /dev/null >"${LOG_DIR}/bag.log" 2>&1 &
BAG_PID=$!
PIDS+=("${BAG_PID}")

# --- compute / perception nodes (headless; RViz from the tmuxp is omitted) ---
launch slam            0  ros2 launch arise_slam_mid360 arize_slam_go2w.launch.py
launch detection       3  ros2 launch semantic_mapping detection_node_go2w.launch
launch semantic_map    3  ros2 launch semantic_mapping semantic_mapping_go2w.launch
launch room_seg        5  ros2 launch tare_planner room_segmentation.launch \
                              use_sim_time:=true scenario:=matterport_bagfile \
                              door_pipeline_log_level:=info

# NOTE: `mission_manager` is NOT present on this branch (az_bags). Left here to
# mirror the tmuxp; it will log an error and the rest of the pipeline continues.
# Add the package (or delete this line) for a complete run.
# launch mission_manager 5  ros2 launch mission_manager mission_manager.launch \
#                               use_sim_time:=true config:=gadm

launch vlm             5  ros2 launch vlm_node vlm_node.launch \
                              use_sim_time:=true config:=vlm_config
launch planner        10  ros2 launch tare_planner explore.launch \
                              use_sim_time:=true scenario:=matterport_bagfile

# --- resume playback once the stack is up ------------------------------------
echo "[run_system] waiting ${AUTOPLAY_DELAY}s for the stack to come up before resuming bag..."
sleep "${AUTOPLAY_DELAY}"
echo "[run_system] resuming bag playback"
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"

# --- wait for the bag to play to completion ----------------------------------
wait "${BAG_PID}"
bag_rc=$?

echo "[run_system] bag finished (rc=${bag_rc}); shutting down."
# give the graph a moment to flush map/output artifacts before cleanup() fires
sleep 5
exit "${bag_rc}"
