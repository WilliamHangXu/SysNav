#!/bin/bash
# Replay a bag recorded with ./record_viz_bag.sh and reproduce the live RViz view
# with no nodes running:
#
#   ./play_viz_bag.sh                              # newest bag under bags/
#   ./play_viz_bag.sh bags/my_run                  # a specific bag
#   ./play_viz_bag.sh bags/my_run --loop --rate 2  # extra args go to `ros2 bag play`
#   ./play_viz_bag.sh --start-paused               # flags-only also works (newest bag)
#
# Plays with --clock and runs RViz on sim time — correct for BOTH sim and real-robot
# bags (RViz follows the recorded stamps, so TF, clouds and markers line up).
# When the bag ends RViz keeps the last rendered state; Ctrl+C closes everything.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"
source ./install/setup.bash

if [ -z "$1" ] || [[ "$1" == -* ]]; then
  BAG="$(ls -td bags/*/ 2>/dev/null | head -1)"
  if [ -z "$BAG" ]; then
    echo "no bags under bags/ — record one with ./record_viz_bag.sh" >&2
    exit 1
  fi
  echo "playing newest bag: $BAG"
else
  BAG="$1"
  shift
fi

rviz2 -d src/exploration_planner/tare_planner/tare_planner_teleop.rviz \
  --ros-args -p use_sim_time:=true &
RVIZ_PID=$!
trap 'kill $RVIZ_PID 2>/dev/null' EXIT
sleep 3                    # let RViz subscribe before the first messages play

ros2 bag play "$BAG" --clock "$@"

echo "bag finished — RViz keeps the last state, Ctrl+C to close"
wait $RVIZ_PID
