#!/bin/bash
# Real robot (Livox Mid-360 + ARISE SLAM, Theta Z1 panorama on /camera/image) +
# teleoperated scene-graph pipeline: sysnav's
# system_real_robot_with_exploration_planner.sh without the exploration planner.
# Drive with the RViz teleop panel / waypoint tool or a joystick on /dev/input/js0.
# The camera driver is started separately (see scene_graph_real_robot.launch).
# Extra args are passed to the launch, e.g.
#   ./system_real_robot_teleop.sh objects:=false
#   ./system_real_robot_teleop.sh bagfile:=true   # then: ros2 bag play <raw-livox bag>  (no --clock)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR"           # semantic_mapping resolves object_file relative to the ws root
source ./install/setup.bash
ros2 launch tare_planner scene_graph_real_robot.launch "$@" &
LAUNCH_PID=$!
# The three ARISE nodes ignore SIGINT (ros2 launch then hangs on them and a
# re-launch would run against stale SLAM publishers); they do exit on SIGTERM.
trap 'pkill -TERM -f arise_slam_mid360/lib 2>/dev/null' INT TERM EXIT
wait $LAUNCH_PID; wait $LAUNCH_PID 2>/dev/null
