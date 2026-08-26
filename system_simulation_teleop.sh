#!/bin/bash
# Unity sim + teleoperated scene-graph pipeline (sysnav's
# system_simulation_with_exploration_planner.sh without the exploration planner).
# Drive with the RViz teleop panel / goalpoint tool or a joystick on /dev/input/js0.
# Extra args are passed to the launch, e.g.  ./system_simulation_teleop.sh objects:=false
# export __NV_PRIME_RENDER_OFFLOAD=0
# export __GLX_VENDOR_LIBRARY_NAME=mesa

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR"           # semantic_mapping resolves object_file relative to the ws root
source ./install/setup.bash
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
UNITY_PID=$!
trap 'kill $UNITY_PID 2>/dev/null' EXIT
sleep 3
ros2 launch tare_planner scene_graph_sim.launch "$@"
