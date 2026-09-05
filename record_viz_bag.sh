#!/bin/bash
# Record every topic the teleop RViz config (tare_planner_teleop.rviz) displays, so replaying
# the bag reproduces the exact live visualization with NO nodes running:
#
#   ./record_viz_bag.sh                       # -> bags/viz_<timestamp>
#   ./record_viz_bag.sh my_run                # -> bags/my_run
#   ./record_viz_bag.sh my_run --with-inputs  # + /registered_scan /state_estimation /camera/image
#                                             #   (enables offline pipeline re-runs; much bigger bag)
#
# Replay with ./play_viz_bag.sh (starts RViz + `ros2 bag play --clock`).
#
# Latched topics (/sempath_plan/*, /sempath_map/markers) keep their transient_local QoS in the
# bag, so a late-started RViz still receives the last map overlay and plan.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"
source ./install/setup.bash

NAME="${1:-viz_$(date +%Y%m%d_%H%M%S)}"

VIZ_TOPICS=(
  # frames (Vehicle axes display + TF tree)
  /tf /tf_static
  # TARE global / local displays
  /keypose_graph_cloud /keypose_graph_edge_marker /planner_cloud /overall_map
  /explore_areas_new /navigation_boundary
  /uncovered_cloud /uncovered_frontier_cloud /way_point /free_paths /path
  # camera + room-mask images
  /annotated_image /room_mask_vis
  # scene graph: rooms / walls / doors / debug clouds
  /trajectory /walls /current_room_boundary /viewpoint_rep_vis_cloud
  /room_boundaries /room_map_cloud /room_cloud /room_type_vis
  /free_cloud_1 /debug_cloud /door_cloud_vis /collision_cloud /door_cloud_in_range
  # object mapper
  /object_visibility_connections /obj_points /obj_labels /obj_boxes /object_node_markers
  # BEV mapper
  /bev_map/grid /bev_map/grid_updates /bev_map/frontiers /bev_map/local
  # sempath_planner: plan path + waypoints + semantic map overlay
  /sempath_plan/path /sempath_plan/markers /sempath_map/markers
)

INPUT_TOPICS=(/registered_scan /state_estimation /camera/image)

TOPICS=("${VIZ_TOPICS[@]}")
if [ "$2" = "--with-inputs" ]; then
  TOPICS+=("${INPUT_TOPICS[@]}")
fi

mkdir -p bags
exec ros2 bag record -o "bags/$NAME" "${TOPICS[@]}"
