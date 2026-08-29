# bev_mapper

Bird's-eye-view occupancy map of the explored floor, built from the SLAM
outputs the scene-graph pipeline already consumes.

```
/registered_scan  (PointCloud2, map frame)  ─┐
                                             ├─► bev_mapper_node ─► /bev_map/grid       nav_msgs/OccupancyGrid (map frame)
/state_estimation (Odometry, map→sensor)    ─┘                    ─► /bev_map/local      sensor_msgs/Image rgb8 448×448
                                                                  ─► /bev_map/frontiers  sensor_msgs/PointCloud2 (map frame)
```

* `lidar_bev_mapper.py`, `coord_utils.py`, `frontier_detector.py` are **verbatim
  copies** of `Navigation-Physical-Experiment/src/vlm_nav_bridge/vlm_nav_bridge/`
  (commit `e033998`). Keep them that way; put changes in the node or the yaml.
* `bev_mapper_node.py` is the only new code: the scan/pose plumbing of that
  package's `vlm_navigator_node.py` without its VLM / camera / target /
  `/way_point` parts. It never publishes anything the planner or the base
  listen to.

Map: `map_size` × `map_size` m (67.2 m) at `map_resolution` (0.05 m), centred
on the first pose; channels occupancy / explored / agent / trajectory. Explored
cells come from Bresenham ray-casting robot→return (stops at occupied cells,
radius `vision_range`); only the component connected to the robot is kept.
`/bev_map/local` is the robot-centred crop rendered the way the VLN training
data was (white explored, black obstacle, grey unknown, blue trail, red arrow,
green frontier dots).

Config knobs that differ from the source package: `hfov_deg: 360` (the source
kept a 79° forward wedge to mimic its training camera), `obstacle_height_min:
-0.4` (relative to the lidar; source used −1.0 for a G1), and two node-side
guards: `self_exclusion_radius` (returns from the platform's own mount at
0.3–0.7 m) and `clear_footprint_radius` (cells the robot drives through are
free).

Known limitation: the mapper only ever sets occupancy and never clears it, so
a person walking with the robot (torso height is inside the obstacle band)
leaves permanent marks along the path — ~55 % of the path cells on the
mecanum test bag. Ray clearing (hit/miss counts) would be the fix.

```
ros2 launch bev_mapper bev_mapper.launch.py            # standalone
ros2 launch tare_planner scene_graph_real_robot.launch # bev:=false to skip
```
