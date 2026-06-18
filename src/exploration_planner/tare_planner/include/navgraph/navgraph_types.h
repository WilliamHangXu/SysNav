//
// Plain-old-data types for the NavGraph, split out so consumers (e.g. the
// scene-graph exporter) can use them without pulling in rclcpp/PCL via the full
// navgraph.h. Depends only on geometry_msgs (a message struct, no ROS runtime).
//

#ifndef NAVGRAPH_NAVGRAPH_TYPES_H
#define NAVGRAPH_NAVGRAPH_TYPES_H

#include <string>

#include <geometry_msgs/msg/point.hpp>

namespace navgraph_ns
{
// A NavGraph node. Its position is copied from a keypose node at birth and is
// frozen forever; its id is monotonic and never reused.
struct NavNode
{
  int id;
  geometry_msgs::msg::Point position;
  int room_id = -1;  // room affiliation (-1 = unknown), tagged from the room mask
  // Scene-graph waypoint id, e.g. "meeting room-room_4-wp_0". Assigned each
  // reconcile from the node's room; this is exactly the id the exporter emits,
  // so RViz labels and the JSON stay in lockstep. Empty if the node has no room.
  std::string name;
};

// An undirected NavGraph edge. (u, v) is canonical with u < v. `meters` is the
// keypose-graph shortest-path distance between the two nodes.
struct NavEdge
{
  int u;
  int v;
  double meters;
};
}  // namespace navgraph_ns

#endif  // NAVGRAPH_NAVGRAPH_TYPES_H
