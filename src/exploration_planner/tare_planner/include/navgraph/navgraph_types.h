//
// Plain-old-data types for the NavGraph, split out so consumers (e.g. the
// scene-graph exporter) can use them without pulling in rclcpp/PCL via the full
// navgraph.h. Depends only on geometry_msgs (a message struct, no ROS runtime).
//

#ifndef NAVGRAPH_NAVGRAPH_TYPES_H
#define NAVGRAPH_NAVGRAPH_TYPES_H

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
