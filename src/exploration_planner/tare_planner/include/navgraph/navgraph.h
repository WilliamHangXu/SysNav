//
// NavGraph: a persistent, lightweight topological graph maintained live as a
// Voronoi contraction of the keypose graph. Nodes are a distance-spread subset
// of keypose-node positions ("in-room waypoints"); an edge means *reachability*
// (not straight-line traversability) with weight = the keypose-graph
// shortest-path distance between the two nodes.
//
// Designed to feed an LLM path planner via the scene graph. The dense keypose
// graph is left untouched; NavGraph is a read-only-ish coarsened view of it,
// reconciled (seed -> label -> delete -> edges -> room-tag) once every N
// planning cycles.
//

#ifndef NAVGRAPH_NAVGRAPH_H
#define NAVGRAPH_NAVGRAPH_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <keypose_graph/keypose_graph.h>
#include <navgraph/navgraph_types.h>

namespace navgraph_ns
{
class NavGraph
{
public:
  explicit NavGraph(rclcpp::Node::SharedPtr nh);
  ~NavGraph() = default;

  void ReadParameters(rclcpp::Node::SharedPtr nh);

  // Throttled reconcile entry point. The planner calls this once per planning
  // cycle; NavGraph self-throttles to every kNavGraphUpdateInterval-th call.
  // `room_mask`/`shift`/`room_resolution` are the planner's room-segmentation
  // mask used to tag each node with a room id (pass an empty mask to skip).
  // `room_keys` maps room id -> scene-graph room key (e.g. "kitchen-room_1") so
  // each node can be named with its eventual scene-graph waypoint id.
  void Update(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
              const cv::Mat& room_mask, const Eigen::Vector3f& shift, float room_resolution,
              const std::map<int, std::string>& room_keys);

  // Read API for downstream consumers (e.g. the scene-graph exporter).
  const std::map<int, NavNode>& GetNodes() const
  {
    return nodes_;
  }
  const std::vector<NavEdge>& GetEdges() const
  {
    return edges_;
  }
  int GetNodeNum() const
  {
    return static_cast<int>(nodes_.size());
  }
  int GetEdgeNum() const
  {
    return static_cast<int>(edges_.size());
  }

private:
  // Full reconcile pass over the keypose graph's connected component:
  // seed (distance-gated) -> label (nearest-node Voronoi) -> hard-delete
  // (empty regions) -> rederive edges (region adjacency).
  void Reconcile(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  // Tag every surviving node with a room id by voxelizing its position into the
  // room mask (mirrors Representation::UpdateViewpointRoomIdsFromMask).
  void TagRooms(const cv::Mat& room_mask, const Eigen::Vector3f& shift, float room_resolution);
  // Assign each node its scene-graph waypoint id ("<room_key>-wp_<n>"), grouping
  // by room and numbering wp_1..N in ascending node-id order -- exactly the order
  // the exporter emits (wp_0 is reserved for the room centroid). Nodes with no
  // room get an empty name.
  void AssignNames(const std::map<int, std::string>& room_keys);
  // (Re)build the kdtree over current NavGraph node positions, smuggling the
  // stable node id through PCL's intensity channel (mirrors KeyposeGraph).
  void BuildKdtree();
  // Nearest existing NavGraph node to p: returns the node id (-1 if none) and
  // sets dist_out to the Euclidean distance.
  int NearestNode(const geometry_msgs::msg::Point& p, double& dist_out) const;
  // Publish nodes (orange POINTS marker) and edges (green LINE_LIST) for RViz.
  void PublishVisualization();

  std::map<int, NavNode> nodes_;  // id-keyed (ordered => deterministic output)
  std::vector<NavEdge> edges_;    // rebuilt from scratch every reconcile
  int next_id_;                   // monotonic id counter; never reused
  int update_call_count_;         // throttle counter

  // Spatial index over current node positions (id stored in intensity).
  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr node_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr label_marker_pub_;

  std::string world_frame_id_;
  double kNavNodeMinDist;          // node spacing (in-room granularity)
  double kNavNodeReanchorDist;     // max gap to salvage an orphaned node by re-anchoring
  int kNavGraphUpdateInterval;     // run full reconcile every Nth Update() call
};
}  // namespace navgraph_ns

#endif  // NAVGRAPH_NAVGRAPH_H
