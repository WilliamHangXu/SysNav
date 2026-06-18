//
// NavGraph implementation. See navgraph/navgraph.h for the design overview.
//

#include <navgraph/navgraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <utility>

#include <nav_msgs/msg/path.hpp>
#include <opencv2/core.hpp>

#include <utils/misc_utils.h>

namespace navgraph_ns
{
NavGraph::NavGraph(rclcpp::Node::SharedPtr nh)
  : next_id_(0)
  , update_call_count_(0)
  , world_frame_id_("map")
  , kNavNodeMinDist(1.25)
  , kNavGraphUpdateInterval(2)
{
  ReadParameters(nh);

  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  clock_ = nh->get_clock();
  node_marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("navgraph/node_marker", 2);
  edge_marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("navgraph/edge_marker", 2);
  label_marker_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("navgraph/label_marker", 2);
}

void NavGraph::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("navigation_graph/kNavNodeMinDist", kNavNodeMinDist);
  nh->get_parameter("navigation_graph/kNavGraphUpdateInterval", kNavGraphUpdateInterval);
  nh->get_parameter("navigation_graph/world_frame_id", world_frame_id_);
  if (kNavGraphUpdateInterval < 1)
  {
    kNavGraphUpdateInterval = 1;
  }
}

void NavGraph::Update(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                      const cv::Mat& room_mask, const Eigen::Vector3f& shift, float room_resolution,
                      const std::map<int, std::string>& room_keys)
{
  if (!keypose_graph)
  {
    return;
  }
  // Self-throttle: run the full reconcile on every kNavGraphUpdateInterval-th
  // call (the first call runs immediately).
  if ((update_call_count_++ % kNavGraphUpdateInterval) != 0)
  {
    return;
  }
  Reconcile(keypose_graph);
  TagRooms(room_mask, shift, room_resolution);
  AssignNames(room_keys);
  PublishVisualization();

  int with_room = 0;
  for (const auto& kv : nodes_)
  {
    if (kv.second.room_id >= 0)
    {
      ++with_room;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("navgraph"), "NavGraph: %d nodes (%d with room), %d edges", GetNodeNum(),
              with_room, GetEdgeNum());
}

void NavGraph::BuildKdtree()
{
  nodes_cloud_->clear();
  for (const auto& kv : nodes_)
  {
    pcl::PointXYZI point;
    point.x = kv.second.position.x;
    point.y = kv.second.position.y;
    point.z = kv.second.position.z;
    point.intensity = static_cast<float>(kv.second.id);  // smuggle stable id
    nodes_cloud_->points.push_back(point);
  }
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);
  }
}

int NavGraph::NearestNode(const geometry_msgs::msg::Point& p, double& dist_out) const
{
  dist_out = std::numeric_limits<double>::max();
  if (nodes_cloud_->points.empty())
  {
    return -1;
  }
  pcl::PointXYZI query;
  query.x = p.x;
  query.y = p.y;
  query.z = p.z;
  query.intensity = 0;
  std::vector<int> nn_indices(1);
  std::vector<float> nn_sqdist(1);
  if (kdtree_nodes_->nearestKSearch(query, 1, nn_indices, nn_sqdist) <= 0)
  {
    return -1;
  }
  dist_out = std::sqrt(static_cast<double>(nn_sqdist[0]));
  return static_cast<int>(nodes_cloud_->points[nn_indices.front()].intensity);
}

void NavGraph::Reconcile(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // --- Gather the keypose graph's connected component -----------------------
  const std::vector<int> connected_inds = keypose_graph->GetConnectedGraphNodeIndices();
  std::vector<geometry_msgs::msg::Point> connected_pos;
  connected_pos.reserve(connected_inds.size());
  for (int ind : connected_inds)
  {
    connected_pos.push_back(keypose_graph->GetNodePosition(ind));
  }

  // --- Phase 1: seed (greedy distance-gated coverage) -----------------------
  // Append a new node wherever a connected keypose node is farther than
  // kNavNodeMinDist from every existing node (kdtree) and every node seeded
  // earlier in this same pass (linear, since that set is small). Existing nodes
  // are never moved or touched -- seeding is purely additive.
  BuildKdtree();  // over existing nodes only
  const double min_dist_sq = kNavNodeMinDist * kNavNodeMinDist;
  std::vector<geometry_msgs::msg::Point> new_node_positions;
  for (const auto& p : connected_pos)
  {
    double d_existing = std::numeric_limits<double>::max();
    NearestNode(p, d_existing);
    bool covered = (d_existing <= kNavNodeMinDist);
    if (!covered)
    {
      for (const auto& np : new_node_positions)
      {
        const double dx = p.x - np.x;
        const double dy = p.y - np.y;
        const double dz = p.z - np.z;
        if (dx * dx + dy * dy + dz * dz <= min_dist_sq)
        {
          covered = true;
          break;
        }
      }
    }
    if (!covered)
    {
      NavNode node;
      node.id = next_id_++;
      node.position = p;
      nodes_[node.id] = node;
      new_node_positions.push_back(p);
    }
  }

  // --- Phase 2: label (nearest-node Voronoi) + member tally -----------------
  // After Phase 1 every connected keypose node is within kNavNodeMinDist of some
  // node, so labeling has no orphan/uncapped case. Region is keyed by keypose
  // node index so the edge phase can look up endpoints directly.
  BuildKdtree();  // now includes the newly seeded nodes
  std::unordered_map<int, int> region;     // keypose node_ind -> nav node id
  std::map<int, int> member_count;         // nav node id -> #connected members
  for (size_t k = 0; k < connected_inds.size(); ++k)
  {
    double d = 0.0;
    const int nav_id = NearestNode(connected_pos[k], d);
    if (nav_id < 0)
    {
      continue;
    }
    region[connected_inds[k]] = nav_id;
    member_count[nav_id]++;
  }

  // --- Phase 3: hard-delete nodes with no connected support this pass --------
  // Newly seeded nodes always have >=1 member (their own seed keypose node), so
  // they survive; every region value therefore still exists after deletion.
  for (auto it = nodes_.begin(); it != nodes_.end();)
  {
    if (member_count.find(it->first) == member_count.end())
    {
      it = nodes_.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // --- Phase 4: rederive edges (region adjacency) ---------------------------
  // An edge u-v exists iff some connected keypose edge crosses from region u
  // into region v. Weight = keypose-graph shortest-path distance between the
  // two nodes (honest traversable distance, summed keypose edge lengths).
  edges_.clear();
  std::set<std::pair<int, int>> seen;
  for (int a : connected_inds)
  {
    const auto ra = region.find(a);
    if (ra == region.end())
    {
      continue;
    }
    const int u = ra->second;
    const std::vector<int>& neighbors = keypose_graph->GetNodeNeighbors(a);
    for (int b : neighbors)
    {
      const auto rb = region.find(b);
      if (rb == region.end())  // neighbor not in the connected/labeled set
      {
        continue;
      }
      const int v = rb->second;
      if (u == v)
      {
        continue;
      }
      const std::pair<int, int> key = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
      if (!seen.insert(key).second)
      {
        continue;  // already added this region pair
      }
      nav_msgs::msg::Path dummy_path;
      const double meters = keypose_graph->GetShortestPath(nodes_.at(key.first).position,
                                                           nodes_.at(key.second).position,
                                                           /*get_path=*/false, dummy_path,
                                                           /*use_connected_nodes=*/true);
      if (meters >= keypose_graph_ns::INF)
      {
        continue;  // no traversable path found (should not happen within a component)
      }
      NavEdge edge;
      edge.u = key.first;
      edge.v = key.second;
      edge.meters = meters;
      edges_.push_back(edge);
    }
  }
}

void NavGraph::TagRooms(const cv::Mat& room_mask, const Eigen::Vector3f& shift, float room_resolution)
{
  // No mask yet (room segmentation hasn't produced one): mark everything unknown.
  if (room_mask.empty() || room_resolution <= 0.0f)
  {
    for (auto& kv : nodes_)
    {
      kv.second.room_id = -1;
    }
    return;
  }
  // Voxelize each node's position into the room mask and read its room id.
  // Mirrors Representation::UpdateViewpointRoomIdsFromMask.
  const float inv_resolution = 1.0f / room_resolution;
  for (auto& kv : nodes_)
  {
    const Eigen::Vector3i voxel = misc_utils_ns::point_to_voxel(kv.second.position, shift, inv_resolution);
    if (voxel.x() >= 0 && voxel.x() < room_mask.rows && voxel.y() >= 0 && voxel.y() < room_mask.cols)
    {
      kv.second.room_id = room_mask.at<int>(voxel.x(), voxel.y());
    }
    else
    {
      kv.second.room_id = -1;
    }
  }
}

void NavGraph::AssignNames(const std::map<int, std::string>& room_keys)
{
  // nodes_ is ordered by id, so iterating it yields ascending node ids within
  // each room -- the same order the exporter emits, so the wp_<n> numbering
  // matches. A node with no (alive) room gets an empty name and is not exported.
  std::map<int, int> next_wp_index;  // room id -> next waypoint index
  for (auto& kv : nodes_)
  {
    NavNode& node = kv.second;
    auto key_it = room_keys.find(node.room_id);
    if (node.room_id < 0 || key_it == room_keys.end())
    {
      node.name.clear();
      continue;
    }
    // wp_0 is reserved for the room centroid, so NavGraph nodes start at wp_1.
    const int idx = ++next_wp_index[node.room_id];
    node.name = key_it->second + "-wp_" + std::to_string(idx);
  }
}

void NavGraph::PublishVisualization()
{
  const rclcpp::Time stamp = clock_->now();

  // Nodes as a fixed-color POINTS marker (orange) -- a marker enforces the color
  // regardless of RViz's point-cloud color transformer.
  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = world_frame_id_;
  node_marker.header.stamp = stamp;
  node_marker.ns = "navgraph_nodes";
  node_marker.id = 0;
  node_marker.type = visualization_msgs::msg::Marker::POINTS;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.scale.x = 0.35;
  node_marker.scale.y = 0.35;
  node_marker.color.r = 1.0;
  node_marker.color.g = 0.5;
  node_marker.color.b = 0.0;
  node_marker.color.a = 1.0;
  node_marker.pose.orientation.w = 1.0;
  for (const auto& kv : nodes_)
  {
    node_marker.points.push_back(kv.second.position);
  }
  node_marker_pub_->publish(node_marker);

  // Edges as a LINE_LIST marker (green, distinct from the keypose graph).
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = world_frame_id_;
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "navgraph_edges";
  edge_marker.id = 0;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.pose.orientation.w = 1.0;
  for (const auto& edge : edges_)
  {
    edge_marker.points.push_back(nodes_.at(edge.u).position);
    edge_marker.points.push_back(nodes_.at(edge.v).position);
  }
  edge_marker_pub_->publish(edge_marker);

  // Per-node text labels = scene-graph waypoint id. DELETEALL first so labels of
  // deleted nodes don't linger. A node with no room shows "(no room) #<id>".
  visualization_msgs::msg::MarkerArray labels;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = world_frame_id_;
  clear_marker.header.stamp = stamp;
  clear_marker.ns = "navgraph_labels";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  labels.markers.push_back(clear_marker);
  for (const auto& kv : nodes_)
  {
    const NavNode& node = kv.second;
    visualization_msgs::msg::Marker text;
    text.header.frame_id = world_frame_id_;
    text.header.stamp = stamp;
    text.ns = "navgraph_labels";
    text.id = node.id;  // stable, unique
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position = node.position;
    text.pose.position.z += 0.3;  // float the label above the node
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.3;  // text height
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.text = node.name.empty() ? ("(no room) #" + std::to_string(node.id)) : node.name;
    labels.markers.push_back(text);
  }
  label_marker_pub_->publish(labels);
}
}  // namespace navgraph_ns
