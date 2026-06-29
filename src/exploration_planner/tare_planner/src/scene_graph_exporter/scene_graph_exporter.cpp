/**
 * @file scene_graph_exporter.cpp
 * @brief Implementation of the GADM-style scene-graph JSON exporter.
 */
#include "scene_graph_exporter/scene_graph_exporter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scene_graph_exporter_ns
{

namespace
{
// Label to use when a room has not been typed by the VLM yet.
const char* kUnknownLabel = "unknown";

// Map a source-frame point into world_frame.
inline Eigen::Vector3d ToWorld(const Eigen::Isometry3d& world_from_source,
                               double x, double y, double z)
{
  return world_from_source * Eigen::Vector3d(x, y, z);
}
}  // namespace

SceneGraphExporter::SceneGraphExporter(SceneGraphExportConfig config)
    : config_(std::move(config))
{
}

std::string SceneGraphExporter::RoomKey(const representation_ns::RoomNodeRep& room)
{
  std::string label = room.GetRoomLabel();
  if (label.empty())
  {
    label = kUnknownLabel;
  }
  return label + "-room_" + std::to_string(room.id_);
}

bool SceneGraphExporter::DoorCentroid(
    const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud, int id_a, int id_b,
    const Eigen::Isometry3d& world_from_source, double& x, double& y, double& z)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  int count = 0;
  for (const auto& point : door_cloud.points)
  {
    // r/g channels carry the two room ids the door connects (order-agnostic).
    if ((point.r == id_a && point.g == id_b) ||
        (point.r == id_b && point.g == id_a))
    {
      // Transform each pixel before averaging so the centroid is in world_frame.
      const Eigen::Vector3d world =
          ToWorld(world_from_source, point.x, point.y, point.z);
      sum_x += world.x();
      sum_y += world.y();
      sum_z += world.z();
      ++count;
    }
  }
  if (count == 0)
  {
    return false;
  }
  x = sum_x / count;
  y = sum_y / count;
  z = sum_z / count;
  return true;
}

nlohmann::json SceneGraphExporter::BuildObjectJson(
    const representation_ns::ObjectNodeRep& object,
    const Eigen::Isometry3d& world_from_source)
{
  std::string label = object.label_.empty() ? kUnknownLabel : object.label_;
  int primary_id = object.object_id_.empty() ? -1 : object.object_id_[0];
  const std::string object_id = label + "_" + std::to_string(primary_id);

  // Object centroid, transformed into world_frame like every other coordinate.
  const geometry_msgs::msg::Point& pos = object.GetPosition();
  const Eigen::Vector3d world =
      ToWorld(world_from_source, pos.x, pos.y, pos.z);

  return nlohmann::json{
      {"object_id", object_id},
      {"type", label},
      {"sgid", primary_id},
      {"waypoint",
       nlohmann::json{
           {"id", object_id + "_wp"},
           {"x", world.x()},
           {"y", world.y()},
           {"z", world.z()},
       }},
  };
}

nlohmann::json SceneGraphExporter::BuildRoomJson(
    const representation_ns::RoomNodeRep& room,
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
    const std::vector<const navgraph_ns::NavNode*>& room_nav_nodes,
    const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
    const Eigen::Isometry3d& world_from_source,
    std::map<int, std::string>& nav_id_to_wpid,
    const navgraph_ns::BuildingAxes& axes) const
{
  const std::string room_key = RoomKey(room);

  // Quadrant origin = the room centroid (the axes are centered there). Both wp_0
  // (below, computed here) and the NavGraph nodes (wp_1..N, tagged upstream) use
  // this same centroid + the same frozen axes, so they agree by construction.
  const Eigen::Vector2d room_centroid(room.centroid_.x(), room.centroid_.y());

  // --- entrances: one per door-adjacent neighbor with door geometry ---
  nlohmann::json entrances = nlohmann::json::array();
  int entrance_count = 0;
  for (int neighbor_id : room.neighbors_)
  {
    auto neighbor_it = rooms.find(neighbor_id);
    if (neighbor_it == rooms.end())
    {
      continue;  // neighbor no longer alive; skip
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!DoorCentroid(door_cloud, room.id_, neighbor_id, world_from_source, x, y,
                      z))
    {
      continue;  // no door pixels for this pair
    }
    ++entrance_count;
    entrances.push_back(nlohmann::json{
        {"id", std::to_string(room.id_) + "_" + std::to_string(neighbor_id) +
                   "_entrance_" + std::to_string(entrance_count)},
        {"connected_to", RoomKey(neighbor_it->second)},
        {"x", x},
        {"y", y},
        {"z", z},
    });
  }

  // --- waypoints: wp_0 = room interior point (pole of inaccessibility, a
  // guaranteed-inside cell), wp_1..N = the room's NavGraph nodes ---
  nlohmann::json waypoints = nlohmann::json::array();
  const Eigen::Vector3d interior_world =
      ToWorld(world_from_source, room.interior_point_.x, room.interior_point_.y,
              room.interior_point_.z);
  // wp_0 area is assigned like any other point (origin = centroid), so it is not a
  // special "center" -- the interior point lies wherever it lies in the quadrants.
  const navgraph_ns::Area wp0_area = navgraph_ns::AssignArea(
      Eigen::Vector2d(room.interior_point_.x, room.interior_point_.y), room_centroid, axes);
  waypoints.push_back(nlohmann::json{
      {"id", room_key + "-wp_0"},
      {"x", interior_world.x()},
      {"y", interior_world.y()},
      {"z", interior_world.z()},
      {"area", navgraph_ns::AreaName(wp0_area)},
  });
  int wp_index = 1;
  for (const navgraph_ns::NavNode* node : room_nav_nodes)  // ascending node id
  {
    const Eigen::Vector3d world = ToWorld(world_from_source, node->position.x,
                                          node->position.y, node->position.z);
    // Use the node's precomputed name (kept in lockstep with RViz labels); fall
    // back to recomputing the same id if it was somehow not assigned.
    const std::string wp_id =
        !node->name.empty() ? node->name
                            : (room_key + "-wp_" + std::to_string(wp_index));
    waypoints.push_back(nlohmann::json{
        {"id", wp_id},
        {"x", world.x()},
        {"y", world.y()},
        {"z", world.z()},
        // Read the area tagged upstream by the NavGraph (same centroid + axes).
        {"area", navgraph_ns::AreaName(node->area)},
    });
    nav_id_to_wpid[node->id] = wp_id;  // so Build() can wire NavGraph edges
    ++wp_index;
  }

  // --- objects: every object assigned to this room ---
  nlohmann::json object_array = nlohmann::json::array();
  for (int object_id : room.GetObjectIndices())
  {
    auto object_it = objects.find(object_id);
    if (object_it == objects.end())
    {
      continue;  // stale link; object no longer present
    }
    object_array.push_back(
        BuildObjectJson(object_it->second, world_from_source));
  }

  return nlohmann::json{
      {"type", room.GetRoomLabel().empty() ? kUnknownLabel : room.GetRoomLabel()},
      {"sgid", room.id_},
      {"entrances", std::move(entrances)},
      {"waypoints", std::move(waypoints)},
      {"objects", std::move(object_array)},
  };
}

void SceneGraphExporter::ComputeDimensions(
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    const Eigen::Isometry3d& world_from_source, double& width, double& height)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  bool any = false;
  for (const auto& id_room : rooms)
  {
    for (const auto& pt : id_room.second.polygon_.polygon.points)
    {
      // Bbox is taken in world_frame, so transform before measuring extents.
      const Eigen::Vector3d world =
          ToWorld(world_from_source, pt.x, pt.y, pt.z);
      any = true;
      min_x = std::min(min_x, world.x());
      min_y = std::min(min_y, world.y());
      max_x = std::max(max_x, world.x());
      max_y = std::max(max_y, world.y());
    }
  }
  width = any ? (max_x - min_x) : 0.0;
  height = any ? (max_y - min_y) : 0.0;
}

bool SceneGraphExporter::ComputeAabbCenterSource(
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    Eigen::Vector3d& center, double& width, double& height)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();
  bool any = false;
  for (const auto& id_room : rooms)
  {
    for (const auto& pt : id_room.second.polygon_.polygon.points)
    {
      // Source frame (NO transform): consistent with the source-frame axes.
      any = true;
      min_x = std::min(min_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      min_z = std::min(min_z, static_cast<double>(pt.z));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      max_y = std::max(max_y, static_cast<double>(pt.y));
      max_z = std::max(max_z, static_cast<double>(pt.z));
    }
  }
  if (!any)
  {
    return false;
  }
  center = Eigen::Vector3d(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.5 * (min_z + max_z));
  width = max_x - min_x;
  height = max_y - min_y;
  return true;
}

nlohmann::json SceneGraphExporter::Build(
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
    const std::map<int, navgraph_ns::NavNode>& nav_nodes,
    const std::vector<navgraph_ns::NavEdge>& nav_edges,
    const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
    const Eigen::Isometry3d& world_from_source,
    const navgraph_ns::BuildingAxes& axes) const
{
  nlohmann::json rooms_json = nlohmann::json::object();
  nlohmann::json all_waypoints = nlohmann::json::array();

  // Group NavGraph nodes by room id (ascending node id within each room, since
  // nav_nodes is an ordered map). Nodes whose room_id is not an alive room are
  // simply never emitted (and edges touching them are dropped below).
  std::map<int, std::vector<const navgraph_ns::NavNode*>> nodes_by_room;
  for (const auto& id_node : nav_nodes)
  {
    nodes_by_room[id_node.second.room_id].push_back(&id_node.second);
  }
  static const std::vector<const navgraph_ns::NavNode*> kNoNavNodes;

  // nav node id -> its emitted waypoint id; populated per room, used for edges.
  std::map<int, std::string> nav_id_to_wpid;

  for (const auto& id_room : rooms)
  {
    const auto& room = id_room.second;
    auto nodes_it = nodes_by_room.find(room.id_);
    const std::vector<const navgraph_ns::NavNode*>& room_nav_nodes =
        (nodes_it != nodes_by_room.end()) ? nodes_it->second : kNoNavNodes;
    nlohmann::json room_json =
        BuildRoomJson(room, rooms, objects, room_nav_nodes, door_cloud,
                      world_from_source, nav_id_to_wpid, axes);
    // Mirror every waypoint id into the flat top-level list.
    for (const auto& wp : room_json["waypoints"])
    {
      all_waypoints.push_back(wp["id"]);
    }
    rooms_json[RoomKey(room)] = std::move(room_json);
  }

  // --- edges: NavGraph edges only (traversable connectivity between nodes) ---
  // Room-centroid (wp_0) adjacency edges are intentionally omitted: the centroid
  // is no longer exported as a waypoint. Skip any edge whose endpoint was not
  // placed in an alive room.
  nlohmann::json edges = nlohmann::json::array();
  for (const auto& edge : nav_edges)
  {
    auto u_it = nav_id_to_wpid.find(edge.u);
    auto v_it = nav_id_to_wpid.find(edge.v);
    if (u_it == nav_id_to_wpid.end() || v_it == nav_id_to_wpid.end())
    {
      continue;
    }
    edges.push_back(nlohmann::json{
        {"u", u_it->second},
        {"v", v_it->second},
        {"meters", edge.meters},
    });
  }

  double width = 0.0;
  double height = 0.0;
  ComputeDimensions(rooms, world_from_source, width, height);

  nlohmann::json metadata = {
      {"units", config_.units},
      {"frame", config_.frame},
      {"building", config_.building},
      {"floor_level", config_.floor_level},
      {"floor_id", config_.floor_id},
      {"dimensions", {{"width", width}, {"height", height}}},
  };

  // Compass: 5 points (center + N/S/E/W tips) describing the global building axes
  // so a frontend can draw an oriented compass. Computed in the source frame then
  // each point transformed by world_from_source (like every other coordinate).
  // Omitted until the axes are frozen / there are rooms.
  Eigen::Vector3d aabb_center;
  double aabb_w = 0.0;
  double aabb_h = 0.0;
  if (axes.valid && ComputeAabbCenterSource(rooms, aabb_center, aabb_w, aabb_h))
  {
    const double radius = (config_.compass_radius_m > 0.0) ? config_.compass_radius_m
                                                           : 0.5 * std::max(aabb_w, aabb_h);
    const Eigen::Vector3d e(axes.east.x(), axes.east.y(), 0.0);
    const Eigen::Vector3d n(axes.north.x(), axes.north.y(), 0.0);
    auto pt = [&world_from_source](const Eigen::Vector3d& s) {
      const Eigen::Vector3d w = world_from_source * s;
      return nlohmann::json{ { "x", w.x() }, { "y", w.y() }, { "z", w.z() } };
    };
    metadata["compass"] = {
        { "center", pt(aabb_center) },
        { "north", pt(aabb_center + radius * n) },
        { "south", pt(aabb_center - radius * n) },
        { "east", pt(aabb_center + radius * e) },
        { "west", pt(aabb_center - radius * e) },
    };
  }

  return nlohmann::json{
      {"map_id", config_.map_id},
      {"warehouse_id", config_.warehouse_id},
      {"name", config_.name},
      {"client_id", config_.client_id},
      {"uploaded_by", config_.uploaded_by},
      {"layout",
       {
           {"zones", {{config_.zone, {{"rooms", std::move(rooms_json)}}}}},
           {"waypoints", std::move(all_waypoints)},
           {"edges", std::move(edges)},
           {"metadata", std::move(metadata)},
       }},
  };
}

}  // namespace scene_graph_exporter_ns
