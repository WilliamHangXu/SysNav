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
    const representation_ns::ObjectNodeRep& object)
{
  std::string label = object.label_.empty() ? kUnknownLabel : object.label_;
  int primary_id = object.object_id_.empty() ? -1 : object.object_id_[0];
  return nlohmann::json{
      {"object_id", label + "_" + std::to_string(primary_id)},
      {"type", label},
      {"sgid", primary_id},
      {"waypoint", nlohmann::json::object()},
  };
}

nlohmann::json SceneGraphExporter::BuildRoomJson(
    const representation_ns::RoomNodeRep& room,
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
    const std::vector<representation_ns::ViewPointRep>& viewpoints,
    const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
    const Eigen::Isometry3d& world_from_source) const
{
  const std::string room_key = RoomKey(room);

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

  // --- waypoints: wp_0 = centroid, wp_1..N = the room's viewpoints ---
  nlohmann::json waypoints = nlohmann::json::array();
  const Eigen::Vector3d centroid_world = ToWorld(
      world_from_source, room.centroid_.x(), room.centroid_.y(),
      room.centroid_.z());
  waypoints.push_back(nlohmann::json{
      {"id", room_key + "-wp_0"},
      {"x", centroid_world.x()},
      {"y", centroid_world.y()},
      {"z", centroid_world.z()},
  });
  int wp_index = 1;
  for (int viewpoint_id : room.viewpoint_indices_)  // std::set => ascending
  {
    if (viewpoint_id < 0 ||
        viewpoint_id >= static_cast<int>(viewpoints.size()))
    {
      continue;
    }
    const auto& position = viewpoints[viewpoint_id].GetPosition();
    const Eigen::Vector3d world =
        ToWorld(world_from_source, position.x, position.y, position.z);
    waypoints.push_back(nlohmann::json{
        {"id", room_key + "-wp_" + std::to_string(wp_index)},
        {"x", world.x()},
        {"y", world.y()},
        {"z", world.z()},
    });
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
    object_array.push_back(BuildObjectJson(object_it->second));
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

nlohmann::json SceneGraphExporter::Build(
    const std::map<int, representation_ns::RoomNodeRep>& rooms,
    const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
    const std::vector<representation_ns::ViewPointRep>& viewpoints,
    const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
    const Eigen::Isometry3d& world_from_source) const
{
  nlohmann::json rooms_json = nlohmann::json::object();
  nlohmann::json all_waypoints = nlohmann::json::array();

  for (const auto& id_room : rooms)
  {
    const auto& room = id_room.second;
    nlohmann::json room_json = BuildRoomJson(room, rooms, objects, viewpoints,
                                             door_cloud, world_from_source);
    // Mirror every waypoint id into the flat top-level list.
    for (const auto& wp : room_json["waypoints"])
    {
      all_waypoints.push_back(wp["id"]);
    }
    rooms_json[RoomKey(room)] = std::move(room_json);
  }

  // --- edges: door-adjacent room pairs only, deduplicated (a < b) ---
  nlohmann::json edges = nlohmann::json::array();
  for (const auto& id_room : rooms)
  {
    const auto& room = id_room.second;
    for (int neighbor_id : room.neighbors_)
    {
      if (neighbor_id <= room.id_)
      {
        continue;  // emit each undirected pair once
      }
      auto neighbor_it = rooms.find(neighbor_id);
      if (neighbor_it == rooms.end())
      {
        continue;
      }
      const auto& neighbor = neighbor_it->second;
      // Measure between world-frame centroids so edges match wp_0 coordinates.
      const Eigen::Vector3d a = ToWorld(world_from_source, room.centroid_.x(),
                                        room.centroid_.y(), room.centroid_.z());
      const Eigen::Vector3d b =
          ToWorld(world_from_source, neighbor.centroid_.x(),
                  neighbor.centroid_.y(), neighbor.centroid_.z());
      const double dx = a.x() - b.x();
      const double dy = a.y() - b.y();
      edges.push_back(nlohmann::json{
          {"u", RoomKey(room) + "-wp_0"},
          {"v", RoomKey(neighbor) + "-wp_0"},
          {"meters", std::sqrt(dx * dx + dy * dy)},
      });
    }
  }

  double width = 0.0;
  double height = 0.0;
  ComputeDimensions(rooms, world_from_source, width, height);

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
           {"metadata",
            {
                {"units", config_.units},
                {"building", config_.building},
                {"floor_level", config_.floor_level},
                {"floor_id", config_.floor_id},
                {"dimensions", {{"width", width}, {"height", height}}},
            }},
       }},
  };
}

}  // namespace scene_graph_exporter_ns
