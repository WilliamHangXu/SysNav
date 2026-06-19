/**
 * @file scene_graph_exporter.h
 * @brief Serializes the planner's in-memory scene graph (rooms / viewpoints /
 *        objects + door geometry) into a GADM-style JSON document.
 *
 * The exporter is a pure transformation from scene-graph data to
 * nlohmann::json. It has no dependency on rclcpp or SensorCoveragePlanner3D, so
 * it can be unit-tested in isolation and keeps the planner translation unit
 * lean. All I/O, timers and ROS-parameter loading live on the planner side.
 */
#pragma once

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "navgraph/navgraph_types.h"
#include "representation/representation.h"

namespace scene_graph_exporter_ns
{

/**
 * @brief Identifiers and behavior knobs for the exporter.
 *
 * Populated by the planner from the dedicated scene_graph_export.yaml. Only the
 * identifier / metadata fields are consumed inside Build(); the behavior fields
 * (enabled, output_root, timers, ...) are interpreted by the planner glue.
 */
struct SceneGraphExportConfig
{
  // --- behavior (interpreted by the planner, not by Build) ---
  bool enabled = true;
  std::string output_root = "output/scene_graph";
  double save_interval_s = 30.0;   // <= 0 disables periodic snapshots
  bool end_of_bag_save = true;     // final snapshot when sim time stalls
  double bag_end_timeout_s = 5.0;  // wall-clock stall before declaring "bag over"
  std::string manual_save_keyword = "save";

  // --- world-frame transform (composed by the planner, applied in Build) ---
  // The scene graph is built in the bag's odom frame (numerically kWorldFrameID
  // = `map`, identity-pinned to the bag odom). To express it in a building-fixed
  // `world` frame, the planner looks up the single static transform
  //   world_T_source = lookupTransform(world_frame, source_frame)
  // at export time (no gravity bridge, no per-run constants). Disabled =>
  // identity => coordinates stay in odom. tf2 strips a leading '/', so give
  // frame names without one. The exporter just receives the transform via Build.
  bool enabled_world_transform = false;
  std::string world_frame = "world";
  std::string source_frame = "map";  // frame the scene-graph coords are in

  // --- written verbatim into the JSON ---
  std::string zone = "all";        // single zone bucket for all rooms
  std::string map_id = "map";
  std::string warehouse_id = "map";
  std::string name = "map";
  std::string client_id;
  std::string uploaded_by;

  // --- layout.metadata ---
  std::string units = "meters";
  // Metadata label for the odom (no-world-transform) case. The planner
  // overwrites the emitted frame at snapshot time to the true coordinate frame:
  // world_frame when the world transform was applied, else this value.
  std::string frame = "odom";
  std::string building;
  int floor_level = 1;
  std::string floor_id;
};

class SceneGraphExporter
{
public:
  explicit SceneGraphExporter(SceneGraphExportConfig config);
  ~SceneGraphExporter() = default;

  /**
   * @brief Build a GADM-style scene-graph JSON snapshot.
   *
   * @param rooms       Persistent rooms keyed by stable id (alive rooms only).
   * @param objects     Persistent objects keyed by primary object id.
   * @param nav_nodes   NavGraph nodes keyed by stable id; each carries a room_id.
   *                    Emitted as the rooms' waypoints (wp_1..N after the wp_0
   *                    centroid). Nodes whose room_id is not an alive room are
   *                    skipped.
   * @param nav_edges   NavGraph edges (traversable connectivity + distance);
   *                    emitted into layout.edges, referencing node waypoint ids.
   * @param door_cloud  Door pixels; r/g channels carry the two room ids, x/y is
   *                    the source-frame position. Used to locate entrances.
   * @param world_from_source  Rigid transform mapping source-frame points into
   *                    world_frame. Pass identity to emit source-frame
   *                    coordinates unchanged.
   * @return The serialized scene graph as a nlohmann::json object.
   */
  nlohmann::json Build(
      const std::map<int, representation_ns::RoomNodeRep>& rooms,
      const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
      const std::map<int, navgraph_ns::NavNode>& nav_nodes,
      const std::vector<navgraph_ns::NavEdge>& nav_edges,
      const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
      const Eigen::Isometry3d& world_from_source =
          Eigen::Isometry3d::Identity()) const;

  // Stable, human-readable room key, e.g. "kitchen-room_1". Public so the
  // planner can name NavGraph nodes with the same key the exporter uses.
  static std::string RoomKey(const representation_ns::RoomNodeRep& room);

private:

  // Average door-pixel position (x, y, z) shared by room id_a and id_b, already
  // expressed in world_frame via world_from_source. Returns false (leaving
  // x/y/z untouched) when no such door pixel exists.
  static bool DoorCentroid(const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
                           int id_a, int id_b,
                           const Eigen::Isometry3d& world_from_source,
                           double& x, double& y, double& z);

  // Emits one room's JSON. The room's NavGraph nodes become wp_1..N (after the
  // wp_0 centroid); nav_id_to_wpid is populated with each node's waypoint id so
  // Build() can wire NavGraph edges to the right waypoints.
  nlohmann::json BuildRoomJson(
      const representation_ns::RoomNodeRep& room,
      const std::map<int, representation_ns::RoomNodeRep>& rooms,
      const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
      const std::vector<const navgraph_ns::NavNode*>& room_nav_nodes,
      const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
      const Eigen::Isometry3d& world_from_source,
      std::map<int, std::string>& nav_id_to_wpid) const;

  static nlohmann::json BuildObjectJson(
      const representation_ns::ObjectNodeRep& object);

  // World-space bounding box (width, height) over every room polygon.
  static void ComputeDimensions(
      const std::map<int, representation_ns::RoomNodeRep>& rooms,
      const Eigen::Isometry3d& world_from_source, double& width,
      double& height);

  SceneGraphExportConfig config_;
};

}  // namespace scene_graph_exporter_ns
