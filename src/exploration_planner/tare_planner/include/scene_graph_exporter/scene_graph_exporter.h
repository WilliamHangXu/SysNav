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
  // The scene graph lives in arise's `map` frame, a disjoint TF tree from the
  // bag's `world`. The planner bridges them via the shared physical LiDAR:
  //   world_T_map = world_T_livox * G * (map_T_sensor)^-1
  // where G (= livox_T_sensor) is the gravity rotation, arise's per-run
  // imu_laser_R_Gravity (identical to cloud_image_fusion.py's R_GRAVITY).
  // tf2 strips a leading '/', so give frame names without one. gravity_matrix is
  // row-major 3x3. The exporter just receives the final transform via Build().
  bool enabled_world_transform = false;
  std::string world_frame = "world";
  std::string livox_frame = "go2w_005/livox_frame";
  std::string map_frame = "map";
  std::string sensor_frame = "sensor";
  std::array<double, 9> gravity_matrix = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  // --- written verbatim into the JSON ---
  std::string zone = "all";        // single zone bucket for all rooms
  std::string map_id = "map";
  std::string warehouse_id = "map";
  std::string name = "map";
  std::string client_id;
  std::string uploaded_by;

  // --- layout.metadata ---
  std::string units = "meters";
  // Which bag frame the coordinates are in ("world" or "odom"). Written
  // verbatim so snapshots stay distinguishable; the exporter cannot see
  // bag_slam_bridge's anchor_frame, so keep the two in sync manually.
  std::string frame = "world";
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
   * @param viewpoints  Viewpoints indexed by id (id == vector index).
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
      const std::vector<representation_ns::ViewPointRep>& viewpoints,
      const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
      const Eigen::Isometry3d& world_from_source =
          Eigen::Isometry3d::Identity()) const;

private:
  // Stable, human-readable room key, e.g. "kitchen-room_1".
  static std::string RoomKey(const representation_ns::RoomNodeRep& room);

  // Average door-pixel position (x, y, z) shared by room id_a and id_b, already
  // expressed in world_frame via world_from_source. Returns false (leaving
  // x/y/z untouched) when no such door pixel exists.
  static bool DoorCentroid(const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
                           int id_a, int id_b,
                           const Eigen::Isometry3d& world_from_source,
                           double& x, double& y, double& z);

  nlohmann::json BuildRoomJson(
      const representation_ns::RoomNodeRep& room,
      const std::map<int, representation_ns::RoomNodeRep>& rooms,
      const std::unordered_map<int, representation_ns::ObjectNodeRep>& objects,
      const std::vector<representation_ns::ViewPointRep>& viewpoints,
      const pcl::PointCloud<pcl::PointXYZRGBL>& door_cloud,
      const Eigen::Isometry3d& world_from_source) const;

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
