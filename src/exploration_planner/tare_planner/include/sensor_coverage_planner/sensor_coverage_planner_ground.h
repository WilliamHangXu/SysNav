/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>
// ROS
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
// Third parties
#include <utils/misc_utils.h>
#include <utils/pointcloud_utils.h>
// Components
#include "grid_world/grid_world.h"
#include "keypose_graph/keypose_graph.h"
#include "navgraph/navgraph.h"
#include "quadrant_manager/quadrant_manager.h"
#include "planning_env/planning_env.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"
#include "viewpoint_manager/viewpoint_manager.h"

#include "representation/representation.h"
#include "scene_graph_exporter/scene_graph_exporter.h"
#include "grid/grid.h"
#include "tare_planner/msg/object_node.hpp"
#include "tare_planner/msg/object_node_list.hpp"
#include "tare_planner/msg/viewpoint_rep.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "tare_planner/msg/room_node.hpp"
#include "tare_planner/msg/room_node_list.hpp"
#include "tare_planner/msg/room_type.hpp"
#include <filesystem>
#include <unordered_map>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace sensor_coverage_planner_3d_ns {
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;

class SensorCoveragePlanner3D : public rclcpp::Node {
public:
  explicit SensorCoveragePlanner3D();
  bool initialize();
  void execute();
  ~SensorCoveragePlanner3D() = default;

private:
  // Parameters
  // String
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_registered_scan_topic_;
  std::string sub_camera_image_topic_;

  // Double
  double kKeyposeCloudDwzFilterLeafSize;

  // Int
  int previous_room_id_;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>
      keypose_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>
      registered_scan_stack_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      registered_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      keypose_graph_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      viewpoint_in_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      point_cloud_manager_neighbor_cloud_;

  nav_msgs::msg::Odometry keypose_;
  geometry_msgs::msg::Point robot_position_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  std::vector<Eigen::Vector3d> visited_positions_;
  int cur_keypose_node_ind_;
  Eigen::Vector3d initial_position_;

  std::shared_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  std::shared_ptr<navgraph_ns::NavGraph> navgraph_;
  std::shared_ptr<quadrant_ns::QuadrantManager> quadrant_mgr_;
  std::shared_ptr<planning_env_ns::PlanningEnv> planning_env_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;
  std::shared_ptr<grid_world_ns::GridWorld> grid_world_;

  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  std::shared_ptr<misc_utils_ns::Marker> grid_world_marker_;

  bool keypose_cloud_update_;
  bool initialized_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;

  int registered_cloud_count_;
  int keypose_count_;

  // First-execute() timestamp; gates the freespace-cloud warm-up in
  // PublishFreespaceCloud (no publish for the first 20 s).
  double start_time_;

  rclcpp::TimerBase::SharedPtr execution_timer_;

  // ROS subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      registered_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      state_estimation_sub_;

  // ROS publishers
  // Debug
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pointcloud_manager_neighbor_cells_origin_pub_;

  void ReadParameters();
  void InitializeData();

  // Callback functions
  void StateEstimationCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg);
  void RegisteredScanCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_cloud_msg);

  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int &uncovered_point_num,
                          int &uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  // Connector-node injection into the keypose graph (feeds the NavGraph):
  // UpdateCellStatus -> UpdateCellKeyposeGraphNodes -> AddPathsInBetweenCells.
  void GlobalPlanning();

  // -------------------------------------------------------------------------------------
  // ========== ROS Subscribers ==========
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr door_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr room_mask_sub_;
  rclcpp::Subscription<tare_planner::msg::RoomNodeList>::SharedPtr room_node_list_sub_;
  rclcpp::Subscription<tare_planner::msg::RoomType>::SharedPtr room_type_sub_;
  rclcpp::Subscription<tare_planner::msg::ObjectNodeList>::SharedPtr object_node_list_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_input_sub_;

  // ========== ROS Publishers ==========
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_node_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_visibility_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr room_type_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoint_room_id_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr viewpoint_visibility_pub_;
  rclcpp::Publisher<tare_planner::msg::RoomType>::SharedPtr room_type_pub_;
  rclcpp::Publisher<tare_planner::msg::ViewpointRep>::SharedPtr viewpoint_rep_pub_;

  // ========== VLM-Related Functions ==========
  // Viewpoint representation
  void UpdateViewpointRep();
  
  // Callback functions
  void ObjectNodeListCallback(const tare_planner::msg::ObjectNodeList::ConstSharedPtr msg);
  void DoorCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr door_cloud_msg);
  void RoomNodeListCallback(const tare_planner::msg::RoomNodeList::ConstSharedPtr room_node_list_msg);
  void RoomMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr room_mask_msg);
  void CameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void RoomTypeCallback(const tare_planner::msg::RoomType::ConstSharedPtr msg);
  void KeyboardInputCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  // Utility functions
  bool CheckRayVisibilityInOccupancyGrid(const Eigen::Vector3i& start_pos, const Eigen::Vector3i& end_pos);
  bool InRange(const Eigen::Vector3i& voxel_index) const;
  std::vector<Eigen::Vector3i> Convert2Voxels(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void GetPoseAtTime(double imageTime, float &lidarX, float &lidarY, float &lidarZ,
                     float &lidarRoll, float &lidarPitch, float &lidarYaw);
  // Non-mutating yaw lookup at an arbitrary time (does NOT advance
  // odomFrontIDPointer, unlike GetPoseAtTime), so it can be sampled on both
  // sides of a capture time for a centered yaw-rate estimate.
  double GetYawAtTime(double queryTime);
  cv::Mat project_pcl_to_image(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_w,
                                float &lidarX, float &lidarY, float &lidarZ,
                                float &lidarRoll, float &lidarPitch, float &lidarYaw,
                                cv::Mat &image, pcl::PointXYZI &room_center, int &room_id);

  // Visualization functions
  void CreateVisibilityMarkers();
  void PublishViewpointRoomIdMarkers();
  void PublishRoomTypeVisualization();
  void PublishObjectNodeMarkers();
  void PublishFreespaceCloud();
  
  // Room management functions
  void SetCurrentRoomId();
  // Evaluate the latest camera frame (motion-gated): attribute it to the room
  // whose floor the current LiDAR sweep observes most within the camera FOV,
  // and admit it into that room's best-3 by pose diversity.
  void UpdateRoomViews();
  // World point -> Go2 front pinhole camera. Returns true iff the point lands
  // inside the image (in front + within 1280x720 after distortion). depth_out =
  // forward distance along the optical axis (for the range gate).
  bool PointToCameraView(const Eigen::Vector3f &p_world,
                         float lidarX, float lidarY, float lidarZ,
                         float lidarRoll, float lidarPitch, float lidarYaw,
                         float &depth_out) const;
  // Emit a room-type query (best-3 image paths + object inventory) for each
  // room whose evidence changed since its last query (rate-limited).
  void PublishRoomTypeQueries();
  // Debug: dump a room-type query's payload (paths + objects + scalars) as JSON.
  void LogRoomTypeQuery(const tare_planner::msg::RoomType &msg);
  // Debug: dump a room-type VLM answer plus the running vote histogram and the
  // resulting label to the same per-run dir. No-op unless enabled.
  void LogRoomTypeAnswer(const tare_planner::msg::RoomType &msg, int room_id,
                         const std::string &previous_label,
                         const std::string &current_label);

  // Object detection and tracking functions
  void UpdateObjectVisibility();
  void UpdateViewpointObjectVisibility();
  void ProcessObjectNodes();
  
  // GADM-style scene-graph snapshot export
  void SaveSceneGraphSnapshot(const std::string &reason);
  void SceneGraphWatchdogCallback();
  bool TryFreezeWorldFromOdom();  // look up & latch world_T_odom once

  // ========== VLM-Related Data Members ==========
  // Representation core
  std::shared_ptr<representation_ns::Representation> representation_;

  // Scene-graph JSON export
  scene_graph_exporter_ns::SceneGraphExportConfig scene_graph_cfg_;
  std::unique_ptr<scene_graph_exporter_ns::SceneGraphExporter> scene_graph_exporter_;
  std::string scene_graph_run_dir_;
  int scene_graph_snapshot_count_;

  // Room-type query debug log (off by default; param room_type_query_log.enabled)
  bool room_type_query_log_enabled_;
  std::string room_type_query_log_dir_;
  int room_type_query_log_seq_;
  int room_type_answer_log_seq_;

  // Per-room best-3 view buffer (stage one). Gated by room_type_query_log.enabled.
  std::string room_views_dir_;          // <run>/room_views
  float room_view_max_range_;           // max useful range for coverage (m)
  float room_view_min_coverage_m2_;     // reject frames seeing less than this
  float room_view_max_yaw_rate_;        // reject frames captured turning faster (rad/s)
  double room_view_yaw_rate_window_s_;  // half-window for centered yaw-rate estimate (s)
  float room_view_object_conf_min_;     // object inventory confidence floor
  double room_type_query_min_interval_s_;  // per-room re-query rate limit
  float room_view_pose_dist_thresh_;    // pose-diversity: min position separation (m)
  float room_view_yaw_thresh_rad_;      // pose-diversity: min heading separation
  float room_view_motion_dist_thresh_;  // intake gate: min move since last eval (m)
  float room_view_motion_yaw_thresh_rad_;// intake gate: min turn since last eval
  bool room_view_have_last_pose_;       // has a previous eval pose been recorded
  float room_view_last_x_, room_view_last_y_, room_view_last_yaw_;
  bool scene_graph_final_saved_;
  bool scene_graph_clock_started_;  // sim clock has advanced at least once (bag playing)
  rclcpp::Time scene_graph_last_sim_time_;
  rclcpp::TimerBase::SharedPtr scene_graph_save_timer_;
  rclcpp::TimerBase::SharedPtr scene_graph_watchdog_timer_;
  // world_T_map for snapshots: composed once via the shared LiDAR across the
  // bag's `world` tree and arise's `map` tree, then frozen. Buffer/listener only
  // created when scene_graph_export.world_transform.enabled.
  std::shared_ptr<tf2_ros::Buffer> scene_graph_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> scene_graph_tf_listener_;
  Eigen::Isometry3d scene_graph_world_from_map_ = Eigen::Isometry3d::Identity();
  bool scene_graph_world_from_map_valid_ = false;
  rclcpp::TimerBase::SharedPtr scene_graph_world_tf_timer_;

  // --- Camera calibration for room-view coverage (PointToCameraView) ---
  // Mirror of semantic_mapping's topic-driven calibration: intrinsics from the
  // rectified camera_info (P matrix, no distortion) + base->camera-optical
  // extrinsic from tf. Falls back to the legacy hardcoded raw model when not
  // calibrated (e.g. empty robot_namespace).
  bool uses_topic_calib_ = false;
  bool camera_calibrated_ = false;
  std::string camera_info_topic_;
  std::string base_frame_;  // <ns>/base, source frame for the camera tf lookup
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  std::shared_ptr<tf2_ros::Buffer> camera_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> camera_tf_listener_;
  Eigen::Matrix3f cam_R_l2c_ = Eigen::Matrix3f::Identity();
  Eigen::Vector3f cam_t_l2c_ = Eigen::Vector3f::Zero();
  float cam_fx_ = 0.f, cam_fy_ = 0.f, cam_cx_ = 0.f, cam_cy_ = 0.f;
  int cam_img_w_ = 0, cam_img_h_ = 0;
  void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool TryCalibrateCamera();

  // Viewpoint representation parameters
  double rep_threshold_;
  int rep_threshold_voxel_num_;
  bool add_viewpoint_rep_;
  int curr_viewpoint_rep_node_ind;
  std::vector<representation_ns::ViewPointRep> viewpoint_reps_;
  std::vector<int> previous_obs_voxel_inds_;
  std::vector<int> current_obs_voxel_inds_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> viewpoint_rep_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> covered_points_all_;
  tare_planner::msg::ViewpointRep viewpoint_rep_msg_;
  
  // Door and room boundary data
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_final_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZRGBL>> door_cloud_vis_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZLNormal>> door_cloud_in_range_;

  // Room state flags
  bool enter_wrong_room_;

  // Room data structures
  Eigen::MatrixXi adjacency_matrix;
  Eigen::Vector3i room_voxel_dimension_;
  Eigen::Vector3f shift_;
  std::vector<representation_ns::RoomNodeRep> room_nodes_;
  std::vector<representation_ns::RoomNodeRep> room_nodes_tmp;
  cv::Mat room_mask_;
  cv::Mat room_mask_old_;
  
  // Room IDs and positions
  int current_room_id_;
  geometry_msgs::msg::Point robot_position_old_;

  // Room counters and parameters
  int room_id_change_counter_;
  int room_finished_counter_;
  float room_resolution_;
  float occupancy_grid_resolution_;
  
  // Object detection parameters
  rclcpp::Time last_object_update_time_;
  double rep_sensor_range;
  std::vector<int> object_ids_to_remove_;
  double obj_score_;

  // Camera and sensor data
  cv::Mat camera_image_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> freespace_cloud_;
  
  // Odometry stack for pose interpolation
  static constexpr int kOdomStackSize = 400;
  float lidarXStack[kOdomStackSize];
  float lidarYStack[kOdomStackSize];
  float lidarZStack[kOdomStackSize];
  float lidarRollStack[kOdomStackSize];
  float lidarPitchStack[kOdomStackSize];
  float lidarYawStack[kOdomStackSize];
  double odomTimeStack[kOdomStackSize];
  int odomLastIDPointer;
  int odomFrontIDPointer;
  double odomTime;
  double imageTime;
  float odomX;
  float odomY;
  float odomZ;
  double PI;
  
  // Miscellaneous flags
  bool tmp_flag_;
};

} // namespace sensor_coverage_planner_3d_ns
