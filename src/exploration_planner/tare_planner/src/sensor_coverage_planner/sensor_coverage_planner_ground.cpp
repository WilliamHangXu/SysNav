/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_set>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
using json = nlohmann::json;

using namespace std::chrono_literals;

namespace sensor_coverage_planner_3d_ns {

// ===== Room lifecycle diagnostics (temporary; grep "[room_dbg]") =============
// Tracks a room from CREATE -> view admission -> QUERY -> ANSWER(apply/drop) ->
// VALIDATE(strip) -> DEATH, plus every MASK_UPDATE, so a single pipeline.log can
// explain why an imaged room ends up unlabeled. Flip ROOM_DBG_ENABLED to 0 to
// silence everything in one place.
#define ROOM_DBG_ENABLED 1
#if ROOM_DBG_ENABLED
#define ROOM_DBG(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#else
#define ROOM_DBG(...) ((void)0)
#endif

// ===========================================================================

void SensorCoveragePlanner3D::ReadParameters() {
  this->declare_parameter<std::string>("sub_state_estimation_topic_",
                                       "/state_estimation_at_scan");
  this->declare_parameter<std::string>("sub_registered_scan_topic_",
                                       "/registered_scan");
  this->declare_parameter<std::string>("sub_camera_image_topic_",
                                       "/camera/image");
  // Double
  this->declare_parameter<double>("kKeyposeCloudDwzFilterLeafSize", 0.2);

  // grid_world
  this->declare_parameter<int>("kGridWorldXNum", 121);
  this->declare_parameter<int>("kGridWorldYNum", 121);
  this->declare_parameter<int>("kGridWorldZNum", 12);
  this->declare_parameter<double>("kGridWorldCellHeight", 8.0);
  this->declare_parameter<int>("kGridWorldNearbyGridNum", 5);
  this->declare_parameter<int>("kMinAddPointNumSmall", 60);
  this->declare_parameter<int>("kMinAddPointNumBig", 100);
  this->declare_parameter<int>("kMinAddFrontierPointNum", 30);
  this->declare_parameter<int>("kCellExploringToCoveredThr", 1);
  this->declare_parameter<int>("kCellCoveredToExploringThr", 10);
  this->declare_parameter<int>("kCellExploringToAlmostCoveredThr", 10);
  this->declare_parameter<int>("kCellAlmostCoveredToExploringThr", 20);
  this->declare_parameter<int>("kCellUnknownToExploringThr", 1);

  // keypose_graph
  this->declare_parameter<double>("keypose_graph/kAddNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddNonKeyposeNodeMinDist",
                                  0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeConnectDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeToLastKeyposeDistThr",
                                  0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeVerticalThreshold",
                                  0.5);
  this->declare_parameter<double>(
      "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckRadius",
                                  0.5);
  this->declare_parameter<int>(
      "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

  // planning_env
  this->declare_parameter<double>("kSurfaceCloudDwzLeafSize", 0.2);
  this->declare_parameter<double>("kCollisionCloudDwzLeafSize", 0.2);
  this->declare_parameter<int>("kKeyposeCloudStackNum", 5);
  this->declare_parameter<int>("kPointCloudRowNum", 20);
  this->declare_parameter<int>("kPointCloudColNum", 20);
  this->declare_parameter<int>("kPointCloudLevelNum", 10);
  this->declare_parameter<int>("kMaxCellPointNum", 100000);
  this->declare_parameter<double>("kPointCloudCellSize", 24.0);
  this->declare_parameter<double>("kPointCloudCellHeight", 3.0);
  this->declare_parameter<int>("kPointCloudManagerNeighborCellNum", 5);
  this->declare_parameter<double>("kCoverCloudZSqueezeRatio", 2.0);
  this->declare_parameter<double>("kFrontierClusterTolerance", 1.0);
  this->declare_parameter<int>("kFrontierClusterMinSize", 30);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnFrontier", false);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnObjectSurface", false);
  this->declare_parameter<bool>("kUseFrontier", true);

  // rolling_occupancy_grid
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_x", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_y", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_z", 0.3);

  // viewpoint_manager
  this->declare_parameter<int>("viewpoint_manager/number_x", 80);
  this->declare_parameter<int>("viewpoint_manager/number_y", 80);
  this->declare_parameter<int>("viewpoint_manager/number_z", 40);
  this->declare_parameter<double>("viewpoint_manager/resolution_x", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_y", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_z", 0.5);
  this->declare_parameter<double>("kConnectivityHeightDiffThr", 0.25);
  this->declare_parameter<double>("kViewPointCollisionMargin", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZPlus", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZMinus", 0.5);
  this->declare_parameter<double>("kCollisionGridZScale", 2.0);
  this->declare_parameter<double>("kCollisionGridResolutionX", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionY", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionZ", 0.5);
  this->declare_parameter<bool>("kLineOfSightStopAtNearestObstacle", true);
  this->declare_parameter<bool>("kCheckDynamicObstacleCollision", true);
  this->declare_parameter<int>("kCollisionFrameCountMax", 3);
  this->declare_parameter<double>("kViewPointHeightFromTerrain", 0.75);
  this->declare_parameter<double>("kViewPointHeightFromTerrainChangeThreshold",
                                  0.6);
  this->declare_parameter<int>("kCollisionPointThr", 3);
  this->declare_parameter<double>("kCoverageOcclusionThr", 1.0);
  this->declare_parameter<double>("kCoverageDilationRadius", 1.0);
  this->declare_parameter<double>("kCoveragePointCloudResolution", 1.0);
  this->declare_parameter<double>("kSensorRange", 10.0);
  this->declare_parameter<double>("kNeighborRange", 3.0);

  // room
  this->declare_parameter<float>("room_resolution");
  this->declare_parameter<int>("room_x");
  this->declare_parameter<int>("room_y");
  this->declare_parameter<int>("room_z");

  this->get_parameter("sub_state_estimation_topic_",
                      sub_state_estimation_topic_);
  this->get_parameter("sub_registered_scan_topic_", sub_registered_scan_topic_);
  this->get_parameter("sub_camera_image_topic_", sub_camera_image_topic_);

  // --- Multi-robot portability: one knob (robot_namespace) ---
  // All robots run the same nav stack, so only the namespace differs. Robot-
  // source inputs become /<robot_namespace>/<suffix>; internal stack topics stay
  // constant. Empty namespace keeps the values read above (pre-namespace
  // behavior, relying on a bag-play remap). Set in the scenario yaml's /** block.
  this->declare_parameter<std::string>("robot_namespace", "");
  this->declare_parameter<std::string>("topic_suffix.registered_scan",
                                       "cloud_registered");
  this->declare_parameter<std::string>("topic_suffix.odometry", "lio/odometry");
  this->declare_parameter<std::string>("topic_suffix.camera_image",
                                       "camera/image_raw");
  {
    const std::string robot_ns =
        this->get_parameter("robot_namespace").as_string();
    if (!robot_ns.empty()) {
      const std::string odom_suffix =
          this->get_parameter("topic_suffix.odometry").as_string();
      sub_registered_scan_topic_ =
          "/" + robot_ns + "/" +
          this->get_parameter("topic_suffix.registered_scan").as_string();
      sub_state_estimation_topic_ = "/" + robot_ns + "/" + odom_suffix;
      sub_camera_image_topic_ =
          "/" + robot_ns + "/" +
          this->get_parameter("topic_suffix.camera_image").as_string();
      RCLCPP_INFO(this->get_logger(),
                  "[robot_namespace=%s] registered_scan=%s "
                  "state_estimation=%s camera=%s",
                  robot_ns.c_str(),
                  sub_registered_scan_topic_.c_str(),
                  sub_state_estimation_topic_.c_str(),
                  sub_camera_image_topic_.c_str());
    }
  }
  this->get_parameter("kKeyposeCloudDwzFilterLeafSize",
                      kKeyposeCloudDwzFilterLeafSize);

  this->declare_parameter<double>("rep_threshold_", 0.1);
  this->get_parameter("rep_threshold_", rep_threshold_);
  this->declare_parameter<double>("kRepSensorRange", 5.0);

  this->get_parameter("room_resolution", room_resolution_);
  this->get_parameter("rolling_occupancy_grid/resolution_x",
                      occupancy_grid_resolution_);
  room_voxel_dimension_.x() = this->get_parameter("room_x").as_int();
  room_voxel_dimension_.y() = this->get_parameter("room_y").as_int();
  room_voxel_dimension_.z() = this->get_parameter("room_z").as_int();

  // SemPathBench snapshot export (tools/sempath_export)
  this->declare_parameter<bool>("export.enabled", sempath_cfg_.enabled);
  this->declare_parameter<std::string>("export.output_dir", sempath_cfg_.output_dir);
  this->declare_parameter<double>("export.interval_s", sempath_cfg_.interval_s);
  this->declare_parameter<std::string>("export.keyword", sempath_cfg_.keyword);
  this->declare_parameter<bool>("export.keep_history", sempath_cfg_.keep_history);
  this->declare_parameter<bool>("export.include_clouds", sempath_cfg_.include_clouds);
  this->declare_parameter<double>("export.cloud_voxel_m", sempath_cfg_.cloud_voxel_m);
  this->declare_parameter<int>("export.mask_crop_margin_cells", sempath_cfg_.mask_crop_margin_cells);
  this->get_parameter("export.enabled", sempath_cfg_.enabled);
  this->get_parameter("export.output_dir", sempath_cfg_.output_dir);
  this->get_parameter("export.interval_s", sempath_cfg_.interval_s);
  this->get_parameter("export.keyword", sempath_cfg_.keyword);
  this->get_parameter("export.keep_history", sempath_cfg_.keep_history);
  this->get_parameter("export.include_clouds", sempath_cfg_.include_clouds);
  this->get_parameter("export.cloud_voxel_m", sempath_cfg_.cloud_voxel_m);
  this->get_parameter("export.mask_crop_margin_cells", sempath_cfg_.mask_crop_margin_cells);
  // The node cwd is whatever launched it (the teleop scripts cd to the repo root); pin the path now.
  sempath_cfg_.output_dir =
      std::filesystem::absolute(std::filesystem::path(sempath_cfg_.output_dir)).lexically_normal().string();
}

// void PlannerData::Initialize(rclcpp::Node::SharedPtr node_)
void SensorCoveragePlanner3D::InitializeData() {
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
          shared_from_this(), "keypose_cloud", kWorldFrameID);
  registered_scan_stack_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
          shared_from_this(), "registered_scan_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "registered_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "collision_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "pointcloud_manager_cloud", kWorldFrameID);
  freespace_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "freespace_cloud", kWorldFrameID);

  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(
      shared_from_this());
  keypose_graph_ =
      std::make_shared<keypose_graph_ns::KeyposeGraph>(shared_from_this());
  planning_env_ =
      std::make_shared<planning_env_ns::PlanningEnv>(shared_from_this());
  grid_world_ = std::make_shared<grid_world_ns::GridWorld>(shared_from_this());
  grid_world_->SetUseKeyposeGraph(true);

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(
      visualization_msgs::msg::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  grid_world_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::msg::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist =
      std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  // ========== VLM-Related Initialization ==========
  
  // Representation core
  representation_ = std::make_shared<representation_ns::Representation>(shared_from_this(), kWorldFrameID);

  // Viewpoint representation initialization
  double resolution = this->get_parameter("rolling_occupancy_grid/resolution_x").as_double();
  rep_sensor_range = this->get_parameter("kRepSensorRange").as_double();
  rep_threshold_voxel_num_ = int(rep_threshold_ * (2.0 / 3.0 * M_PI * std::pow(rep_sensor_range, 3)) / 
                                 (resolution * resolution * resolution));
  add_viewpoint_rep_ = false;
  curr_viewpoint_rep_node_ind = 0;
  
  viewpoint_reps_ = std::vector<representation_ns::ViewPointRep>();
  previous_obs_voxel_inds_ = std::vector<int>();
  current_obs_voxel_inds_ = std::vector<int>();
  
  viewpoint_rep_vis_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
      shared_from_this(), "viewpoint_rep_vis_cloud", kWorldFrameID);
  covered_points_all_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      shared_from_this(), "viewpoint_rep_covered_points", kWorldFrameID);
  viewpoint_rep_msg_ = tare_planner::msg::ViewpointRep();

  // Door and room boundary initialization
  door_cloud_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
  door_cloud_final_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
  door_cloud_vis_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZRGBL>>(
      shared_from_this(), "door_cloud_vis", kWorldFrameID);
  door_cloud_in_range_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZLNormal>>(
      shared_from_this(), "door_cloud_in_range", kWorldFrameID);

  // Room state flags initialization
  enter_wrong_room_ = false;

  // Room data structures initialization
  adjacency_matrix = Eigen::MatrixXi::Zero(200, 200);
  room_voxel_dimension_ = Eigen::Vector3i(
      this->get_parameter("room_x").as_int(),
      this->get_parameter("room_y").as_int(),
      this->get_parameter("room_z").as_int());
  shift_ = Eigen::Vector3f(room_voxel_dimension_.x() / 2.0,
                           room_voxel_dimension_.y() / 2.0,
                           room_voxel_dimension_.z() / 2.0);
  room_mask_ = cv::Mat::zeros(room_voxel_dimension_.x(), room_voxel_dimension_.y(), CV_32S);
  room_mask_old_ = room_mask_.clone();

  // Room IDs and positions initialization
  current_room_id_ = -1;
  previous_room_id_ = -1;
  robot_position_old_ = robot_position_;

  // Room counters and parameters initialization
  room_id_change_counter_ = 0;
  room_finished_counter_ = 0;
  room_resolution_ = this->get_parameter("room_resolution").as_double();
  occupancy_grid_resolution_ = resolution;

  // Object detection parameters initialization
  last_object_update_time_ = this->now();
  object_ids_to_remove_ = std::vector<int>();
  obj_score_ = 0.0;

  // Camera and sensor data initialization
  camera_image_ = cv::Mat::zeros(640, 1920, CV_8UC3);
  freespace_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      shared_from_this(), "freespace_cloud", kWorldFrameID);

  // Odometry stack initialization
  odomLastIDPointer = -1;
  odomFrontIDPointer = 0;
  odomTime = 0.0;
  imageTime = 0.0;
  odomX = 0.0;
  odomY = 0.0;
  odomZ = 0.0;
  PI = 3.14159265358979323846;

  // Miscellaneous flags initialization
  tmp_flag_ = false;
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D()
    : Node("tare_planner_node"), keypose_cloud_update_(false),
      initialized_(false),
      test_point_update_(false), viewpoint_ind_update_(false), step_(false),
      registered_cloud_count_(0), keypose_count_(0), add_viewpoint_rep_(false)
{
  std::cout << "finished constructor" << std::endl;
}

bool SensorCoveragePlanner3D::initialize() {
  ReadParameters();
  InitializeData();

  keypose_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(
      planning_env_->GetPlannerCloudResolution());

  execution_timer_ = this->create_wall_timer(
      1000ms, std::bind(&SensorCoveragePlanner3D::execute, this));

  if (sempath_cfg_.enabled)
  {
    std::error_code ec;
    std::filesystem::create_directories(sempath_cfg_.output_dir, ec);
    if (ec)
    {
      RCLCPP_ERROR(this->get_logger(), "[sempath_export] cannot create %s: %s",
                   sempath_cfg_.output_dir.c_str(), ec.message().c_str());
    }
    if (sempath_cfg_.interval_s > 0.0)
    {
      sempath_export_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(sempath_cfg_.interval_s),
          [this]() { ExportSemPathSnapshot("periodic"); });
    }
    RCLCPP_INFO(this->get_logger(),
                "[sempath_export] snapshots -> %s (every %.0f s, keyword \"%s\", final on shutdown)",
                sempath_cfg_.output_dir.c_str(), sempath_cfg_.interval_s, sempath_cfg_.keyword.c_str());
  }

  registered_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_registered_scan_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::RegisteredScanCallback, this,
                    std::placeholders::_1));
  state_estimation_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      sub_state_estimation_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::StateEstimationCallback, this,
                std::placeholders::_1));
  object_node_list_sub_ = this->create_subscription<tare_planner::msg::ObjectNodeList>(
      "/object_nodes_list", 20,
      std::bind(&SensorCoveragePlanner3D::ObjectNodeListCallback, this,
                std::placeholders::_1));
  door_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/door_cloud", 5,
      std::bind(&SensorCoveragePlanner3D::DoorCloudCallback, this,
                std::placeholders::_1));
  room_node_list_sub_ = this->create_subscription<tare_planner::msg::RoomNodeList>(
      "/room_nodes_list", 5,
      std::bind(&SensorCoveragePlanner3D::RoomNodeListCallback, this,
                std::placeholders::_1));
  room_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/room_mask", 5,
      std::bind(&SensorCoveragePlanner3D::RoomMaskCallback, this,
                std::placeholders::_1));
  camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      sub_camera_image_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::CameraImageCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Room-type query crops use camera topic: %s",
              sub_camera_image_topic_.c_str());
  room_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/room_cloud", 1);
  room_type_sub_ = this->create_subscription<tare_planner::msg::RoomType>(
      "/room_type_answer", 10,
      std::bind(&SensorCoveragePlanner3D::RoomTypeCallback, this,
                std::placeholders::_1));
  keyboard_input_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/keyboard_input", 5,
      std::bind(&SensorCoveragePlanner3D::KeyboardInputCallback, this,
                std::placeholders::_1));

  pointcloud_manager_neighbor_cells_origin_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "pointcloud_manager_neighbor_cells_origin", 1);
  viewpoint_rep_pub_ =
      this->create_publisher<tare_planner::msg::ViewpointRep>("viewpoint_rep_header", 5);
  object_visibility_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "object_visibility_connections", 1);
  viewpoint_visibility_pub_ = this ->create_publisher<std_msgs::msg::String>(
      "viewpoint_object_visibility", 1);
  room_type_pub_ = this->create_publisher<tare_planner::msg::RoomType>(
      "/room_type_query", 10);
  room_type_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/room_type_vis", 5);
  viewpoint_room_id_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "viewpoint_room_ids", 1);
  object_node_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/object_node_markers", 1);

  return true;
}

void SensorCoveragePlanner3D::StateEstimationCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg) {
  robot_position_ = state_estimation_msg->pose.pose.position;
  // Todo: use a boolean
  if (std::abs(initial_position_.x()) < 0.01 &&
      std::abs(initial_position_.y()) < 0.01 &&
      std::abs(initial_position_.z()) < 0.01) {
    initial_position_.x() = robot_position_.x;
    initial_position_.y() = robot_position_.y;
    initial_position_.z() = robot_position_.z;
  }
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat =
      state_estimation_msg->pose.pose.orientation;
  tf2::Matrix3x3(
      tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
      .getRPY(roll, pitch, yaw);

  // Get the timestamp from the state estimation message(for case of ros bag)
  viewpoint_rep_msg_.header.stamp = state_estimation_msg->header.stamp;
  viewpoint_rep_msg_.header.frame_id = kWorldFrameID;
  // initialized_ = true;

  odomTime = rclcpp::Time(state_estimation_msg->header.stamp).seconds();
  odomX = state_estimation_msg->pose.pose.position.x;
  odomY = state_estimation_msg->pose.pose.position.y;
  odomZ = state_estimation_msg->pose.pose.position.z;

  odomLastIDPointer = (odomLastIDPointer + 1) % 400;
  odomTimeStack[odomLastIDPointer] = odomTime;
  lidarXStack[odomLastIDPointer] = odomX;
  lidarYStack[odomLastIDPointer] = odomY;
  lidarZStack[odomLastIDPointer] = odomZ;
  lidarRollStack[odomLastIDPointer] = roll;
  lidarPitchStack[odomLastIDPointer] = pitch;
  lidarYawStack[odomLastIDPointer] = yaw;
}

void SensorCoveragePlanner3D::CameraImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr camera_image_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (camera_image_msg->data.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Camera image data is empty");
    return;
  }
  // 转换成 OpenCV 格式
  camera_image_ = cv_bridge::toCvCopy(camera_image_msg, "bgr8")->image;
  imageTime = rclcpp::Time(camera_image_msg->header.stamp).seconds();
}

void SensorCoveragePlanner3D::RegisteredScanCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  if (!initialized_) {
    return;
  }

  // ---- [PROF] registered-scan callback timing. (disabled)
  // static auto prof_last_cb = std::chrono::steady_clock::now();
  // auto prof_cb_t0 = std::chrono::steady_clock::now();
  // double prof_cb_gap_ms =
  //     std::chrono::duration<double, std::milli>(prof_cb_t0 - prof_last_cb).count();
  // prof_last_cb = prof_cb_t0;

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;

  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  if (registered_scan_tmp->points.empty()) {
    return;
  }
  // size_t prof_in_pts = registered_scan_tmp->points.size();  // [PROF] disabled
  *(registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(
      registered_scan_tmp, kKeyposeCloudDwzFilterLeafSize,
      kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);
  registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(registered_cloud_->cloud_));

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(
      registered_cloud_->cloud_, registered_cloud_count_);

  if (registered_cloud_count_ == 0) {
    // initialized_ = true;
    keypose_.pose.pose.position = robot_position_;
    keypose_.pose.covariance[0] = keypose_count_++;
    cur_keypose_node_ind_ =
        keypose_graph_->AddKeyposeNode(keypose_, *(planning_env_));

    pointcloud_downsizer_.Downsize(
        registered_scan_stack_->cloud_, kKeyposeCloudDwzFilterLeafSize,
        kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);

    keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(registered_scan_stack_->cloud_),
                        *(keypose_cloud_->cloud_));
    // keypose_cloud_->Publish();
    registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }

  // ---- [PROF] RegScanCb log (disabled)
  // double prof_cb_ms = std::chrono::duration<double, std::milli>(
  //                         std::chrono::steady_clock::now() - prof_cb_t0)
  //                         .count();
  // RCLCPP_INFO(this->get_logger(),
  //             "[PROF] RegScanCb gap=%.0fms total=%.1fms in_pts=%zu%s",
  //             prof_cb_gap_ms, prof_cb_ms, prof_in_pts,
  //             registered_cloud_count_ == 0 ? " [keypose]" : "");
}

void SensorCoveragePlanner3D::ObjectNodeListCallback(
    const tare_planner::msg::ObjectNodeList::ConstSharedPtr msg) {
    
  if (!initialized_) {
      return;
  }

  if (msg->nodes.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Received empty ObjectNodeList");
      return;
  }

  // Single timestamp check and logging for the entire batch
  rclcpp::Time now = this->now();
  rclcpp::Duration time_diff = now - msg->header.stamp;
  // RCLCPP_INFO(this->get_logger(), 
  //             "Received ObjectNodeList with %zu objects, time_diff=%.2f seconds", 
  //             msg->nodes.size(), time_diff.seconds());
  
  last_object_update_time_ = msg->header.stamp;
  
  // Process all objects in the batch
  int deleted_count = 0;
  int updated_count = 0;
  int skipped_count = 0;
  
  for (const auto& node : msg->nodes) {
    // Convert to ConstSharedPtr for compatibility with existing UpdateObjectNode
    auto node_ptr = std::make_shared<tare_planner::msg::ObjectNode>(node);
    
    // false for deleted objects, true for updated/new objects
    if (node.status == false) {
      for (auto obj_id : node.object_id) {
        object_ids_to_remove_.push_back(obj_id);
        deleted_count++;
      }
      continue;
    }

    // Skip objects with empty cloud
    if (node.cloud.data.empty()) {
      skipped_count++;
      continue;
    }

    // Update representation
    representation_->UpdateObjectNode(node_ptr);
    representation_->GetLatestObjectNodeIndicesMutable().insert(node.object_id[0]);
    updated_count++;
  }
  
  RCLCPP_DEBUG(this->get_logger(), 
               "Batch processed: %d updated, %d deleted, %d skipped",
               updated_count, deleted_count, skipped_count);
}

void SensorCoveragePlanner3D::DoorCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr door_cloud_msg) {
  if (!initialized_)
  {
    return;
  }
  if (door_cloud_msg->data.empty()) {
    return;
  }
  // reset the adjacency_matrix
  adjacency_matrix.setZero();
  door_cloud_->points.clear();
  door_cloud_vis_->cloud_->points.clear();
      std::set<int>
          room_ids;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZRGBL>());
  pcl::fromROSMsg(*door_cloud_msg, *door_cloud_tmp);

  // only keep the door cloud that are not in collision
  int room_id_0, room_id_1;
  for (auto &point : door_cloud_tmp->points)
  {
    room_ids.insert(point.r);
    room_ids.insert(point.g);
    point.z = robot_position_.z; // set the z coordinate to the robot position z
    if (!planning_env_->DoorInCollision(point.x, point.y, point.z))
    {
      door_cloud_->points.push_back(point);
      pcl::PointXYZRGBL door_point_vis;
      door_point_vis.x = point.x;
      door_point_vis.y = point.y;
      door_point_vis.z = point.z;

      Eigen::Vector3d color_1 = misc_utils_ns::idToColor(point.r);
      Eigen::Vector3d color_2 = misc_utils_ns::idToColor(point.g);
      // find the average color of the two rooms
      door_point_vis.b = (color_1.x() + color_2.x()) / 2 * 255;
      door_point_vis.g = (color_1.y() + color_2.y()) / 2 * 255;
      door_point_vis.r = (color_1.z() + color_2.z()) / 2 * 255;
      door_point_vis.label = point.label; // label
      door_cloud_vis_->cloud_->points.push_back(door_point_vis);
    }
  }

  door_cloud_vis_->Publish();

  for (auto &point : door_cloud_->points)
  {
      room_id_0 = point.r;
      room_id_1 = point.g;
      adjacency_matrix(room_id_0 - 1, room_id_1 - 1) = 1;
      adjacency_matrix(room_id_1 - 1, room_id_0 - 1) = 1;
  }
}

void SensorCoveragePlanner3D::RoomNodeListCallback(
    const tare_planner::msg::RoomNodeList::ConstSharedPtr room_node_list_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (room_node_list_msg->nodes.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Room node list is empty");
    return;
  }
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    id_to_room_node.second.SetAlive(false);
  }
  for (const auto &room_node_msg : room_node_list_msg->nodes) {
    // 如果 room node 不存在，使用 AddRoomNode 创建新的
    if (!representation_->HasRoomNode(room_node_msg.id)) {
      representation_->AddRoomNode(room_node_msg);
      ROOM_DBG("[room_dbg] CREATE id=%d show_id=%d centroid=(%.2f,%.2f,%.2f) area=%.2f neighbors=%zu",
               room_node_msg.id, room_node_msg.show_id, room_node_msg.centroid.x,
               room_node_msg.centroid.y, room_node_msg.centroid.z, room_node_msg.area,
               room_node_msg.neighbors.size());
    } else {
      representation_->GetRoomNode(room_node_msg.id).UpdateRoomNode(room_node_msg);
    }
  }
  for (auto it = representation_->GetRoomNodesMapMutable().begin(); it != representation_->GetRoomNodesMapMutable().end();)
  {
    if (!it->second.IsAlive())
    {
      ROOM_DBG("[room_dbg] DEATH id=%d labeled=%d label='%s' objects=%zu",
               it->first, it->second.IsLabeled() ? 1 : 0,
               it->second.GetRoomLabel().c_str(),
               it->second.GetObjectIndices().size());
      it = representation_->GetRoomNodesMapMutable().erase(it);
    }
    else
    {
      ++it;
    }
  }
  // sysnav anchor validation (ported from rsb): a labeled room whose anchor
  // point no longer samples its own id in room_mask_ has been re-segmented;
  // clear its labels so it is re-queried from scratch.
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    auto &room_node = id_to_room_node.second;
    if (!room_node.IsLabeled())
    {
      continue;
    }
    Eigen::Vector3f anchor_point(room_node.anchor_point_.x, room_node.anchor_point_.y, room_node.anchor_point_.z);
    Eigen::Vector3i anchor_point_voxel = misc_utils_ns::point_to_voxel(anchor_point, shift_, 1.0 / room_resolution_);
    if (anchor_point_voxel.x() < 0 || anchor_point_voxel.x() >= room_mask_.rows ||
        anchor_point_voxel.y() < 0 || anchor_point_voxel.y() >= room_mask_.cols)
    {
      RCLCPP_ERROR(this->get_logger(), "Anchor point of room %d is out of room mask bounds", room_id);
      room_node.ClearRoomLabels();
      continue;
    }
    int room_id_in_mask = room_mask_.at<int>(anchor_point_voxel.x(), anchor_point_voxel.y());
    if (room_id_in_mask != room_id)
    {
      RCLCPP_ERROR(this->get_logger(), "Anchor point of room %d is not in the room, removing labels", room_id);
      room_node.ClearRoomLabels();
    }
  }
}

void SensorCoveragePlanner3D::RoomMaskCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr room_mask_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (room_mask_msg->data.empty())
  {
    return;
  }
  // store the current room mask to room_mask_prev
  // convert the room mask to a cv::Mat
  cv::Mat room_mask(room_mask_msg->height, room_mask_msg->width, CV_32S,
                    const_cast<uint8_t *>(room_mask_msg->data.data()));
  // resize the room mask to the room voxel dimension
  cv::resize(room_mask, room_mask, cv::Size(room_voxel_dimension_.x(), room_voxel_dimension_.y()),
             0, 0, cv::INTER_NEAREST);
  // update the room mask
  room_mask.copyTo(room_mask_);

  // Diagnostic: cadence + extent of the mask raster, so its update timing can be
  // correlated against the room-list/centroid updates and answer arrivals (the
  // two sides arrive on independent topics and can be a cycle out of step).
  ROOM_DBG("[room_dbg] MASK_UPDATE dims=%dx%d nonzero_cells=%d",
           room_mask_.rows, room_mask_.cols, cv::countNonZero(room_mask_));

  viewpoint_manager_->SetRoomMask(room_mask_);
  grid_world_->SetRoomMask(room_mask_);
  if (initialized_ && representation_)
  {
    representation_->UpdateViewpointRoomIdsFromMask(room_mask_, shift_, room_resolution_);
  }
}

void SensorCoveragePlanner3D::RoomTypeCallback(
    const tare_planner::msg::RoomType::ConstSharedPtr room_type_msg)
{
  // sysnav answer apply (ported from rsb): re-resolve the room by sampling the
  // mask at the query's anchor point, then accumulate a coverage-weighted vote.
  Eigen::Vector3f anchor_point(
      room_type_msg->anchor_point.x, room_type_msg->anchor_point.y,
      room_type_msg->anchor_point.z);
  Eigen::Vector3i anchor_point_voxel = misc_utils_ns::point_to_voxel(
      anchor_point, shift_, 1.0 / room_resolution_);
  if (anchor_point_voxel.x() < 0 || anchor_point_voxel.x() >= room_mask_.rows ||
      anchor_point_voxel.y() < 0 || anchor_point_voxel.y() >= room_mask_.cols)
  {
    RCLCPP_ERROR(this->get_logger(), "Anchor point is out of room mask bounds");
    return;
  }
  int room_id = room_mask_.at<int>(anchor_point_voxel.x(),
                                   anchor_point_voxel.y());
  if (!representation_->HasRoomNode(room_id))
  {
    RCLCPP_ERROR(this->get_logger(), "Room id %d is out of bounds",
                 room_id);
    return;
  }
  std::string room_type = room_type_msg->room_type;
  std::string current_room_type_ = representation_->GetRoomNode(room_id).GetRoomLabel();
  RCLCPP_INFO(this->get_logger(), "Room id: %d, Room type: %s, Current room type: %s",
              room_id, room_type.c_str(), current_room_type_.c_str());
  representation_->GetRoomNode(room_id).GetLabelsMutable()[room_type] += room_type_msg->voxel_num; // accumulate the voxel number for each label
  representation_->GetRoomNode(room_id).SetIsLabeled(true); // mark the room as labeled
  std::string current_room_type_new_ = representation_->GetRoomNode(room_id).GetRoomLabel();
  ROOM_DBG("[room_dbg] ANSWER_APPLY msg_room_id=%d resolved_id=%d (by anchor) type='%s' label='%s'->'%s'",
           room_type_msg->room_id, room_id, room_type.c_str(),
           current_room_type_.c_str(), current_room_type_new_.c_str());
  if (current_room_type_new_ != current_room_type_)
  {
    for (auto object_id : representation_->GetRoomNode(room_id).GetObjectIndices())
    {
      if (representation_->HasObjectNode(object_id)) {
        representation_->GetObjectNodeRep(object_id).SetIsConsidered(false);
      }
    }
  }
}

void SensorCoveragePlanner3D::KeyboardInputCallback(const std_msgs::msg::String::ConstSharedPtr keyboard_input_msg)
{
  if (keyboard_input_msg->data == "reset")
  {
    tmp_flag_ = true;
  }
  else if (keyboard_input_msg->data == sempath_cfg_.keyword)
  {
    ExportSemPathSnapshot("manual");
  }
}

void SensorCoveragePlanner3D::SetCurrentRoomId()
{
  enter_wrong_room_ = false;
  viewpoint_manager_->SetEnterWrongRoom(enter_wrong_room_);
  // find the current room id
  Eigen::Vector3f robot_position_tmp(robot_position_.x, robot_position_.y, robot_position_.z);
  Eigen::Vector3f robot_position_old_tmp(robot_position_old_.x, robot_position_old_.y, robot_position_old_.z);
  Eigen::Vector3i robot_position_voxel_new = misc_utils_ns::point_to_voxel(
      robot_position_tmp, shift_, 1.0 / room_resolution_);
  Eigen::Vector3i robot_position_voxel_old = misc_utils_ns::point_to_voxel(
      robot_position_old_tmp, shift_, 1.0 / room_resolution_);
  // Bulletproofing: room_mask_ comes from room_segmentation and is empty until the
  // first mask arrives; the robot voxel can also fall outside it. An unguarded
  // cv::Mat::at() on an empty mat / out-of-range index throws and would kill the node
  // (and with it the scene-graph JSON). Skip the room-id update this cycle if we can't
  // index safely -- the rest of execute()'s scene-graph work still runs.
  if (room_mask_.empty() ||
      robot_position_voxel_new.x() < 0 || robot_position_voxel_new.x() >= room_mask_.rows ||
      robot_position_voxel_new.y() < 0 || robot_position_voxel_new.y() >= room_mask_.cols ||
      robot_position_voxel_old.x() < 0 || robot_position_voxel_old.x() >= room_mask_.rows ||
      robot_position_voxel_old.y() < 0 || robot_position_voxel_old.y() >= room_mask_.cols)
  {
    return;
  }
  int room_id_tmp_ = room_mask_.at<int>(robot_position_voxel_new.x(),
                                        robot_position_voxel_new.y());

  if (room_id_tmp_ <= 0)
  {
    // RCLCPP_INFO(this->get_logger(), "Robot is not in any room");
    return; // maybe just across a door, no need to update the room id
  }

  if ((room_id_tmp_ == current_room_id_) || current_room_id_ == -1)
  {
    // If the robot is still in the same room, no need to update the room id
    current_room_id_ = room_id_tmp_;
    robot_position_old_ = robot_position_;
    room_mask_old_ = room_mask_.clone();
    viewpoint_manager_->SetCurrentRoomId(current_room_id_);
    grid_world_->SetCurrentRoomId(current_room_id_);
    return;
  }
  // 意外走进新房间分为两种大情况：
  // 1. 机器人在走入该房间前就已经知道该房间是一个新房间（即虽然room_id_tmp_!= current_room_id_，但在room_mask_old_上该位置的值也与current_room_id_不同），对于这种情况，一定要绕回原房间
  // 2. 机器人在走入该房间前并不知道该房间是一个新房间（即room_id_tmp_ != current_room_id_，但在room_mask_old_上该位置的值与current_room_id_相同），对于这种情况，需要分类讨论
  //    2.1 因为merge导致的房间id变化，这种情况可以立即更新房间id
  //    2.2 因为split导致的房间id变化，这种情况需要设置ask_vlm_change_room_为true，等待VLM的确认
  // 新旧的第一个下标表示mask新旧，第二个下标表示robot position新旧
  if (room_id_tmp_ != current_room_id_)
  {
    int room_label_old_new = room_mask_old_.at<int>(robot_position_voxel_new.x(),
                                                    robot_position_voxel_new.y());
    if (room_label_old_new != current_room_id_)
    {
      // 机器人在走入该房间前就已经知道该房间是一个新房间，一定要绕回原房间。
      // RCLCPP_INFO(this->get_logger(), "Robot enters a wrong room %d, but already knows it is a new room before entering.", room_id_tmp_);
      enter_wrong_room_ = true;
      viewpoint_manager_->SetEnterWrongRoom(enter_wrong_room_);
      return;
    }
    else
    {
      // 机器人在走入该房间前并不知道该房间是一个新房间
      cv::Mat room_mask_new_new = (room_mask_ == room_id_tmp_);
      cv::Mat room_mask_old_old = (room_mask_old_ == current_room_id_);
      cv::Mat room_mask_and;
      cv::bitwise_and(room_mask_new_new, room_mask_old_old, room_mask_and);
      int num_1 = cv::countNonZero(room_mask_and);
      int num_2 = cv::countNonZero(room_mask_old_old);
      // 1. 被merge（新id在新mask上的mask and 旧id在旧mask上的mask 占据 旧id在旧mask上的mask的绝大部分） 
      // 2. 被分割出去（新id在新mask上的mask and 旧id在旧mask上的mask 占据 旧id在旧mask上的mask的一小部分部分）
      if (num_1 > num_2 * 0.8)
      {
        // 1. 被merge
        // RCLCPP_INFO(this->get_logger(), "Room %d is merged into room %d", current_room_id_, room_id_tmp_);
        current_room_id_ = room_id_tmp_;
        robot_position_old_ = robot_position_;
        room_mask_old_ = room_mask_.clone();
        viewpoint_manager_->SetCurrentRoomId(current_room_id_);
        grid_world_->SetCurrentRoomId(current_room_id_);
        // RCLCPP_INFO(this->get_logger(), "Current room id: %d", current_room_id_);
        return;
      }
      else
      {
        // 2. 被分割出去
        // RCLCPP_INFO(this->get_logger(), "Room %d is split into room %d", current_room_id_, room_id_tmp_);
        current_room_id_ = room_id_tmp_;
        robot_position_old_ = robot_position_;
        room_mask_old_ = room_mask_.clone();
        viewpoint_manager_->SetCurrentRoomId(current_room_id_);
        grid_world_->SetCurrentRoomId(current_room_id_);
        return;
      }
    }
  }
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph() {
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  keypose_graph_->GetMarker(keypose_graph_node_marker_->marker_,
                            keypose_graph_edge_marker_->marker_);
  // keypose_graph_node_marker_->Publish();
  keypose_graph_edge_marker_->Publish();
  keypose_graph_vis_cloud_->cloud_->clear();
  keypose_graph_->CheckLocalCollision(robot_position_, viewpoint_manager_);
  keypose_graph_->CheckConnectivity(robot_position_);
  keypose_graph_->GetVisualizationCloud(keypose_graph_vis_cloud_->cloud_);
  keypose_graph_vis_cloud_->Publish();

  update_keypose_graph_timer.Stop(false);
}

int SensorCoveragePlanner3D::UpdateViewPoints() {
  misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
  collision_cloud_timer.Start();
  collision_cloud_->cloud_ = planning_env_->GetCollisionCloud();
  collision_cloud_timer.Stop(false);

  misc_utils_ns::Timer viewpoint_manager_update_timer(
      "update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  viewpoint_manager_->CheckViewPointCollision(collision_cloud_->cloud_);
  viewpoint_manager_->CheckViewPointRoomBoundaryCollision();
  viewpoint_manager_->CheckViewPointLineOfSight();
  viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = viewpoint_manager_->GetViewPointCandidate();

  UpdateVisitedPositions();
  viewpoint_manager_->UpdateViewPointVisited(visited_positions_);
  viewpoint_manager_->UpdateViewPointVisited(grid_world_); // only used for multi-robot exploration

  // For visualization
  collision_cloud_->Publish();
  // collision_grid_cloud_->Publish();
  viewpoint_manager_->GetCollisionViewPointVisCloud(
      viewpoint_in_collision_cloud_->cloud_);
  viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage() {
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(
      planning_env_->GetDiffCloud());
  viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
      planning_env_->GetStackedCloud());
  // Update robot coverage
  robot_viewpoint_.ResetCoverage();
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position = robot_position_;
  robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(false);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
      planning_env_->GetCollisionCloud();
  for (const auto &point : cloud->points) {
    if (viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(robot_position_.x, robot_position_.y,
                            robot_position_.z))) {
      robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(
    int &uncovered_point_num, int &uncovered_frontier_point_num) {
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  planning_env_->UpdateCoveredArea(robot_viewpoint_, viewpoint_manager_);

  update_coverage_area_timer.Stop(false);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  planning_env_->GetUncoveredArea(viewpoint_manager_, uncovered_point_num,
                                  uncovered_frontier_point_num);

  get_uncovered_area_timer.Stop(false);
  planning_env_->PublishUncoveredCloud();
  planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions() {
  Eigen::Vector3d robot_current_position(robot_position_.x, robot_position_.y,
                                         robot_position_.z);
  bool existing = false;
  for (int i = 0; i < visited_positions_.size(); i++) {
    // TODO: parameterize this
    if ((robot_current_position - visited_positions_[i]).norm() < 1) {
      existing = true;
      break;
    }
  }
  if (!existing) {
    visited_positions_.push_back(robot_current_position);
  }
}

void SensorCoveragePlanner3D::UpdateObjectVisibility()
{
  std::vector<int> visible_object_ids = {};
  for (auto &object_id : representation_->GetLatestObjectNodeIndicesMutable())
  {
    if (!representation_->HasObjectNode(object_id)) {
      RCLCPP_WARN(this->get_logger(), "Object with id %d does not exist in representation, skip", object_id);
      continue;
    }
    auto &object = representation_->GetObjectNodeRep(object_id);
    geometry_msgs::msg::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);

    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);
    for (auto &viewpoint : representation_->GetViewPointRepsMutable())
    {
      if (viewpoint.HasObjectIndex(object_id))
      {
        object.AddVisibleViewpoint(viewpoint.GetId());
        continue;
      }
      // if timestamp is very close, connect
      // if the viewpoint id is in the object visible viewpoint list, connect
      // if ((std::abs((viewpoint.GetTimestamp() - object.GetTimestamp()).seconds()) < 0.5) || std::find(object.GetVisibleViewpointIndices().begin(),
      //                                                                                         object.GetVisibleViewpointIndices().end(),
      //                                                                                         viewpoint.GetId()) != object.GetVisibleViewpointIndices().end())
      if (std::find(object.GetVisibleViewpointIndices().begin(), object.GetVisibleViewpointIndices().end(), viewpoint.GetId()) != object.GetVisibleViewpointIndices().end())
      {
        viewpoint.AddObjectIndex(object_id);
        viewpoint.AddDirectObjectIndex(object_id);
        object.AddVisibleViewpoint(viewpoint.GetId());
        visible_object_ids.push_back(object_id);
        // RCLCPP_INFO(this->get_logger(),
        //             "Object ID %d (%s) is detected at viewpoint %d (new object)",
        //             object.GetObjectId(),
        //             object.GetLabel().c_str(),
        //             viewpoint.GetId());
        continue;
      }
      else
      {
        // do ray-casting to check visibility
        if (viewpoint.HasObjectIndex(object_id))
        {
          visible_object_ids.push_back(object_id);
          continue;
        }
        Eigen::Vector3d current_viewpoint_pos(viewpoint.GetPosition().x,
                                              viewpoint.GetPosition().y,
                                              viewpoint.GetPosition().z + 0.265); // make this to the height of the robot camera
        Eigen::Vector3i curr_viewpoint_voxel = planning_env_->Pos2Sub(current_viewpoint_pos);

        if ((current_viewpoint_pos - object_pos).norm() > rep_sensor_range)
        {
          continue;
        }
        bool is_visible = false; // Initialize visibility flag
        for (auto &pt : voxels)
        {
          is_visible = CheckRayVisibilityInOccupancyGrid(curr_viewpoint_voxel, pt);
          if (is_visible)
          {
            break; // if any voxel is visible, we consider the object visible
          }
        }
        if (is_visible)
        {
          viewpoint.AddObjectIndex(object_id);
          object.AddVisibleViewpoint(viewpoint.GetId());
          visible_object_ids.push_back(object_id);
          // RCLCPP_INFO(this->get_logger(),
          //             "Object ID %d (%s) is visible from viewpoint %d",
          //             object.GetObjectId(),
          //             object.GetLabel().c_str(),
          //             viewpoint.GetId());
        }
        // else
        // {
        //   RCLCPP_INFO(this->get_logger(),
        //               "Object ID %d (%s) is NOT visible from viewpoint %d",
        //               object.GetObjectId(),
        //               object.GetLabel().c_str(),
        //               viewpoint.GetId());
        // }
      }
    }

    // ---------- set the obj-room relation-----------
    Eigen::Vector3f object_pos_f(obj_position.x,
                                 obj_position.y,
                                 obj_position.z);
    Eigen::Vector3i object_pos_voxel = misc_utils_ns::point_to_voxel(
        object_pos_f, shift_, 1.0 / room_resolution_);
    if (object_pos_voxel.x() >= 0 && object_pos_voxel.x() < room_mask_.cols &&
        object_pos_voxel.y() >= 0 && object_pos_voxel.y() < room_mask_.rows)
    {
      int object_room_id = room_mask_.at<int>(object_pos_voxel.x(),
                                              object_pos_voxel.y());
      if (object_room_id == 0)
      {
        // // if the object is visible from any viewpoint, use the room id of that viewpoint
        // if (!object.GetVisibleViewpointIndices().empty())
        // {
        //   int vp_id = *object.GetVisibleViewpointIndices().begin();
        //   object_room_id = representation_->GetViewPointRepNode(vp_id).GetRoomId();
        // }
        // else
        {
          // if the object is not visible from any viewpoint, we need to dilate the object position by 2 voxels to get a non-zero room id
          int dilation_size = 2;
          bool found = false;
          for (int dx = -dilation_size; dx <= dilation_size && !found; dx++)
          {
            for (int dy = -dilation_size; dy <= dilation_size; dy++)
            {
              int nx = object_pos_voxel.x() + dx;
              int ny = object_pos_voxel.y() + dy;
              if (nx >= 0 && nx < room_mask_.cols &&
                  ny >= 0 && ny < room_mask_.rows)
              {
                int neighbor_room_id = room_mask_.at<int>(nx, ny);
                if (neighbor_room_id > 0)
                {
                  object_room_id = neighbor_room_id;
                  found = true;
                  break;  
                }
              }
            }
          }
        }
      }
      representation_->SetObjectRoomRelation(object_id, object_room_id);
    }
    // ---------- set the obj-room relation-----------
  }

  // check the current position visibility
  obj_score_ = 0.0;
  Eigen::Vector3d current_pos(robot_position_.x, robot_position_.y, robot_position_.z + 0.265);
  Eigen::Vector3i current_voxel = planning_env_->Pos2Sub(current_pos);
  // print the latest_object_node_indices size
  // RCLCPP_ERROR(this->get_logger(),
  //                   "Latest object node rep map size: %zu",
  //                   representation_->GetLatestObjectNodeIndices().size());
  for (auto &object_id : representation_->GetLatestObjectNodeIndicesMutable())
  {
    if (!representation_->HasObjectNode(object_id)) {
      RCLCPP_WARN(this->get_logger(), "Object with id %d does not exist in representation, skip", object_id);
      continue;
    }
    auto &object = representation_->GetObjectNodeRep(object_id);
    geometry_msgs::msg::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);
    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);

    if ((current_pos - object_pos).norm() > rep_sensor_range)
    {
      continue;
    }
    bool is_visible = false; // Initialize visibility flag
    for (auto &pt : voxels)
    {
      is_visible = CheckRayVisibilityInOccupancyGrid(current_voxel, pt);
      if (is_visible)
      {
        break; // if any voxel is visible, we consider the object visible
      }
    }
    if (is_visible && object.GetVisibleViewpointIndices().empty())
    {
      obj_score_ += 1.0;
    }
    else if (is_visible && !object.GetVisibleViewpointIndices().empty())
    {
      obj_score_ += 0.2;
    }
  }
      
  misc_utils_ns::UniquifyIntVector(visible_object_ids);
  // remove the visible objects from the latest_object_node_indices
  for (int &object_id : visible_object_ids)
  {
    representation_->GetLatestObjectNodeIndicesMutable().erase(object_id);
  }
}  

void SensorCoveragePlanner3D::UpdateViewpointObjectVisibility()
{
  representation_ns::ViewPointRep &current_viewpoint = representation_->GetViewPointRepNode(curr_viewpoint_rep_node_ind);
  Eigen::Vector3d current_viewpoint_pos(current_viewpoint.GetPosition().x,
                                        current_viewpoint.GetPosition().y,
                                        current_viewpoint.GetPosition().z + 0.265); // make this to the height of the robot camera
  Eigen::Vector3i curr_viewpoint_voxel = planning_env_->Pos2Sub(current_viewpoint_pos);
  for (auto &id_object_pair : representation_->GetObjectNodeRepMapMutable())
  {
    const int &object_id = id_object_pair.first;
    auto &object = id_object_pair.second;
    // if the object is already in the viewpoint's visibility list, skip it
    if (current_viewpoint.HasObjectIndex(object_id))
    {
      continue;
    }
    geometry_msgs::msg::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);

    if ((current_viewpoint_pos - object_pos).norm() > rep_sensor_range)
    {
      continue;
    }
    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);
    bool is_visible = false; // Initialize visibility flag
    for (auto &pt : voxels)
    {
      is_visible = CheckRayVisibilityInOccupancyGrid(curr_viewpoint_voxel, pt);
      if (is_visible)
      {
        break; // if any voxel is visible, we consider the object visible
      }
    }
    if (is_visible)
    {
      current_viewpoint.AddObjectIndex(object.GetObjectId());
      object.AddVisibleViewpoint(current_viewpoint.GetId());
      // RCLCPP_INFO(this->get_logger(),
      //             "Object ID %d (%s) is visible from viewpoint %d",
      //             object.GetObjectId(),
      //             object.GetLabel().c_str(),
      //             current_viewpoint.GetId());
    }
    // else
    // {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Object ID %d (%s) is NOT visible from viewpoint %d",
    //               object.GetObjectId(),
    //               object.GetLabel().c_str(),
    //               current_viewpoint.GetId());
    // }
  }  
}

// Helper function to check visibility using occupancy grid
bool SensorCoveragePlanner3D::CheckRayVisibilityInOccupancyGrid(const Eigen::Vector3i& start_pos, 
                                                                const Eigen::Vector3i& end_pos) {

  return planning_env_->CheckLineOfSightInOccupancyGrid(start_pos, end_pos);
}

bool SensorCoveragePlanner3D::InRange(const Eigen::Vector3i& voxel_index) const {
  return planning_env_->InRange(voxel_index);
}

std::vector<Eigen::Vector3i> SensorCoveragePlanner3D::Convert2Voxels(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<Eigen::Vector3i> voxel_vector;
    for (const auto& point : cloud->points) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        Eigen::Vector3i voxel_index = planning_env_->Pos2Sub(pos);
        bool is_valid = InRange(voxel_index);
        if (is_valid) {
            voxel_vector.push_back(voxel_index);
        } else {
            RCLCPP_WARN(this->get_logger(), "Voxel index out of range: (%d, %d, %d)", 
                        voxel_index.x(), voxel_index.y(), voxel_index.z());
        }
    }
    // Remove duplicates
    std::sort(voxel_vector.begin(), voxel_vector.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    });
    voxel_vector.erase(std::unique(voxel_vector.begin(), voxel_vector.end()), voxel_vector.end());
    
    return voxel_vector;
}

void SensorCoveragePlanner3D::CreateVisibilityMarkers() {
    if (!initialized_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = kWorldFrameID;
    delete_marker.header.stamp = this->now();
    delete_marker.ns = "visibility_lines";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int unique_marker_id = 1;
    
    for (const auto& viewpoint : representation_->GetViewPointReps()) {
        Eigen::Vector3d viewpoint_pos(
            viewpoint.GetPosition().x,
            viewpoint.GetPosition().y,
            viewpoint.GetPosition().z
        );

        const auto& visible_object_indices = viewpoint.GetObjectIndices();
        for (int object_index : visible_object_indices) {
            bool has_object = representation_->HasObjectNode(object_index);
            if (!has_object) {
                RCLCPP_WARN(this->get_logger(),
                            "Object ID %d not found in object_node_rep_map", object_index);
                continue;
            }

            const auto& object_node = representation_->GetObjectNodeRep(object_index);
            geometry_msgs::msg::Point obj_position = object_node.GetPosition();
            Eigen::Vector3d object_pos(obj_position.x, obj_position.y, obj_position.z);

            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = kWorldFrameID;
            line_marker.header.stamp = this->now();
            line_marker.ns = "visibility_lines";
            
            line_marker.id = ++unique_marker_id; // Unique ID for each line marker

            line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set the duration of the line marker
            // line_marker.lifetime = rclcpp::Duration::from_seconds(10.0);

            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = viewpoint_pos.x();
            start_point.y = viewpoint_pos.y();
            start_point.z = viewpoint_pos.z();
            end_point.x = object_pos.x();
            end_point.y = object_pos.y();
            end_point.z = object_pos.z();

            line_marker.points.push_back(start_point);
            line_marker.points.push_back(end_point);

            line_marker.scale.x = 0.08; // Thinner lines since there will be many
            if (viewpoint.HasDirectObjectIndex(object_index))
            {
                line_marker.color.r = 0.0;
                line_marker.color.g = 0.0;
                line_marker.color.b = 1.0; // Blue for direct visibility
            }
            else
            {
              line_marker.color.r = 0.0;
              line_marker.color.g = 1.0;
              line_marker.color.b = 0.0;
            }
            line_marker.color.a = 0.6; // Slightly transparent

            marker_array.markers.push_back(line_marker);

            RCLCPP_DEBUG(this->get_logger(),
                "Created visibility line from viewpoint to object %d (%s)",
                object_node.GetObjectId(),
                object_node.GetLabel().c_str()
            );
        }
    }

    // Publish the marker array
    if (!marker_array.markers.empty()) {
        object_visibility_marker_pub_->publish(marker_array);
        // RCLCPP_INFO(this->get_logger(),
        //             "Published %zu visibility markers from all viewpoints", 
        //             marker_array.markers.size() - 1); // -1 for DELETEALL marker
    } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "No visibility connections to publish");
    }
}

void SensorCoveragePlanner3D::PublishViewpointRoomIdMarkers() {
    if (!initialized_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = kWorldFrameID;
    delete_marker.header.stamp = this->now();
    delete_marker.ns = "viewpoint_room_ids";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // int marker_id = 1;
    
    for (const auto& viewpoint : representation_->GetViewPointReps()) {
        // Create text marker for room ID
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = kWorldFrameID;
        text_marker.header.stamp = this->now();
        text_marker.ns = "viewpoint_room_ids";
        text_marker.id = viewpoint.GetId() + 1; // Use viewpoint ID as marker ID
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position the text slightly above the viewpoint
        text_marker.pose.position.x = viewpoint.GetPosition().x;
        text_marker.pose.position.y = viewpoint.GetPosition().y;
        text_marker.pose.position.z = viewpoint.GetPosition().z + 0.5; // 0.5m above viewpoint
        
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        
        // Set text content
        text_marker.text = "V" + std::to_string(viewpoint.GetId()) + "R" + std::to_string(viewpoint.GetRoomId());
        
        // Set size and color
        text_marker.scale.z = 0.4; // Text height
        // Set color to red
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0; // Red text
        text_marker.color.a = 1.0;
        
        // Set lifetime
        // text_marker.lifetime = rclcpp::Duration::from_seconds(10.0);
        
        marker_array.markers.push_back(text_marker);
  
    }

    // Publish the marker array
    if (!marker_array.markers.empty()) {
        viewpoint_room_id_marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(this->get_logger(),
                     "Published %zu viewpoint room ID markers", 
                     (marker_array.markers.size() - 1) / 2); // -1 for DELETEALL, /2 for text+sphere pairs
    }
}

// void SensorCoveragePlanner3D::PublishViewpointObjectVisibility() {
//     std_msgs::msg::String visibility_msg;
//     std::string json_data = "[";  // array of all viewpoints

//     std::unordered_map<int, std::vector<const representation_ns::ObjectNodeRep*>> vp_to_objects;

//     for (const auto& obj : object_node_reps_) {
//         for (const auto& vp : obj.GetVisibleViewpoints()) {
//             vp_to_objects[vp].push_back(&obj);
//         }
//     }
//     bool first_viewpoint = true;
//     for (const auto& [vp_id, obj_list] : vp_to_objects) {
//         if (!first_viewpoint) json_data += ",";
//         json_data += "{";
//         json_data += "\"viewpoint_id\":" + std::to_string(vp_id) + ",";
//         json_data += "\"visible_objects\":[";
        
//         bool first_obj = true;
//         for (const auto* obj_ptr : obj_list) {
//             if (!first_obj) json_data += ",";
//             json_data += "{";
//             json_data += "\"object_id\":" + std::to_string(obj_ptr->GetObjectId()) + ",";
//             json_data += "\"label\":\"" + obj_ptr->GetLabel() + "\",";
//             json_data += "\"total_viewpoints\":" + std::to_string(obj_ptr->GetVisibleViewpoints().size());

//             auto cloud_msg = obj_ptr->GetCloud();
//             pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//             pcl::fromROSMsg(cloud_msg, *obj_cloud);
//             json_data += ",\"point_count\":" + std::to_string(obj_cloud->points.size());

//             json_data += "}";
//             first_obj = false;
//         }

//         json_data += "]}";
//         first_viewpoint = false;
//     }

//     json_data += "]";

//     visibility_msg.data = json_data;
//     viewpoint_visibility_pub_->publish(visibility_msg);
    
//     RCLCPP_INFO(this->get_logger(), "Published all viewpoint-object visibility");
// }


void SensorCoveragePlanner3D::UpdateGlobalRepresentation() {
  bool viewpoint_rollover = viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  if (!grid_world_->Initialized() || viewpoint_rollover) {
    grid_world_->UpdateNeighborCells(robot_position_);
  }

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->GetVisualizationPointCloud(point_cloud_manager_neighbor_cloud_->cloud_);
  point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::msg::PointStamped
      pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = this->now();
  pointcloud_manager_neighbor_cells_origin_point.point.x =
      pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y =
      pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z =
      pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_->publish(
      pointcloud_manager_neighbor_cells_origin_point);

  planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(
      keypose_cloud_->cloud_);

  int closest_node_ind = keypose_graph_->GetClosestNodeInd(robot_position_);
  geometry_msgs::msg::Point closest_node_position =
      keypose_graph_->GetClosestNodePosition(robot_position_);
  grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  grid_world_->UpdateRobotPosition(robot_position_);
  if (!grid_world_->HomeSet()) {
    grid_world_->SetHomePosition(initial_position_);
  }

  // // Representation
  // // add keypose_cloud_ to point_cloud_all_
  // *(point_cloud_new_->cloud_) = *(keypose_cloud_->cloud_);
  // int size;
  // planning_env_->GetObsVoxelNumber(size);
  // RCLCPP_INFO(
  //     this->get_logger(), "Number of occupied voxels in the occupancy grid: %d",
  //     size);
  // if (size - voxel_num_ > rep_threshold_)
  // {
  //   voxel_num_ = size;
  //   // add the robot position to viewpoint_rep_vis_cloud_
  //   pcl::PointXYZI robot_point;
  //   robot_point.x = robot_position_.x;
  //   robot_point.y = robot_position_.y;
  //   robot_point.z = robot_position_.z;
  //   robot_point.intensity = 1.0; // Set intensity to 1.0 for visibility
  //   viewpoint_rep_vis_cloud_->cloud_->points.push_back(robot_point);
  //   viewpoint_rep_vis_cloud_->Publish();
  // }
  // current_obs_voxel_inds_.clear();
  // planning_env_->GetUpdatedVoxelInds(current_obs_voxel_inds_);
}

void SensorCoveragePlanner3D::GlobalPlanning() {
  grid_world_->UpdateCellStatus(viewpoint_manager_);
  grid_world_->UpdateCellKeyposeGraphNodes(keypose_graph_);
  grid_world_->AddPathsInBetweenCells(viewpoint_manager_, keypose_graph_);
}

void SensorCoveragePlanner3D::PublishFreespaceCloud() {
  viewpoint_manager_->GetFreespaceCloud(freespace_cloud_->cloud_);
  double current_time = this->now().seconds();
  double delta_time = current_time - start_time_;
  if (delta_time > 20)
  {
    freespace_cloud_->Publish();
  }
}

void SensorCoveragePlanner3D::execute() {
  if (!initialized_) {
    start_time_ = this->now().seconds();
    if(start_time_ == 0.0){
      RCLCPP_ERROR(this->get_logger(), "Start time is zero, time source (use_time_time) not set correctly. Exiting...");
      exit(1);
    }
    initialized_ = true;
    return;
  }

  ProcessObjectNodes();

  if (tmp_flag_)
  {
    tmp_flag_ = false;
    viewpoint_manager_->ResetViewPointCoverage();
    RCLCPP_ERROR(this->get_logger(), "Reset the viewpoint coverage");
  }

  if (keypose_cloud_update_) {
    keypose_cloud_update_ = false;
    UpdateRoomLabel();
    SetCurrentRoomId();

    UpdateObjectVisibility();

    // Update grid world
    UpdateGlobalRepresentation();
    UpdateViewpointRep();
    // Draw the current viewpoint representation's room index
    PublishViewpointRoomIdMarkers();

    if (add_viewpoint_rep_)
    {
      UpdateViewpointObjectVisibility();
      add_viewpoint_rep_ = false;
    }

    // Update the visibility markers after updating the object visibility
    CreateVisibilityMarkers();
    
    int viewpoint_candidate_count = UpdateViewPoints();
    // SCENE-GRAPH BULLETPROOFING: a zero candidate-viewpoint count is a *motion-
    // planning* condition, not a scene-graph one. This used to `return` here, which
    // also skipped the keypose-graph / room-finishing work below -- i.e.
    // froze the scene graph (and thus the exported JSON). Instead, keep building the
    // scene graph every cycle and only skip the parts that genuinely need candidate
    // viewpoints (the TSP path planning + coverage update, which only steer the robot
    // and never touch the scene graph). UpdateKeyposeGraph() below does not
    // iterate the candidate set, so they are safe to run with zero candidates.
    bool have_viewpoints = (viewpoint_candidate_count > 0);
    if (!have_viewpoints) {
      RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                  "No candidate viewpoints this cycle; skipping motion planning, "
                  "still updating the scene graph");
    }

    UpdateKeyposeGraph();
    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    if (have_viewpoints) {
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
    }

    // Connector-node injection: bin the candidate viewpoints into grid_world cells
    // and add the inter-cell paths into the keypose graph (these non-keypose nodes
    // are kept as part of the traversability roadmap). Only meaningful when there are candidates.
    if (have_viewpoints) {
      GlobalPlanning();
    }

    // Reset current_room_id_ if the room disappeared from the representation
    if (current_room_id_ != -1 && !representation_->HasRoomNode(current_room_id_))
    {
      RCLCPP_WARN(this->get_logger(), "Current room with id %d does not exist in representation, reset to -1", current_room_id_);
      current_room_id_ = -1;
    }

    PublishFreespaceCloud();

    PublishRoomTypeVisualization();
    PublishObjectNodeMarkers();
  }
}

void SensorCoveragePlanner3D::UpdateViewpointRep(){
  if (!initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Planner not initialized, cannot update viewpoint representation");
    return;
  }

  planning_env_->GetUpdatedVoxelInds(current_obs_voxel_inds_);

  std::vector<int> intersection_voxel_inds;
  // get the intersection of current_obs_voxel_inds_ and previous_obs_voxel_inds_
  misc_utils_ns::SetIntersection(current_obs_voxel_inds_,
                                 previous_obs_voxel_inds_,
                                 intersection_voxel_inds);
  int intersection_voxel_num = intersection_voxel_inds.size();
  int current_obs_voxel_num = current_obs_voxel_inds_.size();
  int previous_obs_voxel_num = previous_obs_voxel_inds_.size();
  double ratio = intersection_voxel_num / (double)current_obs_voxel_num;

  if ((intersection_voxel_num < rep_threshold_voxel_num_ && ratio < rep_threshold_))
  {
    // If the intersection is less than 20% of the current obs voxel number,
    // we update the viewpoint representation
    // RCLCPP_INFO(rclcpp::get_logger("UpdateViewpointRep"), "Intersection voxel number is low, updating viewpoint representation.");
    add_viewpoint_rep_ = true;
  }
  // RCLCPP_ERROR(rclcpp::get_logger("UpdateViewpointRep"), "Object score: %f.", obj_score_);
  if (obj_score_ > 4.0)
  {
    // RCLCPP_INFO(rclcpp::get_logger("UpdateViewpointRep"), "Object score is high, adding viewpoint representation.");
    add_viewpoint_rep_ = true;
  }

  if (add_viewpoint_rep_) 
  {
    // RCLCPP_INFO(this->get_logger(), "Intersection voxel number: %d, Current obs voxel number: %d, Pre obs voxel number: %d",
    //             intersection_voxel_num, current_obs_voxel_num, previous_obs_voxel_num);
    // RCLCPP_INFO(this->get_logger(), "Intersection ratio: %f, Threshold : %d", ratio, rep_threshold_voxel_num_);
    // RCLCPP_INFO(this->get_logger(), "Updating viewpoint representation.");
    // add_viewpoint_rep_ = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr covered_cloud = planning_env_->GetUpdatedCloudInRange();
    int prev_size = representation_->GetViewPointReps().size();
    curr_viewpoint_rep_node_ind = representation_->AddViewPointRep(robot_position_, keypose_cloud_->cloud_, covered_cloud, viewpoint_rep_msg_.header.stamp);
    int curr_size = representation_->GetViewPointReps().size();
    representation_->GetViewPointRepNode(curr_viewpoint_rep_node_ind).SetRoomId(current_room_id_);
    geometry_msgs::msg::Point current_viewpoint_rep_node_pos = representation_->GetViewPointRepNodePos(curr_viewpoint_rep_node_ind);

    // publish viewpoint_rep_header_ only if we are actually adding a new viewpoint representation on rviz
    if (prev_size != curr_size) 
    {
      viewpoint_rep_msg_.viewpoint_id = curr_viewpoint_rep_node_ind;
      viewpoint_rep_pub_->publish(viewpoint_rep_msg_);
      // Update the viewpoint representation cloud
      Eigen::Vector3d origin(current_viewpoint_rep_node_pos.x,
                             current_viewpoint_rep_node_pos.y,
                             current_viewpoint_rep_node_pos.z);
      planning_env_->UpdateCoveredVoxels(origin);
      planning_env_->GetCurrentObsVoxelInds(previous_obs_voxel_inds_);

      representation_->GetLatestObjectNodeIndicesMutable().clear();
    }
  }
  viewpoint_rep_vis_cloud_->cloud_ = representation_->GetViewPointRepCloud();
  viewpoint_rep_vis_cloud_->Publish();
  covered_points_all_->cloud_ = representation_->GetCoveredPointsAllCloud();
  covered_points_all_->Publish();
}

void SensorCoveragePlanner3D::GetPoseAtTime(double imageTime, float &lidarX, float &lidarY, float &lidarZ, float &lidarRoll, float &lidarPitch, float &lidarYaw)
{
  while (odomFrontIDPointer != odomLastIDPointer)
  {
    if (odomTimeStack[odomFrontIDPointer] > imageTime)
    {
      break;
    }
    odomFrontIDPointer = (odomFrontIDPointer + 1) % 400;
  }
  if (odomTimeStack[odomFrontIDPointer] < imageTime)
  {
    lidarX = lidarXStack[odomFrontIDPointer];
    lidarY = lidarYStack[odomFrontIDPointer];
    lidarZ = lidarZStack[odomFrontIDPointer];
    lidarRoll = lidarRollStack[odomFrontIDPointer];
    lidarPitch = lidarPitchStack[odomFrontIDPointer];
    lidarYaw = lidarYawStack[odomFrontIDPointer];
  }
  else
  {
    int odomBackIDPointer = (odomFrontIDPointer - 1) % 400;
    float ratioFront = (imageTime - odomTimeStack[odomBackIDPointer]) / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);
    float ratioBack = (odomTimeStack[odomFrontIDPointer] - imageTime) / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);

    if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] > PI)
    {
      lidarYawStack[odomBackIDPointer] += 2 * PI;
    }
    else if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] < -PI)
    {
      lidarYawStack[odomBackIDPointer] -= 2 * PI;
    }

    lidarX = lidarXStack[odomFrontIDPointer] * ratioFront + lidarXStack[odomBackIDPointer] * ratioBack;
    lidarY = lidarYStack[odomFrontIDPointer] * ratioFront + lidarYStack[odomBackIDPointer] * ratioBack;
    lidarZ = lidarZStack[odomFrontIDPointer] * ratioFront + lidarZStack[odomBackIDPointer] * ratioBack;
    lidarRoll = lidarRollStack[odomFrontIDPointer] * ratioFront + lidarRollStack[odomBackIDPointer] * ratioBack;
    lidarPitch = lidarPitchStack[odomFrontIDPointer] * ratioFront + lidarPitchStack[odomBackIDPointer] * ratioBack;
    lidarYaw = lidarYawStack[odomFrontIDPointer] * ratioFront + lidarYawStack[odomBackIDPointer] * ratioBack;
  }
}

// Interpolated yaw at an arbitrary time, scanning the odom ring buffer backward
// from the newest sample. Unlike GetPoseAtTime this does NOT advance
// odomFrontIDPointer, so it is safe to call for times on both sides of a capture
// instant (and in any order). Clamps to the buffer's [oldest, newest] range and
// stops at the first non-monotonic slot (guards against uninitialized entries
// before the ring has filled).
void SensorCoveragePlanner3D::UpdateRoomLabel()
{
  // sysnav room typing (ported from rsb, navigation early-stop hooks removed):
  // bin the covered cloud into rooms via room_mask_; on first coverage, or when
  // coverage grew by >20 voxels / area by >5 m^2 since the last query, crop the
  // panorama around the room's covered points and ask the VLM.
  if (room_mask_.empty())
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr covered_cloud = planning_env_->GetUpdatedCloudInRange();
  std::unordered_map<int, int> room_counts;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>> room_cloud_in_range;
  std::unordered_map<int, Eigen::Vector3f> room_centers;
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    room_counts[room_id] = 0;
    room_cloud_in_range[room_id] = pcl::PointCloud<pcl::PointXYZI>();
    room_centers[room_id] = Eigen::Vector3f(0.0, 0.0, 0.0);
  }
  for (const auto &point : covered_cloud->points)
  {
    Eigen::Vector3f point_pos(point.x, point.y, point.z);
    Eigen::Vector3i point_voxel_ind = misc_utils_ns::point_to_voxel(point_pos, shift_, 1.0 / room_resolution_);
    if (point_voxel_ind.x() < 0 || point_voxel_ind.x() >= room_mask_.rows ||
        point_voxel_ind.y() < 0 || point_voxel_ind.y() >= room_mask_.cols)
    {
      continue;
    }
    int room_id = room_mask_.at<int>(point_voxel_ind.x(), point_voxel_ind.y());
    if (representation_->HasRoomNode(room_id))
    {
      room_counts[room_id]++;
      room_cloud_in_range[room_id].points.push_back(point);
      room_centers[room_id] += Eigen::Vector3f(point.x, point.y, point.z);
    }
  }
  std::vector<int> labled_rooms = {};
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    auto &room_node = id_to_room_node.second;
    // First deal with the unlabeled rooms
    if (room_counts[room_id] == 0)
    {
      continue;
    }
    else if (representation_->GetRoomNode(room_id).IsLabeled())
    {
      labled_rooms.push_back(room_id);
      continue;
    }
    else
    {
      room_centers[room_id] /= room_counts[room_id];
      representation_->GetRoomNode(room_id).SetVoxelNum(room_counts[room_id]);
      representation_->GetRoomNode(room_id).SetIsLabeled(true);

      pcl::PointCloud<pcl::PointXYZI>::Ptr room_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointXYZI room_center;
      room_center.x = room_centers[room_id].x();
      room_center.y = room_centers[room_id].y();
      room_center.z = robot_position_.z; // use the robot z position as the room center z
      room_center.intensity = 10.0;
      pcl::copyPointCloud((room_cloud_in_range[room_id]), *room_cloud_tmp);
      room_cloud_tmp->push_back(room_center);

      // publish it with room_cloud_pub_
      sensor_msgs::msg::PointCloud2 room_cloud_msg;
      pcl::toROSMsg(*room_cloud_tmp, room_cloud_msg);
      room_cloud_msg.header.frame_id = kWorldFrameID;
      room_cloud_msg.header.stamp = this->now();
      room_cloud_pub_->publish(room_cloud_msg);

      float lidarRoll = 0, lidarPitch = 0, lidarYaw = 0;
      float lidarX = 0, lidarY = 0, lidarZ = 0;
      GetPoseAtTime(imageTime, lidarX, lidarY, lidarZ, lidarRoll, lidarPitch, lidarYaw);
      cv::Mat camera_image_tmp = camera_image_.clone();
      cv::Mat cropped_img = project_pcl_to_image(room_cloud_tmp, lidarX, lidarY, lidarZ,
                                          lidarRoll, lidarPitch, lidarYaw,
                                          camera_image_tmp, room_center, room_id);
      room_node.SetImage(cropped_img);

      geometry_msgs::msg::Point anchor_point;
      anchor_point.x = room_center.x;
      anchor_point.y = room_center.y;
      anchor_point.z = robot_position_.z; // use the robot z position as the anchor point z
      room_node.SetAnchorPoint(anchor_point);
      room_node.SetLastArea(room_node.area_);

      tare_planner::msg::RoomType room_type_msg;
      room_type_msg.header.frame_id = kWorldFrameID;
      room_type_msg.header.stamp = this->now();
      room_type_msg.anchor_point = anchor_point;
      room_type_msg.room_id = room_id;
      room_type_msg.in_room = (room_id == current_room_id_);
      auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cropped_img).toImageMsg();
      auto room_mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", room_node.room_mask_).toImageMsg();
      room_type_msg.image = *img_msg;
      room_type_msg.room_mask = *room_mask_msg;
      room_type_msg.room_type = "";
      room_type_msg.voxel_num = room_counts[room_id];

      room_type_pub_->publish(room_type_msg);
      ROOM_DBG("[room_dbg] QUERY_PUB id=%d first=1 voxels=%d anchor=(%.2f,%.2f) in_room=%d",
               room_id, room_counts[room_id], anchor_point.x, anchor_point.y, room_type_msg.in_room ? 1 : 0);
    }
  }
  for(int room_id : labled_rooms)
  {
    if (!representation_->HasRoomNode(room_id)) {
      RCLCPP_WARN(this->get_logger(), "Room with id %d does not exist in representation, skip", room_id);
      continue;
    }
    auto &room_node = representation_->GetRoomNode(room_id);
    if (room_counts[room_id] - representation_->GetRoomNode(room_id).GetVoxelNum() > 20 ||
        room_node.area_ - room_node.last_area_ > 5.0)
    {
      bool flag1 = (room_counts[room_id] - representation_->GetRoomNode(room_id).GetVoxelNum() > 20);

      room_centers[room_id] /= room_counts[room_id];
      representation_->GetRoomNode(room_id).SetVoxelNum(room_counts[room_id]);
      representation_->GetRoomNode(room_id).SetIsLabeled(true);

      pcl::PointCloud<pcl::PointXYZI>::Ptr room_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointXYZI room_center;
      room_center.x = room_centers[room_id].x();
      room_center.y = room_centers[room_id].y();
      room_center.z = robot_position_.z; // use the robot z position as the room center z
      room_center.intensity = 10.0;
      pcl::copyPointCloud((room_cloud_in_range[room_id]), *room_cloud_tmp);
      room_cloud_tmp->push_back(room_center);

      // publish it with room_cloud_pub_
      sensor_msgs::msg::PointCloud2 room_cloud_msg;
      pcl::toROSMsg(*room_cloud_tmp, room_cloud_msg);
      room_cloud_msg.header.frame_id = kWorldFrameID;
      room_cloud_msg.header.stamp = this->now();
      room_cloud_pub_->publish(room_cloud_msg);

      cv::Mat cropped_img;
      geometry_msgs::msg::Point anchor_point;
      if (flag1)
      {
        float lidarRoll = 0, lidarPitch = 0, lidarYaw = 0;
        float lidarX = 0, lidarY = 0, lidarZ = 0;
        GetPoseAtTime(imageTime, lidarX, lidarY, lidarZ, lidarRoll, lidarPitch, lidarYaw);
        cv::Mat camera_image_tmp = camera_image_.clone();
        cropped_img = project_pcl_to_image(room_cloud_tmp, lidarX, lidarY, lidarZ,
                                          lidarRoll, lidarPitch, lidarYaw,
                                          camera_image_tmp, room_center, room_id);
        room_node.SetImage(cropped_img);

        // choose the first point in the room_cloud_in_range[room_id - 1] as the anchor point
        anchor_point.x = room_center.x;
        anchor_point.y = room_center.y;
        anchor_point.z = robot_position_.z; // use the robot z position as the anchor point z
        room_node.SetAnchorPoint(anchor_point);
      }
      else
      {
        cropped_img = room_node.GetImage();
        anchor_point = room_node.GetAnchorPoint();
      }

      room_node.SetLastArea(room_node.area_);

      tare_planner::msg::RoomType room_type_msg;
      room_type_msg.anchor_point = anchor_point;
      room_type_msg.room_id = room_id;
      room_type_msg.in_room = (room_id == current_room_id_);
      auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cropped_img).toImageMsg();
      auto room_mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", room_node.room_mask_).toImageMsg();
      room_type_msg.image = *img_msg;
      room_type_msg.room_mask = *room_mask_msg;
      room_type_msg.room_type = "";
      room_type_msg.voxel_num = room_counts[room_id];

      room_type_pub_->publish(room_type_msg);
      ROOM_DBG("[room_dbg] QUERY_PUB id=%d first=0 voxels=%d new_image=%d anchor=(%.2f,%.2f) in_room=%d",
               room_id, room_counts[room_id], flag1 ? 1 : 0, anchor_point.x, anchor_point.y, room_type_msg.in_room ? 1 : 0);
    }
  }
}

void SensorCoveragePlanner3D::PublishRoomTypeVisualization()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = kWorldFrameID;
  clear_marker.header.stamp = this->now();
  clear_marker.ns = "room_type";
  clear_marker.id = 0;
  clear_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  for (const auto &id_room_node_pair : representation_->GetRoomNodesMap())
  {
    const representation_ns::RoomNodeRep &room_node = id_room_node_pair.second;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kWorldFrameID;
    marker.header.stamp = this->now();
    marker.ns = "room_type";
    marker.id = room_node.show_id_;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // Label sits on the canonical interior point (stable, guaranteed inside),
    // not the drifting anchor_point_ (nav's in-range mean) or the centroid
    // (which can land in a wall for a non-convex room).
    marker.pose.position.x = room_node.centroid_.x();
    marker.pose.position.y = room_node.centroid_.y();
    marker.pose.position.z = room_node.centroid_.z();
    marker.pose.orientation.w = 0.65;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    Eigen::Vector3d color = misc_utils_ns::idToColor(room_node.GetId());
    marker.color.b = color[0] / 255.0;
    marker.color.g = color[1] / 255.0;
    marker.color.r = color[2] / 255.0;
    // Room id shows as soon as the room exists; the VLM label is appended once
    // it arrives (is_labeled_ is set only by RoomTypeCallback).
    marker.text = std::to_string(room_node.GetId());
    if (room_node.IsLabeled())
    {
      marker.text += " " + room_node.GetRoomLabel();
    }
    marker_array.markers.push_back(marker);
  }
  room_type_vis_pub_->publish(marker_array);
}

cv::Mat SensorCoveragePlanner3D::project_pcl_to_image(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_w,
    float &lidarX, float &lidarY, float &lidarZ, float &lidarRoll, float &lidarPitch, float &lidarYaw,
    cv::Mat &image, pcl::PointXYZI &room_center, int &room_id)
{
  cv::Mat image_projected = image.clone();
  cv::Mat rotated_image = image.clone();
  const float PI = 3.1415926f;
  const float camX = -0.12f, camY = -0.075f, camZ = 0.265f;
  const float camRoll = -1.5707963f, camPitch = 0.0f, camYaw = -1.5707963f;
  const int imageWidth = 1920;
  const int imageHeight = 640;
  // sysnav's crop assumes the 1920x640 360-degree panorama. On any other camera
  // (e.g. a pinhole test bag) the cv::Rect slices below would throw and take the
  // node down, so hand the frame back uncropped instead (logged once).
  if (image.cols != imageWidth || image.rows != imageHeight)
  {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "[project_pcl_to_image] camera image is %dx%d, not the %dx%d "
                     "panorama; sending the uncropped frame",
                     image.cols, image.rows, imageWidth, imageHeight);
    return image.clone();
  }
  const float minRange = 0.5f, maxRange = 10.0f;

  int imagePixelNum = imageWidth * imageHeight;

  float sinCamRoll = sin(camRoll);
  float cosCamRoll = cos(camRoll);
  float sinCamPitch = sin(camPitch);
  float cosCamPitch = cos(camPitch);
  float sinCamYaw = sin(camYaw);
  float cosCamYaw = cos(camYaw);

  float sinLidarRoll = sin(lidarRoll);
  float cosLidarRoll = cos(lidarRoll);
  float sinLidarPitch = sin(lidarPitch);
  float cosLidarPitch = cos(lidarPitch);
  float sinLidarYaw = sin(lidarYaw);
  float cosLidarYaw = cos(lidarYaw);

  int cloud_wSize = cloud_w->points.size();

  std::vector<int> hori_coords; // 收集所有有效的horiPixelID
  int hori_coord_room_center = -1;

  for (int i = 0; i <= cloud_wSize; i++)
  {
    float x1, y1, z1;
    if (i < cloud_wSize)
    {
      x1 = cloud_w->points[i].x - lidarX;
      y1 = cloud_w->points[i].y - lidarY;
      z1 = cloud_w->points[i].z - lidarZ;
    }
    else
    {
      x1 = room_center.x - lidarX;
      y1 = room_center.y - lidarY;
      z1 = room_center.z - lidarZ;
    }

    float x2 = x1 * cosLidarYaw + y1 * sinLidarYaw;
    float y2 = -x1 * sinLidarYaw + y1 * cosLidarYaw;
    float z2 = z1;

    float x3 = x2 * cosLidarPitch - z2 * sinLidarPitch;
    float y3 = y2;
    float z3 = x2 * sinLidarPitch + z2 * cosLidarPitch;

    float x4 = x3;
    float y4 = y3 * cosLidarRoll + z3 * sinLidarRoll;
    float z4 = -y3 * sinLidarRoll + z3 * cosLidarRoll;

    float x5 = x4 - camX;
    float y5 = y4 - camY;
    float z5 = z4 - camZ;

    float x6 = x5 * cosCamYaw + y5 * sinCamYaw;
    float y6 = -x5 * sinCamYaw + y5 * cosCamYaw;
    float z6 = z5;

    float x7 = x6 * cosCamPitch - z6 * sinCamPitch;
    float y7 = y6;
    float z7 = x6 * sinCamPitch + z6 * cosCamPitch;

    float x8 = x7;
    float y8 = y7 * cosCamRoll + z7 * sinCamRoll;
    float z8 = -y7 * sinCamRoll + z7 * cosCamRoll;

    int horiPixelID = -1, vertPixelID = -1;
    float horiDis = sqrt(x8 * x8 + z8 * z8);

    horiPixelID = imageWidth / (2 * PI) * atan2(x8, z8) + imageWidth / 2 + 1;
    vertPixelID = imageWidth / (2 * PI) * atan(y8 / horiDis) + imageHeight / 2 + 1;

    int pixelVal = 255 * (horiDis - minRange) / (maxRange - minRange);

    if (i < cloud_wSize)
    {
      if (horiPixelID >= 0 && horiPixelID <= imageWidth - 1 && vertPixelID >= 0 && vertPixelID <= imageHeight - 1)
      {
        for (int du = -1; du <= 1; ++du)
        {
          for (int dv = -1; dv <= 1; ++dv)
          {
            int uu = std::min(imageWidth - 1, std::max(0, horiPixelID + du));
            int vv = std::min(imageHeight - 1, std::max(0, vertPixelID + dv));
            int idx = vv * imageWidth + uu;
            {
              image_projected.at<cv::Vec3b>(vv, uu)[0] = pixelVal;
              image_projected.at<cv::Vec3b>(vv, uu)[1] = 255 - pixelVal;
              image_projected.at<cv::Vec3b>(vv, uu)[2] = 0;
            }
          }
        }
        hori_coords.push_back(horiPixelID);
      }
    }
    else
    {
      vertPixelID = imageHeight / 2; // room_center投影到图像中央
      for (int du = -5; du <= 5; ++du)
      {
        for (int dv = -5; dv <= 5; ++dv)
        {
          int uu = std::min(imageWidth - 1, std::max(0, horiPixelID + du));
          int vv = std::min(imageHeight - 1, std::max(0, vertPixelID + dv));
          int idx = vv * imageWidth + uu;
          {
            image_projected.at<cv::Vec3b>(vv, uu)[0] = 0;
            image_projected.at<cv::Vec3b>(vv, uu)[1] = 0;
            image_projected.at<cv::Vec3b>(vv, uu)[2] = 255;
          }
        }
      }
      hori_coord_room_center = horiPixelID;
    }
  }

  // Rotate image to center room_center at middle
  if (!(hori_coord_room_center >= 0 && hori_coord_room_center < imageWidth))
  {
    RCLCPP_ERROR(this->get_logger(), "[project_pcl_to_image] Error: hori_coord_room_center (%d) is out of bounds [0, %d).", hori_coord_room_center, imageWidth);
    return rotated_image; // 返回原图像
  }
  else
  {
    int shift = imageWidth / 2 - hori_coord_room_center;
    shift = (shift + imageWidth) % imageWidth; // wrap to [0, imageWidth)

    if (shift != 0)
    {
      // 1. 平移图像
      cv::Mat right_part = image(cv::Rect(imageWidth - shift, 0, shift, imageHeight)).clone();
      cv::Mat left_part = image(cv::Rect(0, 0, imageWidth - shift, imageHeight)).clone();
      cv::hconcat(right_part, left_part, rotated_image);

      // 2. 平移所有水平投影坐标
      for (int &coord : hori_coords)
      {
        coord = (coord + shift) % imageWidth;
      }
      hori_coord_room_center = (hori_coord_room_center + shift) % imageWidth;
    }
  }
  std::sort(hori_coords.begin(), hori_coords.end());
  int u_start = 0, u_end = imageWidth - 1;
  if (!hori_coords.empty())
  {
    int hori_min = hori_coords.front();
    int hori_max = hori_coords.back();

    u_start = hori_min;
    u_end = hori_max;
    u_start = std::max(u_start - 50, 0);
    u_end = std::min(u_end + 50, imageWidth - 1);

    MY_ASSERT(u_end > u_start);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[project_pcl_to_image] Error: No valid horizontal coordinates found in the point cloud.");
  }

  if (u_end > u_start)
  {
    cv::Mat cropped = rotated_image(cv::Rect(u_start, 0, u_end - u_start, imageHeight)).clone();
    return cropped;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[project_pcl_to_image] Error: u_end (%d) is not greater than u_start (%d).", u_end, u_start);
    cv::Mat cropped = rotated_image.clone();
    return cropped; // 返回原图像
  }
}

void SensorCoveragePlanner3D::PublishObjectNodeMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = kWorldFrameID;
  clear_marker.header.stamp = this->now();
  clear_marker.ns = "object_nodes_room";
  clear_marker.id = 0;
  clear_marker.type = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  for (const auto &id_object_node_pair : representation_->GetObjectNodeRepMap())
  {
    const auto &object_node = id_object_node_pair.second;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kWorldFrameID;
    marker.header.stamp = this->now();
    marker.ns = "object_nodes_room";
    marker.id = object_node.object_id_[0];
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = object_node.position_;
    marker.pose.position.z += 0.5; // raise the text a bit
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.text = "O" + std::to_string(object_node.object_id_[0]) + " R" + std::to_string(object_node.room_id_);
    marker_array.markers.push_back(marker);
  }
  object_node_marker_pub_->publish(marker_array);
}

void SensorCoveragePlanner3D::ProcessObjectNodes()
{
  for (auto &obj_id : object_ids_to_remove_)
  {
    representation_->GetObjectNodeRepMapMutable().erase(obj_id);
    representation_->GetLatestObjectNodeIndicesMutable().erase(obj_id);

    for (auto &viewpoint : representation_->GetViewPointRepsMutable())
    {
      viewpoint.DeleteObjectIndex(obj_id);
      viewpoint.DeleteDirectObjectIndex(obj_id);
    }
    for (auto &id_room_pair : representation_->GetRoomNodesMapMutable())
    {
      id_room_pair.second.DeleteObjectIndex(obj_id);
    }
  }
  object_ids_to_remove_.clear();
}

// ===========================================================================
// SemPathBench snapshot export (consumed offline by tools/sempath_export).
// Raw dump of the in-memory scene graph; the converter builds the layered map.
// ===========================================================================

namespace
{
json PointToJson(const geometry_msgs::msg::Point& p)
{
  return json::array({p.x, p.y, p.z});
}

double Round3(double v)
{
  return std::round(v * 1000.0) / 1000.0;
}
}  // namespace

bool SensorCoveragePlanner3D::WriteTextAtomic(const std::filesystem::path& path, const std::string& text,
                                              rclcpp::Logger log)
{
  const std::filesystem::path tmp = path.parent_path() / ("." + path.filename().string() + ".tmp");
  {
    std::ofstream out(tmp);
    if (!out)
    {
      RCLCPP_ERROR(log, "[sempath_export] cannot write %s", tmp.c_str());
      return false;
    }
    out << text;
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec)
  {
    RCLCPP_ERROR(log, "[sempath_export] cannot finalize %s: %s", path.c_str(), ec.message().c_str());
    std::filesystem::remove(tmp, ec);
    return false;
  }
  return true;
}

bool SensorCoveragePlanner3D::WriteRoomMaskPng(const std::filesystem::path& out_path, json& geometry) const
{
  geometry = nullptr;
  if (room_mask_.empty())
  {
    return true;
  }
  const cv::Mat nonzero = room_mask_ != 0;  // CV_8U
  const int nonzero_cells = cv::countNonZero(nonzero);
  if (nonzero_cells == 0)
  {
    return true;
  }
  std::vector<cv::Point> points;
  cv::findNonZero(nonzero, points);
  cv::Rect bb = cv::boundingRect(points);  // x = col, y = row
  const int margin = std::max(0, sempath_cfg_.mask_crop_margin_cells);
  const int x0 = std::max(0, bb.x - margin);
  const int y0 = std::max(0, bb.y - margin);
  const int x1 = std::min(room_mask_.cols, bb.x + bb.width + margin);
  const int y1 = std::min(room_mask_.rows, bb.y + bb.height + margin);
  const cv::Rect crop_rect(x0, y0, x1 - x0, y1 - y0);
  const cv::Mat crop = room_mask_(crop_rect);
  double min_id = 0.0, max_id = 0.0;
  cv::minMaxLoc(crop, &min_id, &max_id);

  std::error_code ec;
  std::filesystem::path final_path = out_path;
  std::string dtype = "uint16";
  if (min_id >= 0.0 && max_id < 65536.0)
  {
    cv::Mat crop16;
    crop.convertTo(crop16, CV_16U);
    const std::filesystem::path tmp = out_path.parent_path() / ("." + out_path.stem().string() + ".tmp.png");
    if (!cv::imwrite(tmp.string(), crop16))
    {
      RCLCPP_ERROR(this->get_logger(), "[sempath_export] cannot write %s", tmp.c_str());
      return false;
    }
    std::filesystem::rename(tmp, final_path, ec);
  }
  else
  {
    // Ids beyond uint16 (never seen in practice): raw little-endian int32 rows.
    dtype = "int32";
    final_path = out_path.parent_path() / (out_path.stem().string() + ".bin");
    const std::filesystem::path tmp = out_path.parent_path() / ("." + out_path.stem().string() + ".tmp.bin");
    {
      std::ofstream out(tmp, std::ios::binary);
      if (!out)
      {
        RCLCPP_ERROR(this->get_logger(), "[sempath_export] cannot write %s", tmp.c_str());
        return false;
      }
      for (int r = 0; r < crop.rows; ++r)
      {
        out.write(reinterpret_cast<const char*>(crop.ptr<int>(r)), sizeof(int) * crop.cols);
      }
    }
    std::filesystem::rename(tmp, final_path, ec);
  }
  if (ec)
  {
    RCLCPP_ERROR(this->get_logger(), "[sempath_export] cannot finalize %s: %s", final_path.c_str(),
                 ec.message().c_str());
    return false;
  }
  geometry = {
    {"file", final_path.filename().string()},
    {"dtype", dtype},
    {"crop", {{"row0", crop_rect.y}, {"col0", crop_rect.x}, {"rows", crop_rect.height}, {"cols", crop_rect.width}}},
    {"nonzero_cells", nonzero_cells},
    {"max_id", static_cast<int>(max_id)},
  };
  return true;
}

json SensorCoveragePlanner3D::BuildSemPathSnapshotJson(const std::string& reason, const json& mask_geometry) const
{
  json snapshot;
  snapshot["schema"] = "sysnav_scene_graph_dump/1";
  snapshot["reason"] = reason;
  {
    const auto wall = std::chrono::system_clock::now();
    const std::time_t wall_t = std::chrono::system_clock::to_time_t(wall);
    std::ostringstream iso;
    iso << std::put_time(std::gmtime(&wall_t), "%Y-%m-%dT%H:%M:%SZ");
    snapshot["stamp"] = {
      {"ros_sec", this->now().seconds()},
      {"wall_unix", std::chrono::duration<double>(wall.time_since_epoch()).count()},
      {"wall_iso", iso.str()},
    };
  }
  snapshot["frame"] = kWorldFrameID;
  snapshot["robot_position"] = {{"x", robot_position_.x}, {"y", robot_position_.y}, {"z", robot_position_.z}};
  snapshot["room_grid"] = {
    {"room_resolution", room_resolution_},
    {"shift", json::array({shift_.x(), shift_.y(), shift_.z()})},
    {"dims", {{"rows", room_mask_.rows}, {"cols", room_mask_.cols}}},
    {"layout", "row=x_index, col=y_index"},
    {"index_formula", "ix=floor(x/res+shift[0]); iy=floor(y/res+shift[1]); id=mask[ix][iy]"},
  };
  snapshot["room_mask"] = mask_geometry;

  json rooms = json::array();
  for (const auto& id_room : representation_->GetRoomNodesMap())
  {
    const auto& room = id_room.second;
    json polygon = json::array();
    for (const auto& pt : room.polygon_.polygon.points)
    {
      polygon.push_back(json::array({Round3(pt.x), Round3(pt.y)}));
    }
    rooms.push_back({
      {"id", room.id_},
      {"show_id", room.show_id_},
      {"label", room.GetRoomLabel()},
      {"label_votes", room.GetLabels()},
      {"is_labeled", room.IsLabeled()},
      {"is_connected", room.is_connected_},
      {"centroid", json::array({room.centroid_.x(), room.centroid_.y(), room.centroid_.z()})},
      {"anchor_point", PointToJson(room.anchor_point_)},
      {"area_m2", room.area_},
      {"polygon_xy", polygon},
      {"neighbors", std::vector<int>(room.neighbors_.begin(), room.neighbors_.end())},
    });
  }
  snapshot["rooms"] = rooms;

  json objects = json::array();
  for (const auto& id_obj : representation_->GetObjectNodeRepMap())
  {
    const auto& obj = id_obj.second;
    json corners = json::array();
    for (const auto& corner : obj.bbox3d_)
    {
      corners.push_back(json::array({Round3(corner.x), Round3(corner.y), Round3(corner.z)}));
    }
    json entry = {
      {"id", obj.object_id_.empty() ? id_obj.first : obj.object_id_.front()},
      {"ids", obj.object_id_},
      {"label", obj.label_},
      {"confidence", obj.confidence_},
      {"status", obj.status_},
      {"room_id", obj.room_id_},
      {"img_path", obj.img_path_},
      {"timestamp", obj.timestamp_.seconds()},
      {"position", PointToJson(obj.position_)},
      {"bbox3d", corners},
      {"cloud_xyz", nullptr},
      {"cloud_points_raw", static_cast<int>(obj.cloud_.width) * static_cast<int>(obj.cloud_.height)},
    };
    if (sempath_cfg_.include_clouds && obj.cloud_.width * obj.cloud_.height > 0)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(obj.cloud_, *cloud);
      if (sempath_cfg_.cloud_voxel_m > 0.0 && cloud->size() > 1)
      {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        const float leaf = static_cast<float>(sempath_cfg_.cloud_voxel_m);
        voxel.setLeafSize(leaf, leaf, leaf);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.filter(*filtered);
        cloud = filtered;
      }
      json cloud_xyz = json::array();
      for (const auto& pt : cloud->points)
      {
        cloud_xyz.push_back(json::array({Round3(pt.x), Round3(pt.y), Round3(pt.z)}));
      }
      entry["cloud_xyz"] = cloud_xyz;
    }
    objects.push_back(entry);
  }
  snapshot["objects"] = objects;

  json doors = json::array();
  if (door_cloud_)
  {
    for (const auto& pt : door_cloud_->points)
    {
      doors.push_back({{"x", Round3(pt.x)}, {"y", Round3(pt.y)}, {"z", Round3(pt.z)},
                       {"room_a", static_cast<int>(pt.r)}, {"room_b", static_cast<int>(pt.g)},
                       {"label", static_cast<int>(pt.label)}});
    }
  }
  snapshot["doors"] = doors;

  json adjacency = json::array();
  for (int i = 0; i < adjacency_matrix.rows(); ++i)
  {
    for (int j = i + 1; j < adjacency_matrix.cols(); ++j)
    {
      if (adjacency_matrix(i, j) != 0 || adjacency_matrix(j, i) != 0)
      {
        adjacency.push_back(json::array({i + 1, j + 1}));
      }
    }
  }
  snapshot["room_adjacency"] = adjacency;
  return snapshot;
}

bool SensorCoveragePlanner3D::ExportSemPathSnapshot(const std::string& reason)
{
  if (!sempath_cfg_.enabled || !initialized_ || !representation_)
  {
    return false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const std::filesystem::path dir(sempath_cfg_.output_dir);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  // Mask first, JSON last: a reader that sees a fresh JSON always finds its matching PNG.
  json mask_geometry;
  if (!WriteRoomMaskPng(dir / "room_mask_latest.png", mask_geometry))
  {
    return false;
  }
  const json snapshot = BuildSemPathSnapshotJson(reason, mask_geometry);
  if (!WriteTextAtomic(dir / "scene_graph_latest.json", snapshot.dump(), this->get_logger()))
  {
    return false;
  }
  if (sempath_cfg_.keep_history)
  {
    const std::filesystem::path history = dir / "history";
    std::filesystem::create_directories(history, ec);
    char suffix[64];
    std::snprintf(suffix, sizeof(suffix), "%04d_%s", sempath_export_count_, reason.c_str());
    std::filesystem::copy_file(dir / "scene_graph_latest.json", history / ("scene_graph_" + std::string(suffix) + ".json"),
                               std::filesystem::copy_options::overwrite_existing, ec);
    if (!mask_geometry.is_null())
    {
      const std::string mask_file = mask_geometry["file"].get<std::string>();
      std::filesystem::copy_file(dir / mask_file,
                                 history / ("room_mask_" + std::string(suffix) + std::filesystem::path(mask_file).extension().string()),
                                 std::filesystem::copy_options::overwrite_existing, ec);
    }
  }
  const json latest = {
    {"stamp", snapshot["stamp"]},
    {"reason", reason},
    {"count", sempath_export_count_},
    {"files", json::array({"scene_graph_latest.json",
                           mask_geometry.is_null() ? json(nullptr) : mask_geometry["file"], "bev_latest.npz"})},
  };
  WriteTextAtomic(dir / "latest.json", latest.dump(2), this->get_logger());
  ++sempath_export_count_;
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(this->get_logger(), "[sempath_export] saved %s snapshot #%d -> %s (%zu rooms, %zu objects, %zu doors, %.0f ms)",
              reason.c_str(), sempath_export_count_ - 1, (dir / "scene_graph_latest.json").c_str(),
              snapshot["rooms"].size(), snapshot["objects"].size(), snapshot["doors"].size(), ms);
  return true;
}

} // namespace sensor_coverage_planner_3d_ns
