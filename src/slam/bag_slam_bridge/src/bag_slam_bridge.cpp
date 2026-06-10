// Bridge a rosbag's own SLAM onto the SysNav interface topics.
//
// C++ port of bag_slam_bridge/bag_slam_bridge.py (kept alongside as the
// reference implementation, installed as `bag_slam_bridge_py`). The Python
// node burned ~half a core on per-point message access at 15 Hz x ~13k pts;
// this port is functionally identical. See the Python module docstring for
// the full design rationale (why the LIO topic instead of /tf, deskew by
// per-point offset_time, arise's body blind-zone, frame conventions).
//
// Pipeline summary:
//   /go2w_005/lio/odometry  -> (compose static world<-odom)  -> /state_estimation
//   /go2w_005/livox/lidar   -> body/range filter -> time-bucketed deskew with
//                              slerp/lerp-interpolated LIO poses -> world
//                           -> /registered_scan
// Sweeps wait in a small queue until the first LIO pose after their last
// point (<= one sweep of latency). Both outputs are stamped `output_frame`
// (default "map"), defined identical to the bag's `world` via a static
// identity transform.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace
{

// arise's blind-zone filter (featureExtraction.cpp:270, livox_mid360_go2w.yaml),
// in the raw lidar frame: a box around the robot body plus a thin disk that
// catches ground/chassis returns right at sensor height.
constexpr double kBodyBoxXBack = -0.45, kBodyBoxXFront = 0.15;
constexpr double kBodyBoxYRight = -0.15, kBodyBoxYLeft = 0.15;
constexpr double kBodyDiskZLow = -0.05, kBodyDiskZHigh = 0.05, kBodyDiskRadius = 0.5;

std::string stripSlash(const std::string & frame)
{
  return (!frame.empty() && frame[0] == '/') ? frame.substr(1) : frame;
}

struct Pose
{
  double t;
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
};

struct PendingSweep
{
  double t0;
  double t_end;
  builtin_interfaces::msg::Time stamp;
  std::string lidar_frame;
  std::vector<Eigen::Vector3f> xyz;
  std::vector<float> intensity;  // empty => publish xyz-only cloud
  std::vector<float> off;        // empty => no per-point times (single pose)
};

}  // namespace

class BagSlamBridge : public rclcpp::Node
{
public:
  BagSlamBridge()
  : Node("bag_slam_bridge")
  {
    lidar_msg_type_ = declare_parameter<std::string>("lidar_msg_type", "livox");
    const std::string default_lidar =
      lidar_msg_type_ == "livox" ? "/go2w_005/livox/lidar" : "/go2w_005/lidar";
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", default_lidar);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/go2w_005/lio/odometry");
    registered_scan_topic_ =
      declare_parameter<std::string>("registered_scan_topic", "/registered_scan");
    state_estimation_topic_ =
      declare_parameter<std::string>("state_estimation_topic", "/state_estimation");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    lidar_frame_param_ = declare_parameter<std::string>("lidar_frame", "go2w_005/livox_frame");
    base_frame_param_ = declare_parameter<std::string>("base_frame", "go2w_005/base");
    odom_frame_param_ = declare_parameter<std::string>("odom_frame", "go2w_005/odom");
    // Max extrapolation past either end of the pose history before a scan is dropped.
    pose_match_tol_ = declare_parameter<double>("pose_match_tol", 0.05);
    // Max spacing between the two poses bracketing a point time; a bigger
    // gap means the LIO stream skipped and interpolation would be fiction.
    max_pose_gap_ = declare_parameter<double>("max_pose_gap", 0.2);
    // Number of time buckets a sweep is split into for deskewing.
    deskew_buckets_ = static_cast<int>(declare_parameter<int64_t>("deskew_buckets", 12));
    // Replicate arise's robot-body blind zone (livox path only).
    body_filter_ = declare_parameter<bool>("body_filter", true);
    // Drop returns closer than this to the sensor (Livox emits (0,0,0) for no-return).
    min_range_ = declare_parameter<double>("min_range", 0.1);

    base_frame_ = base_frame_param_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(registered_scan_topic_, 5);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(state_estimation_topic_, 10);

    const auto sensor_qos = rclcpp::QoS(10).best_effort();
    if (lidar_msg_type_ == "livox") {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_, sensor_qos,
        std::bind(&BagSlamBridge::livoxCallback, this, std::placeholders::_1));
    } else {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, sensor_qos,
        std::bind(&BagSlamBridge::pointcloudCallback, this, std::placeholders::_1));
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, std::bind(&BagSlamBridge::odomCallback, this, std::placeholders::_1));

    publishIdentityWorldToOutput();
    RCLCPP_INFO(
      get_logger(),
      "bag_slam_bridge (c++): lidar='%s' (%s) -> %s, %s -> %s; world='%s', output='%s', "
      "deskew_buckets=%d, body_filter=%s.",
      lidar_topic_.c_str(), lidar_msg_type_.c_str(), registered_scan_topic_.c_str(),
      odom_topic_.c_str(), state_estimation_topic_.c_str(), world_frame_.c_str(),
      output_frame_.c_str(), deskew_buckets_, body_filter_ ? "true" : "false");
  }

private:
  void publishIdentityWorldToOutput()
  {
    if (output_frame_ == world_frame_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id = output_frame_;
    tf_msg.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(tf_msg);
  }

  // Static (time-independent) target<-source.
  bool lookupStatic(
    const std::string & target, const std::string & source,
    Eigen::Quaterniond & q, Eigen::Vector3d & t)
  {
    try {
      const auto tf_msg =
        tf_buffer_->lookupTransform(stripSlash(target), stripSlash(source), tf2::TimePointZero);
      const auto & tr = tf_msg.transform;
      q = Eigen::Quaterniond(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z);
      q.normalize();
      t = Eigen::Vector3d(tr.translation.x, tr.translation.y, tr.translation.z);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string odom_frame =
      msg->header.frame_id.empty() ? odom_frame_param_ : msg->header.frame_id;
    base_frame_ = msg->child_frame_id.empty() ? base_frame_param_ : msg->child_frame_id;

    if (!have_world_odom_) {
      if (!lookupStatic(world_frame_, odom_frame, q_wo_, t_wo_)) {
        return;
      }
      have_world_odom_ = true;
    }

    Pose pose;
    pose.t = rclcpp::Time(msg->header.stamp).seconds();
    const auto & q = msg->pose.pose.orientation;
    const auto & p = msg->pose.pose.position;
    pose.q = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    pose.p = Eigen::Vector3d(p.x, p.y, p.z);
    odom_hist_.push_back(pose);
    while (odom_hist_.size() > 200) {
      odom_hist_.pop_front();
    }

    const Eigen::Quaterniond q_wb = q_wo_ * pose.q;
    const Eigen::Vector3d t_wb = q_wo_ * pose.p + t_wo_;

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = output_frame_;
    out.child_frame_id = stripSlash(base_frame_);
    out.pose.pose.position.x = t_wb.x();
    out.pose.pose.position.y = t_wb.y();
    out.pose.pose.position.z = t_wb.z();
    out.pose.pose.orientation.x = q_wb.x();
    out.pose.pose.orientation.y = q_wb.y();
    out.pose.pose.orientation.z = q_wb.z();
    out.pose.pose.orientation.w = q_wb.w();
    out.twist = msg->twist;
    odom_pub_->publish(out);

    drainPending();
  }

  // odom<-base at time t by slerp/lerp over the pose history.
  bool interpOdom(double t, Eigen::Quaterniond & q, Eigen::Vector3d & p)
  {
    if (odom_hist_.empty()) {
      return false;
    }
    auto it = std::lower_bound(
      odom_hist_.begin(), odom_hist_.end(), t,
      [](const Pose & a, double val) {return a.t < val;});
    if (it == odom_hist_.begin()) {
      if (odom_hist_.front().t - t > pose_match_tol_) {
        return false;
      }
      q = odom_hist_.front().q;
      p = odom_hist_.front().p;
    } else if (it == odom_hist_.end()) {
      if (t - odom_hist_.back().t > pose_match_tol_) {
        return false;
      }
      q = odom_hist_.back().q;
      p = odom_hist_.back().p;
    } else {
      const Pose & hi = *it;
      const Pose & lo = *std::prev(it);
      const double dt = hi.t - lo.t;
      if (dt > max_pose_gap_) {
        return false;
      }
      const double a = (t - lo.t) / std::max(dt, 1e-9);
      q = lo.q.slerp(a, hi.q);
      p = (1.0 - a) * lo.p + a * hi.p;
    }
    return true;
  }

  // Register every buffered sweep whose interval the pose history now covers.
  void drainPending()
  {
    if (!have_world_odom_ || odom_hist_.empty()) {
      return;
    }
    while (!pending_.empty() && odom_hist_.back().t >= pending_.front().t_end) {
      PendingSweep sweep = std::move(pending_.front());
      pending_.pop_front();
      registerAndPublish(sweep);
    }
  }

  // Deskews by time bucket (one interpolated world<-lidar pose per bucket)
  // and publishes the sweep in the output frame.
  void registerAndPublish(const PendingSweep & sweep)
  {
    if (!have_base_lidar_ || cached_lidar_frame_ != sweep.lidar_frame) {
      if (!lookupStatic(base_frame_, sweep.lidar_frame, q_bl_, t_bl_)) {
        return;
      }
      have_base_lidar_ = true;
      cached_lidar_frame_ = sweep.lidar_frame;
    }

    const size_t n = sweep.xyz.size();
    const bool has_off = !sweep.off.empty();
    const int nb = has_off ? deskew_buckets_ : 1;

    float off_max = 0.0f;
    if (has_off) {
      off_max = *std::max_element(sweep.off.begin(), sweep.off.end());
    }
    const double step = (static_cast<double>(off_max) + 1e-6) / nb;

    // One world<-lidar transform per non-empty bucket (drop the sweep if any
    // needed pose is missing, same as the Python node).
    std::vector<bool> bucket_used(nb, !has_off);
    if (has_off) {
      for (const float off : sweep.off) {
        bucket_used[std::min(nb - 1, static_cast<int>(off / step))] = true;
      }
    }
    std::vector<Eigen::Matrix3f> R_wl(nb);
    std::vector<Eigen::Vector3f> t_wl(nb);
    for (int b = 0; b < nb; ++b) {
      if (!bucket_used[b]) {
        continue;
      }
      const double t_bucket = sweep.t0 + (has_off ? (b + 0.5) * step : 0.0);
      Eigen::Quaterniond q_ob;
      Eigen::Vector3d t_ob;
      if (!interpOdom(t_bucket, q_ob, t_ob)) {
        if (!scan_drop_warned_) {
          RCLCPP_WARN(get_logger(),
            "No bracketing LIO poses for a scan; dropping until odom aligns.");
          scan_drop_warned_ = true;
        }
        return;
      }
      const Eigen::Quaterniond q_wb = q_wo_ * q_ob;
      const Eigen::Vector3d t_wb = q_wo_ * t_ob + t_wo_;
      const Eigen::Quaterniond q_wlb = q_wb * q_bl_;
      const Eigen::Vector3d t_wlb = q_wb * t_bl_ + t_wb;
      R_wl[b] = q_wlb.toRotationMatrix().cast<float>();
      t_wl[b] = t_wlb.cast<float>();
    }
    scan_drop_warned_ = false;

    const bool has_intensity = !sweep.intensity.empty();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = sweep.stamp;
    cloud.header.frame_id = output_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(n);
    cloud.fields.resize(has_intensity ? 4 : 3);
    const char * names[] = {"x", "y", "z", "intensity"};
    for (size_t f = 0; f < cloud.fields.size(); ++f) {
      cloud.fields[f].name = names[f];
      cloud.fields[f].offset = static_cast<uint32_t>(4 * f);
      cloud.fields[f].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[f].count = 1;
    }
    cloud.is_bigendian = false;
    cloud.point_step = has_intensity ? 16 : 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(static_cast<size_t>(cloud.row_step));

    float * out = reinterpret_cast<float *>(cloud.data.data());
    const int stride = has_intensity ? 4 : 3;
    for (size_t i = 0; i < n; ++i) {
      const int b = has_off
        ? std::min(nb - 1, static_cast<int>(sweep.off[i] / step)) : 0;
      const Eigen::Vector3f pw = R_wl[b] * sweep.xyz[i] + t_wl[b];
      float * dst = out + i * stride;
      dst[0] = pw.x();
      dst[1] = pw.y();
      dst[2] = pw.z();
      if (has_intensity) {
        dst[3] = sweep.intensity[i];
      }
    }
    scan_pub_->publish(cloud);
  }

  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }
    PendingSweep sweep;
    sweep.lidar_frame =
      stripSlash(msg->header.frame_id.empty() ? lidar_frame_param_ : msg->header.frame_id);
    sweep.stamp = msg->header.stamp;
    sweep.t0 = rclcpp::Time(msg->header.stamp).seconds();
    sweep.xyz.reserve(msg->points.size());
    sweep.intensity.reserve(msg->points.size());
    sweep.off.reserve(msg->points.size());

    const double min_r2 = min_range_ * min_range_;
    float off_max = 0.0f;
    for (const auto & pt : msg->points) {
      const double r2 = static_cast<double>(pt.x) * pt.x +
        static_cast<double>(pt.y) * pt.y + static_cast<double>(pt.z) * pt.z;
      if (r2 < min_r2) {
        continue;
      }
      if (body_filter_) {
        const bool in_box = pt.x > kBodyBoxXBack && pt.x < kBodyBoxXFront &&
          pt.y > kBodyBoxYRight && pt.y < kBodyBoxYLeft;
        const bool in_disk = pt.z > kBodyDiskZLow && pt.z < kBodyDiskZHigh &&
          r2 < kBodyDiskRadius * kBodyDiskRadius;
        if (in_box || in_disk) {
          continue;
        }
      }
      sweep.xyz.emplace_back(pt.x, pt.y, pt.z);
      sweep.intensity.push_back(static_cast<float>(pt.reflectivity));
      const float off = static_cast<float>(pt.offset_time) * 1e-9f;
      sweep.off.push_back(off);
      off_max = std::max(off_max, off);
    }
    if (sweep.xyz.empty()) {
      return;
    }
    sweep.t_end = sweep.t0 + off_max;
    pending_.push_back(std::move(sweep));
    while (pending_.size() > 40) {
      pending_.pop_front();
    }
    drainPending();
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PendingSweep sweep;
    sweep.lidar_frame =
      stripSlash(msg->header.frame_id.empty() ? lidar_frame_param_ : msg->header.frame_id);
    sweep.stamp = msg->header.stamp;
    sweep.t0 = rclcpp::Time(msg->header.stamp).seconds();
    sweep.t_end = sweep.t0;  // no per-point times: single interpolated pose at the stamp

    bool has_intensity = false;
    for (const auto & f : msg->fields) {
      if (f.name == "intensity") {
        has_intensity = true;
      }
    }
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> ii;
    if (has_intensity) {
      ii = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    }
    const double min_r2 = min_range_ * min_range_;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      float inten = 0.0f;
      if (ii) {
        inten = *(*ii);
        ++(*ii);
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y +
        static_cast<double>(z) * z;
      if (r2 < min_r2) {
        continue;
      }
      sweep.xyz.emplace_back(x, y, z);
      if (has_intensity) {
        sweep.intensity.push_back(inten);
      }
    }
    if (sweep.xyz.empty()) {
      return;
    }
    pending_.push_back(std::move(sweep));
    while (pending_.size() > 40) {
      pending_.pop_front();
    }
    drainPending();
  }

  // parameters
  std::string lidar_msg_type_, lidar_topic_, odom_topic_;
  std::string registered_scan_topic_, state_estimation_topic_;
  std::string world_frame_, output_frame_;
  std::string lidar_frame_param_, base_frame_param_, odom_frame_param_;
  double pose_match_tol_{0.05}, max_pose_gap_{0.2}, min_range_{0.1};
  int deskew_buckets_{12};
  bool body_filter_{true};

  // state
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  bool have_world_odom_{false};
  Eigen::Quaterniond q_wo_;       // world <- odom (static, cached)
  Eigen::Vector3d t_wo_;
  bool have_base_lidar_{false};
  Eigen::Quaterniond q_bl_;       // base <- lidar (static sensor mount, cached)
  Eigen::Vector3d t_bl_;
  std::string cached_lidar_frame_;
  std::string base_frame_;
  std::deque<Pose> odom_hist_;
  std::deque<PendingSweep> pending_;
  bool scan_drop_warned_{false};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagSlamBridge>());
  rclcpp::shutdown();
  return 0;
}
