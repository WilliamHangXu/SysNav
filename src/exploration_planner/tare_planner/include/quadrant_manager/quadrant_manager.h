//
// QuadrantManager: fits a single GLOBAL building orientation (two orthonormal
// axes in the source/map frame) from all room polygons via cv::minAreaRect, and
// FREEZES it once stable -- the building orientation is static, so this is a
// latch-once quantity (mirrors the intent of the planner's TryFreezeWorldFromOdom,
// but triggered on warmup stability rather than a TF lookup).
//
// It owns the per-room quadrant cross-line RViz visualization. The frozen
// `BuildingAxes` it produces is handed by the planner to both the NavGraph (to tag
// each node's `area` for live node-coloring) and the scene-graph exporter (to tag
// each waypoint + draw the JSON compass) -- so all three agree by construction.
//
// This is the one place that depends on OpenCV's imgproc (the fit). The exporter
// stays OpenCV-free: it only consumes the resulting BuildingAxes, never fits.
//

#ifndef QUADRANT_MANAGER_QUADRANT_MANAGER_H
#define QUADRANT_MANAGER_QUADRANT_MANAGER_H

#include <map>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <navgraph/building_axes.h>
#include <representation/representation.h>
#include <tare_planner/msg/wall_axis.hpp>

namespace quadrant_ns
{
class QuadrantManager
{
public:
  explicit QuadrantManager(rclcpp::Node::SharedPtr nh);
  ~QuadrantManager() = default;

  void ReadParameters(rclcpp::Node::SharedPtr nh);

  // Self-throttled (every kUpdateInterval-th call). While not frozen: the PRIMARY
  // axis source is the latest /wall_axis (dominant building-wall orientation),
  // frozen once it is confident + stable; the min-rect FitAxes fallback is used
  // ONLY at the hard cap. Once frozen, stops fitting and only republishes the
  // per-room cross-lines. `debug_log` gates verbose warmup/fit logs; the one-time
  // FREEZE line is always shown.
  void Update(const std::map<int, representation_ns::RoomNodeRep>& rooms, bool debug_log = false);

  // The global axes. valid == false until the warmup freeze completes.
  const navgraph_ns::BuildingAxes& GetAxes() const
  {
    return axes_;
  }
  bool IsFrozen() const
  {
    return frozen_;
  }

private:
  // Fit + canonicalize the global axes from all alive-room polygon vertices.
  // Returns false (leaving the previous `out` untouched) on too little/degenerate
  // geometry this cycle. On success `out.valid == true`.
  bool FitAxes(const std::map<int, representation_ns::RoomNodeRep>& rooms,
               navgraph_ns::BuildingAxes& out) const;
  // Build canonicalized axes from a grid angle theta (the wall-axis source): reduce
  // theta to (-45,45] deg so east is within +/-45 deg of map +X, north = +90 CCW.
  // Identical canonicalization to FitAxes, so downstream is source-agnostic.
  navgraph_ns::BuildingAxes AxesFromAngle(double theta) const;
  // Draw, per alive room, the two axis lines through its centroid spanning the
  // room (one LINE_LIST marker, rewritten each publish so dead rooms vanish).
  // No-op until the axes are valid.
  void PublishCrossLines(const std::map<int, representation_ns::RoomNodeRep>& rooms);

  navgraph_ns::BuildingAxes axes_;  // frozen result (valid == false until freeze)
  bool frozen_ = false;
  int update_call_count_ = 0;       // self-throttle counter
  int warmup_cycles_ = 0;           // successful-fit cycles before freeze
  int stable_count_ = 0;            // consecutive angle-stable fits
  double last_angle_rad_ = 0.0;     // last grid angle tracked for stability
  bool have_last_angle_ = false;

  // Latest /wall_axis measurement (the primary axis source). Written by the
  // subscription callback, read in Update(). Both run on the planner's
  // single-threaded executor, so no mutex is needed.
  struct WallAxisState
  {
    double yaw_rad = 0.0;
    double confidence = 0.0;
    double support_length = 0.0;
    bool valid = false;
  } wall_axis_;
  rclcpp::Subscription<tare_planner::msg::WallAxis>::SharedPtr wall_axis_sub_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cross_marker_pub_;

  std::string world_frame_id_;  // marker frame, default "map"
  int kUpdateInterval_;
  int kWarmupMinRooms_;
  int kWarmupMinVertices_;
  int kFreezeStableCycles_;
  int kMaxWarmupCycles_;
  double kFreezeAngleEpsRad_;  // angle-stability tolerance (param given in deg)
  double kCrossLineWidth_;
  double kWallMinConfidence_;  // min /wall_axis confidence to trust it as primary
  double kWallMinSupportM_;    // min /wall_axis aligned wall length to trust it (m)
};
}  // namespace quadrant_ns

#endif  // QUADRANT_MANAGER_QUADRANT_MANAGER_H
