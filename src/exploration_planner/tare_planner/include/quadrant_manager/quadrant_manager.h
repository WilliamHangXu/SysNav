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

namespace quadrant_ns
{
class QuadrantManager
{
public:
  explicit QuadrantManager(rclcpp::Node::SharedPtr nh);
  ~QuadrantManager() = default;

  void ReadParameters(rclcpp::Node::SharedPtr nh);

  // Self-throttled (every kUpdateInterval-th call). While not frozen, fits the
  // global building axes from all alive-room polygons and freezes on warmup
  // stability; once frozen, stops fitting and only republishes the per-room
  // quadrant cross-lines. `debug_log` (a runtime planner param, passed in each
  // call) gates verbose warmup/fit logs; the one-time FREEZE line is always shown.
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
  // Draw, per alive room, the two axis lines through its centroid spanning the
  // room (one LINE_LIST marker, rewritten each publish so dead rooms vanish).
  // No-op until the axes are valid.
  void PublishCrossLines(const std::map<int, representation_ns::RoomNodeRep>& rooms);

  navgraph_ns::BuildingAxes axes_;  // frozen result (valid == false until freeze)
  bool frozen_ = false;
  int update_call_count_ = 0;       // self-throttle counter
  int warmup_cycles_ = 0;           // successful-fit cycles before freeze
  int stable_count_ = 0;            // consecutive angle-stable fits
  double last_angle_rad_ = 0.0;     // east angle of the previous fit
  bool have_last_angle_ = false;

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
};
}  // namespace quadrant_ns

#endif  // QUADRANT_MANAGER_QUADRANT_MANAGER_H
