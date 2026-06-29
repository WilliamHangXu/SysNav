//
// QuadrantManager implementation. See quadrant_manager/quadrant_manager.h for the
// design overview.
//

#include <quadrant_manager/quadrant_manager.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace quadrant_ns
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

// Smallest difference of two east-axis angles, wrapped to [0, pi/4]. The axes are
// defined only mod 90 deg (after canonicalization a 90-deg rect rotation is the
// same orientation), so stability is measured mod 90 deg.
double AngleDiff(double a, double b)
{
  double d = std::fmod(std::abs(a - b), kPi / 2.0);
  if (d < 0.0)
  {
    d += kPi / 2.0;
  }
  if (d > kPi / 4.0)
  {
    d = kPi / 2.0 - d;
  }
  return d;
}
}  // namespace

QuadrantManager::QuadrantManager(rclcpp::Node::SharedPtr nh)
  : world_frame_id_("map")
  , kUpdateInterval_(4)
  , kWarmupMinRooms_(2)
  , kWarmupMinVertices_(8)
  , kFreezeStableCycles_(5)
  , kMaxWarmupCycles_(60)
  , kFreezeAngleEpsRad_(2.0 * kPi / 180.0)
  , kCrossLineWidth_(0.08)
{
  ReadParameters(nh);
  clock_ = nh->get_clock();
  cross_marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("quadrant/cross_marker", 2);
}

void QuadrantManager::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("quadrant/kUpdateInterval", kUpdateInterval_);
  nh->get_parameter("quadrant/kWarmupMinRooms", kWarmupMinRooms_);
  nh->get_parameter("quadrant/kWarmupMinVertices", kWarmupMinVertices_);
  nh->get_parameter("quadrant/kFreezeStableCycles", kFreezeStableCycles_);
  nh->get_parameter("quadrant/kMaxWarmupCycles", kMaxWarmupCycles_);
  nh->get_parameter("quadrant/kCrossLineWidth", kCrossLineWidth_);
  nh->get_parameter("quadrant/world_frame_id", world_frame_id_);
  double eps_deg = kFreezeAngleEpsRad_ * 180.0 / kPi;
  nh->get_parameter("quadrant/kFreezeAngleEpsDeg", eps_deg);
  kFreezeAngleEpsRad_ = eps_deg * kPi / 180.0;
  if (kUpdateInterval_ < 1)
  {
    kUpdateInterval_ = 1;
  }
}

void QuadrantManager::Update(const std::map<int, representation_ns::RoomNodeRep>& rooms, bool debug_log)
{
  // Self-throttle (rooms evolve slowly; fitting + viz need not run every cycle).
  if ((update_call_count_++ % kUpdateInterval_) != 0)
  {
    return;
  }

  // Once frozen, never refit -- only keep the cross-lines current as rooms move.
  if (frozen_)
  {
    PublishCrossLines(rooms);
    return;
  }

  // Warmup-evidence gate: enough alive rooms + polygon vertices before fitting.
  int alive_rooms = 0;
  int total_vertices = 0;
  for (const auto& id_room : rooms)
  {
    if (!id_room.second.IsAlive())
    {
      continue;
    }
    ++alive_rooms;
    total_vertices += static_cast<int>(id_room.second.GetPolygon().polygon.points.size());
  }

  navgraph_ns::BuildingAxes cand;
  if (alive_rooms < kWarmupMinRooms_ || total_vertices < kWarmupMinVertices_ || !FitAxes(rooms, cand))
  {
    if (debug_log)
    {
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] warmup waiting: rooms=%d (need>=%d) verts=%d (need>=%d) fit_ok=%d",
                  alive_rooms, kWarmupMinRooms_, total_vertices, kWarmupMinVertices_,
                  static_cast<int>(cand.valid));
    }
    return;  // axes_ stays invalid -> no cross-lines, nodes grey
  }

  const double ang = std::atan2(cand.east.y(), cand.east.x());
  if (have_last_angle_ && AngleDiff(ang, last_angle_rad_) <= kFreezeAngleEpsRad_)
  {
    ++stable_count_;
  }
  else
  {
    stable_count_ = 0;
  }
  last_angle_rad_ = ang;
  have_last_angle_ = true;
  ++warmup_cycles_;

  if (stable_count_ >= kFreezeStableCycles_ || warmup_cycles_ >= kMaxWarmupCycles_)
  {
    axes_ = cand;  // latch (cand.valid == true); never refit after this
    frozen_ = true;
    RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                "[quadrant] axes FROZEN after %d cycles (stable=%d): east=(%.3f,%.3f) "
                "north=(%.3f,%.3f) angle=%.1f deg",
                warmup_cycles_, stable_count_, axes_.east.x(), axes_.east.y(), axes_.north.x(),
                axes_.north.y(), ang * 180.0 / kPi);
    PublishCrossLines(rooms);
  }
  else if (debug_log)
  {
    RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                "[quadrant] fitting: angle=%.1f deg stable=%d/%d warmup=%d/%d", ang * 180.0 / kPi,
                stable_count_, kFreezeStableCycles_, warmup_cycles_, kMaxWarmupCycles_);
  }
}

bool QuadrantManager::FitAxes(const std::map<int, representation_ns::RoomNodeRep>& rooms,
                              navgraph_ns::BuildingAxes& out) const
{
  std::vector<cv::Point2f> verts;
  for (const auto& id_room : rooms)
  {
    if (!id_room.second.IsAlive())
    {
      continue;
    }
    for (const auto& p : id_room.second.GetPolygon().polygon.points)
    {
      verts.emplace_back(p.x, p.y);
    }
  }
  if (verts.size() < 3)
  {
    return false;
  }

  const cv::RotatedRect rr = cv::minAreaRect(verts);
  if (rr.size.width < 1e-3f || rr.size.height < 1e-3f)
  {
    return false;  // collinear / degenerate -> no reliable orientation
  }

  // Use the rect CORNERS (boxPoints), not RotatedRect::angle: the angle range
  // flipped between OpenCV 4.4 and 4.5, but the corner geometry is stable.
  cv::Point2f box[4];
  rr.points(box);
  Eigen::Vector2d a(box[1].x - box[0].x, box[1].y - box[0].y);
  Eigen::Vector2d b(box[2].x - box[1].x, box[2].y - box[1].y);  // a perpendicular b
  if (a.norm() < 1e-6 || b.norm() < 1e-6)
  {
    return false;
  }
  a.normalize();
  b.normalize();
  // east = the rect side more aligned with map +X (|x| larger), flipped into the
  // +/-45 deg-of-+X half (e.x >= 0). This absorbs the 90-deg rect ambiguity:
  // a 90-deg rotation swaps which side is "more horizontal", and we re-select the
  // within-+/-45-deg one, yielding an identical east.
  Eigen::Vector2d e = (std::abs(a.x()) >= std::abs(b.x())) ? a : b;
  if (e.x() < 0.0)
  {
    e = -e;
  }
  out.east = e;
  out.north = Eigen::Vector2d(-e.y(), e.x());  // +90 deg CCW => north.y = e.x >= 0 ("up")
  out.valid = true;
  return true;
}

void QuadrantManager::PublishCrossLines(const std::map<int, representation_ns::RoomNodeRep>& rooms)
{
  if (!axes_.valid)
  {
    return;
  }
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_id_;
  marker.header.stamp = clock_->now();
  marker.ns = "quadrant_cross";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = kCrossLineWidth_;
  marker.color.a = 1.0;  // fallback; per-vertex colors below take precedence
  marker.pose.orientation.w = 1.0;

  std_msgs::msg::ColorRGBA east_color;
  east_color.r = 1.0;
  east_color.g = 0.0;
  east_color.b = 0.0;
  east_color.a = 1.0;  // east axis = red
  std_msgs::msg::ColorRGBA north_color;
  north_color.r = 0.1;
  north_color.g = 0.4;
  north_color.b = 1.0;
  north_color.a = 1.0;  // north axis = blue

  const Eigen::Vector2d e = axes_.east;
  const Eigen::Vector2d n = axes_.north;
  for (const auto& id_room : rooms)
  {
    const representation_ns::RoomNodeRep& room = id_room.second;
    if (!room.IsAlive())
    {
      continue;
    }
    const auto& pts = room.GetPolygon().polygon.points;
    if (pts.size() < 2)
    {
      continue;
    }
    const double cx = room.centroid_.x();
    const double cy = room.centroid_.y();
    const double cz = room.centroid_.z();
    double te_min = std::numeric_limits<double>::max();
    double te_max = std::numeric_limits<double>::lowest();
    double tn_min = std::numeric_limits<double>::max();
    double tn_max = std::numeric_limits<double>::lowest();
    for (const auto& p : pts)
    {
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      const double te = dx * e.x() + dy * e.y();
      const double tn = dx * n.x() + dy * n.y();
      te_min = std::min(te_min, te);
      te_max = std::max(te_max, te);
      tn_min = std::min(tn_min, tn);
      tn_max = std::max(tn_max, tn);
    }
    auto make_point = [cz](double x, double y) {
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = cz;
      return pt;
    };
    // East axis line: spans [te_min, te_max] along east, through the centroid.
    if (te_max - te_min > 1e-3)
    {
      marker.points.push_back(make_point(cx + te_min * e.x(), cy + te_min * e.y()));
      marker.points.push_back(make_point(cx + te_max * e.x(), cy + te_max * e.y()));
      marker.colors.push_back(east_color);
      marker.colors.push_back(east_color);
    }
    // North axis line.
    if (tn_max - tn_min > 1e-3)
    {
      marker.points.push_back(make_point(cx + tn_min * n.x(), cy + tn_min * n.y()));
      marker.points.push_back(make_point(cx + tn_max * n.x(), cy + tn_max * n.y()));
      marker.colors.push_back(north_color);
      marker.colors.push_back(north_color);
    }
  }
  cross_marker_pub_->publish(marker);
}
}  // namespace quadrant_ns
