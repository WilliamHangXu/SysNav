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
  , kWarmupMinRooms_(1)
  , kWarmupMinVertices_(8)
  , kFreezeStableCycles_(5)
  , kMaxWarmupCycles_(60)
  , kFreezeAngleEpsRad_(2.0 * kPi / 180.0)
  , kCrossLineWidth_(0.08)
  , kWallMinConfidence_(0.5)
  , kWallMinSupportM_(3.0)
  , kCenterFraction_(1.0 / 3.0)
{
  ReadParameters(nh);
  clock_ = nh->get_clock();
  cross_marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("quadrant/cross_marker", 2);
  // Primary axis source: the dominant wall orientation from room_segmentation.
  // Single-threaded executor (this callback + Update both on the planner node) =>
  // no mutex needed to store the latest measurement.
  wall_axis_sub_ = nh->create_subscription<tare_planner::msg::WallAxis>(
      "/wall_axis", 5, [this](const tare_planner::msg::WallAxis::SharedPtr m) {
        wall_axis_.yaw_rad = m->yaw_rad;
        wall_axis_.confidence = m->confidence;
        wall_axis_.support_length = m->support_length;
        wall_axis_.valid = m->valid;
      });
}

void QuadrantManager::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("quadrant/kUpdateInterval", kUpdateInterval_);
  nh->get_parameter("quadrant/kWarmupMinRooms", kWarmupMinRooms_);
  nh->get_parameter("quadrant/kWarmupMinVertices", kWarmupMinVertices_);
  nh->get_parameter("quadrant/kFreezeStableCycles", kFreezeStableCycles_);
  nh->get_parameter("quadrant/kMaxWarmupCycles", kMaxWarmupCycles_);
  nh->get_parameter("quadrant/kCrossLineWidth", kCrossLineWidth_);
  nh->get_parameter("quadrant/kWallMinConfidence", kWallMinConfidence_);
  nh->get_parameter("quadrant/kWallMinSupportM", kWallMinSupportM_);
  nh->get_parameter("quadrant/kCenterFraction", kCenterFraction_);
  nh->get_parameter("quadrant/world_frame_id", world_frame_id_);
  double eps_deg = kFreezeAngleEpsRad_ * 180.0 / kPi;
  nh->get_parameter("quadrant/kFreezeAngleEpsDeg", eps_deg);
  kFreezeAngleEpsRad_ = eps_deg * kPi / 180.0;
  if (kUpdateInterval_ < 1)
  {
    kUpdateInterval_ = 1;
  }
}

navgraph_ns::BuildingAxes QuadrantManager::AxesFromAngle(double theta) const
{
  navgraph_ns::BuildingAxes out;
  // Reduce to (-pi/4, pi/4] so east is within +/-45 deg of map +X. std::remainder
  // rounds half-to-even, sending exactly +/-45 deg to +45 deg deterministically.
  const double tp = std::remainder(theta, kPi / 2.0);
  out.east = Eigen::Vector2d(std::cos(tp), std::sin(tp));    // east.x = cos(tp) > 0
  out.north = Eigen::Vector2d(-std::sin(tp), std::cos(tp));  // north.y = cos(tp) > 0 ("up")
  out.valid = true;
  return out;
}

void QuadrantManager::Update(const std::map<int, representation_ns::RoomNodeRep>& rooms, bool debug_log)
{
  // Self-throttle (rooms/walls evolve slowly; fitting + viz need not run every cycle).
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

  // Geometry presence (>=1 alive room with enough polygon vertices). This is NOT a
  // hard gate on the freeze: the wall-stability path below runs on a confident wall
  // axis ALONE, so the building orientation can freeze BEFORE any room is segmented
  // (the cross-lines/areas just render once rooms appear). Geometry is only required
  // for (a) the min-rect cap fallback, which needs room polygons, and (b) as an
  // alternative cap-timer driver so the timeout still fires when there is no wall.
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
  const bool geometry_ok = (alive_rooms >= kWarmupMinRooms_ && total_vertices >= kWarmupMinVertices_);

  // PRIMARY source: the latest /wall_axis, when confident enough to trust.
  const WallAxisState wa = wall_axis_;
  const bool wall_ok =
      wa.valid && wa.confidence >= kWallMinConfidence_ && wa.support_length >= kWallMinSupportM_;

  // Nothing usable yet (no trustworthy wall AND no geometry) -> wait. warmup_cycles_
  // (the cap timer) only advances once we have at least one source, so the hard cap
  // is a real timeout for both the wall and the min-rect paths.
  if (!wall_ok && !geometry_ok)
  {
    if (debug_log)
    {
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] waiting: no confident wall axis and no geometry yet (rooms=%d verts=%d)",
                  alive_rooms, total_vertices);
    }
    return;  // pre-freeze: axes_ invalid -> nothing shown (grey nodes, no cross-lines)
  }
  ++warmup_cycles_;
  if (wall_ok)
  {
    if (have_last_angle_ && AngleDiff(wa.yaw_rad, last_angle_rad_) <= kFreezeAngleEpsRad_)
    {
      ++stable_count_;
    }
    else
    {
      stable_count_ = 0;
    }
    last_angle_rad_ = wa.yaw_rad;
    have_last_angle_ = true;
    if (stable_count_ >= kFreezeStableCycles_)
    {
      axes_ = AxesFromAngle(wa.yaw_rad);
      frozen_ = true;
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] axes FROZEN source=wall after %d cycles (stable=%d): yaw=%.1f deg "
                  "conf=%.2f support=%.2f m | east=(%.3f,%.3f) north=(%.3f,%.3f)",
                  warmup_cycles_, stable_count_, wa.yaw_rad * 180.0 / kPi, wa.confidence,
                  wa.support_length, axes_.east.x(), axes_.east.y(), axes_.north.x(),
                  axes_.north.y());
      PublishCrossLines(rooms);
      return;
    }
  }
  else
  {
    // Not trustworthy this cycle -> break the stability streak. Freezing requires a
    // CONTIGUOUS confident streak, so a flickering wall axis can never accumulate one.
    stable_count_ = 0;
    have_last_angle_ = false;
  }

  // HARD CAP: time's up. This is the ONLY place the min-rect fallback is used.
  if (warmup_cycles_ >= kMaxWarmupCycles_)
  {
    if (wall_ok)
    {
      axes_ = AxesFromAngle(wa.yaw_rad);  // confident wall now wins over min-rect
      frozen_ = true;
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] axes FROZEN source=wall_cap after %d cycles: yaw=%.1f deg conf=%.2f",
                  warmup_cycles_, wa.yaw_rad * 180.0 / kPi, wa.confidence);
      PublishCrossLines(rooms);
    }
    else if (geometry_ok && FitAxes(rooms, axes_))  // min-rect fallback (needs room polygons)
    {
      const double ang = std::atan2(axes_.east.y(), axes_.east.x());
      frozen_ = true;
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] axes FROZEN source=minrect_cap after %d cycles (no confident wall): "
                  "angle=%.1f deg | east=(%.3f,%.3f)",
                  warmup_cycles_, ang * 180.0 / kPi, axes_.east.x(), axes_.east.y());
      PublishCrossLines(rooms);
    }
    else if (debug_log)
    {
      RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                  "[quadrant] cap reached but no axis source yet (rooms=%d) -- retrying next cycle",
                  alive_rooms);
    }
    return;
  }

  if (debug_log)
  {
    RCLCPP_INFO(rclcpp::get_logger("quadrant"),
                "[quadrant] fitting: wall_ok=%d geom=%d yaw=%.1f deg conf=%.2f stable=%d/%d warmup=%d/%d",
                static_cast<int>(wall_ok), static_cast<int>(geometry_ok), wa.yaw_rad * 180.0 / kPi,
                wa.confidence, stable_count_, kFreezeStableCycles_, warmup_cycles_, kMaxWarmupCycles_);
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

std::map<int, navgraph_ns::RoomGrid> QuadrantManager::BuildRoomGrids(
    const std::map<int, representation_ns::RoomNodeRep>& rooms) const
{
  std::map<int, navgraph_ns::RoomGrid> grids;
  for (const auto& id_room : rooms)
  {
    const representation_ns::RoomNodeRep& room = id_room.second;
    if (!room.IsAlive())
    {
      continue;
    }
    const double cx = room.centroid_.x();
    const double cy = room.centroid_.y();
    // Oriented bbox extents = polygon vertices projected onto east/north, relative
    // to the centroid (the grid origin). MakeRoomGrid returns an invalid grid if
    // axes_ is not frozen or the extent is degenerate (< 2 vertices).
    double e_min = std::numeric_limits<double>::max();
    double e_max = std::numeric_limits<double>::lowest();
    double n_min = std::numeric_limits<double>::max();
    double n_max = std::numeric_limits<double>::lowest();
    for (const auto& p : room.GetPolygon().polygon.points)
    {
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      e_min = std::min(e_min, dx * axes_.east.x() + dy * axes_.east.y());
      e_max = std::max(e_max, dx * axes_.east.x() + dy * axes_.east.y());
      n_min = std::min(n_min, dx * axes_.north.x() + dy * axes_.north.y());
      n_max = std::max(n_max, dx * axes_.north.x() + dy * axes_.north.y());
    }
    grids[room.GetId()] = navgraph_ns::MakeRoomGrid(Eigen::Vector2d(cx, cy), axes_, e_min, e_max,
                                                    n_min, n_max, kCenterFraction_);
  }
  return grids;
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

  std_msgs::msg::ColorRGBA east_color;  // separators running ALONG east (the n_lo/n_hi cuts)
  east_color.r = 1.0;
  east_color.g = 0.0;
  east_color.b = 0.0;
  east_color.a = 1.0;  // red
  std_msgs::msg::ColorRGBA north_color;  // separators running ALONG north (the e_lo/e_hi cuts)
  north_color.r = 0.1;
  north_color.g = 0.4;
  north_color.b = 1.0;
  north_color.a = 1.0;  // blue

  // "#" overhang: each separator extends this fraction of the center-cell size past
  // the cell corners, so the four lines read as a # rather than a closed box.
  constexpr double kOverhangFrac = 0.45;

  const Eigen::Vector2d e = axes_.east;
  const Eigen::Vector2d n = axes_.north;
  const std::map<int, navgraph_ns::RoomGrid> grids = BuildRoomGrids(rooms);
  for (const auto& id_room : rooms)
  {
    const representation_ns::RoomNodeRep& room = id_room.second;
    if (!room.IsAlive())
    {
      continue;
    }
    const auto git = grids.find(room.GetId());
    if (git == grids.end() || !git->second.valid)
    {
      continue;
    }
    const navgraph_ns::RoomGrid& g = git->second;
    const double cx = room.centroid_.x();
    const double cy = room.centroid_.y();
    const double cz = room.centroid_.z();
    const double oh_e = kOverhangFrac * (g.e_hi - g.e_lo);
    const double oh_n = kOverhangFrac * (g.n_hi - g.n_lo);
    // (e,n) in the axes frame (origin = centroid) -> world point at the centroid z.
    auto pt = [&](double pe, double pn) {
      geometry_msgs::msg::Point p;
      p.x = cx + pe * e.x() + pn * n.x();
      p.y = cy + pe * e.y() + pn * n.y();
      p.z = cz;
      return p;
    };
    // Two separators running along NORTH (constant e = e_lo / e_hi), short with
    // overhang in n -> the two vertical strokes of the #. Color: north (blue).
    marker.points.push_back(pt(g.e_lo, g.n_lo - oh_n));
    marker.points.push_back(pt(g.e_lo, g.n_hi + oh_n));
    marker.points.push_back(pt(g.e_hi, g.n_lo - oh_n));
    marker.points.push_back(pt(g.e_hi, g.n_hi + oh_n));
    // Two separators running along EAST (constant n = n_lo / n_hi) -> the two
    // horizontal strokes. Color: east (red).
    marker.points.push_back(pt(g.e_lo - oh_e, g.n_lo));
    marker.points.push_back(pt(g.e_hi + oh_e, g.n_lo));
    marker.points.push_back(pt(g.e_lo - oh_e, g.n_hi));
    marker.points.push_back(pt(g.e_hi + oh_e, g.n_hi));
    for (int k = 0; k < 4; ++k)
    {
      marker.colors.push_back(north_color);  // first 2 segments (4 verts) = north strokes
    }
    for (int k = 0; k < 4; ++k)
    {
      marker.colors.push_back(east_color);  // next 2 segments (4 verts) = east strokes
    }
  }
  cross_marker_pub_->publish(marker);
}
}  // namespace quadrant_ns
