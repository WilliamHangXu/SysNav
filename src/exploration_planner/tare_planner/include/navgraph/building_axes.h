//
// Building axes + room "area" (quadrant) primitives. Lowest-level, shared header:
// Eigen-only, ROS-free, OpenCV-free, so the ROS-free NavNode (navgraph_types.h)
// and the rclcpp/PCL-free scene-graph exporter can both use it.
//
// A single GLOBAL building orientation (two orthonormal axes in the source/map
// frame) is fit elsewhere (QuadrantManager, via cv::minAreaRect) and frozen once
// stable. Given that orientation and a room's centroid as origin, AssignArea()
// buckets any point into one of four quadrants (NE/NW/SE/SW). The fitting itself
// lives where OpenCV + room geometry are available; this header only DEFINES the
// axes/area data and the (pure, branchless) assignment, so consumers that must
// stay free of OpenCV/ROS can reuse it.
//

#ifndef NAVGRAPH_BUILDING_AXES_H
#define NAVGRAPH_BUILDING_AXES_H

#include <Eigen/Core>

namespace navgraph_ns
{
// Which room quadrant a point falls in, relative to the room's centroid and the
// global building axes. kUnknown = axes not yet frozen, or no room.
enum class Area
{
  kUnknown = -1,
  kNorthEast,
  kNorthWest,
  kSouthEast,
  kSouthWest
};

// Canonical string form, emitted verbatim into the scene-graph JSON and used in
// the RViz labels, so the two can never drift.
inline const char* AreaName(Area area)
{
  switch (area)
  {
    case Area::kNorthEast:
      return "northeast";
    case Area::kNorthWest:
      return "northwest";
    case Area::kSouthEast:
      return "southeast";
    case Area::kSouthWest:
      return "southwest";
    default:
      return "unknown";
  }
}

// The global building axes in the source/map frame (XY only; the stack is
// gravity-aligned). `east` is canonicalized to within +/-45 deg of map +X and
// `north` = east rotated +90 deg CCW (so north.y > 0, "up"). `valid` is false
// until the warmup freeze completes; a default-constructed instance is therefore
// a safe "not ready" sentinel that makes AssignArea return kUnknown.
struct BuildingAxes
{
  Eigen::Vector2d east{ 1.0, 0.0 };
  Eigen::Vector2d north{ 0.0, 1.0 };
  bool valid = false;
};

// Bucket `p` relative to `origin` (the room centroid) by the SIGN of the
// projections onto the building axes. Tiebreak: a projection of exactly 0 counts
// as the positive (east / north) side, so the assignment is deterministic. Only
// XY is used; pass the x,y of 3D positions.
inline Area AssignArea(const Eigen::Vector2d& p, const Eigen::Vector2d& origin, const BuildingAxes& axes)
{
  if (!axes.valid)
  {
    return Area::kUnknown;
  }
  const Eigen::Vector2d d = p - origin;
  const bool east_pos = d.dot(axes.east) >= 0.0;
  const bool north_pos = d.dot(axes.north) >= 0.0;
  if (north_pos)
  {
    return east_pos ? Area::kNorthEast : Area::kNorthWest;
  }
  return east_pos ? Area::kSouthEast : Area::kSouthWest;
}
}  // namespace navgraph_ns

#endif  // NAVGRAPH_BUILDING_AXES_H
