//
// Building axes + room "area" (3x3 grid cell) primitives. Lowest-level, shared
// header: Eigen-only, ROS-free, OpenCV-free, so the ROS-free NavNode
// (navgraph_types.h) and the rclcpp/PCL-free scene-graph exporter can both use it.
//
// A single GLOBAL building orientation (two orthonormal axes in the source/map
// frame) is fit elsewhere (QuadrantManager, primarily from /wall_axis) and frozen
// once stable. Each room is divided into a 3x3 grid aligned to those axes: the
// room's oriented bounding box is split into thirds along each axis, and a point
// is bucketed into one of nine cells -- the center cell plus the eight compass
// directions. The grid geometry (per-room band boundaries) is built into a
// RoomGrid by the caller (which has the room polygon); this header only DEFINES
// the data and the pure assignment, so OpenCV/ROS-free consumers can reuse it.
//

#ifndef NAVGRAPH_BUILDING_AXES_H
#define NAVGRAPH_BUILDING_AXES_H

#include <Eigen/Core>

namespace navgraph_ns
{
// Which of a room's 3x3 grid cells a point falls in. kCenter is the middle cell;
// the other eight are compass directions. kUnknown = axes/grid not ready, or no
// room.
enum class Area
{
  kUnknown = -1,
  kCenter,     // 0
  kNorth,      // 1
  kNorthEast,  // 2
  kEast,       // 3
  kSouthEast,  // 4
  kSouth,      // 5
  kSouthWest,  // 6
  kWest,       // 7
  kNorthWest   // 8
};

// Canonical string form, emitted verbatim into the scene-graph JSON and used in
// the RViz labels, so the two can never drift.
inline const char* AreaName(Area area)
{
  switch (area)
  {
    case Area::kCenter:
      return "center";
    case Area::kNorth:
      return "north";
    case Area::kNorthEast:
      return "northeast";
    case Area::kEast:
      return "east";
    case Area::kSouthEast:
      return "southeast";
    case Area::kSouth:
      return "south";
    case Area::kSouthWest:
      return "southwest";
    case Area::kWest:
      return "west";
    case Area::kNorthWest:
      return "northwest";
    default:
      return "unknown";
  }
}

// The global building axes in the source/map frame (XY only; the stack is
// gravity-aligned). `east` is canonicalized to within +/-45 deg of map +X and
// `north` = east rotated +90 deg CCW (so north.y > 0, "up"). `valid` is false
// until the freeze completes; a default-constructed instance is a safe "not ready"
// sentinel.
struct BuildingAxes
{
  Eigen::Vector2d east{ 1.0, 0.0 };
  Eigen::Vector2d north{ 0.0, 1.0 };
  bool valid = false;
};

// A room's 3x3 grid: the global axes, the projection origin (the room centroid),
// and the two third-boundaries on each axis (origin-relative, in the axes frame).
// `valid` is false until both the axes are frozen and the room has a real extent.
struct RoomGrid
{
  Eigen::Vector2d origin{ 0.0, 0.0 };
  BuildingAxes axes;
  double e_lo = 0.0, e_hi = 0.0;  // east-axis low/high third boundaries
  double n_lo = 0.0, n_hi = 0.0;  // north-axis low/high third boundaries
  bool valid = false;
};

// Build a room's 3x3 grid from its oriented-bounding-box extents (the min/max of
// its polygon vertices projected onto east/north, relative to `origin`). The
// center band spans `center_fraction` of each extent, centered on the extent
// midpoint (center_fraction = 1/3 => equal thirds). Invalid axes or a degenerate
// extent => an invalid grid (AssignArea then returns kUnknown).
inline RoomGrid MakeRoomGrid(const Eigen::Vector2d& origin, const BuildingAxes& axes, double e_min,
                             double e_max, double n_min, double n_max, double center_fraction)
{
  RoomGrid g;
  g.origin = origin;
  g.axes = axes;
  if (!axes.valid || e_max <= e_min || n_max <= n_min)
  {
    g.valid = false;
    return g;
  }
  const double lo_f = 0.5 - 0.5 * center_fraction;
  const double hi_f = 0.5 + 0.5 * center_fraction;
  g.e_lo = e_min + lo_f * (e_max - e_min);
  g.e_hi = e_min + hi_f * (e_max - e_min);
  g.n_lo = n_min + lo_f * (n_max - n_min);
  g.n_hi = n_min + hi_f * (n_max - n_min);
  g.valid = true;
  return g;
}

// Bucket `p` into one of the 9 cells of the room's 3x3 grid: project (p - origin)
// onto the axes, classify each axis into low / middle / high by the grid's
// third-boundaries, then map the pair to a compass cell (middle,middle = center).
// Boundary points fall into the middle (center-ward) band. Only XY is used.
inline Area AssignArea(const Eigen::Vector2d& p, const RoomGrid& grid)
{
  if (!grid.valid || !grid.axes.valid)
  {
    return Area::kUnknown;
  }
  const Eigen::Vector2d d = p - grid.origin;
  const double e = d.dot(grid.axes.east);
  const double n = d.dot(grid.axes.north);
  const int eb = (e < grid.e_lo) ? -1 : (e > grid.e_hi) ? 1 : 0;  // west / mid / east
  const int nb = (n < grid.n_lo) ? -1 : (n > grid.n_hi) ? 1 : 0;  // south / mid / north
  if (nb > 0)
  {
    return (eb < 0) ? Area::kNorthWest : (eb > 0) ? Area::kNorthEast : Area::kNorth;
  }
  if (nb < 0)
  {
    return (eb < 0) ? Area::kSouthWest : (eb > 0) ? Area::kSouthEast : Area::kSouth;
  }
  return (eb < 0) ? Area::kWest : (eb > 0) ? Area::kEast : Area::kCenter;
}
}  // namespace navgraph_ns

#endif  // NAVGRAPH_BUILDING_AXES_H
