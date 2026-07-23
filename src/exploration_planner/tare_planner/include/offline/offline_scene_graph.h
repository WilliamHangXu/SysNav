/**
 * @file offline_scene_graph.h
 * @brief The assembler side of the offline pipeline: loaders for one floor's
 *        offline_room_segmentation outputs (room_mask.png + mask_meta.json +
 *        rooms.json + doors.json), per-floor assembly + the multifloor merge,
 *        and the navgraph-over-rooms debug overlay.
 *
 * The output is the multifloor GADM-style JSON: one zones.floor_<M> entry per
 * floor and ONE metadata block (single building-wide compass, fit once from the
 * largest-footprint floor and shared by every floor's area tags). Ids are
 * floor-qualified so a multifloor building never collides:
 *   room_<M>_<N>            room N on floor M
 *   wp_<M>_<N>_<K>          waypoint K in that room (K=0 = interior point)
 *   entrance_<M>_<A>_<B>_<k> k-th door from room A to room B on floor M
 * The room label lives ONLY in the room's "type" field ("unknown" until the
 * labeling stage exists), so ids stay stable when labels arrive.
 *
 * All cross-layer relationships live HERE by contract: waypoint-in-room tagging
 * (mask lookup), waypoint naming, per-room 3x3 areas, the building-axes compass,
 * and the debug overlay (navgraph drawn over the room mask). Layer producers
 * (offline_room_segmentation.h, offline_navgraph.h) stay relationship-free.
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <navgraph/building_axes.h>  // Eigen-only, ROS-free (shared with online)

#include "offline/offline_navgraph.h"

namespace offline_scene_graph {

// ---- one floor's rooms-layer outputs -----------------------------------------

// room_mask.png + the pixel<->world transform from mask_meta.json. The mask is
// the bbox-CROPPED 16-bit label image in RAW grid orientation (row = world +x,
// col = world +y); 0 = explored background, pixel value = room id.
struct RoomMaskData {
    cv::Mat mask;  // CV_16U
    Eigen::Vector3f origin_shift = Eigen::Vector3f::Zero();  // full-grid voxel shift
    int bbox_row0 = 0, bbox_col0 = 0;  // crop offset in full-grid voxels
    float resolution = 0.1f;           // meters per pixel
    float robot_z = 0.0f;
    std::string frame;
    std::string floor_name;

    // Room id at world point p (mirrors the online NavGraph::TagRooms lookup,
    // adjusted for the cropped mask): 0 = explored background, -1 = outside.
    int RoomIdAt(const Eigen::Vector3d &p) const;
};

// One room from rooms.json. Coordinates are world-frame; z = robot_z.
struct RoomEntry {
    int id = 0;
    double area_m2 = 0.0;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d interior_point = Eigen::Vector3d::Zero();  // PIA, guaranteed inside
    std::vector<Eigen::Vector2d> polygon;  // world XY, outer contour
    std::vector<int> neighbors;            // door-connected room ids
};

// One door from doors.json.
struct DoorEntry {
    int id = 0;
    int door_id = 0;             // per-room-pair instance index
    int room_a = 0, room_b = 0;  // room_a < room_b
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    int pixel_count = 0;
};

struct FloorRoomData {
    RoomMaskData mask;
    std::vector<RoomEntry> rooms;  // ascending id
    std::vector<DoorEntry> doors;
};

// Load room_mask.png + mask_meta.json from a floor's segmentation output dir.
// Throws std::runtime_error on missing/malformed files.
RoomMaskData LoadRoomMask(const std::string &floor_dir);

// LoadRoomMask + rooms.json + doors.json.
FloorRoomData LoadFloorRoomData(const std::string &floor_dir);

// ---- assembly ----------------------------------------------------------------

struct AssemblerConfig {
    // Names the graph: top-level name/map_id/warehouse_id ("map" when empty).
    std::string building;
    std::string client_id;
    std::string uploaded_by;
    std::string units = "meters";
    double compass_radius_m = 0.0;       // <= 0 => auto (half the larger AABB extent)
    double center_fraction = 1.0 / 3.0;  // 3x3 grid center-band size
};

// "floor_2" -> 2; fallback when the name carries no trailing number.
int FloorLevelFromName(const std::string &name, int fallback);

// Global building axes from all room-polygon vertices via cv::minAreaRect,
// canonicalized exactly like QuadrantManager::FitAxes (east within +/-45 deg of
// map +X, north = east rotated +90 deg CCW). Invalid (default) on degenerate
// geometry -- areas then come out "unknown" and the compass is omitted.
navgraph_ns::BuildingAxes FitBuildingAxes(const std::vector<RoomEntry> &rooms);

// The single per-building compass: axes + the metadata slice derived from the
// fit floor (compass points, AABB dimensions, frame label). Fit ONCE per run
// from the largest-footprint floor; every floor's area tags use the same axes.
struct BuildingCompass {
    navgraph_ns::BuildingAxes axes;
    nlohmann::json compass;     // null on degenerate geometry (then omitted)
    nlohmann::json dimensions;  // {width, height} of the fit floor's AABB
    std::string frame;          // frame label carried into layout.metadata
};

// Fit the building compass from one floor's rooms. Compass point z = the
// floor's robot_z.
BuildingCompass FitBuildingCompass(const FloorRoomData &floor, double compass_radius_m);

// One floor assembled with floor-qualified ids: the rooms object destined for
// zones.floor_<level> plus this floor's slice of the flat waypoint/edge lists.
struct FloorAssembly {
    int level = 1;           // M in floor_<M> / room_<M>_<N> / wp_<M>_<N>_<K>
    std::string floor_name;  // source floor name, for logs
    nlohmann::json rooms;    // zones.floor_<M>.rooms object
    nlohmann::json waypoint_ids;  // array of this floor's waypoint ids
    nlohmann::json edges;         // array of {u, v, meters} between them
};

// Assemble one floor. Coordinates (including z) pass through unchanged --
// everything already shares the map frame. `axes` is the building-wide fit.
FloorAssembly BuildFloorAssembly(const FloorRoomData &floor, const NavGraphData &nav,
                                 const navgraph_ns::BuildingAxes &axes,
                                 const AssemblerConfig &cfg, int floor_level);

// Merge the assembled floors into the final scene-graph JSON: one
// zones.floor_<M> per floor, concatenated waypoint/edge lists, one metadata
// block (units/frame/building/floors/dimensions/compass). Throws on duplicate
// floor levels (zone keys would collide).
nlohmann::json BuildSceneGraph(const std::vector<FloorAssembly> &floors,
                               const BuildingCompass &compass,
                               const AssemblerConfig &cfg);

// ---- debug overlay -----------------------------------------------------------

// Draw the navgraph over a floor's room_mask_vis.png ->
// <out_dir>/debug/navgraph_overlay.png, in the established debug orientation
// (transpose + vertical flip). Untagged nodes draw grey so frame or coverage
// problems jump out. Returns the number of nodes whose position lands inside a
// room (RoomIdAt > 0) -- the frame-consistency coverage figure. Throws
// std::runtime_error if the vis image can't be read.
int SaveNavGraphOverlay(const NavGraphData &nav, const RoomMaskData &mask,
                        const std::string &vis_png_path, const std::string &out_dir);

}  // namespace offline_scene_graph
