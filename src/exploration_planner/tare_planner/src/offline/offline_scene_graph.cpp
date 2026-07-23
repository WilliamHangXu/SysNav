/**
 * @file offline_scene_graph.cpp
 * @brief Assembler-side implementation (see offline_scene_graph.h): rooms-layer
 *        file parsing, per-floor assembly + the multifloor merge (JSON shape
 *        follows the online SceneGraphExporter::Build/BuildRoomJson port, with
 *        floor-qualified ids and one zones.floor_<M> per floor; the axes fit
 *        ports QuadrantManager::FitAxes/BuildRoomGrids), and the
 *        navgraph-over-rooms debug overlay.
 */

#include "offline/offline_scene_graph.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace offline_scene_graph {

using json = nlohmann::json;

// ---- rooms-layer file parsing --------------------------------------------------

namespace {

json LoadJsonFile(const std::string &path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    try {
        return json::parse(in);
    } catch (const json::parse_error &e) {
        throw std::runtime_error(path + ": " + e.what());
    }
}

Eigen::Vector3d ToVector3d(const json &arr)
{
    if (!arr.is_array() || arr.size() != 3) {
        throw std::runtime_error("expected [x,y,z] array in rooms/doors json");
    }
    return { arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>() };
}

}  // namespace

int RoomMaskData::RoomIdAt(const Eigen::Vector3d &p) const
{
    // Same voxelization as the online tagger (misc_utils point_to_voxel: scale,
    // shift, floor), then shifted into the cropped mask.
    const float inv = 1.0f / resolution;
    const int row = static_cast<int>(std::floor(static_cast<float>(p.x()) * inv +
                                                origin_shift.x())) - bbox_row0;
    const int col = static_cast<int>(std::floor(static_cast<float>(p.y()) * inv +
                                                origin_shift.y())) - bbox_col0;
    if (mask.empty() || row < 0 || row >= mask.rows || col < 0 || col >= mask.cols) {
        return -1;
    }
    return static_cast<int>(mask.at<uint16_t>(row, col));
}

RoomMaskData LoadRoomMask(const std::string &floor_dir)
{
    RoomMaskData data;

    const json meta = LoadJsonFile(floor_dir + "/mask_meta.json");
    data.frame = meta.value("frame", std::string());
    data.floor_name = meta.value("floor", std::string());
    data.robot_z = meta.value("robot_z", 0.0f);
    data.resolution = meta.value("resolution", 0.1f);
    const json &shift = meta.at("origin_shift");
    data.origin_shift = Eigen::Vector3f(shift[0].get<float>(), shift[1].get<float>(),
                                        shift[2].get<float>());
    data.bbox_row0 = meta.at("bbox").at("row")[0].get<int>();
    data.bbox_col0 = meta.at("bbox").at("col")[0].get<int>();

    const std::string mask_path = floor_dir + "/room_mask.png";
    data.mask = cv::imread(mask_path, cv::IMREAD_UNCHANGED);
    if (data.mask.empty()) {
        throw std::runtime_error("cannot read " + mask_path);
    }
    if (data.mask.type() != CV_16U) {
        throw std::runtime_error(mask_path + ": expected 16-bit label image, got type " +
                                 std::to_string(data.mask.type()));
    }
    return data;
}

FloorRoomData LoadFloorRoomData(const std::string &floor_dir)
{
    FloorRoomData data;
    data.mask = LoadRoomMask(floor_dir);

    const json rooms_root = LoadJsonFile(floor_dir + "/rooms.json");
    for (const json &r : rooms_root.at("rooms")) {
        RoomEntry room;
        room.id = r.at("id").get<int>();
        room.area_m2 = r.at("area_m2").get<double>();
        room.centroid = ToVector3d(r.at("centroid"));
        room.interior_point = ToVector3d(r.at("interior_point"));
        for (const json &v : r.at("polygon")) {
            room.polygon.emplace_back(v[0].get<double>(), v[1].get<double>());
        }
        for (const json &nb : r.at("neighbors")) {
            room.neighbors.push_back(nb.get<int>());
        }
        data.rooms.push_back(std::move(room));
    }
    std::sort(data.rooms.begin(), data.rooms.end(),
              [](const RoomEntry &a, const RoomEntry &b) { return a.id < b.id; });

    const json doors_root = LoadJsonFile(floor_dir + "/doors.json");
    for (const json &d : doors_root.at("doors")) {
        DoorEntry door;
        door.id = d.at("id").get<int>();
        door.door_id = d.value("door_id", 0);
        door.room_a = d.at("rooms")[0].get<int>();
        door.room_b = d.at("rooms")[1].get<int>();
        if (door.room_a > door.room_b) {
            std::swap(door.room_a, door.room_b);
        }
        door.centroid = ToVector3d(d.at("centroid"));
        door.pixel_count = d.value("pixel_count", 0);
        data.doors.push_back(door);
    }

    std::printf("[room_io] %s: %zu rooms, %zu doors, mask %dx%d @ %.2f m/px\n",
                floor_dir.c_str(), data.rooms.size(), data.doors.size(),
                data.mask.mask.rows, data.mask.mask.cols, data.mask.resolution);
    return data;
}

// ---- assembly ------------------------------------------------------------------

namespace {

// Label used while the offline room-labeling stage doesn't exist. Lives ONLY in
// the room's "type" field -- ids below are label-free so they never change when
// labels arrive.
const char *kUnknownLabel = "unknown";

// Floor-qualified ids (see offline_scene_graph.h): unique across a whole
// multifloor building, since room ids restart at 1 on every floor.
std::string RoomKey(int level, int room_id)
{
    return "room_" + std::to_string(level) + "_" + std::to_string(room_id);
}

std::string WaypointKey(int level, int room_id, int wp_index)
{
    return "wp_" + std::to_string(level) + "_" + std::to_string(room_id) + "_" +
           std::to_string(wp_index);
}

std::string EntranceKey(int level, int room_a, int room_b, int pair_index)
{
    return "entrance_" + std::to_string(level) + "_" + std::to_string(room_a) + "_" +
           std::to_string(room_b) + "_" + std::to_string(pair_index);
}

// Per-room 3x3 grid from its polygon's oriented-bbox extents (port of
// QuadrantManager::BuildRoomGrids, minus the room lifecycle).
std::map<int, navgraph_ns::RoomGrid> BuildRoomGrids(const std::vector<RoomEntry> &rooms,
                                                    const navgraph_ns::BuildingAxes &axes,
                                                    double center_fraction)
{
    std::map<int, navgraph_ns::RoomGrid> grids;
    for (const RoomEntry &room : rooms) {
        double e_min = std::numeric_limits<double>::max();
        double e_max = std::numeric_limits<double>::lowest();
        double n_min = std::numeric_limits<double>::max();
        double n_max = std::numeric_limits<double>::lowest();
        for (const Eigen::Vector2d &p : room.polygon) {
            const Eigen::Vector2d d = p - room.centroid.head<2>();
            e_min = std::min(e_min, d.dot(axes.east));
            e_max = std::max(e_max, d.dot(axes.east));
            n_min = std::min(n_min, d.dot(axes.north));
            n_max = std::max(n_max, d.dot(axes.north));
        }
        grids[room.id] = navgraph_ns::MakeRoomGrid(room.centroid.head<2>(), axes, e_min,
                                                   e_max, n_min, n_max, center_fraction);
    }
    return grids;
}

}  // namespace

int FloorLevelFromName(const std::string &name, int fallback)
{
    size_t end = name.size();
    while (end > 0 && std::isdigit(static_cast<unsigned char>(name[end - 1]))) {
        --end;
    }
    if (end == name.size()) {
        return fallback;
    }
    return std::stoi(name.substr(end));
}

navgraph_ns::BuildingAxes FitBuildingAxes(const std::vector<RoomEntry> &rooms)
{
    navgraph_ns::BuildingAxes axes;  // default: invalid

    std::vector<cv::Point2f> verts;
    for (const RoomEntry &room : rooms) {
        for (const Eigen::Vector2d &p : room.polygon) {
            verts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
        }
    }
    if (verts.size() < 3) {
        return axes;
    }
    const cv::RotatedRect rr = cv::minAreaRect(verts);
    if (rr.size.width < 1e-3f || rr.size.height < 1e-3f) {
        return axes;  // collinear / degenerate
    }

    // Corners (boxPoints), not RotatedRect::angle: the angle convention flipped
    // between OpenCV versions, the corner geometry did not.
    cv::Point2f box[4];
    rr.points(box);
    Eigen::Vector2d a(box[1].x - box[0].x, box[1].y - box[0].y);
    Eigen::Vector2d b(box[2].x - box[1].x, box[2].y - box[1].y);
    if (a.norm() < 1e-6 || b.norm() < 1e-6) {
        return axes;
    }
    a.normalize();
    b.normalize();
    // east = the rect side more aligned with map +X, flipped into the
    // +/-45-deg-of-+X half; north = +90 deg CCW. Same canonicalization as the
    // online fit, so per-run areas agree with what a live run would tag.
    Eigen::Vector2d e = (std::abs(a.x()) >= std::abs(b.x())) ? a : b;
    if (e.x() < 0.0) {
        e = -e;
    }
    axes.east = e;
    axes.north = Eigen::Vector2d(-e.y(), e.x());
    axes.valid = true;
    return axes;
}

BuildingCompass FitBuildingCompass(const FloorRoomData &floor, double compass_radius_m)
{
    BuildingCompass bc;
    bc.frame = floor.mask.frame;
    bc.axes = FitBuildingAxes(floor.rooms);

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    bool any_vertex = false;
    for (const RoomEntry &room : floor.rooms) {
        for (const Eigen::Vector2d &p : room.polygon) {
            any_vertex = true;
            min_x = std::min(min_x, p.x());
            min_y = std::min(min_y, p.y());
            max_x = std::max(max_x, p.x());
            max_y = std::max(max_y, p.y());
        }
    }
    bc.dimensions = { { "width", any_vertex ? (max_x - min_x) : 0.0 },
                      { "height", any_vertex ? (max_y - min_y) : 0.0 } };
    if (bc.axes.valid && any_vertex) {
        const Eigen::Vector3d center(0.5 * (min_x + max_x), 0.5 * (min_y + max_y),
                                     floor.mask.robot_z);
        const double radius = (compass_radius_m > 0.0)
                                  ? compass_radius_m
                                  : 0.5 * std::max(max_x - min_x, max_y - min_y);
        const Eigen::Vector3d e(bc.axes.east.x(), bc.axes.east.y(), 0.0);
        const Eigen::Vector3d n(bc.axes.north.x(), bc.axes.north.y(), 0.0);
        const auto pt = [](const Eigen::Vector3d &p) {
            return json{ { "x", p.x() }, { "y", p.y() }, { "z", p.z() } };
        };
        bc.compass = {
            { "center", pt(center) },
            { "north", pt(center + radius * n) },
            { "south", pt(center - radius * n) },
            { "east", pt(center + radius * e) },
            { "west", pt(center - radius * e) },
        };
    }
    return bc;
}

FloorAssembly BuildFloorAssembly(const FloorRoomData &floor, const NavGraphData &nav,
                                 const navgraph_ns::BuildingAxes &axes,
                                 const AssemblerConfig &cfg, int floor_level)
{
    FloorAssembly out;
    out.level = floor_level;
    out.floor_name = floor.mask.floor_name;

    const std::map<int, navgraph_ns::RoomGrid> grids =
        BuildRoomGrids(floor.rooms, axes, cfg.center_fraction);

    // --- cross-layer tagging: nav node -> room id (mask lookup) --------------
    std::map<int, std::vector<const NavGraphNode *>> nodes_by_room;  // ascending id
    int untagged = 0;
    for (const NavGraphNode &node : nav.nodes) {
        const int room_id = floor.mask.RoomIdAt(node.position);
        if (room_id > 0) {
            nodes_by_room[room_id].push_back(&node);
        } else {
            ++untagged;  // background/outside; not emitted (edges drop below)
        }
    }

    json rooms_json = json::object();
    json waypoint_ids = json::array();
    std::map<int, std::string> nav_id_to_wpid;

    std::map<int, const RoomEntry *> rooms_by_id;
    for (const RoomEntry &room : floor.rooms) {
        rooms_by_id[room.id] = &room;
    }

    int entrances_total = 0;
    for (const RoomEntry &room : floor.rooms) {
        const std::string room_key = RoomKey(floor_level, room.id);

        // --- entrances: one per door touching this room, indexed per neighbor
        //     pair (a double doorway to the same neighbor gets k = 1, 2, ...) --
        json entrances = json::array();
        std::map<int, int> pair_count;
        for (const DoorEntry &door : floor.doors) {
            if (door.room_a != room.id && door.room_b != room.id) {
                continue;
            }
            const int neighbor_id = (door.room_a == room.id) ? door.room_b : door.room_a;
            if (rooms_by_id.find(neighbor_id) == rooms_by_id.end()) {
                continue;
            }
            const int k = ++pair_count[neighbor_id];
            ++entrances_total;
            entrances.push_back(json{
                { "id", EntranceKey(floor_level, room.id, neighbor_id, k) },
                { "connected_to", RoomKey(floor_level, neighbor_id) },
                { "x", door.centroid.x() },
                { "y", door.centroid.y() },
                { "z", door.centroid.z() },
            });
        }

        // --- waypoints: wp_M_N_0 = interior point, then the room's nav nodes --
        const navgraph_ns::RoomGrid grid =
            grids.count(room.id) ? grids.at(room.id) : navgraph_ns::RoomGrid{};
        json waypoints = json::array();
        const navgraph_ns::Area wp0_area =
            navgraph_ns::AssignArea(room.interior_point.head<2>(), grid);
        waypoints.push_back(json{
            { "id", WaypointKey(floor_level, room.id, 0) },
            { "x", room.interior_point.x() },
            { "y", room.interior_point.y() },
            { "z", room.interior_point.z() },
            { "area", navgraph_ns::AreaName(wp0_area) },
        });
        int wp_index = 1;
        const auto nodes_it = nodes_by_room.find(room.id);
        if (nodes_it != nodes_by_room.end()) {
            for (const NavGraphNode *node : nodes_it->second) {
                const std::string wp_id = WaypointKey(floor_level, room.id, wp_index);
                const navgraph_ns::Area area =
                    navgraph_ns::AssignArea(node->position.head<2>(), grid);
                waypoints.push_back(json{
                    { "id", wp_id },
                    { "x", node->position.x() },
                    { "y", node->position.y() },
                    { "z", node->position.z() },
                    { "area", navgraph_ns::AreaName(area) },
                });
                nav_id_to_wpid[node->id] = wp_id;
                ++wp_index;
            }
        }

        for (const json &wp : waypoints) {
            waypoint_ids.push_back(wp["id"]);
        }
        rooms_json[room_key] = json{
            { "type", kUnknownLabel },  // labels are the missing (future) stage
            { "sgid", room.id },
            { "entrances", std::move(entrances) },
            { "waypoints", std::move(waypoints) },
            { "objects", json::array() },  // offline object layer doesn't exist yet
        };
    }

    // --- edges: navgraph edges between emitted waypoints ----------------------
    json edges = json::array();
    int edges_dropped = 0;
    for (const NavGraphEdge &edge : nav.edges) {
        const auto u_it = nav_id_to_wpid.find(edge.u);
        const auto v_it = nav_id_to_wpid.find(edge.v);
        if (u_it == nav_id_to_wpid.end() || v_it == nav_id_to_wpid.end()) {
            ++edges_dropped;  // an endpoint wasn't placed in any room
            continue;
        }
        edges.push_back(json{
            { "u", u_it->second },
            { "v", v_it->second },
            { "meters", edge.meters },
        });
    }

    std::printf("[assembler] %s (floor_%d): %zu rooms, %d entrances, %zu waypoints "
                "(%zu nav nodes tagged, %d untagged), %zu edges (%d dropped)\n",
                floor.mask.floor_name.c_str(), floor_level, floor.rooms.size(),
                entrances_total, waypoint_ids.size(), nav_id_to_wpid.size(), untagged,
                edges.size(), edges_dropped);

    out.rooms = std::move(rooms_json);
    out.waypoint_ids = std::move(waypoint_ids);
    out.edges = std::move(edges);
    return out;
}

json BuildSceneGraph(const std::vector<FloorAssembly> &floors,
                     const BuildingCompass &compass, const AssemblerConfig &cfg)
{
    // Deterministic floor order in every list, whatever order the caller built.
    std::vector<const FloorAssembly *> ordered;
    for (const FloorAssembly &f : floors) {
        ordered.push_back(&f);
    }
    std::sort(ordered.begin(), ordered.end(),
              [](const FloorAssembly *a, const FloorAssembly *b) {
                  return a->level < b->level;
              });

    json zones = json::object();
    json waypoints = json::array();
    json edges = json::array();
    json floor_levels = json::array();
    for (const FloorAssembly *f : ordered) {
        const std::string zone_key = "floor_" + std::to_string(f->level);
        if (zones.contains(zone_key)) {
            throw std::runtime_error("duplicate floor level " + std::to_string(f->level) +
                                     " (" + f->floor_name + "): zone keys would collide");
        }
        zones[zone_key] = { { "rooms", f->rooms } };
        for (const json &wp : f->waypoint_ids) {
            waypoints.push_back(wp);
        }
        for (const json &e : f->edges) {
            edges.push_back(e);
        }
        floor_levels.push_back(f->level);
    }

    json metadata = {
        { "units", cfg.units },
        { "frame", compass.frame },
        { "building", cfg.building },
        { "floors", std::move(floor_levels) },
        { "dimensions", compass.dimensions },
    };
    if (!compass.compass.is_null()) {
        metadata["compass"] = compass.compass;
    }

    const std::string graph_name = cfg.building.empty() ? "map" : cfg.building;
    return json{
        { "map_id", graph_name },
        { "warehouse_id", graph_name },
        { "name", graph_name },
        { "client_id", cfg.client_id },
        { "uploaded_by", cfg.uploaded_by },
        { "update", true },
        { "layout",
          {
              { "zones", std::move(zones) },
              { "waypoints", std::move(waypoints) },
              { "edges", std::move(edges) },
              { "metadata", std::move(metadata) },
          } },
    };
}

// ---- debug overlay -------------------------------------------------------------

namespace {

// Upscale factor: the masks are ~0.1 m/px, too small for readable node ids.
constexpr int kOverlayScale = 4;

}  // namespace

int SaveNavGraphOverlay(const NavGraphData &nav, const RoomMaskData &mask,
                        const std::string &vis_png_path, const std::string &out_dir)
{
    cv::Mat base = cv::imread(vis_png_path, cv::IMREAD_COLOR);
    if (base.empty()) {
        throw std::runtime_error("cannot read " + vis_png_path);
    }

    // To debug orientation FIRST (transpose + vertical flip, exactly like the
    // segmentation saveDebugImage), so node labels are drawn upright.
    cv::Mat canvas;
    cv::transpose(base, canvas);
    cv::flip(canvas, canvas, 0);
    cv::resize(canvas, canvas, cv::Size(), kOverlayScale, kOverlayScale,
               cv::INTER_NEAREST);

    // World -> debug-canvas pixel. Continuous raw grid coords (u = row axis,
    // w = col axis); the transpose+flip maps raw (u, w) to canvas (x = u,
    // y = W - w) with W = raw col count.
    const float inv = 1.0f / mask.resolution;
    const double raw_cols = static_cast<double>(mask.mask.cols);
    const auto to_canvas = [&](const Eigen::Vector3d &p) {
        const double u = p.x() * inv + mask.origin_shift.x() - mask.bbox_row0;
        const double w = p.y() * inv + mask.origin_shift.y() - mask.bbox_col0;
        return cv::Point(static_cast<int>(std::lround(u * kOverlayScale)),
                         static_cast<int>(std::lround((raw_cols - w) * kOverlayScale)));
    };

    for (const NavGraphEdge &edge : nav.edges) {
        cv::line(canvas, to_canvas(nav.nodes[edge.u].position),
                 to_canvas(nav.nodes[edge.v].position), cv::Scalar(0, 128, 0), 2,
                 cv::LINE_AA);
    }

    int in_room = 0;
    for (const NavGraphNode &node : nav.nodes) {
        const cv::Point c = to_canvas(node.position);
        const bool tagged = mask.RoomIdAt(node.position) > 0;
        in_room += tagged ? 1 : 0;
        const cv::Scalar fill = tagged ? cv::Scalar(0, 140, 255) : cv::Scalar(128, 128, 128);
        cv::circle(canvas, c, 7, fill, cv::FILLED, cv::LINE_AA);
        cv::circle(canvas, c, 7, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        const std::string label = std::to_string(node.id);
        const cv::Point text_at = c + cv::Point(9, 4);
        cv::putText(canvas, label, text_at, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(canvas, label, text_at, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    const std::string debug_dir = out_dir + "/debug";
    std::filesystem::create_directories(debug_dir);
    const std::string out_path = debug_dir + "/navgraph_overlay.png";
    cv::imwrite(out_path, canvas);
    std::printf("[navgraph] overlay -> %s (%d of %zu nodes inside a room)\n",
                out_path.c_str(), in_room, nav.nodes.size());
    return in_room;
}

}  // namespace offline_scene_graph
