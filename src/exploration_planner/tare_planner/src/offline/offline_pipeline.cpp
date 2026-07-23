/**
 * @file offline_pipeline.cpp
 * @brief In-process offline scene-graph DAG (see offline_pipeline.h).
 */

#include "offline/offline_pipeline.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "offline/offline_room_segmentation.h"
#include "offline/offline_scene_graph.h"

namespace offline_scene_graph {

namespace fs = std::filesystem;
namespace ors = offline_room_segmentation;

NavGraphConfig LoadNavGraphConfig(const std::string &yaml_path)
{
    NavGraphConfig cfg;
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const char *key = "navigation_graph/kNavNodeMinDist";
    for (const char *scope : { "tare_planner_node", "/**" }) {
        if (root[scope] && root[scope]["ros__parameters"] &&
            root[scope]["ros__parameters"][key]) {
            cfg.nav_node_min_dist = root[scope]["ros__parameters"][key].as<double>();
            return cfg;
        }
    }
    if (root[key]) {
        cfg.nav_node_min_dist = root[key].as<double>();
    }
    return cfg;
}

PipelineResult RunOfflinePipeline(const PipelineConfig &pc)
{
    if (pc.session_dir.empty()) {
        throw std::runtime_error("session_dir is empty");
    }
    const std::string pcd_path = pc.session_dir + "/scans.pcd";
    const std::string blueprint_path = pc.session_dir + "/blueprint.yaml";
    if (!fs::exists(pcd_path)) {
        throw std::runtime_error(pcd_path + " not found");
    }
    if (!fs::exists(blueprint_path)) {
        throw std::runtime_error(blueprint_path + " not found");
    }

    PipelineResult result;
    result.output_dir =
        pc.output_dir.empty() ? pc.session_dir + "/scene_graph" : pc.output_dir;

    // --- stage 1: rooms layer, all requested floors in one pcd pass ----------
    const ors::OfflineConfig seg_cfg = ors::LoadOfflineConfig(pc.config_yaml);
    const std::vector<ors::FloorSpec> floors =
        ors::LoadFloorSpecs(blueprint_path, seg_cfg);
    ors::RunRoomSegmentation(pcd_path, floors, seg_cfg, result.output_dir,
                             pc.only_floor);

    const NavGraphConfig nav_cfg =
        pc.config_yaml.empty() ? NavGraphConfig{} : LoadNavGraphConfig(pc.config_yaml);

    // --- stage 2 per floor: navgraph + rooms-layer loading --------------------
    struct ReadyFloor {
        int level;
        FloorRoomData data;
        NavGraphData nav;
    };
    std::vector<ReadyFloor> ready;  // blueprint order (ascending floors)

    for (const ors::FloorSpec &spec : floors) {
        if (!pc.only_floor.empty() && spec.name != pc.only_floor) {
            continue;
        }
        FloorResult fr;
        fr.floor = spec.name;
        const std::string floor_dir = result.output_dir + "/" + spec.name;

        if (!fs::exists(floor_dir + "/mask_meta.json")) {
            fr.skipped_reason = "room segmentation produced no output";
            result.floors.push_back(std::move(fr));
            continue;
        }
        const std::string graph_path =
            pc.session_dir + "/" + spec.name + "/keypose_graph.json";
        if (!fs::exists(graph_path)) {
            fr.skipped_reason = "no keypose_graph.json in session "
                                "(publish 'skg' on /keyboard_input during the run)";
            result.floors.push_back(std::move(fr));
            continue;
        }

        const KeyposeGraphData keypose_graph = LoadKeyposeGraph(graph_path);
        NavGraphData nav = BuildNavGraph(keypose_graph, nav_cfg);
        SaveNavGraphJson(nav, floor_dir + "/navgraph.json");

        FloorRoomData floor_data = LoadFloorRoomData(floor_dir);
        fr.nodes_in_rooms = SaveNavGraphOverlay(
            nav, floor_data.mask, floor_dir + "/room_mask_vis.png", floor_dir);
        if (!nav.frame.empty() && nav.frame != floor_data.mask.frame) {
            std::printf("[pipeline] note: navgraph frame label '%s' != rooms frame "
                        "label '%s' (same physical frame for this pipeline)\n",
                        nav.frame.c_str(), floor_data.mask.frame.c_str());
        }
        // The per-floor scene_graph.json of the pre-multifloor layout; remove so
        // a re-run over an old output dir can't leave a stale second format.
        fs::remove(floor_dir + "/scene_graph.json");

        fr.rooms = static_cast<int>(floor_data.rooms.size());
        fr.nav_nodes = static_cast<int>(nav.nodes.size());
        fr.nav_edges = static_cast<int>(nav.edges.size());
        result.floors.push_back(std::move(fr));

        const int level =
            FloorLevelFromName(spec.name, static_cast<int>(ready.size()) + 1);
        ready.push_back({ level, std::move(floor_data), std::move(nav) });
    }

    // --- stage 3: one building compass, then the multifloor assembly ----------
    if (!ready.empty()) {
        AssemblerConfig asm_cfg;
        asm_cfg.building = pc.building;

        // One compass per building: fit once from the floor with the largest
        // room footprint (ties -> the lower floor, ready is in ascending order).
        const ReadyFloor *compass_floor = &ready.front();
        double best_area = -1.0;
        for (const ReadyFloor &f : ready) {
            double area = 0.0;
            for (const RoomEntry &room : f.data.rooms) {
                area += room.area_m2;
            }
            if (area > best_area) {
                best_area = area;
                compass_floor = &f;
            }
        }
        const BuildingCompass compass =
            FitBuildingCompass(compass_floor->data, asm_cfg.compass_radius_m);
        std::printf("[pipeline] building compass fit from floor_%d "
                    "(largest footprint, %.1f m^2)\n",
                    compass_floor->level, best_area);

        std::vector<FloorAssembly> assemblies;
        for (const ReadyFloor &f : ready) {
            assemblies.push_back(
                BuildFloorAssembly(f.data, f.nav, compass.axes, asm_cfg, f.level));
        }
        result.scene_graph = BuildSceneGraph(assemblies, compass, asm_cfg);

        result.scene_graph_path = result.output_dir + "/scene_graph.json";
        std::ofstream out(result.scene_graph_path);
        if (!out) {
            throw std::runtime_error("cannot write " + result.scene_graph_path);
        }
        out << result.scene_graph.dump(2) << "\n";
        std::printf("[pipeline] wrote %s\n", result.scene_graph_path.c_str());
    }

    int completed = 0;
    for (const FloorResult &fr : result.floors) {
        if (!fr.skipped_reason.empty()) {
            std::fprintf(stderr, "[pipeline] %s SKIPPED: %s\n", fr.floor.c_str(),
                         fr.skipped_reason.c_str());
        } else {
            ++completed;
        }
    }
    std::printf("[pipeline] done: %d of %zu floor(s) in the scene graph -> %s\n",
                completed, result.floors.size(),
                result.scene_graph_path.empty() ? "(none)"
                                                : result.scene_graph_path.c_str());
    return result;
}

}  // namespace offline_scene_graph
