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

    // --- stages 2+3 per floor: navgraph, then assembly ------------------------
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
        const NavGraphData nav = BuildNavGraph(keypose_graph, nav_cfg);
        SaveNavGraphJson(nav, floor_dir + "/navgraph.json");

        const FloorRoomData floor_data = LoadFloorRoomData(floor_dir);
        fr.nodes_in_rooms = SaveNavGraphOverlay(
            nav, floor_data.mask, floor_dir + "/room_mask_vis.png", floor_dir);
        if (!nav.frame.empty() && nav.frame != floor_data.mask.frame) {
            std::printf("[pipeline] note: navgraph frame label '%s' != rooms frame "
                        "label '%s' (same physical frame for this pipeline)\n",
                        nav.frame.c_str(), floor_data.mask.frame.c_str());
        }

        AssemblerConfig asm_cfg;
        asm_cfg.building = pc.building;
        asm_cfg.floor_level = FloorLevelFromName(spec.name, asm_cfg.floor_level);
        fr.scene_graph = BuildSceneGraph(floor_data, nav, asm_cfg);

        fr.scene_graph_path = floor_dir + "/scene_graph.json";
        std::ofstream out(fr.scene_graph_path);
        if (!out) {
            throw std::runtime_error("cannot write " + fr.scene_graph_path);
        }
        out << fr.scene_graph.dump(2) << "\n";
        std::printf("[pipeline] wrote %s\n", fr.scene_graph_path.c_str());

        fr.rooms = static_cast<int>(floor_data.rooms.size());
        fr.nav_nodes = static_cast<int>(nav.nodes.size());
        fr.nav_edges = static_cast<int>(nav.edges.size());
        result.floors.push_back(std::move(fr));
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
    std::printf("[pipeline] done: %d of %zu floor(s) have a scene graph -> %s\n",
                completed, result.floors.size(), result.output_dir.c_str());
    return result;
}

}  // namespace offline_scene_graph
