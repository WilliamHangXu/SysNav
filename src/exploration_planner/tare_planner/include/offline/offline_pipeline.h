/**
 * @file offline_pipeline.h
 * @brief The whole offline scene-graph DAG as one in-process call:
 *
 *   room segmentation (scans.pcd + blueprint.yaml, all floors)
 *     -> per floor: navgraph (<session>/<floor>/keypose_graph.json)
 *     -> building compass (fit once, largest-footprint floor)
 *     -> assembly: ONE multifloor scene_graph.json at the output root
 *        (zones.floor_<M> per floor, floor-qualified ids)
 *
 * ROS-free by design: the production entry point is the thin
 * offline_scene_graph_node wrapper (signal in -> scene graph out), and the
 * same call serves the CLI (`offline_cli run`) or any future embedding.
 * Per-floor problems (missing keypose dump, failed segmentation) are reported
 * in the result, not thrown; session-level problems (missing pcd/blueprint,
 * bad config) throw.
 */

#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "offline/offline_navgraph.h"

namespace offline_scene_graph {

struct PipelineConfig {
    // Session folder: scans.pcd + blueprint.yaml + <floor>/keypose_graph.json.
    std::string session_dir;
    // Pipeline yaml (config/offline_scene_graph.yaml; "" = compiled defaults).
    std::string config_yaml;
    // Output root; "" => <session_dir>/scene_graph. Per-floor subdirs inside.
    std::string output_dir;
    // Scene-graph metadata.
    std::string building;
    // Process a single floor ("" = all floors in the blueprint).
    std::string only_floor;
};

struct FloorResult {
    std::string floor;
    std::string skipped_reason;  // empty <=> the floor is in the scene graph
    int rooms = 0;
    int nav_nodes = 0;
    int nav_edges = 0;
    int nodes_in_rooms = 0;      // overlay coverage (frame-consistency check)
};

struct PipelineResult {
    std::string output_dir;
    std::string scene_graph_path;  // set iff at least one floor completed
    nlohmann::json scene_graph;    // the merged multifloor graph (null otherwise)
    std::vector<FloorResult> floors;
    // True iff at least one floor made it into the scene graph.
    bool AnyCompleted() const
    {
        for (const FloorResult &f : floors) {
            if (f.skipped_reason.empty()) {
                return true;
            }
        }
        return false;
    }
};

// Run the full DAG. Throws std::runtime_error on session-level failures.
PipelineResult RunOfflinePipeline(const PipelineConfig &cfg);

// navigation_graph/kNavNodeMinDist from the pipeline yaml (the LIVE parameter
// name, flat or under the tare_planner_node / wildcard scopes). Missing key =>
// compiled default; unreadable file throws.
NavGraphConfig LoadNavGraphConfig(const std::string &yaml_path);

}  // namespace offline_scene_graph
