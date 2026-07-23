/**
 * @file offline_navgraph.h
 * @brief The navgraph layer of the offline scene-graph pipeline: keypose-graph
 *        dump loading, the one-shot downsampling algorithm, and navgraph.json
 *        serialization.
 *
 * Naming contract (project-wide): "navgraph" is only the coarsened keypose
 * graph -- nodes {id, position} and reachability edges [u,v,meters]. Cross-layer
 * relationships (waypoint-in-room tagging, wp_<n> naming, area tagging) belong
 * to the scene-graph ASSEMBLER (offline_scene_graph.h), so nothing here knows
 * about rooms.
 *
 * Input: keypose_graph.json, written online by
 * SensorCoveragePlanner3D::SaveKeyposeGraphJson() on the "skg" /keyboard_input
 * trigger. Schema:
 *   metadata   frame / stamp_sec / node_count / edge_count / connected_node_count
 *   nodes      [[x,y,z], ...]   index == keypose node_ind (dense 0..N-1)
 *   edges      [[u,v,d], ...]   undirected, u < v once, d = traversable meters
 *   connected  [...]            GetConnectedGraphNodeIndices() verbatim
 *
 * `connected` is dumped verbatim and CONTAINS DUPLICATES (the online DFS appends
 * on every pop). The loader dedups preserving first-occurrence order -- that is
 * the online flood order, so offline seeding visits nodes in the same sequence
 * the live NavGraph would have. `connected` must be REPLAYED, never recomputed
 * from adjacency: the online CheckConnectivity applies collision pruning, so raw
 * adjacency overstates traversability.
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace offline_scene_graph {

// ---- keypose-graph dump ------------------------------------------------------

struct KeyposeGraphData {
    std::string frame;
    double stamp_sec = 0.0;
    std::vector<Eigen::Vector3d> positions;           // index = keypose node_ind
    std::vector<std::vector<int>> adjacency;          // symmetric neighbor lists
    std::vector<std::vector<double>> adjacency_dist;  // parallel edge lengths (m)
    std::vector<int> connected;  // deduped, first-occurrence (= online flood) order
    int edge_count = 0;          // undirected edges actually loaded
};

// Parse + validate a keypose_graph.json. Throws std::runtime_error on structural
// problems (unreadable file, malformed rows, out-of-range edge indices); warns on
// stderr and continues on soft issues (metadata count mismatch, out-of-range
// connected entries).
KeyposeGraphData LoadKeyposeGraph(const std::string &path);

// ---- navgraph ----------------------------------------------------------------

// A navgraph node. Position is copied verbatim from the keypose node it was
// seeded from; seed_keypose_ind is kept as provenance back into the dump.
struct NavGraphNode {
    int id = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    int seed_keypose_ind = -1;
};

// An undirected reachability edge, canonical u < v. `meters` approximates the
// traversable (keypose-graph) distance between the two nodes, not straight-line.
struct NavGraphEdge {
    int u = 0, v = 0;
    double meters = 0.0;
};

struct NavGraphData {
    // Provenance, copied from the source keypose dump + build parameters.
    std::string frame;
    double stamp_sec = 0.0;
    double nav_node_min_dist = 0.0;

    std::vector<NavGraphNode> nodes;  // ascending id (== seed order)
    std::vector<NavGraphEdge> edges;  // canonical u < v, sorted
};

struct NavGraphConfig {
    // Node spacing (in-room granularity). Same knob as the live
    // navigation_graph/kNavNodeMinDist parameter, so tuning transfers.
    double nav_node_min_dist = 2.5;
};

// One-shot offline port of the live NavGraph reconcile (src/navgraph/
// navgraph.cpp) minus the incremental machinery, leaving three phases:
//   1. seed   greedy distance-gated coverage of the connected keypose nodes,
//             visited in the dump's flood order;
//   2. label  geodesic Voronoi via multi-source BFS along keypose edges only
//             (wall-leak-proof: keypose edges are collision-checked);
//   3. edges  region adjacency; weight = min over crossing keypose edges of
//             ||navnode_u - a|| + len(a,b) + ||b - navnode_v||.
// Pure function: no I/O, deterministic (same input -> same output).
NavGraphData BuildNavGraph(const KeyposeGraphData &graph, const NavGraphConfig &cfg);

// Write/read navgraph.json (metadata / nodes / edges; one node or edge per
// line, like keypose_graph.json). Load throws std::runtime_error on malformed
// input.
void SaveNavGraphJson(const NavGraphData &g, const std::string &path);
NavGraphData LoadNavGraphJson(const std::string &path);

}  // namespace offline_scene_graph
