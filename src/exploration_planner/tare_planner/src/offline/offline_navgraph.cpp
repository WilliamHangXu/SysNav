/**
 * @file offline_navgraph.cpp
 * @brief Navgraph layer implementation (see offline_navgraph.h): dump parsing,
 *        the one-shot downsampler (phase logic mirrors NavGraph::Reconcile),
 *        and navgraph.json serialization (same hand formatting as
 *        keypose_graph.json -- one node/edge row per line).
 */

#include "offline/offline_navgraph.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>

namespace offline_scene_graph {

using json = nlohmann::json;

// ---- keypose-graph dump ------------------------------------------------------

KeyposeGraphData LoadKeyposeGraph(const std::string &path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open keypose graph: " + path);
    }
    json root;
    try {
        root = json::parse(in);
    } catch (const json::parse_error &e) {
        throw std::runtime_error(path + ": " + e.what());
    }

    KeyposeGraphData g;
    if (root.contains("metadata")) {
        const json &m = root["metadata"];
        g.frame = m.value("frame", std::string());
        g.stamp_sec = m.value("stamp_sec", 0.0);
    }

    const json &nodes = root.at("nodes");
    g.positions.reserve(nodes.size());
    for (const json &row : nodes) {
        if (!row.is_array() || row.size() != 3) {
            throw std::runtime_error(path + ": node row is not [x,y,z]");
        }
        g.positions.emplace_back(row[0].get<double>(), row[1].get<double>(),
                                 row[2].get<double>());
    }
    const int n = static_cast<int>(g.positions.size());
    g.adjacency.assign(n, {});
    g.adjacency_dist.assign(n, {});

    std::set<std::pair<int, int>> seen_edges;
    for (const json &row : root.at("edges")) {
        if (!row.is_array() || row.size() != 3) {
            throw std::runtime_error(path + ": edge row is not [u,v,d]");
        }
        const int u = row[0].get<int>();
        const int v = row[1].get<int>();
        const double d = row[2].get<double>();
        if (u < 0 || u >= n || v < 0 || v >= n || u == v) {
            throw std::runtime_error(path + ": edge [" + std::to_string(u) + "," +
                                     std::to_string(v) + "] out of range");
        }
        if (!seen_edges.insert(std::minmax(u, v)).second) {
            continue;  // duplicate row; keep the first
        }
        g.adjacency[u].push_back(v);
        g.adjacency_dist[u].push_back(d);
        g.adjacency[v].push_back(u);
        g.adjacency_dist[v].push_back(d);
        ++g.edge_count;
    }

    int raw_connected = 0;
    int dropped_connected = 0;
    std::unordered_set<int> seen;
    for (const json &ind_json : root.at("connected")) {
        const int ind = ind_json.get<int>();
        ++raw_connected;
        if (ind < 0 || ind >= n) {
            ++dropped_connected;
            continue;
        }
        if (seen.insert(ind).second) {
            g.connected.push_back(ind);
        }
    }
    if (dropped_connected > 0) {
        std::fprintf(stderr, "[keypose_io] WARNING %s: %d connected entries out of range\n",
                     path.c_str(), dropped_connected);
    }

    if (root.contains("metadata")) {
        const json &m = root["metadata"];
        // connected_node_count is the raw (duplicated) length by contract, so it
        // is compared against the raw count, not the deduped one.
        if (m.value("node_count", n) != n ||
            m.value("edge_count", g.edge_count) != g.edge_count ||
            m.value("connected_node_count", raw_connected) != raw_connected) {
            std::fprintf(stderr,
                         "[keypose_io] WARNING %s: metadata counts disagree with contents "
                         "(nodes %d, edges %d, connected raw %d)\n",
                         path.c_str(), n, g.edge_count, raw_connected);
        }
    }

    std::printf("[keypose_io] %s: %d nodes, %d edges, %zu connected (%d raw), frame '%s'\n",
                path.c_str(), n, g.edge_count, g.connected.size(), raw_connected,
                g.frame.c_str());
    return g;
}

// ---- builder -----------------------------------------------------------------

NavGraphData BuildNavGraph(const KeyposeGraphData &graph, const NavGraphConfig &cfg)
{
    NavGraphData out;
    out.frame = graph.frame;
    out.stamp_sec = graph.stamp_sec;
    out.nav_node_min_dist = cfg.nav_node_min_dist;

    // --- Phase 1: seed (greedy distance-gated coverage) ----------------------
    // Walk the connected nodes in flood order; seed a nav node wherever a
    // keypose node is farther than nav_node_min_dist from every earlier seed.
    // Linear scan instead of the live kdtree: the node counts are tiny offline.
    const double min_dist_sq = cfg.nav_node_min_dist * cfg.nav_node_min_dist;
    for (int ind : graph.connected) {
        const Eigen::Vector3d &p = graph.positions[ind];
        bool covered = false;
        for (const NavGraphNode &node : out.nodes) {
            if ((p - node.position).squaredNorm() <= min_dist_sq) {
                covered = true;
                break;
            }
        }
        if (!covered) {
            NavGraphNode node;
            node.id = static_cast<int>(out.nodes.size());
            node.position = p;
            node.seed_keypose_ind = ind;
            out.nodes.push_back(node);
        }
    }

    // --- Phase 2: label (geodesic Voronoi via multi-source BFS) --------------
    // First source to reach a keypose node (fewest hops) claims it. Expansion is
    // gated on connected-set membership: adjacency still lists neighbors the
    // online connectivity flood excluded (collision-pruned).
    const std::unordered_set<int> connected_set(graph.connected.begin(),
                                                graph.connected.end());
    std::unordered_map<int, int> region;  // keypose node_ind -> nav node id
    std::vector<int> bfs_queue;
    bfs_queue.reserve(graph.connected.size());
    for (const NavGraphNode &node : out.nodes) {
        region[node.seed_keypose_ind] = node.id;
        bfs_queue.push_back(node.seed_keypose_ind);
    }
    for (size_t head = 0; head < bfs_queue.size(); ++head) {
        const int a = bfs_queue[head];
        const int label = region[a];
        for (int b : graph.adjacency[a]) {
            if (connected_set.find(b) == connected_set.end() ||
                region.find(b) != region.end()) {
                continue;
            }
            region[b] = label;
            bfs_queue.push_back(b);
        }
    }

    // --- Phase 3: edges (region adjacency) ------------------------------------
    // An edge u-v exists iff some keypose edge crosses from region u into region
    // v. Weight = the shortest crossing: ||navnode_u - a|| + len(a,b) +
    // ||b - navnode_v||, minimum over all crossing keypose edges (both endpoints
    // lie within nav_node_min_dist of their nav node by the Phase-1 invariant).
    std::map<std::pair<int, int>, double> edge_weight;  // canonical (u<v) -> meters
    for (int a : graph.connected) {
        const auto ra = region.find(a);
        if (ra == region.end()) {
            continue;  // island in the connected list no BFS source reached
        }
        const int u = ra->second;
        const Eigen::Vector3d &pos_a = graph.positions[a];
        const std::vector<int> &neighbors = graph.adjacency[a];
        const std::vector<double> &neighbor_dists = graph.adjacency_dist[a];
        for (size_t i = 0; i < neighbors.size(); ++i) {
            const int b = neighbors[i];
            const auto rb = region.find(b);
            if (rb == region.end() || rb->second == u) {
                continue;
            }
            const int v = rb->second;
            const Eigen::Vector3d &pos_b = graph.positions[b];
            const double crossing = (out.nodes[u].position - pos_a).norm() +
                                    neighbor_dists[i] +
                                    (pos_b - out.nodes[v].position).norm();
            const std::pair<int, int> key = (u < v) ? std::make_pair(u, v)
                                                    : std::make_pair(v, u);
            const auto it = edge_weight.find(key);
            if (it == edge_weight.end() || crossing < it->second) {
                edge_weight[key] = crossing;
            }
        }
    }
    out.edges.reserve(edge_weight.size());
    for (const auto &kv : edge_weight) {
        out.edges.push_back({ kv.first.first, kv.first.second, kv.second });
    }

    std::printf("[navgraph] %zu connected keypose nodes -> %zu nav nodes, %zu edges "
                "(min_dist %.2f m, %zu labeled)\n",
                graph.connected.size(), out.nodes.size(), out.edges.size(),
                cfg.nav_node_min_dist, region.size());
    return out;
}

// ---- navgraph.json -----------------------------------------------------------

void SaveNavGraphJson(const NavGraphData &g, const std::string &path)
{
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path);
    }

    const json metadata = { { "frame", g.frame },
                            { "stamp_sec", g.stamp_sec },
                            { "nav_node_min_dist", g.nav_node_min_dist },
                            { "node_count", static_cast<int>(g.nodes.size()) },
                            { "edge_count", static_cast<int>(g.edges.size()) } };

    out << "{\n  \"metadata\": " << metadata.dump() << ",\n";
    out << "  \"nodes\": [";
    for (size_t i = 0; i < g.nodes.size(); i++) {
        const NavGraphNode &n = g.nodes[i];
        const json row = { { "id", n.id },
                           { "position", { n.position.x(), n.position.y(), n.position.z() } },
                           { "seed_keypose_ind", n.seed_keypose_ind } };
        out << (i == 0 ? "\n    " : ",\n    ") << row.dump();
    }
    out << (g.nodes.empty() ? "]" : "\n  ]") << ",\n";
    out << "  \"edges\": [";
    for (size_t i = 0; i < g.edges.size(); i++) {
        const json row = { g.edges[i].u, g.edges[i].v, g.edges[i].meters };
        out << (i == 0 ? "\n    " : ",\n    ") << row.dump();
    }
    out << (g.edges.empty() ? "]" : "\n  ]") << "\n}\n";
}

NavGraphData LoadNavGraphJson(const std::string &path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open navgraph: " + path);
    }
    json root;
    try {
        root = json::parse(in);
    } catch (const json::parse_error &e) {
        throw std::runtime_error(path + ": " + e.what());
    }

    NavGraphData g;
    if (root.contains("metadata")) {
        const json &m = root["metadata"];
        g.frame = m.value("frame", std::string());
        g.stamp_sec = m.value("stamp_sec", 0.0);
        g.nav_node_min_dist = m.value("nav_node_min_dist", 0.0);
    }
    for (const json &row : root.at("nodes")) {
        NavGraphNode node;
        node.id = row.at("id").get<int>();
        const json &p = row.at("position");
        node.position = Eigen::Vector3d(p[0].get<double>(), p[1].get<double>(),
                                        p[2].get<double>());
        node.seed_keypose_ind = row.value("seed_keypose_ind", -1);
        if (node.id != static_cast<int>(g.nodes.size())) {
            throw std::runtime_error(path + ": node ids must be dense 0..N-1");
        }
        g.nodes.push_back(node);
    }
    const int n = static_cast<int>(g.nodes.size());
    for (const json &row : root.at("edges")) {
        if (!row.is_array() || row.size() != 3) {
            throw std::runtime_error(path + ": edge row is not [u,v,meters]");
        }
        NavGraphEdge edge{ row[0].get<int>(), row[1].get<int>(), row[2].get<double>() };
        if (edge.u < 0 || edge.u >= n || edge.v < 0 || edge.v >= n || edge.u >= edge.v) {
            throw std::runtime_error(path + ": bad edge [" + std::to_string(edge.u) +
                                     "," + std::to_string(edge.v) + "]");
        }
        g.edges.push_back(edge);
    }
    return g;
}

}  // namespace offline_scene_graph
