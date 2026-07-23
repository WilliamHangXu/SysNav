/**
 * @file offline_cli.cpp
 * @brief Debug CLI for the offline scene-graph pipeline: every stage as a
 *        subcommand of one binary, all thin wrappers over the same library the
 *        production offline_scene_graph_node uses.
 *
 *   offline_cli run       --session <dir> [--out <dir>] [--config <yaml>]
 *                         [--building NAME] [--floor <name>]       (whole DAG)
 *   offline_cli seg       --pcd <scans.pcd> --floors <blueprint.yaml> --out <dir>
 *                         [--config <yaml>] [--floor <name>]
 *   offline_cli navgraph  --graph <keypose_graph.json> --out <dir>
 *                         [--config <yaml>] [--rooms <seg floor dir>]
 *   offline_cli assemble  --rooms <seg floor dir> --navgraph <navgraph.json>
 *                         --out <scene_graph.json> [--building NAME]
 *                         [--floor-level N] [--floor-id ID] [--map-name NAME]
 */

#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "offline/offline_navgraph.h"
#include "offline/offline_pipeline.h"
#include "offline/offline_room_segmentation.h"
#include "offline/offline_scene_graph.h"

namespace ors = offline_room_segmentation;
namespace osg = offline_scene_graph;

namespace {

int Usage()
{
    std::fprintf(
        stderr,
        "Usage: offline_cli <subcommand> [options]\n"
        "  run       --session <dir> [--out <dir>] [--config <yaml>]\n"
        "            [--building NAME] [--floor <name>]           whole pipeline\n"
        "  seg       --pcd <scans.pcd> --floors <blueprint.yaml> --out <dir>\n"
        "            [--config <yaml>] [--floor <name>]           rooms layer\n"
        "  navgraph  --graph <keypose_graph.json> --out <dir>\n"
        "            [--config <yaml>] [--rooms <seg floor dir>]  navgraph layer\n"
        "  assemble  --rooms <seg floor dir> --navgraph <navgraph.json>\n"
        "            --out <scene_graph.json> [--building NAME]\n"
        "            [--floor-level N] [--floor-id ID] [--map-name NAME]\n");
    return 2;
}

// Flag parsing shared by all subcommands: every option takes one value.
bool ParseFlags(int argc, char **argv, std::map<std::string, std::string> &flags)
{
    for (int i = 2; i < argc; i += 2) {
        if (argv[i][0] != '-' || i + 1 >= argc) {
            return false;
        }
        flags[argv[i]] = argv[i + 1];
    }
    return true;
}

int RunPipeline(const std::map<std::string, std::string> &flags)
{
    osg::PipelineConfig cfg;
    cfg.session_dir = flags.count("--session") ? flags.at("--session") : "";
    if (cfg.session_dir.empty()) {
        return Usage();
    }
    if (flags.count("--out")) cfg.output_dir = flags.at("--out");
    if (flags.count("--config")) cfg.config_yaml = flags.at("--config");
    if (flags.count("--building")) cfg.building = flags.at("--building");
    if (flags.count("--floor")) cfg.only_floor = flags.at("--floor");
    const osg::PipelineResult result = osg::RunOfflinePipeline(cfg);
    return result.AnyCompleted() ? 0 : 1;
}

int RunSeg(const std::map<std::string, std::string> &flags)
{
    if (!flags.count("--pcd") || !flags.count("--floors") || !flags.count("--out")) {
        return Usage();
    }
    const std::string config = flags.count("--config") ? flags.at("--config") : "";
    const std::string only_floor = flags.count("--floor") ? flags.at("--floor") : "";
    const ors::OfflineConfig cfg = ors::LoadOfflineConfig(config);
    const std::vector<ors::FloorSpec> floors =
        ors::LoadFloorSpecs(flags.at("--floors"), cfg);
    const ors::SegmentationRunResult result = ors::RunRoomSegmentation(
        flags.at("--pcd"), floors, cfg, flags.at("--out"), only_floor);
    if (result.processed == 0) {
        const std::string hint =
            only_floor.empty() ? "" : " (--floor " + only_floor + " not found?)";
        std::fprintf(stderr, "[offline_seg] no floor processed%s\n", hint.c_str());
        return 1;
    }
    return result.failed > 0 ? 1 : 0;
}

int RunNavGraph(const std::map<std::string, std::string> &flags)
{
    if (!flags.count("--graph") || !flags.count("--out")) {
        return Usage();
    }
    osg::NavGraphConfig cfg;
    if (flags.count("--config")) {
        cfg = osg::LoadNavGraphConfig(flags.at("--config"));
    }
    const osg::KeyposeGraphData graph = osg::LoadKeyposeGraph(flags.at("--graph"));
    const osg::NavGraphData nav = osg::BuildNavGraph(graph, cfg);

    const std::string out_dir = flags.at("--out");
    std::filesystem::create_directories(out_dir);
    const std::string out_path = out_dir + "/navgraph.json";
    osg::SaveNavGraphJson(nav, out_path);
    std::printf("[navgraph] wrote %s\n", out_path.c_str());

    if (flags.count("--rooms")) {
        const std::string rooms_dir = flags.at("--rooms");
        const osg::RoomMaskData mask = osg::LoadRoomMask(rooms_dir);
        osg::SaveNavGraphOverlay(nav, mask, rooms_dir + "/room_mask_vis.png", out_dir);
    }
    return 0;
}

int RunAssemble(const std::map<std::string, std::string> &flags)
{
    if (!flags.count("--rooms") || !flags.count("--navgraph") || !flags.count("--out")) {
        return Usage();
    }
    const osg::FloorRoomData floor = osg::LoadFloorRoomData(flags.at("--rooms"));
    const osg::NavGraphData nav = osg::LoadNavGraphJson(flags.at("--navgraph"));

    // Same physical frame either way (FAST-LIO map); the labels just differ by
    // producer default ("map" vs "odom"). Surface it, don't fail on it.
    if (!nav.frame.empty() && nav.frame != floor.mask.frame) {
        std::printf("[assembler] note: navgraph frame label '%s' != rooms frame "
                    "label '%s' (same physical frame for this pipeline)\n",
                    nav.frame.c_str(), floor.mask.frame.c_str());
    }

    osg::AssemblerConfig cfg;
    if (flags.count("--building")) cfg.building = flags.at("--building");
    if (flags.count("--floor-id")) cfg.floor_id = flags.at("--floor-id");
    if (flags.count("--map-name")) cfg.name = flags.at("--map-name");
    cfg.floor_level = flags.count("--floor-level")
                          ? std::stoi(flags.at("--floor-level"))
                          : osg::FloorLevelFromName(floor.mask.floor_name, cfg.floor_level);

    const nlohmann::json scene_graph = osg::BuildSceneGraph(floor, nav, cfg);

    const std::string out_path = flags.at("--out");
    if (out_path.find('/') != std::string::npos) {
        std::filesystem::create_directories(std::filesystem::path(out_path).parent_path());
    }
    std::ofstream out(out_path);
    if (!out) {
        throw std::runtime_error("cannot write " + out_path);
    }
    out << scene_graph.dump(2) << "\n";  // same pretty format as online snapshots
    std::printf("[assembler] wrote %s\n", out_path.c_str());
    return 0;
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2) {
        return Usage();
    }
    std::map<std::string, std::string> flags;
    if (!ParseFlags(argc, argv, flags)) {
        return Usage();
    }
    const std::string subcommand = argv[1];
    try {
        if (subcommand == "run") return RunPipeline(flags);
        if (subcommand == "seg") return RunSeg(flags);
        if (subcommand == "navgraph") return RunNavGraph(flags);
        if (subcommand == "assemble") return RunAssemble(flags);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[offline_cli] ERROR: %s\n", e.what());
        return 1;
    }
    return Usage();
}
