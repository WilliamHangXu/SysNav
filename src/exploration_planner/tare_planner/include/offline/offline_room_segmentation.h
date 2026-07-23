/**
 * @file offline_room_segmentation.h
 * @brief Library entry points into the offline room segmentation stage (the
 *        rooms layer), so the pipeline (offline_pipeline.h) can run it
 *        in-process. Implementation lives in offline_room_segmentation.cpp.
 */

#pragma once

#include <string>
#include <vector>

#include "offline/offline_types.h"

namespace offline_room_segmentation {

// Parse a scenario/flat yaml into an OfflineConfig ("" => compiled defaults).
// Throws on an unreadable/unparsable file.
OfflineConfig LoadOfflineConfig(const std::string &path);

// blueprint.yaml -> per-floor slabs (sorted by z, slab tops clamped by the next
// floor). Throws on a malformed file.
std::vector<FloorSpec> LoadFloorSpecs(const std::string &path, const OfflineConfig &cfg);

struct SegmentationRunResult {
    int processed = 0;
    int failed = 0;
};

// Segment every floor (or just `only_floor`) of the pcd into <out_dir>/<floor>/.
// Throws if the pcd can't be loaded; per-floor failures are reported on the
// console and counted, not thrown.
SegmentationRunResult RunRoomSegmentation(const std::string &pcd_path,
                                          const std::vector<FloorSpec> &floors,
                                          const OfflineConfig &cfg,
                                          const std::string &out_dir,
                                          const std::string &only_floor = std::string());

}  // namespace offline_room_segmentation
