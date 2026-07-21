/**
 * @file offline_types.h
 * @brief PODs + pure-math helpers for the offline room segmentation tool.
 *
 * Everything here is COPIED (not linked) from the online stack so the offline
 * tool stays ROS-free:
 *   - PlaneInfo:            include/room_segmentation/room_segmentation_node.h
 *   - voxel/color helpers:  src/utils/misc_utils.cpp (Eigen-only overloads)
 *   - cropRoomMask:         representation.h RoomNodeRep::SetRoomMask
 */

#pragma once

#include <array>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace offline_room_segmentation {

// Copied verbatim from room_segmentation_node.h (PlaneInfo).
struct PlaneInfo {
    int id;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud;
    std::vector<Eigen::Vector3i> voxel_indices;
    Eigen::Vector3f normal;
    Eigen::Vector3f centroid;
    Eigen::Vector3f u_dir;
    Eigen::Vector3f v_dir;
    float width;
    float height;
    std::array<Eigen::Vector3f, 4> corners;
    bool alive = true;
    bool merged = false;
};

// One segmented room (offline replacement for RoomNodeRep's structural slots).
// All raster coordinates are in the CROPPED (bbox) frame.
struct OfflineRoom {
    int id = 0;       // watershed seed id, 1..N, one-shot (no lifecycle)
    int show_id = 0;  // == id offline
    std::vector<cv::Point> points;          // cropped raster cells (x=col, y=row)
    std::vector<Eigen::Vector2f> polygon;   // world XY, largest outer contour
    cv::Mat room_mask;                      // bbox-cropped CV_8U per-room mask
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();        // world, z = robot_z
    Eigen::Vector3f interior_point = Eigen::Vector3f::Zero();  // PIA, world, z = robot_z
    float area = 0.0f;                      // m^2
    std::set<int> neighbors;                // door-connected room ids
};

struct OfflineDoor {
    int id = 0;              // global sequential door index
    int door_id = 0;         // per-room-pair instance index (online door_cloud label)
    int room_a = 0, room_b = 0;  // room_a < room_b
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();  // world, z = robot_z
    int pixel_count = 0;
};

struct FloorSpec {
    std::string name;
    float robot_z = 0.0f;
    float slab_z_min = 0.0f;      // robot_z - slab_below
    float slab_z_max = 0.0f;      // min(robot_z + ceilingHeight, next floor's slab bottom)
    float wall_thres_height = 0.0f;  // absolute: robot_z + wall_thres_height_ base
    float ceiling_height = 0.0f;     // absolute: robot_z + ceilingHeight_ base
};

// Same yaml keys as the online room_segmentation node where a knob survives
// offline; the formerly hard-coded constants keep their online values as
// defaults so a scenario yaml tunes both pipelines identically.
struct OfflineConfig {
    float explored_area_voxel_size = 0.1f;  // exploredAreaVoxelSize
    float room_resolution = 0.1f;           // room_resolution
    float ceiling_height_base = 2.0f;       // ceilingHeight_
    float wall_thres_height_base = 0.1f;    // wall_thres_height_
    int dilation_iteration = 4;             // dilation_iteration
    float outward_distance_0 = 0.5f;        // outward_distance_0
    float outward_distance_1 = 0.3f;        // outward_distance_1
    float distance_threshold = 2.5f;        // distance_threshold
    float distance_angel_threshold = 0.3f;  // distance_angel_threshold
    float angle_threshold_deg = 6.0f;       // angle_threshold_deg
    int min_room_size = 40;                 // min_room_size
    int normal_search_num = 50;             // normal_search_num
    // Formerly hard-coded in the online node (values = the online constants):
    int rg_min_cluster_size = 300;
    int rg_neighbors = 50;
    float rg_smoothness_deg = 3.0f;
    float rg_curvature = 1.0f;
    float plane_min_height = 1.5f;
    float min_hole_area = 400.0f;
    int min_component_area = 100;
    float hist_threshold_factor = 0.5f;
    // Offline-only:
    float slab_below = 1.0f;        // slab bottom = robot_z - slab_below
    int grid_margin_px = 20;        // auto-sized grid margin
    // Fuse same-pair door fragments whose nearest pixels are <= this apart.
    // The 3x3 junction blanking leaves gaps up to ~3-4 px (diagonals included),
    // so this must clear ~0.35 m while staying below any real wall pier.
    float door_merge_gap_m = 0.4f;
    float wall_stage_leaf_size = 0.0f;  // >voxel size => coarser cloud for normals/planes
    std::string frame = "map";      // frame label recorded in mask_meta.json
};

// ---- helpers copied from misc_utils.cpp (Eigen-only overloads) --------------

inline Eigen::Vector3d HSVtoRGB(double h, double s, double v)
{
    double c = v * s;
    double x = c * (1 - std::fabs(fmod(h / 60.0, 2) - 1));
    double m = v - c;
    double r_, g_, b_;

    if (h < 60) {
        r_ = c; g_ = x; b_ = 0;
    } else if (h < 120) {
        r_ = x; g_ = c; b_ = 0;
    } else if (h < 180) {
        r_ = 0; g_ = c; b_ = x;
    } else if (h < 240) {
        r_ = 0; g_ = x; b_ = c;
    } else if (h < 300) {
        r_ = x; g_ = 0; b_ = c;
    } else {
        r_ = c; g_ = 0; b_ = x;
    }

    int r = static_cast<int>((r_ + m) * 255.0);
    int g = static_cast<int>((g_ + m) * 255.0);
    int b = static_cast<int>((b_ + m) * 255.0);

    return Eigen::Vector3d(b, g, r);  // BGR, matching OpenCV
}

inline Eigen::Vector3d idToColor(int id)
{
    if (id == 0) {
        return Eigen::Vector3d(255, 255, 255);
    }
    double hue = (id * 57) % 360;
    double saturation = 0.85;
    double value = 0.95;
    return HSVtoRGB(hue, saturation, value);
}

inline Eigen::Vector3i point_to_voxel(const Eigen::Vector3f &pt,
                                      const Eigen::Vector3f &origin_shift,
                                      float inv_resolution)
{
    Eigen::Vector3f shifted = pt * inv_resolution + origin_shift;
    return shifted.array().floor().cast<int>();
}

inline Eigen::Vector3f voxel_to_point(const Eigen::Vector3i &voxel_index,
                                      const Eigen::Vector3f &origin_shift,
                                      float resolution)
{
    Eigen::Vector3f pt = voxel_index.cast<float>() - origin_shift;
    return pt * resolution;
}

inline Eigen::Vector3i point_to_voxel_cropped(const Eigen::Vector3f &pt,
                                              const Eigen::Vector3f &origin_shift,
                                              float inv_resolution,
                                              const std::vector<Eigen::Vector2i> &bbox)
{
    Eigen::Vector3f shifted = pt * inv_resolution + origin_shift;
    shifted.x() = shifted.x() - bbox[0].x();
    shifted.y() = shifted.y() - bbox[0].y();
    return shifted.array().floor().cast<int>();
}

inline Eigen::Vector3f voxel_to_point_cropped(const Eigen::Vector3i &voxel_index,
                                              const Eigen::Vector3f &origin_shift,
                                              float resolution,
                                              const std::vector<Eigen::Vector2i> &bbox)
{
    Eigen::Vector3f voxel_idx = voxel_index.cast<float>();
    voxel_idx.x() += bbox[0].x();
    voxel_idx.y() += bbox[0].y();
    Eigen::Vector3f pt = voxel_idx - origin_shift;
    return pt * resolution;
}

// Replica of RoomNodeRep::SetRoomMask (representation.h): bbox-crop the room's
// binary mask with a 10 px margin. Returns a clone (the original returned a
// view into a temporary).
inline cv::Mat cropRoomMask(const cv::Mat &room_mask)
{
    cv::Mat room_mask_tmp = room_mask.clone();
    room_mask_tmp.convertTo(room_mask_tmp, CV_8UC1);
    std::vector<cv::Point> non_zero_points;
    cv::findNonZero(room_mask_tmp, non_zero_points);
    if (non_zero_points.empty()) {
        return cv::Mat();
    }
    cv::Rect rect = cv::boundingRect(non_zero_points);
    std::vector<Eigen::Vector2i> bbox(2);
    bbox[0] = {rect.tl().y, rect.tl().x};
    bbox[1] = {rect.br().y, rect.br().x};
    int margin = 10;
    bbox[0] = (bbox[0] - Eigen::Vector2i(margin, margin)).cwiseMax(Eigen::Vector2i(0, 0));
    bbox[1] = (bbox[1] + Eigen::Vector2i(margin, margin))
                  .cwiseMin(Eigen::Vector2i(room_mask_tmp.rows - 1, room_mask_tmp.cols - 1));
    return room_mask_tmp.rowRange(bbox[0][0], bbox[1][0] + 1)
                        .colRange(bbox[0][1], bbox[1][1] + 1)
                        .clone();
}

}  // namespace offline_room_segmentation
