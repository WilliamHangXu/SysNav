/**
 * @file offline_room_segmentation.cpp
 * @brief Offline (batch, non-ROS) room segmentation.
 *
 * Full-building PCD + per-floor z index (blueprint.yaml) in; per-floor room
 * mask / rooms.json / doors.json + debug images out.
 *
 * The segmentation core (two-source wall extraction -> watershed -> doors ->
 * per-room polygon/centroid/interior point) is lifted from the online node,
 * src/room_segmentation/room_segmentation.cpp. Deliberately absent relative to
 * the online node:
 *   - the freespace/occupancy machinery (updateFreespace / updateStateVoxel /
 *     state_map_): it corrects online accumulation artifacts (dynamics, glass)
 *     that a one-shot clean map does not have;
 *   - all incremental state: room lifecycle reconciliation, monotonic ids,
 *     cross-frame plane merge/prune, incremental normals. One shot: ids = 1..N;
 *   - robot_position_: output z comes from the floor's robot_z, there is no
 *     current-room / is_connected concept.
 *
 * Usage:
 *   offline_room_segmentation --pcd scans.pcd --floors blueprint.yaml \
 *       --config <scenario or flat yaml> --out <dir> [--floor <name>]
 */

#include "offline/offline_types.h"
#include "offline/offline_room_segmentation.h"

#include <chrono>
#include <stdexcept>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace ors = offline_room_segmentation;
using json = nlohmann::json;

namespace {

double NowSec()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class OfflineRoomSegmenter {
public:
    OfflineRoomSegmenter(const ors::OfflineConfig &cfg, const ors::FloorSpec &floor,
                         const std::string &out_dir)
        : cfg_(cfg), floor_(floor), out_dir_(out_dir), debug_dir_(out_dir + "/debug")
    {
        room_resolution_ = cfg_.room_resolution;
        room_resolution_inv_ = 1.0f / room_resolution_;
        wall_thres_height_ = floor_.wall_thres_height;
        ceiling_height_ = floor_.ceiling_height;
        std::filesystem::create_directories(debug_dir_);
    }

    bool run(const pcl::PointCloud<pcl::PointXYZ>::Ptr &building_cloud)
    {
        const double t_start = NowSec();
        if (!prepareCloud(building_cloud)) {
            writeEmptyOutputs("empty slab");
            return false;
        }
        initGrid();
        updateVoxelMap();
        segmentFloor();
        writeOutputs();
        std::printf("[offline_seg] %s: %zu rooms, %zu doors, %.1f s\n",
                    floor_.name.c_str(), rooms_.size(), doors_.size(),
                    NowSec() - t_start);
        return true;
    }

private:
    // ---------------- Stage 1: slab crop + downsample + one-shot normals ----
    bool prepareCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &building_cloud)
    {
        const double t0 = NowSec();
        pcl::PointCloud<pcl::PointXYZ>::Ptr slab(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(floor_.slab_z_min, floor_.slab_z_max);
        pass.setInputCloud(building_cloud);
        pass.filter(*slab);

        if (slab->size() < 1000) {
            std::fprintf(stderr,
                         "[offline_seg] %s: slab z=[%.2f, %.2f] has only %zu points; "
                         "check robot_z in the floors yaml\n",
                         floor_.name.c_str(), floor_.slab_z_min, floor_.slab_z_max,
                         slab->size());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr slab_ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(cfg_.explored_area_voxel_size, cfg_.explored_area_voxel_size,
                       cfg_.explored_area_voxel_size);
        vg.setInputCloud(slab);
        vg.filter(*slab_ds);

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::copyPointCloud(*slab_ds, *cloud_);
        estimateNormals(cloud_);

        // Optional coarser copy for the normal/plane stage only (perf knob).
        wall_cloud_ = cloud_;
        if (cfg_.wall_stage_leaf_size > cfg_.explored_area_voxel_size) {
            wall_cloud_.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::VoxelGrid<pcl::PointXYZINormal> vg_wall;
            vg_wall.setLeafSize(cfg_.wall_stage_leaf_size, cfg_.wall_stage_leaf_size,
                                cfg_.wall_stage_leaf_size);
            vg_wall.setInputCloud(cloud_);
            vg_wall.filter(*wall_cloud_);
            estimateNormals(wall_cloud_);  // averaged normals are non-unit; re-estimate
        }

        std::printf("[offline_seg] %s: slab %zu pts -> downsampled %zu pts "
                    "(wall stage %zu), normals in %.1f s\n",
                    floor_.name.c_str(), slab->size(), cloud_->size(),
                    wall_cloud_->size(), NowSec() - t0);
        return true;
    }

    void estimateNormals(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud)
    {
        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZINormal>);
        pcl::NormalEstimationOMP<pcl::PointXYZINormal, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(cfg_.normal_search_num);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*normals);
        for (size_t i = 0; i < cloud->size(); ++i) {
            cloud->points[i].normal_x = normals->points[i].normal_x;
            cloud->points[i].normal_y = normals->points[i].normal_y;
            cloud->points[i].normal_z = normals->points[i].normal_z;
            cloud->points[i].curvature = normals->points[i].curvature;
        }
    }

    // ---------------- Stage 2: auto-sized grid (replaces room_x/y/z) --------
    void initGrid()
    {
        pcl::PointXYZINormal min_pt, max_pt;
        pcl::getMinMax3D(*cloud_, min_pt, max_pt);

        const int m = cfg_.grid_margin_px;
        const int dims_x =
            static_cast<int>(std::ceil((max_pt.x - min_pt.x) * room_resolution_inv_)) + 2 * m;
        const int dims_y =
            static_cast<int>(std::ceil((max_pt.y - min_pt.y) * room_resolution_inv_)) + 2 * m;
        const int dims_z = static_cast<int>(std::ceil(
                               (floor_.slab_z_max - floor_.slab_z_min) * room_resolution_inv_)) +
                           2;
        room_voxel_dimension_ = {std::max(dims_x, 1), std::max(dims_y, 1),
                                 std::max(dims_z, 1)};
        shift_ = Eigen::Vector3f(m - min_pt.x * room_resolution_inv_,
                                 m - min_pt.y * room_resolution_inv_,
                                 1 - floor_.slab_z_min * room_resolution_inv_);

        navigable_voxels_.assign(static_cast<size_t>(room_voxel_dimension_[0]) *
                                     room_voxel_dimension_[1] * room_voxel_dimension_[2],
                                 0);
        navigable_map_all_ =
            cv::Mat::zeros(room_voxel_dimension_[0], room_voxel_dimension_[1], CV_32F);
        wall_hist_all_ =
            cv::Mat::zeros(room_voxel_dimension_[0], room_voxel_dimension_[1], CV_32F);
        room_mask_ =
            cv::Mat::zeros(room_voxel_dimension_[0], room_voxel_dimension_[1], CV_32S);
        room_mask_vis_ =
            cv::Mat(room_voxel_dimension_[0], room_voxel_dimension_[1], CV_8UC3,
                    cv::Scalar(255, 255, 255));

        bbox_.clear();
        bbox_.emplace_back(Eigen::Vector2i(0, 0));
        bbox_.emplace_back(Eigen::Vector2i(room_voxel_dimension_[0] - 1,
                                           room_voxel_dimension_[1] - 1));

        std::printf("[offline_seg] %s: grid %dx%dx%d @ %.2f m (shift %.1f, %.1f, %.1f)\n",
                    floor_.name.c_str(), room_voxel_dimension_[0], room_voxel_dimension_[1],
                    room_voxel_dimension_[2], room_resolution_, shift_.x(), shift_.y(),
                    shift_.z());
    }

    int toIndex(int x, int y, int z)
    {
        return x * room_voxel_dimension_[1] * room_voxel_dimension_[2] +
               y * room_voxel_dimension_[2] + z;
    }

    // ---------------- Stage 3: lifted updateVoxelMap (state_map guard gone) --
    void updateVoxelMap()
    {
        for (const auto &p : cloud_->points) {
            const Eigen::Vector3f pt(p.x, p.y, p.z);
            auto idx = ors::point_to_voxel(pt, shift_, room_resolution_inv_);
            idx[0] = std::clamp(idx[0], 0, room_voxel_dimension_[0] - 1);
            idx[1] = std::clamp(idx[1], 0, room_voxel_dimension_[1] - 1);
            idx[2] = std::clamp(idx[2], 0, room_voxel_dimension_[2] - 1);

            if (navigable_voxels_[toIndex(idx[0], idx[1], idx[2])] == 0) {
                navigable_voxels_[toIndex(idx[0], idx[1], idx[2])] = 1;
                navigable_map_all_.at<float>(idx[0], idx[1]) += 1.0f;

                if (wall_thres_height_ < pt.z() && pt.z() < ceiling_height_) {
                    wall_hist_all_.at<float>(idx[0], idx[1]) += 1.0f;
                }
            }
        }

        std::vector<cv::Point> non_zero_points;
        cv::findNonZero(navigable_map_all_, non_zero_points);
        if (non_zero_points.empty()) {
            return;
        }

        cv::Rect rect = cv::boundingRect(non_zero_points);
        bbox_[0] = Eigen::Vector2i(rect.tl().y, rect.tl().x);
        bbox_[1] = Eigen::Vector2i(rect.br().y, rect.br().x);

        const int margin = 20;
        bbox_[0] = (bbox_[0] - Eigen::Vector2i(margin, margin))
                       .cwiseMax(Eigen::Vector2i(0, 0));
        bbox_[1] = (bbox_[1] + Eigen::Vector2i(margin, margin))
                       .cwiseMin(Eigen::Vector2i(room_voxel_dimension_[0] - 1,
                                                 room_voxel_dimension_[1] - 1));

        navigable_map_ = navigable_map_all_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                             .colRange(bbox_[0][1], bbox_[1][1] + 1);
        wall_hist_ = wall_hist_all_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                         .colRange(bbox_[0][1], bbox_[1][1] + 1);
    }

    // ---------------- lifted isPlaneSame / mergePlanes ----------------------
    bool isPlaneSame(const ors::PlaneInfo &a, const ors::PlaneInfo &b)
    {
        float angle = std::acos(std::abs(a.normal.dot(b.normal))) * 180.0f / M_PI;
        if (angle > cfg_.angle_threshold_deg)
            return false;

        Eigen::Vector3f centroid_diff = b.centroid - a.centroid;
        float angel_distance = centroid_diff.dot(a.normal);
        if (std::abs(angel_distance) > cfg_.distance_angel_threshold)
            return false;

        float center_dist = (a.centroid - b.centroid).norm();
        float actual_dist = center_dist - (a.width + b.width) / 2.0f;
        if (actual_dist > cfg_.distance_threshold)
            return false;

        return true;
    }

    void mergePlanes(std::vector<ors::PlaneInfo> &plane_infos, int idx_0,
                     ors::PlaneInfo &compare)
    {
        ors::PlaneInfo &base = plane_infos[idx_0];
        compare.merged = true;

        size_t size_0 = base.cloud->size();
        size_t size_1 = compare.cloud->size();
        base.centroid = (base.centroid * size_0 + compare.centroid * size_1) /
                        (size_0 + size_1);
        base.cloud->insert(base.cloud->end(), compare.cloud->begin(),
                           compare.cloud->end());

        pcl::VoxelGrid<pcl::PointXYZINormal> dwz;
        dwz.setLeafSize(cfg_.explored_area_voxel_size, cfg_.explored_area_voxel_size,
                        cfg_.explored_area_voxel_size);
        dwz.setInputCloud(base.cloud);
        dwz.filter(*base.cloud);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr merged_cloud(
            new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::copyPointCloud(*base.cloud, *merged_cloud);

        pcl::SACSegmentation<pcl::PointXYZINormal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(merged_cloud);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        seg.segment(inliers, coefficients);

        if (inliers.indices.size() > 0) {
            Eigen::Vector3f normal(coefficients.values[0], coefficients.values[1],
                                   coefficients.values[2]);
            normal = (normal - normal.dot(Eigen::Vector3f::UnitZ()) *
                                   Eigen::Vector3f::UnitZ())
                         .normalized();
            base.normal = normal;

            base.cloud->clear();
            base.voxel_indices.clear();
            for (const auto &index : inliers.indices) {
                pcl::PointXYZINormal pt = merged_cloud->points[index];
                base.cloud->push_back(pt);
                Eigen::Vector3f pt_f(pt.x, pt.y, pt.z);
                base.voxel_indices.push_back(
                    ors::point_to_voxel(pt_f, shift_, room_resolution_inv_));
            }
        }

        base.u_dir = base.normal.cross(Eigen::Vector3f::UnitZ()).normalized();
        base.v_dir = base.normal.cross(base.u_dir).normalized();

        Eigen::Vector4f centroid4f = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*base.cloud, centroid4f);
        base.centroid = centroid4f.head<3>();

        float u_min = FLT_MAX, u_max = -FLT_MAX;
        float v_min = FLT_MAX, v_max = -FLT_MAX;
        for (const auto &point : base.cloud->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f relative = p - base.centroid;
            float u = relative.dot(base.u_dir);
            float v = relative.dot(base.v_dir);

            u_min = std::min(u_min, u);
            u_max = std::max(u_max, u);
            v_min = std::min(v_min, v);
            v_max = std::max(v_max, v);
        }

        std::array<Eigen::Vector3f, 4> corners = {
            base.centroid + base.u_dir * u_min + base.v_dir * v_min,
            base.centroid + base.u_dir * u_max + base.v_dir * v_min,
            base.centroid + base.u_dir * u_max + base.v_dir * v_max,
            base.centroid + base.u_dir * u_min + base.v_dir * v_max};

        base.centroid = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;
        base.corners = corners;
        base.width = u_max - u_min;
        base.height = v_max - v_min;
    }

    // ---------------- Stage 4: lifted getWall (one-shot) --------------------
    // Dropped vs online: in-range plane kill (no robot), cross-frame merge into
    // a persistent list (single pass), state-map free-cull (no freespace), the
    // /walls publisher (replaced by a debug image).
    cv::Mat getWall(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud)
    {
        const double t0 = NowSec();
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normals->resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            const pcl::PointXYZINormal &pt = cloud->points[i];
            (*normals)[i].normal_x = pt.normal_x;
            (*normals)[i].normal_y = pt.normal_y;
            (*normals)[i].normal_z = pt.normal_z;
            (*normals)[i].curvature = pt.curvature;
        }

        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud, *indices);

        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZINormal>);
        pcl::RegionGrowing<pcl::PointXYZINormal, pcl::Normal> reg;
        reg.setMinClusterSize(cfg_.rg_min_cluster_size);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(cfg_.rg_neighbors);
        reg.setInputCloud(cloud);
        reg.setIndices(indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(cfg_.rg_smoothness_deg / 180.0 * M_PI);
        reg.setCurvatureThreshold(cfg_.rg_curvature);
        reg.setSmoothModeFlag(true);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        std::vector<ors::PlaneInfo> plane_infos_new;
        plane_infos_new.reserve(clusters.size() / 4);

        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto &cluster = clusters[i];

            Eigen::Vector3f centroid(0, 0, 0);
            Eigen::Vector3f normal_sum(0, 0, 0);
            int valid_points = 0;

            for (int idx : cluster.indices) {
                const pcl::PointXYZINormal &pt = cloud->points[idx];
                centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);

                const pcl::Normal &n = normals->points[idx];
                if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) &&
                    std::isfinite(n.normal_z)) {
                    normal_sum += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
                    valid_points++;
                }
            }

            if (valid_points == 0 || normal_sum.norm() < 1e-3)
                continue;

            centroid /= static_cast<float>(cluster.indices.size());
            Eigen::Vector3f avg_normal = normal_sum.normalized();

            float dot = std::abs(avg_normal.dot(Eigen::Vector3f::UnitZ()));
            if (dot > std::cos(80.0f * M_PI / 180.0f))
                continue;

            float mean_dist = 0.0f;
            float m2 = 0.0f;
            int n = 0;

            for (int idx : cluster.indices) {
                const pcl::PointXYZINormal &pt = cloud->points[idx];
                Eigen::Vector3f p(pt.x - centroid.x(), pt.y - centroid.y(),
                                  pt.z - centroid.z());
                float dist = p.dot(avg_normal);

                n++;
                float delta = dist - mean_dist;
                mean_dist += delta / n;
                float delta2 = dist - mean_dist;
                m2 += delta * delta2;
            }

            float variance = (n > 1) ? (m2 / n) : 0.0f;
            if (variance > 0.1f)
                continue;

            avg_normal = (avg_normal - avg_normal.dot(Eigen::Vector3f::UnitZ()) *
                                           Eigen::Vector3f::UnitZ())
                             .normalized();

            std::vector<Eigen::Vector3i> voxel_indices;
            voxel_indices.reserve(cluster.indices.size());

            Eigen::Vector3f u_dir = avg_normal.cross(Eigen::Vector3f::UnitZ()).normalized();
            Eigen::Vector3f v_dir = avg_normal.cross(u_dir).normalized();

            float u_min = FLT_MAX, u_max = -FLT_MAX;
            float v_min = FLT_MAX, v_max = -FLT_MAX;

            for (int idx : cluster.indices) {
                const pcl::PointXYZINormal &point = cloud->points[idx];
                Eigen::Vector3f p(point.x, point.y, point.z);
                Eigen::Vector3f relative = p - centroid;
                float u = relative.dot(u_dir);
                float v = relative.dot(v_dir);

                u_min = std::min(u_min, u);
                u_max = std::max(u_max, u);
                v_min = std::min(v_min, v);
                v_max = std::max(v_max, v);

                voxel_indices.emplace_back(
                    ors::point_to_voxel(p, shift_, room_resolution_inv_));
            }

            float height = v_max - v_min;
            if (height < cfg_.plane_min_height)
                continue;

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster_cloud(
                new pcl::PointCloud<pcl::PointXYZINormal>);
            cluster_cloud->points.reserve(cluster.indices.size());
            for (int idx : cluster.indices) {
                cluster_cloud->points.push_back(cloud->points[idx]);
            }
            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;

            std::array<Eigen::Vector3f, 4> corners = {
                centroid + u_dir * u_min + v_dir * v_min,
                centroid + u_dir * u_max + v_dir * v_min,
                centroid + u_dir * u_max + v_dir * v_max,
                centroid + u_dir * u_min + v_dir * v_max};

            centroid = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;
            float width = u_max - u_min;

            plane_infos_new.push_back({static_cast<int>(i), cluster_cloud,
                                       std::move(voxel_indices), avg_normal, centroid,
                                       u_dir, v_dir, width, height, corners, true,
                                       false});
        }

        // One-shot: the fresh detections ARE the plane list; the only merge is
        // the same-pass self-merge (one physical wall grown as two clusters).
        plane_infos_ = std::move(plane_infos_new);
        for (size_t i = 0; i < plane_infos_.size(); ++i) {
            if (!plane_infos_[i].alive)
                continue;
            for (size_t j = i + 1; j < plane_infos_.size(); ++j) {
                if (!plane_infos_[j].alive)
                    continue;
                if (isPlaneSame(plane_infos_[i], plane_infos_[j])) {
                    mergePlanes(plane_infos_, static_cast<int>(i), plane_infos_[j]);
                    plane_infos_[j].alive = false;
                    plane_infos_[i].alive = true;
                }
            }
        }
        plane_infos_.erase(std::remove_if(plane_infos_.begin(), plane_infos_.end(),
                                          [](const ors::PlaneInfo &plane) {
                                              return (!plane.alive);
                                          }),
                           plane_infos_.end());

        std::printf("[offline_seg] %s: getWall %zu clusters -> %zu planes in %.1f s\n",
                    floor_.name.c_str(), clusters.size(), plane_infos_.size(),
                    NowSec() - t0);

        // Debug: per-plane footprint colors (offline stand-in for /walls).
        cv::Mat planes_color(room_voxel_dimension_[0], room_voxel_dimension_[1],
                             CV_8UC3, cv::Scalar(0, 0, 0));
        for (const auto &plane : plane_infos_) {
            Eigen::Vector3d color = ors::idToColor(plane.id + 1);
            for (const auto &vi : plane.voxel_indices) {
                if (vi[0] >= 0 && vi[0] < room_voxel_dimension_[0] && vi[1] >= 0 &&
                    vi[1] < room_voxel_dimension_[1]) {
                    planes_color.at<cv::Vec3b>(vi[0], vi[1]) =
                        cv::Vec3b(static_cast<uchar>(color[0]),
                                  static_cast<uchar>(color[1]),
                                  static_cast<uchar>(color[2]));
                }
            }
        }
        saveDebugImage(planes_color.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                           .colRange(bbox_[0][1], bbox_[1][1] + 1),
                       "wall_planes_color.png");

        // Project walls to the 2D map (lifted verbatim; full grid, crop at end).
        cv::Mat wall_mask(room_voxel_dimension_[0], room_voxel_dimension_[1], CV_8U,
                          cv::Scalar(0));

        for (const auto &plane : plane_infos_) {
            if (plane.merged)
                continue;

            std::vector<cv::Point> polygon_2d;
            polygon_2d.reserve(4);
            Eigen::Vector3f outward_1 = plane.normal * cfg_.outward_distance_1;

            for (int i = 0; i < 4; i++) {
                Eigen::Vector3f corner = plane.corners[i];
                if (i == 0)
                    corner = corner - outward_1;
                else if (i == 1)
                    corner = corner - outward_1;
                else if (i == 2)
                    corner = corner + outward_1;
                else
                    corner = corner + outward_1;

                Eigen::Vector3i idx =
                    ors::point_to_voxel(corner, shift_, room_resolution_inv_);
                polygon_2d.emplace_back(idx[1], idx[0]);
            }
            cv::fillPoly(wall_mask, std::vector<std::vector<cv::Point>>{polygon_2d},
                         cv::Scalar(255));

            polygon_2d.clear();
            polygon_2d.reserve(4);

            for (int i = 0; i < 2; ++i) {
                Eigen::Vector3f corner = plane.corners[i];
                Eigen::Vector3f pt_0 = corner;
                Eigen::Vector3f pt_1;
                if (i == 0) {
                    pt_1 = corner - plane.u_dir * cfg_.outward_distance_0;
                } else {
                    pt_1 = corner + plane.u_dir * cfg_.outward_distance_0;
                }

                Eigen::Vector3i idx_0 =
                    ors::point_to_voxel(pt_0, shift_, room_resolution_inv_);
                Eigen::Vector3i idx_1 =
                    ors::point_to_voxel(pt_1, shift_, room_resolution_inv_);
                cv::Point pt_2d_0(idx_0[1], idx_0[0]);
                cv::Point pt_2d_1(idx_1[1], idx_1[0]);

                cv::LineIterator it(wall_mask, pt_2d_0, pt_2d_1, 8);

                cv::Point pt_found = pt_2d_1;
                for (int j = 0; j < it.count; ++j, ++it) {
                    cv::Point pt = it.pos();
                    if (pt.x <= 0 || pt.x >= wall_mask.cols - 1 || pt.y <= 0 ||
                        pt.y >= wall_mask.rows - 1) {
                        pt_found = pt;
                        break;
                    }
                    if (wall_mask.at<uchar>(pt) == 255) {
                        pt_found = pt;
                        break;
                    }
                }
                if (pt_found.x >= 0 && pt_found.y >= 0) {
                    Eigen::Vector3i idx_found(pt_found.y, pt_found.x, 0);
                    Eigen::Vector3f pt_found_world =
                        ors::voxel_to_point(idx_found, shift_, room_resolution_);
                    Eigen::Vector3f pt_outward_0 =
                        pt_found_world + plane.normal * cfg_.outward_distance_1;
                    Eigen::Vector3f pt_outward_1 =
                        pt_found_world - plane.normal * cfg_.outward_distance_1;
                    Eigen::Vector3i idx_outward_0 =
                        ors::point_to_voxel(pt_outward_0, shift_, room_resolution_inv_);
                    Eigen::Vector3i idx_outward_1 =
                        ors::point_to_voxel(pt_outward_1, shift_, room_resolution_inv_);
                    polygon_2d.emplace_back(idx_outward_0[1], idx_outward_0[0]);
                    polygon_2d.emplace_back(idx_outward_1[1], idx_outward_1[0]);
                }
            }
            if (!polygon_2d.empty())
                cv::fillPoly(wall_mask, std::vector<std::vector<cv::Point>>{polygon_2d},
                             cv::Scalar(255));
        }

        wall_mask = wall_mask.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                        .colRange(bbox_[0][1], bbox_[1][1] + 1);
        saveDebugImage(wall_mask, "wall_mask_from_planes.png");

        return wall_mask;
    }

    // ---------------- Stage 5: lifted roomSegmentation core -----------------
    void segmentFloor()
    {
        // Outside boundary from the navigable map.
        cv::Mat hist_full = navigable_map_.clone();
        cv::normalize(hist_full, hist_full, 0, 255, cv::NORM_MINMAX);
        cv::Mat outside_boundary = cv::Mat::zeros(hist_full.size(), CV_8U);
        cv::threshold(hist_full, outside_boundary, 0, 255, cv::THRESH_BINARY);
        saveDebugImage(outside_boundary, "full_map_1.png");

        if (outside_boundary.type() != CV_8U)
            outside_boundary.convertTo(outside_boundary, CV_8U);

        // Wall source 1: region-grown vertical planes.
        cv::Mat wall_from_plane = getWall(wall_cloud_);

        // Wall source 2: the wall-band column histogram. The gate is applied on
        // the FLOAT hist (hist_threshold_factor x max) -- same semantics as the
        // online normalize->CV_8U rounding at factor 0.5, but a real knob.
        double hist_max_raw = 0;
        cv::minMaxLoc(wall_hist_, nullptr, &hist_max_raw);
        cv::Mat hist_norm;
        cv::normalize(wall_hist_, hist_norm, 0, 255, cv::NORM_MINMAX);
        hist_norm.convertTo(hist_norm, CV_8U);
        saveDebugImage(hist_norm, "walls_skeleton_hist_1_raw.png");

        cv::Mat wall_from_hist;
        if (hist_max_raw > 0.0) {
            wall_from_hist = (wall_hist_ >= cfg_.hist_threshold_factor * hist_max_raw);
        } else {
            wall_from_hist = cv::Mat::zeros(wall_hist_.size(), CV_8U);
        }

        cv::normalize(wall_from_plane, wall_from_plane, 0, 255, cv::NORM_MINMAX);
        wall_from_plane.convertTo(wall_from_plane, CV_8U);
        cv::threshold(wall_from_plane, wall_from_plane, 0, 255, cv::THRESH_BINARY);
        std::printf("[offline_seg] %s: hist max=%.0f kept=%d cells | plane cells=%d\n",
                    floor_.name.c_str(), hist_max_raw, cv::countNonZero(wall_from_hist),
                    cv::countNonZero(wall_from_plane));
        saveDebugImage(wall_from_plane, "wall_from_plane.png");
        saveDebugImage(wall_from_hist, "wall_from_hist.png");

        // Combine (no state-map cull offline).
        cv::Mat walls_skeleton_hist_connected = wall_from_hist.clone();
        wall_from_plane = wall_from_plane | wall_from_hist;

        // Contour processing: keep outer contours, punch back holes >= min area.
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(outside_boundary.clone(), contours, hierarchy, cv::RETR_CCOMP,
                         cv::CHAIN_APPROX_SIMPLE);

        outside_boundary = cv::Mat::zeros(outside_boundary.size(), CV_8U);

        for (size_t i = 0; i < contours.size(); ++i) {
            if (hierarchy[i][3] == -1) {
                cv::drawContours(outside_boundary, contours, i, cv::Scalar(255),
                                 cv::FILLED);

                int child = hierarchy[i][2];
                while (child != -1) {
                    double area = cv::contourArea(contours[child]);
                    if (area >= static_cast<double>(cfg_.min_hole_area)) {
                        cv::drawContours(outside_boundary, contours, child,
                                         cv::Scalar(0), cv::FILLED);
                    }
                    child = hierarchy[child][0];
                }
            }
        }

        saveDebugImage(wall_from_plane, "wall_all.png");

        cv::Mat outside_boundary_connected = outside_boundary.clone();

        // Two variants, deliberately different (G4): the walls-subtracted one
        // seeds the rooms; the hist-only-subtracted one is the watershed
        // background + flood image.
        outside_boundary.setTo(0, wall_from_plane);
        outside_boundary_connected.setTo(0, walls_skeleton_hist_connected);

        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(outside_boundary, labels,
                                                          stats, centroids, 8);

        cv::Mat area_mask = cv::Mat::zeros(outside_boundary.size(), CV_8U);
        for (int i = 1; i < num_labels; ++i) {
            if (stats.at<int>(i, cv::CC_STAT_AREA) > cfg_.min_component_area) {
                cv::Mat label_mask = (labels == i);
                area_mask.setTo(255, label_mask);
            }
        }
        outside_boundary = area_mask.clone();

        num_labels = cv::connectedComponentsWithStats(outside_boundary_connected,
                                                      labels, stats, centroids, 8);
        area_mask = cv::Mat::zeros(outside_boundary_connected.size(), CV_8U);
        for (int i = 1; i < num_labels; ++i) {
            if (stats.at<int>(i, cv::CC_STAT_AREA) > cfg_.min_component_area) {
                cv::Mat label_mask = (labels == i);
                area_mask.setTo(255, label_mask);
            }
        }
        outside_boundary_connected = area_mask.clone();

        cv::Mat full_map, full_map_connected;
        cv::bitwise_not(outside_boundary, full_map);
        cv::bitwise_not(outside_boundary_connected, full_map_connected);
        cv::Mat boundary_mask = full_map.clone();

        saveDebugImage(full_map, "full_map.png");
        saveDebugImage(full_map_connected, "full_map_connected.png");

        // Dilate + seed extraction.
        std::vector<cv::Mat> found_region_masks;
        cv::Mat kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::Mat new_boundary;
        cv::dilate(boundary_mask, new_boundary, kernel_, cv::Point(-1, -1),
                   cfg_.dilation_iteration);

        cv::Mat new_boundary_inv;
        cv::bitwise_not(new_boundary, new_boundary_inv);

        saveDebugImage(new_boundary, "new_boundary.png");

        cv::Mat new_labels, new_stats, centers;
        int num_rooms = cv::connectedComponentsWithStats(new_boundary_inv, new_labels,
                                                         new_stats, centers, 8);

        for (int label = 1; label < num_rooms; ++label) {
            cv::Mat mask = (new_labels == label);
            int area = new_stats.at<int>(label, cv::CC_STAT_AREA);
            if (area > cfg_.min_room_size) {
                new_boundary.setTo(255, mask);
                mask.convertTo(mask, CV_8U, 255);
                found_region_masks.push_back(mask);
            }
        }

        num_seeds_ = static_cast<int>(found_region_masks.size());
        if (num_seeds_ == 0) {
            std::fprintf(stderr,
                         "[offline_seg] %s: no room seeds survived (min_room_size=%d, "
                         "dilation_iteration=%d) -- writing empty outputs\n",
                         floor_.name.c_str(), cfg_.min_room_size,
                         cfg_.dilation_iteration);
            room_mask_cropped_ = room_mask_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                                     .colRange(bbox_[0][1], bbox_[1][1] + 1);
            room_mask_vis_cropped_ =
                room_mask_vis_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                    .colRange(bbox_[0][1], bbox_[1][1] + 1);
            return;
        }

        // Watershed.
        cv::Mat markers = cv::Mat::zeros(boundary_mask.size(), CV_32S);
        cv::Mat bg_mask = (full_map_connected != 0);
        markers.setTo(num_seeds_ + 1, bg_mask);

        for (int i = 0; i < num_seeds_; ++i) {
            cv::Mat mask = (found_region_masks[i] == 255);
            markers.setTo(i + 1, mask);
        }

        {
            cv::Mat seed_vis(markers.size(), CV_8UC3, cv::Scalar(255, 255, 255));
            for (int i = 0; i < num_seeds_; ++i) {
                Eigen::Vector3d color = ors::idToColor(i + 1);
                seed_vis.setTo(cv::Scalar(color[0], color[1], color[2]),
                               found_region_masks[i]);
            }
            saveDebugImage(seed_vis, "seed_mask.png");
        }

        cv::Mat full_map_color;
        cv::cvtColor(full_map_connected, full_map_color, cv::COLOR_GRAY2BGR);
        saveDebugImage(full_map_color, "full_map_color.png");
        cv::watershed(full_map_color, markers);
        std::printf("[offline_seg] %s: %d watershed seeds\n", floor_.name.c_str(),
                    num_seeds_);

        {
            cv::Mat ws_vis(markers.size(), CV_8UC3, cv::Scalar(255, 255, 255));
            for (int r = 0; r < markers.rows; ++r) {
                for (int c = 0; c < markers.cols; ++c) {
                    int v = markers.at<int>(r, c);
                    if (v == -1) {
                        ws_vis.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);
                    } else if (v > 0 && v <= num_seeds_) {
                        Eigen::Vector3d color = ors::idToColor(v);
                        ws_vis.at<cv::Vec3b>(r, c) =
                            cv::Vec3b(static_cast<uchar>(color[0]),
                                      static_cast<uchar>(color[1]),
                                      static_cast<uchar>(color[2]));
                    }
                }
            }
            saveDebugImage(ws_vis, "watershed_markers_vis.png");
        }

        // Door pixels = the watershed borders; image-border doors zeroed.
        cv::Mat door_mask = (markers == -1);
        saveDebugImage(door_mask, "door_mask_raw.png");

        for (int c = 0; c < door_mask.cols; ++c) {
            if (door_mask.at<uchar>(0, c) != 0)
                door_mask.at<uchar>(0, c) = 0;
            if (door_mask.at<uchar>(door_mask.rows - 1, c) != 0)
                door_mask.at<uchar>(door_mask.rows - 1, c) = 0;
        }
        for (int r = 0; r < door_mask.rows; ++r) {
            if (door_mask.at<uchar>(r, 0) != 0)
                door_mask.at<uchar>(r, 0) = 0;
            if (door_mask.at<uchar>(r, door_mask.cols - 1) != 0)
                door_mask.at<uchar>(r, door_mask.cols - 1) = 0;
        }

        // Background and borders -> 0; the labels 1..N are the final room ids
        // (one shot: no reconciliation with a previous cycle).
        markers.setTo(0, markers == num_seeds_ + 1);
        markers.setTo(0, markers == -1);

        room_mask_cropped_ = room_mask_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                                 .colRange(bbox_[0][1], bbox_[1][1] + 1);
        room_mask_vis_cropped_ = room_mask_vis_.rowRange(bbox_[0][0], bbox_[1][0] + 1)
                                     .colRange(bbox_[0][1], bbox_[1][1] + 1);
        markers.copyTo(room_mask_cropped_);

        {
            cv::Mat rm_new = (room_mask_cropped_ > 0);
            saveDebugImage(rm_new, "room_mask_new.png");
        }

        finalizeRooms();
        detectDoors(door_mask);
        annotate();
    }

    // ---------------- Stage 6: per-room finalization (lift of the online
    // updateRooms second pass; cropped coords + _cropped helpers throughout) --
    void finalizeRooms()
    {
        long pia_total_us = 0;
        int pia_count = 0;

        for (int id = 1; id <= num_seeds_; ++id) {
            cv::Mat mask_new = (room_mask_cropped_ == id);
            if (cv::countNonZero(mask_new) == 0) {
                continue;
            }

            ors::OfflineRoom room;
            room.id = id;
            room.show_id = id;
            room.room_mask = ors::cropRoomMask(mask_new);
            room.polygon = polygonFromMask(mask_new);
            room.area = cv::countNonZero(mask_new) * room_resolution_ * room_resolution_;
            cv::findNonZero(mask_new, room.points);

            Eigen::Vector3f room_centroid(0.0, 0.0, 0.0);
            for (const auto &pt : room.points) {
                Eigen::Vector3i pt_voxel(pt.y, pt.x, 0);
                Eigen::Vector3f pt_position = ors::voxel_to_point_cropped(
                    pt_voxel, shift_, room_resolution_, bbox_);
                room_centroid.x() += pt_position.x();
                room_centroid.y() += pt_position.y();
            }
            room_centroid.x() /= room.points.size();
            room_centroid.y() /= room.points.size();
            room_centroid.z() = floor_.robot_z;
            room.centroid = room_centroid;

            // Interior point = pole of inaccessibility (lifted verbatim; the
            // medoid of the near-max clearance ridge, guaranteed inside).
            {
                auto t_pia_0 = std::chrono::high_resolution_clock::now();
                cv::Rect rect = cv::boundingRect(mask_new);
                const int pia_margin = 2;
                rect.x = std::max(0, rect.x - pia_margin);
                rect.y = std::max(0, rect.y - pia_margin);
                rect.width = std::min(mask_new.cols - rect.x, rect.width + 2 * pia_margin);
                rect.height =
                    std::min(mask_new.rows - rect.y, rect.height + 2 * pia_margin);
                cv::Mat sub = mask_new(rect).clone();
                cv::Mat dist;
                cv::distanceTransform(sub, dist, cv::DIST_L2, 3);
                double max_dist = 0.0;
                cv::minMaxLoc(dist, nullptr, &max_dist, nullptr, nullptr);
                const float pia_eps = 0.5f;
                std::vector<cv::Point> ridge;
                for (int r = 0; r < dist.rows; ++r) {
                    const float *drow = dist.ptr<float>(r);
                    for (int c = 0; c < dist.cols; ++c) {
                        if (drow[c] >= max_dist - pia_eps) {
                            ridge.emplace_back(c, r);
                        }
                    }
                }
                if (!ridge.empty()) {
                    double mean_x = 0.0, mean_y = 0.0;
                    for (const auto &p : ridge) {
                        mean_x += p.x;
                        mean_y += p.y;
                    }
                    mean_x /= ridge.size();
                    mean_y /= ridge.size();
                    cv::Point pia_cell = ridge.front();
                    double best_d2 = 1e18;
                    for (const auto &p : ridge) {
                        double d2 = (p.x - mean_x) * (p.x - mean_x) +
                                    (p.y - mean_y) * (p.y - mean_y);
                        if (d2 < best_d2) {
                            best_d2 = d2;
                            pia_cell = p;
                        }
                    }
                    Eigen::Vector3i pia_voxel(rect.y + pia_cell.y, rect.x + pia_cell.x,
                                              0);
                    Eigen::Vector3f pia_pos = ors::voxel_to_point_cropped(
                        pia_voxel, shift_, room_resolution_, bbox_);
                    room.interior_point =
                        Eigen::Vector3f(pia_pos.x(), pia_pos.y(), floor_.robot_z);
                } else {
                    room.interior_point = room.centroid;
                }
                auto t_pia_1 = std::chrono::high_resolution_clock::now();
                pia_total_us += std::chrono::duration_cast<std::chrono::microseconds>(
                                    t_pia_1 - t_pia_0)
                                    .count();
                pia_count++;
            }

            Eigen::Vector3d color = ors::idToColor(id);
            room_mask_vis_cropped_.setTo(cv::Scalar(color[0], color[1], color[2]),
                                         mask_new);

            rooms_[id] = std::move(room);
        }

        if (pia_count > 0) {
            std::printf("[offline_seg] %s: interior points for %d room(s) in %.3f ms\n",
                        floor_.name.c_str(), pia_count, pia_total_us / 1000.0);
        }
    }

    std::vector<Eigen::Vector2f> polygonFromMask(const cv::Mat &mask)
    {
        std::vector<std::vector<cv::Point>> current_room_contours;
        cv::findContours(mask, current_room_contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> largest_contour;
        if (!current_room_contours.empty()) {
            largest_contour = current_room_contours[0];
            for (const auto &contour : current_room_contours) {
                if (cv::contourArea(contour) > cv::contourArea(largest_contour)) {
                    largest_contour = contour;
                }
            }
        }

        std::vector<Eigen::Vector2f> polygon;
        polygon.reserve(largest_contour.size());
        for (const auto &pt : largest_contour) {
            Eigen::Vector3i pt_voxel(pt.y, pt.x, 0);
            Eigen::Vector3f pt_position =
                ors::voxel_to_point_cropped(pt_voxel, shift_, room_resolution_, bbox_);
            polygon.emplace_back(pt_position.x(), pt_position.y());
        }
        return polygon;
    }

    // ---------------- Stage 7: lifted door pass ------------------------------
    static float minPixelGap(const std::vector<cv::Point> &a,
                             const std::vector<cv::Point> &b)
    {
        float best_sq = std::numeric_limits<float>::max();
        for (const auto &pa : a) {
            for (const auto &pb : b) {
                const float dx = static_cast<float>(pa.x - pb.x);
                const float dy = static_cast<float>(pa.y - pb.y);
                best_sq = std::min(best_sq, dx * dx + dy * dy);
            }
        }
        return std::sqrt(best_sq);
    }

    void detectDoors(cv::Mat &door_mask)
    {
        assert(room_mask_cropped_.size() == door_mask.size());

        // 3x3 label filter: 1 distinct id -> not a door; >2 -> ambiguous junction.
        for (int r = 1; r < door_mask.rows - 1; ++r) {
            for (int c = 1; c < door_mask.cols - 1; ++c) {
                if (door_mask.at<uchar>(r, c) != 0) {
                    std::set<int> neighborLabels;

                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            if (r + dr < 0 || r + dr >= door_mask.rows || c + dc < 0 ||
                                c + dc >= door_mask.cols || (dr == 0 && dc == 0)) {
                                continue;
                            }
                            int label = room_mask_cropped_.at<int>(r + dr, c + dc);
                            if (label > 0) {
                                neighborLabels.insert(label);
                            }
                        }
                    }

                    if (neighborLabels.size() == 1) {
                        door_mask.at<uchar>(r, c) = 0;
                    }
                    if (neighborLabels.size() > 2) {
                        for (int dr = -1; dr <= 1; ++dr) {
                            for (int dc = -1; dc <= 1; ++dc) {
                                if (r + dr < 0 || r + dr >= door_mask.rows ||
                                    c + dc < 0 || c + dc >= door_mask.cols) {
                                    continue;
                                }
                                door_mask.at<uchar>(r + dr, c + dc) = 0;
                            }
                        }
                    }
                }
            }
        }
        saveDebugImage(door_mask, "door_mask_filtered.png");

        cv::Mat labels, stats, centroids;
        int num_labels =
            cv::connectedComponentsWithStats(door_mask, labels, stats, centroids, 8);

        // Border components touching exactly two rooms are door FRAGMENTS: the
        // 3x3 filter above can chop one physical opening into several pieces,
        // so same-pair fragments closer than door_merge_gap_m get fused before
        // the doors are finalized. A genuine double doorway (real wall pier
        // between the openings) stays split.
        struct DoorFragment {
            int room_a, room_b;
            std::vector<cv::Point> pixels;
        };
        std::vector<DoorFragment> fragments;

        std::vector<cv::Point> non_zero_points;
        for (int label = 1; label < num_labels; ++label) {
            cv::Mat label_mask = (labels == label);
            non_zero_points.clear();
            cv::findNonZero(label_mask, non_zero_points);

            std::set<int> neighborLabels;

            for (const auto &point : non_zero_points) {
                int r = point.y;
                int c = point.x;

                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        if (r + dr < 0 || r + dr >= door_mask.rows || c + dc < 0 ||
                            c + dc >= door_mask.cols || (dr == 0 && dc == 0)) {
                            continue;
                        }
                        neighborLabels.insert(
                            room_mask_cropped_.at<int>(r + dr, c + dc));
                    }
                }
            }

            neighborLabels.erase(0);
            if (neighborLabels.empty()) {
                continue;
            }
            if (neighborLabels.size() != 2) {
                std::fprintf(stderr,
                             "[offline_seg] %s: door component %d touches %zu rooms, "
                             "skipping\n",
                             floor_.name.c_str(), label, neighborLabels.size());
                continue;
            }

            int room_label_1 = *neighborLabels.begin();
            int room_label_2 = *std::next(neighborLabels.begin());

            if (room_label_1 > room_label_2) {
                std::swap(room_label_1, room_label_2);
            }
            if (rooms_.count(room_label_1) == 0 || rooms_.count(room_label_2) == 0) {
                continue;
            }

            rooms_[room_label_1].neighbors.insert(room_label_2);
            rooms_[room_label_2].neighbors.insert(room_label_1);

            fragments.push_back({room_label_1, room_label_2, non_zero_points});
        }

        // Greedy transitive merge. min-gap(A, B∪C) = min(gap(A,B), gap(A,C)),
        // so absorbing j and re-scanning from i+1 reaches the full transitive
        // closure without a union-find.
        const float merge_gap_px = cfg_.door_merge_gap_m * room_resolution_inv_;
        const size_t fragments_before = fragments.size();
        for (size_t i = 0; i < fragments.size(); ++i) {
            for (size_t j = i + 1; j < fragments.size();) {
                if (fragments[j].room_a == fragments[i].room_a &&
                    fragments[j].room_b == fragments[i].room_b &&
                    minPixelGap(fragments[i].pixels, fragments[j].pixels) <=
                        merge_gap_px) {
                    fragments[i].pixels.insert(fragments[i].pixels.end(),
                                               fragments[j].pixels.begin(),
                                               fragments[j].pixels.end());
                    fragments.erase(fragments.begin() +
                                    static_cast<std::ptrdiff_t>(j));
                    j = i + 1;
                } else {
                    ++j;
                }
            }
        }
        if (fragments.size() < fragments_before) {
            std::printf("[offline_seg] %s: merged %zu door fragment(s) into %zu "
                        "door(s) (gap <= %.2f m)\n",
                        floor_.name.c_str(), fragments_before, fragments.size(),
                        cfg_.door_merge_gap_m);
        }

        Eigen::MatrixXi adjacency_matrix = Eigen::MatrixXi::Zero(num_seeds_, num_seeds_);
        for (const auto &frag : fragments) {
            ors::OfflineDoor door;
            door.id = static_cast<int>(doors_.size());
            door.door_id = adjacency_matrix(frag.room_a - 1, frag.room_b - 1);
            adjacency_matrix(frag.room_a - 1, frag.room_b - 1) += 1;
            adjacency_matrix(frag.room_b - 1, frag.room_a - 1) += 1;
            door.room_a = frag.room_a;
            door.room_b = frag.room_b;
            door.pixel_count = static_cast<int>(frag.pixels.size());

            Eigen::Vector3f centroid_sum = Eigen::Vector3f::Zero();
            for (const auto &point : frag.pixels) {
                Eigen::Vector3i door_idx(point.y, point.x, 0);
                Eigen::Vector3f door_position = ors::voxel_to_point_cropped(
                    door_idx, shift_, room_resolution_, bbox_);
                centroid_sum += Eigen::Vector3f(door_position.x(), door_position.y(),
                                                floor_.robot_z);

                room_mask_vis_cropped_.at<cv::Vec3b>(point.y, point.x) =
                    cv::Vec3b(0, 0, 255);
            }
            door.centroid = centroid_sum / static_cast<float>(frag.pixels.size());
            doors_.push_back(door);
        }
    }

    // ---------------- annotated final vis ------------------------------------
    void annotate()
    {
        cv::Mat annotated = room_mask_vis_cropped_.clone();
        for (const auto &[id, room] : rooms_) {
            cv::Mat mask = (room_mask_cropped_ == id);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::drawContours(annotated, contours, -1, cv::Scalar(0, 0, 0), 1);

            Eigen::Vector3i pia_px = ors::point_to_voxel_cropped(
                room.interior_point, shift_, room_resolution_inv_, bbox_);
            cv::circle(annotated, cv::Point(pia_px[1], pia_px[0]), 2,
                       cv::Scalar(0, 0, 0), -1);
            cv::putText(annotated, std::to_string(id),
                        cv::Point(pia_px[1] + 3, pia_px[0] + 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
        }
        for (const auto &door : doors_) {
            Eigen::Vector3i door_px = ors::point_to_voxel_cropped(
                door.centroid, shift_, room_resolution_inv_, bbox_);
            cv::circle(annotated, cv::Point(door_px[1], door_px[0]), 3,
                       cv::Scalar(255, 0, 0), 1);
        }
        saveDebugImage(annotated, "final_annotated.png");
        saveDebugImage(room_mask_vis_cropped_, "room_segmentation_visualization.png");
    }

    // ---------------- outputs ------------------------------------------------
    // Debug images keep the online transpose+flip viewing convention; machine
    // artifacts (room_mask.png / room_mask_vis.png) stay in raw grid
    // orientation, documented in mask_meta.json.
    void saveDebugImage(const cv::Mat &image, const std::string &filename)
    {
        if (image.empty()) {
            return;
        }
        cv::Mat flipped_image;
        cv::transpose(image, flipped_image);
        cv::flip(flipped_image, flipped_image, 0);
        cv::imwrite(debug_dir_ + "/" + filename, flipped_image);
    }

    json metaJson() const
    {
        const cv::Size cropped(bbox_[1][1] - bbox_[0][1] + 1,
                               bbox_[1][0] - bbox_[0][0] + 1);
        json meta;
        meta["floor"] = floor_.name;
        meta["frame"] = cfg_.frame;
        meta["robot_z"] = floor_.robot_z;
        meta["slab_z"] = {floor_.slab_z_min, floor_.slab_z_max};
        meta["wall_band_z"] = {floor_.wall_thres_height, floor_.ceiling_height};
        meta["resolution"] = room_resolution_;
        meta["grid_dims"] = {room_voxel_dimension_[0], room_voxel_dimension_[1],
                             room_voxel_dimension_[2]};
        meta["origin_shift"] = {shift_.x(), shift_.y(), shift_.z()};
        meta["bbox"] = {{"row", {bbox_[0][0], bbox_[1][0]}},
                        {"col", {bbox_[0][1], bbox_[1][1]}}};
        meta["image_dims"] = {{"rows", cropped.height}, {"cols", cropped.width}};
        meta["pixel_to_world"] =
            "x = (row + bbox.row[0] - origin_shift[0]) * resolution; "
            "y = (col + bbox.col[0] - origin_shift[1]) * resolution "
            "(cell corner; +resolution/2 for center)";
        meta["orientation_note"] =
            "room_mask.png / room_mask_vis.png are raw grid orientation "
            "(row = world +x, col = world +y). debug/*.png are transposed + "
            "vertically flipped for viewing, matching the online node's dumps.";
        meta["params"] = {
            {"exploredAreaVoxelSize", cfg_.explored_area_voxel_size},
            {"room_resolution", cfg_.room_resolution},
            {"ceilingHeight_", cfg_.ceiling_height_base},
            {"wall_thres_height_", cfg_.wall_thres_height_base},
            {"dilation_iteration", cfg_.dilation_iteration},
            {"outward_distance_0", cfg_.outward_distance_0},
            {"outward_distance_1", cfg_.outward_distance_1},
            {"distance_threshold", cfg_.distance_threshold},
            {"distance_angel_threshold", cfg_.distance_angel_threshold},
            {"angle_threshold_deg", cfg_.angle_threshold_deg},
            {"min_room_size", cfg_.min_room_size},
            {"normal_search_num", cfg_.normal_search_num},
            {"rg_min_cluster_size", cfg_.rg_min_cluster_size},
            {"rg_neighbors", cfg_.rg_neighbors},
            {"rg_smoothness_deg", cfg_.rg_smoothness_deg},
            {"rg_curvature", cfg_.rg_curvature},
            {"plane_min_height", cfg_.plane_min_height},
            {"min_hole_area", cfg_.min_hole_area},
            {"min_component_area", cfg_.min_component_area},
            {"hist_threshold_factor", cfg_.hist_threshold_factor},
            {"slab_below", cfg_.slab_below},
            {"grid_margin_px", cfg_.grid_margin_px},
            {"door_merge_gap_m", cfg_.door_merge_gap_m},
            {"wall_stage_leaf_size", cfg_.wall_stage_leaf_size}};
        return meta;
    }

    void writeOutputs()
    {
        // room_mask.png: 16-bit labels, raw orientation (CV_32S is not
        // PNG-writable; N << 65535 always).
        cv::Mat mask16;
        room_mask_cropped_.convertTo(mask16, CV_16U);
        cv::imwrite(out_dir_ + "/room_mask.png", mask16);
        cv::imwrite(out_dir_ + "/room_mask_vis.png", room_mask_vis_cropped_);

        std::ofstream(out_dir_ + "/mask_meta.json") << metaJson().dump(2) << "\n";

        json rooms_json;
        rooms_json["floor"] = floor_.name;
        rooms_json["robot_z"] = floor_.robot_z;
        rooms_json["rooms"] = json::array();
        for (const auto &[id, room] : rooms_) {
            json r;
            r["id"] = room.id;
            r["show_id"] = room.show_id;
            r["area_m2"] = room.area;
            r["pixel_count"] = static_cast<int>(room.points.size());
            r["centroid"] = {room.centroid.x(), room.centroid.y(), room.centroid.z()};
            r["interior_point"] = {room.interior_point.x(), room.interior_point.y(),
                                   room.interior_point.z()};
            r["polygon"] = json::array();
            for (const auto &p : room.polygon) {
                r["polygon"].push_back({p.x(), p.y()});
            }
            r["neighbors"] = room.neighbors;
            rooms_json["rooms"].push_back(r);
        }
        std::ofstream(out_dir_ + "/rooms.json") << rooms_json.dump(2) << "\n";

        json doors_json;
        doors_json["floor"] = floor_.name;
        doors_json["doors"] = json::array();
        for (const auto &door : doors_) {
            json d;
            d["id"] = door.id;
            d["door_id"] = door.door_id;
            d["rooms"] = {door.room_a, door.room_b};
            d["centroid"] = {door.centroid.x(), door.centroid.y(), door.centroid.z()};
            d["pixel_count"] = door.pixel_count;
            doors_json["doors"].push_back(d);
        }
        std::ofstream(out_dir_ + "/doors.json") << doors_json.dump(2) << "\n";
    }

    void writeEmptyOutputs(const std::string &reason)
    {
        json rooms_json = {{"floor", floor_.name},
                           {"robot_z", floor_.robot_z},
                           {"error", reason},
                           {"rooms", json::array()}};
        std::ofstream(out_dir_ + "/rooms.json") << rooms_json.dump(2) << "\n";
        json doors_json = {
            {"floor", floor_.name}, {"error", reason}, {"doors", json::array()}};
        std::ofstream(out_dir_ + "/doors.json") << doors_json.dump(2) << "\n";
    }

    // ---------------- members (names mirror the online node) ----------------
    ors::OfflineConfig cfg_;
    ors::FloorSpec floor_;
    std::string out_dir_;
    std::string debug_dir_;

    float room_resolution_ = 0.1f;
    float room_resolution_inv_ = 10.0f;
    float wall_thres_height_ = 0.0f;  // absolute (robot_z + base)
    float ceiling_height_ = 0.0f;     // absolute (robot_z + base)

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_;       // slab, downsampled, normals
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr wall_cloud_;  // = cloud_ or coarser copy

    std::vector<int> room_voxel_dimension_ = {1, 1, 1};
    Eigen::Vector3f shift_ = Eigen::Vector3f::Zero();
    std::vector<int> navigable_voxels_;
    cv::Mat navigable_map_all_, navigable_map_;
    cv::Mat wall_hist_all_, wall_hist_;
    cv::Mat room_mask_, room_mask_vis_;
    cv::Mat room_mask_cropped_, room_mask_vis_cropped_;  // views into the above
    std::vector<Eigen::Vector2i> bbox_;
    std::vector<ors::PlaneInfo> plane_infos_;

    int num_seeds_ = 0;
    std::map<int, ors::OfflineRoom> rooms_;
    std::vector<ors::OfflineDoor> doors_;
};

// ---------------- config / blueprint / CLI parsing ---------------------------

// Key lookup priority: `room_segmentation: ros__parameters:` -> `/**:
// ros__parameters:` -> flat top level, so both the existing scenario yamls
// (e.g. go2w_bag_direct.yaml) and a plain flat yaml work unchanged.
YAML::Node findKey(const YAML::Node &root, const std::string &key)
{
    const char *scopes[] = {"room_segmentation", "/**"};
    for (const char *scope : scopes) {
        if (root[scope] && root[scope]["ros__parameters"] &&
            root[scope]["ros__parameters"][key]) {
            return root[scope]["ros__parameters"][key];
        }
    }
    if (root[key]) {
        return root[key];
    }
    return YAML::Node(YAML::NodeType::Undefined);
}

template <typename T>
void readParam(const YAML::Node &root, const std::string &key, T &out)
{
    YAML::Node node = findKey(root, key);
    if (node.IsDefined() && !node.IsNull()) {
        out = node.as<T>();
    }
}

ors::OfflineConfig loadConfig(const std::string &path)
{
    ors::OfflineConfig cfg;
    if (path.empty()) {
        return cfg;
    }
    YAML::Node root = YAML::LoadFile(path);
    readParam(root, "exploredAreaVoxelSize", cfg.explored_area_voxel_size);
    readParam(root, "room_resolution", cfg.room_resolution);
    readParam(root, "ceilingHeight_", cfg.ceiling_height_base);
    readParam(root, "wall_thres_height_", cfg.wall_thres_height_base);
    readParam(root, "dilation_iteration", cfg.dilation_iteration);
    readParam(root, "outward_distance_0", cfg.outward_distance_0);
    readParam(root, "outward_distance_1", cfg.outward_distance_1);
    readParam(root, "distance_threshold", cfg.distance_threshold);
    readParam(root, "distance_angel_threshold", cfg.distance_angel_threshold);
    readParam(root, "angle_threshold_deg", cfg.angle_threshold_deg);
    readParam(root, "min_room_size", cfg.min_room_size);
    readParam(root, "normal_search_num", cfg.normal_search_num);
    readParam(root, "rg_min_cluster_size", cfg.rg_min_cluster_size);
    readParam(root, "rg_neighbors", cfg.rg_neighbors);
    readParam(root, "rg_smoothness_deg", cfg.rg_smoothness_deg);
    readParam(root, "rg_curvature", cfg.rg_curvature);
    readParam(root, "plane_min_height", cfg.plane_min_height);
    readParam(root, "min_hole_area", cfg.min_hole_area);
    readParam(root, "min_component_area", cfg.min_component_area);
    readParam(root, "hist_threshold_factor", cfg.hist_threshold_factor);
    readParam(root, "slab_below", cfg.slab_below);
    readParam(root, "grid_margin_px", cfg.grid_margin_px);
    readParam(root, "door_merge_gap_m", cfg.door_merge_gap_m);
    readParam(root, "wall_stage_leaf_size", cfg.wall_stage_leaf_size);
    readParam(root, "frame", cfg.frame);
    return cfg;
}

// blueprint.yaml: floors[].name + collision_range[0] (= robot z on that floor).
// Everything else in the file belongs to the on-robot blueprint tool.
std::vector<ors::FloorSpec> loadFloors(const std::string &path,
                                       const ors::OfflineConfig &cfg)
{
    YAML::Node root = YAML::LoadFile(path);
    if (!root["floors"] || !root["floors"].IsSequence()) {
        throw std::runtime_error("floors yaml has no 'floors:' sequence");
    }

    std::vector<ors::FloorSpec> floors;
    for (const auto &f : root["floors"]) {
        ors::FloorSpec spec;
        spec.name = f["name"].as<std::string>();
        if (!f["collision_range"] || !f["collision_range"].IsSequence() ||
            f["collision_range"].size() < 1) {
            throw std::runtime_error("floor '" + spec.name +
                                     "' has no collision_range");
        }
        spec.robot_z = f["collision_range"][0].as<float>();
        floors.push_back(spec);
    }

    std::sort(floors.begin(), floors.end(),
              [](const ors::FloorSpec &a, const ors::FloorSpec &b) {
                  return a.robot_z < b.robot_z;
              });

    for (size_t i = 0; i < floors.size(); ++i) {
        ors::FloorSpec &spec = floors[i];
        spec.slab_z_min = spec.robot_z - cfg.slab_below;
        spec.slab_z_max = spec.robot_z + cfg.ceiling_height_base;
        if (i + 1 < floors.size()) {
            // Bound by the next floor's own slab bottom (just below its floor
            // surface) so the slabs stay disjoint in stairwells.
            spec.slab_z_max =
                std::min(spec.slab_z_max, floors[i + 1].robot_z - cfg.slab_below);
        }
        spec.wall_thres_height = spec.robot_z + cfg.wall_thres_height_base;
        spec.ceiling_height = spec.robot_z + cfg.ceiling_height_base;
    }
    return floors;
}

}  // namespace

// ---------------- library entry points (room_segmentation_run.h) -------------

namespace offline_room_segmentation {

OfflineConfig LoadOfflineConfig(const std::string &path)
{
    return loadConfig(path);
}

std::vector<FloorSpec> LoadFloorSpecs(const std::string &path, const OfflineConfig &cfg)
{
    return loadFloors(path, cfg);
}

SegmentationRunResult RunRoomSegmentation(const std::string &pcd_path,
                                          const std::vector<FloorSpec> &floors,
                                          const OfflineConfig &cfg,
                                          const std::string &out_dir,
                                          const std::string &only_floor)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr building(new pcl::PointCloud<pcl::PointXYZ>);
    const double t_load = NowSec();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *building) < 0) {
        throw std::runtime_error("failed to load " + pcd_path);
    }
    std::printf("[offline_seg] loaded %zu points from %s in %.1f s\n",
                building->size(), pcd_path.c_str(), NowSec() - t_load);

    SegmentationRunResult result;
    for (const auto &floor : floors) {
        if (!only_floor.empty() && floor.name != only_floor) {
            continue;
        }
        std::printf("[offline_seg] === %s: robot_z=%.3f slab=[%.2f, %.2f] "
                    "wall_band=[%.2f, %.2f] ===\n",
                    floor.name.c_str(), floor.robot_z, floor.slab_z_min,
                    floor.slab_z_max, floor.wall_thres_height, floor.ceiling_height);
        OfflineRoomSegmenter seg(cfg, floor, out_dir + "/" + floor.name);
        if (seg.run(building)) {
            result.processed++;
        } else {
            result.failed++;
        }
    }
    std::printf("[offline_seg] done: %d floor(s) processed, %d failed\n",
                result.processed, result.failed);
    return result;
}

}  // namespace offline_room_segmentation
