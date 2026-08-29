"""
bev_mapper_node
===============
Thin ROS 2 wrapper around LidarBEVMapper (+ FrontierDetector).

Subscribes
  /registered_scan   sensor_msgs/PointCloud2   SLAM-registered scan in the map frame
  /state_estimation  nav_msgs/Odometry         robot pose from SLAM

Publishes (timer-driven, rates in config)
  /bev_map/grid       nav_msgs/OccupancyGrid   full map, map frame: -1 unknown / 0 free / 100 occupied
  /bev_map/local      sensor_msgs/Image rgb8   robot-centred crop (output_size x output_size, north up)
                                               with trajectory, agent arrow and frontier dots
  /bev_map/frontiers  sensor_msgs/PointCloud2  frontier centres in the map frame

The map/frontier code (lidar_bev_mapper.py, coord_utils.py,
frontier_detector.py) is copied verbatim from
Navigation-Physical-Experiment/src/vlm_nav_bridge. This node is the
scan/pose plumbing of that package's vlm_navigator_node.py
(_pose_callback / _lookup_pose_at / _scan_callback / _parse_pointcloud2)
without the VLM, camera, target-detection and /way_point parts: it only
reads the two SLAM topics and never steers the robot.
"""

import math
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header

from .lidar_bev_mapper import LidarBEVMapper, BEVMapperConfig
from .frontier_detector import FrontierDetector, FrontierConfig
from .coord_utils import quaternion_to_yaw, global_cell_to_world, world_to_global_cell


class BEVMapperNode(Node):

    def __init__(self):
        super().__init__('bev_mapper')
        self._declare_parameters()
        p = self.get_parameter

        bev_cfg = BEVMapperConfig(
            resolution=p('map_resolution').value,
            map_size=p('map_size').value,
            vision_range=p('vision_range').value,
            obstacle_height_min=p('obstacle_height_min').value,
            obstacle_height_max=p('obstacle_height_max').value,
            map_pred_threshold=p('map_pred_threshold').value,
            exp_pred_threshold=p('exp_pred_threshold').value,
            explored_max_rays_per_scan=p('explored_max_rays_per_scan').value,
            scan_denoise_enable=p('scan_denoise_enable').value,
            scan_denoise_voxel_size=p('scan_denoise_voxel_size').value,
            scan_denoise_min_points=p('scan_denoise_min_points').value,
            obstacle_render_dilate_ksize=p('obstacle_render_dilate_ksize').value,
            output_size=p('output_size').value,
            hfov_deg=p('hfov_deg').value,
        )
        self.mapper = LidarBEVMapper(bev_cfg)

        self.publish_frontiers = bool(p('publish_frontiers').value)
        frontier_cfg = FrontierConfig(
            exp_threshold=p('frontier_exp_threshold').value,
            map_pred_threshold=bev_cfg.map_pred_threshold,
            dilate_wall_ksize=p('frontier_dilate_wall_ksize').value,
            close_explore_ksize=p('frontier_close_explore_ksize').value,
            min_frontier_area=p('frontier_min_area').value,
            clear_border_px=p('frontier_clear_border_px').value,
            min_distance_m=p('frontier_min_distance_m').value,
            top_k=p('frontier_top_k').value,
            resolution=bev_cfg.resolution,
            output_size=bev_cfg.output_size,
        )
        self.frontier_detector = FrontierDetector(frontier_cfg)

        self.grid_frame = str(p('grid_frame').value)
        self.grid_z_offset = float(p('grid_z_offset').value)
        self.max_sensor_skew_sec = float(p('max_sensor_skew_sec').value)
        self.self_exclusion_radius = float(p('self_exclusion_radius').value)
        self.clear_footprint_radius = float(p('clear_footprint_radius').value)
        self.pose_update_period = 1.0 / max(1e-3, float(p('pose_update_rate').value))
        # The FOV wedge only makes sense for a forward camera.
        self.draw_fov = bev_cfg.hfov_deg < 360.0

        # ---- state -------------------------------------------------------
        self._pose_history = deque()          # (t, x, y, z, yaw), last 10 s
        self.latest_pose = None               # (x, y, z, yaw)
        self.latest_pose_stamp_s: Optional[float] = None
        self.odom_frame = 'map'
        self.start_z: Optional[float] = None
        self._last_pose_update_t = 0.0
        self._scan_seq = 0                    # scans integrated so far
        self._grid_seq = -1                   # scan_seq at last grid publish
        self._local_seq = -1
        self._frontiers_world = np.empty((0, 2), dtype=np.float32)
        self._frontiers_local = np.empty((0, 2), dtype=np.float32)
        # timing summary
        self._upd_ms_sum = 0.0
        self._upd_ms_max = 0.0
        self._upd_n = 0
        self._stale_pose_n = 0

        # ---- I/O ---------------------------------------------------------
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, p('scan_topic').value,
                                 self._scan_callback, scan_qos)
        self.create_subscription(Odometry, p('odom_topic').value,
                                 self._pose_callback, 10)

        self.grid_pub = self.create_publisher(OccupancyGrid, '/bev_map/grid', 1)
        self.local_pub = self.create_publisher(Image, '/bev_map/local', 1)
        self.frontier_pub = self.create_publisher(PointCloud2, '/bev_map/frontiers', 1)

        self.create_timer(1.0 / max(1e-3, float(p('local_rate').value)), self._publish_local)
        self.create_timer(1.0 / max(1e-3, float(p('grid_rate').value)), self._publish_grid)
        self.create_timer(5.0, self._log_summary)

        self.get_logger().info(
            f'bev_mapper: {self.mapper.global_cells}x{self.mapper.global_cells} cells @ '
            f'{bev_cfg.resolution} m ({bev_cfg.map_size} m), hfov={bev_cfg.hfov_deg} deg, '
            f'obstacle z in [{bev_cfg.obstacle_height_min:+.2f}, '
            f'{bev_cfg.obstacle_height_max:+.2f}] m rel. to the sensor, self-exclusion '
            f'{self.self_exclusion_radius} m; waiting for '
            f'{p("odom_topic").value} + {p("scan_topic").value}')

    # ------------------------------------------------------------------
    def _declare_parameters(self):
        d = self.declare_parameter
        d('scan_topic', '/registered_scan')
        d('odom_topic', '/state_estimation')
        d('grid_frame', '')             # '' -> frame_id of the odometry messages
        d('grid_z_offset', -0.5)        # OccupancyGrid z = start pose z + this
        d('local_rate', 5.0)            # Hz, /bev_map/local (+ frontier extraction)
        d('grid_rate', 1.0)             # Hz, /bev_map/grid + /bev_map/frontiers
        d('pose_update_rate', 10.0)     # Hz cap for pose-only map updates (agent/trail channels)
        d('max_sensor_skew_sec', 0.5)
        d('self_exclusion_radius', 0.8)  # m; drop returns this close (2-D) to the sensor = robot body
        d('clear_footprint_radius', 0.4)  # m; cells the robot drives through are forced free
        d('publish_frontiers', True)
        # BEVMapperConfig
        d('map_resolution', 0.05)
        d('map_size', 67.2)
        d('vision_range', 100)
        d('obstacle_height_min', -0.4)
        d('obstacle_height_max', 1.0)
        d('map_pred_threshold', 1.0)
        d('exp_pred_threshold', 1.0)
        d('explored_max_rays_per_scan', 512)
        d('scan_denoise_enable', True)
        d('scan_denoise_voxel_size', 0.10)
        d('scan_denoise_min_points', 2)
        d('obstacle_render_dilate_ksize', 3)
        d('output_size', 448)
        d('hfov_deg', 360.0)
        # FrontierConfig
        d('frontier_exp_threshold', 0.1)
        d('frontier_dilate_wall_ksize', 1)
        d('frontier_close_explore_ksize', 1)
        d('frontier_min_area', 4)
        d('frontier_clear_border_px', 2)
        d('frontier_min_distance_m', 0.7)
        d('frontier_top_k', 32)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _pose_callback(self, msg: Odometry):
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.latest_pose = (float(pos.x), float(pos.y), float(pos.z), float(yaw))
        self.latest_pose_stamp_s = t
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id
        self._pose_history.append((t,) + self.latest_pose)
        while self._pose_history and t - self._pose_history[0][0] > 10.0:
            self._pose_history.popleft()

        if not self.mapper.is_initialised:
            self.start_z = float(pos.z)
            self.mapper.reset(pos.x, pos.y, pos.z)
            self.get_logger().info(
                f'Map initialised at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) in '
                f'frame "{self.odom_frame}".')
            return

        # Keep the agent/trail channels and the local crop moving between
        # scans (as the source node does), but capped at pose_update_rate.
        now = time.monotonic()
        if now - self._last_pose_update_t >= self.pose_update_period:
            self._last_pose_update_t = now
            self._clear_footprint(self.latest_pose[0], self.latest_pose[1])
            self.mapper.update(None, *self.latest_pose)

    def _lookup_pose_at(self, stamp_s: float):
        """(x, y, z, yaw, matched_stamp) interpolated from the pose history."""
        if not self._pose_history:
            return self.latest_pose + (self.latest_pose_stamp_s,)
        history = list(self._pose_history)
        before = [e for e in history if e[0] <= stamp_s]
        after = [e for e in history if e[0] > stamp_s]
        if not before:
            t, x, y, z, yaw = after[0]
            return x, y, z, yaw, t
        if not after:
            t, x, y, z, yaw = before[-1]
            return x, y, z, yaw, t
        t0, x0, y0, z0, yaw0 = before[-1]
        t1, x1, y1, z1, yaw1 = after[0]
        a = (stamp_s - t0) / max(t1 - t0, 1e-9)
        dyaw = ((yaw1 - yaw0) + math.pi) % (2.0 * math.pi) - math.pi
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), z0 + a * (z1 - z0),
                yaw0 + a * dyaw, stamp_s)

    def _scan_callback(self, msg: PointCloud2):
        if self.latest_pose is None:
            return
        pts = self._parse_pointcloud2(msg)
        if pts is None or len(pts) == 0:
            return
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        x, y, z, yaw, matched = self._lookup_pose_at(stamp_s)
        if not all(np.isfinite(v) for v in (x, y, z, yaw)):
            x, y, z, yaw = self.latest_pose
            matched = self.latest_pose_stamp_s
        if matched is not None and abs(stamp_s - matched) > self.max_sensor_skew_sec:
            self._stale_pose_n += 1

        if self.self_exclusion_radius > 0.0:
            # Returns from the platform itself (mount, chair, rider) would be
            # painted as obstacles under the robot; the mapper's own guard is
            # only 0.1 m (sized for a Go2/G1 whose body sits below the lidar).
            d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
            pts = pts[d2 >= self.self_exclusion_radius ** 2]
            if len(pts) == 0:
                return

        t0 = time.monotonic()
        self._clear_footprint(x, y)
        self.mapper.update(pts, x, y, z, yaw)
        ms = (time.monotonic() - t0) * 1e3
        self._upd_ms_sum += ms
        self._upd_ms_max = max(self._upd_ms_max, ms)
        self._upd_n += 1
        self._scan_seq += 1

    def _clear_footprint(self, x: float, y: float):
        """Force the cells under the robot free: the mapper only ever sets
        occupancy, so anything seen at a spot the robot later drives through
        (a person walking alongside, at torso height in the obstacle band)
        would stay painted on its path forever."""
        m = self.mapper
        if m.full_map is None or self.clear_footprint_radius <= 0.0:
            return
        rc = int(math.ceil(self.clear_footprint_radius / m.cfg.resolution))
        gc, gr = world_to_global_cell(x, y, m.map_origin_x, m.map_origin_y, m.cfg.resolution)
        n = m.global_cells
        r0, r1 = max(0, gr - rc), min(n, gr + rc + 1)
        c0, c1 = max(0, gc - rc), min(n, gc + rc + 1)
        if r1 <= r0 or c1 <= c0:
            return
        yy, xx = np.ogrid[r0:r1, c0:c1]
        disk = (yy - gr) ** 2 + (xx - gc) ** 2 <= rc * rc
        m.full_map[0, r0:r1, c0:c1][disk] = 0.0
        m.full_map[1, r0:r1, c0:c1][disk] = 1.0

    @staticmethod
    def _parse_pointcloud2(msg: PointCloud2) -> Optional[np.ndarray]:
        """PointCloud2 -> (N, 3) float32 xyz, NaN/inf rows dropped."""
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            return None
        n = msg.width * msg.height
        if n == 0:
            return None
        step = msg.point_step
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
        dt = np.dtype(('>' if msg.is_bigendian else '<') + 'f4')
        cols = []
        for k in ('x', 'y', 'z'):
            off = fields[k].offset
            cols.append(np.frombuffer(np.ascontiguousarray(raw[:, off:off + 4]).tobytes(), dtype=dt))
        pts = np.stack(cols, axis=1).astype(np.float32)
        return pts[np.isfinite(pts).all(axis=1)]

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def _frame(self) -> str:
        return self.grid_frame or self.odom_frame

    def _update_frontiers(self):
        m = self.mapper
        if not self.publish_frontiers or m.full_map is None:
            return
        local = self.frontier_detector.extract_from_global(
            full_map=m.full_map, robot_g_row=int(m.robot_g_row),
            robot_g_col=int(m.robot_g_col), robot_x=m.robot_x, robot_y=m.robot_y,
            output_size=m.cfg.output_size)
        self._frontiers_local = local
        centre = m.cfg.output_size / 2.0
        world = []
        for lp_row, lp_col in local:
            # inverse of FrontierDetector step 6: local pixel -> global cell -> world
            g_col = int(round(m.robot_g_col + (lp_col - centre)))
            g_row = int(round(m.robot_g_row + (centre - lp_row)))
            world.append(global_cell_to_world(g_col, g_row, m.map_origin_x,
                                              m.map_origin_y, m.cfg.resolution))
        self._frontiers_world = np.array(world, dtype=np.float32).reshape(-1, 2)

    def _publish_local(self):
        if self.mapper.local_map is None or self._scan_seq == self._local_seq:
            return
        self._local_seq = self._scan_seq
        self._update_frontiers()
        rgb = self.mapper.render_local_bev(
            frontier_centers_2d=self._frontiers_local if len(self._frontiers_local) else None,
            draw_fov=self.draw_fov)
        msg = Image()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame())
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        self.local_pub.publish(msg)

    def _publish_grid(self):
        m = self.mapper
        if m.full_map is None or self._scan_seq == self._grid_seq:
            return
        self._grid_seq = self._scan_seq
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame())

        # full_map[ch, row, col] with row = +Y, col = +X from the map origin =
        # OccupancyGrid's row-major (y, x) layout, no flip needed.
        occ = m.full_map[0] >= m.cfg.map_pred_threshold
        exp = m.full_map[1] >= m.cfg.exp_pred_threshold
        data = np.full(m.full_map.shape[1:], -1, dtype=np.int8)
        data[exp] = 0
        data[occ] = 100
        grid = OccupancyGrid()
        grid.header = header
        grid.info.map_load_time = header.stamp
        grid.info.resolution = float(m.cfg.resolution)
        grid.info.width = int(m.global_cells)
        grid.info.height = int(m.global_cells)
        grid.info.origin.position.x = float(m.map_origin_x)
        grid.info.origin.position.y = float(m.map_origin_y)
        grid.info.origin.position.z = float((self.start_z or 0.0) + self.grid_z_offset)
        grid.info.origin.orientation.w = 1.0
        grid.data = data.reshape(-1).tolist()
        self.grid_pub.publish(grid)

        if self.publish_frontiers:
            z = float((self.start_z or 0.0) + self.grid_z_offset)
            pts = [(float(x), float(y), z) for x, y in self._frontiers_world]
            self.frontier_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def _log_summary(self):
        if self._upd_n == 0:
            if self.latest_pose is None:
                self.get_logger().info('waiting for odometry')
            return
        m = self.mapper
        explored = int((m.full_map[1] >= m.cfg.exp_pred_threshold).sum()) if m.full_map is not None else 0
        occupied = int((m.full_map[0] >= m.cfg.map_pred_threshold).sum()) if m.full_map is not None else 0
        self.get_logger().info(
            f'scans={self._scan_seq} update_ms mean={self._upd_ms_sum / self._upd_n:.1f} '
            f'max={self._upd_ms_max:.1f} | explored={explored * m.cfg.resolution ** 2:.1f} m2 '
            f'occupied={occupied} cells frontiers={len(self._frontiers_world)}'
            + (f' | stale_pose={self._stale_pose_n}' if self._stale_pose_n else ''))
        self._upd_ms_sum = self._upd_ms_max = 0.0
        self._upd_n = 0
        self._stale_pose_n = 0


def main(args=None):
    rclpy.init(args=args)
    node = BEVMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
