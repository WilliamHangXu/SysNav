#!/usr/bin/env python3
"""Bridge a rosbag's own SLAM onto the SysNav interface topics.

The office_building bag already carries a single, connected, gravity-aligned TF
tree (`world -> go2w_005/odom -> go2w_005/base -> sensors`) and its LIO pose on
`/go2w_005/lio/odometry`. Instead of running a *second* estimator (arise_slam)
and reconciling two drifting `map`/`world` frames with a static transform, this
node feeds the stack directly from the bag's own SLAM:

  * republishes the LIO pose as `/state_estimation` (base-in-world), and
  * transforms the raw LiDAR into world and republishes `/registered_scan`.

LiDAR source: arise consumed the *raw* Livox stream `/go2w_005/livox/lidar`
(livox_ros_driver2/CustomMsg, ~13k pts/sweep), NOT the decimated PointCloud2
`/go2w_005/lidar` (~3.8k pts). To match arise's registered-scan density (which
3D lifting and room segmentation depend on) we default to the CustomMsg stream;
a PointCloud2 source is still supported via `lidar_msg_type:=pointcloud2`.

The dynamic pose comes from the LIO *topic* (not from `/tf`), so we never do a
time-sensitive dynamic TF lookup (fragile under sim time and the bag's ~one-frame
stamp skew). Only the *static* legs are taken from TF:

    world -> odom   (static, ~identity)   cached once
    base  -> lidar  (static sensor mount) cached once (keyed by the lidar frame)

and composed with the per-message LIO pose:

    world<-base  = (world<-odom) . (odom<-base)_lio
    world<-lidar = (world<-base) . (base<-lidar)

Both outputs are stamped with `output_frame` (default "map"), defined to coincide
*exactly* with the bag's `world`; a static identity `world -> map` unifies the
labels. Every downstream consumer (`semantic_mapping`, `room_segmentation`, the
occupancy grid, `tare_planner`) keeps working unchanged — it only reads these two
topics.
"""

from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2

from tf2_ros import StaticTransformBroadcaster

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # only needed when lidar_msg_type == 'livox'
    CustomMsg = None


def strip_slash(frame):
    """tf2 rejects frame ids that start with '/'."""
    return frame[1:] if frame.startswith('/') else frame


def quat_to_matrix(qx, qy, qz, qw):
    """Unit-quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


def matrix_to_quat(R):
    """3x3 rotation matrix -> (x,y,z,w) quaternion."""
    qw = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if qw > 1e-8:
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(max(1e-12, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
            qw, qx, qy, qz = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(max(1e-12, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) * 2.0
            qw, qx, qy, qz = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(max(1e-12, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) * 2.0
            qw, qx, qy, qz = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return qx, qy, qz, qw


def transform_to_Rt(tf_msg: TransformStamped):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def compose(Rt_a, Rt_b):
    """(R_a, t_a) . (R_b, t_b)  ->  applies b first, then a."""
    Ra, ta = Rt_a
    Rb, tb = Rt_b
    return Ra @ Rb, Ra @ tb + ta


class BagSlamBridge(Node):
    def __init__(self):
        super().__init__('bag_slam_bridge')

        self.lidar_msg_type = self.declare_parameter('lidar_msg_type', 'livox').value  # 'livox' | 'pointcloud2'
        default_lidar = '/go2w_005/livox/lidar' if self.lidar_msg_type == 'livox' else '/go2w_005/lidar'
        self.lidar_topic = self.declare_parameter('lidar_topic', default_lidar).value
        self.odom_topic = self.declare_parameter('odom_topic', '/go2w_005/lio/odometry').value
        self.registered_scan_topic = self.declare_parameter('registered_scan_topic', '/registered_scan').value
        self.state_estimation_topic = self.declare_parameter('state_estimation_topic', '/state_estimation').value
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.output_frame = self.declare_parameter('output_frame', 'map').value
        self.lidar_frame_param = self.declare_parameter('lidar_frame', 'go2w_005/livox_frame').value
        self.base_frame_param = self.declare_parameter('base_frame', 'go2w_005/base').value
        self.odom_frame_param = self.declare_parameter('odom_frame', 'go2w_005/odom').value
        self.pose_match_tol = float(self.declare_parameter('pose_match_tol', 0.05).value)
        # Drop returns closer than this to the sensor (Livox emits (0,0,0) for no-return).
        self.min_range = float(self.declare_parameter('min_range', 0.1).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.T_world_odom = None          # world <- odom (static, cached)
        self.T_base_lidar = None          # base  <- lidar (static, cached)
        self._cached_lidar_frame = None
        self.base_frame = self.base_frame_param
        self.odom_hist = deque(maxlen=200)  # (t_sec, R_ob, t_ob)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.scan_pub = self.create_publisher(PointCloud2, self.registered_scan_topic, 5)
        self.odom_pub = self.create_publisher(Odometry, self.state_estimation_topic, 10)

        if self.lidar_msg_type == 'livox':
            if CustomMsg is None:
                raise RuntimeError("lidar_msg_type='livox' but livox_ros_driver2 is not importable.")
            self.create_subscription(CustomMsg, self.lidar_topic, self.livox_callback, sensor_qos)
        else:
            self.create_subscription(PointCloud2, self.lidar_topic, self.pointcloud_callback, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        self._publish_identity_world_to_output()
        self._scan_drop_warned = False
        self.get_logger().info(
            f"bag_slam_bridge: lidar='{self.lidar_topic}' ({self.lidar_msg_type}) -> "
            f"{self.registered_scan_topic}, {self.odom_topic} -> {self.state_estimation_topic}; "
            f"world='{self.world_frame}', output='{self.output_frame}'.")

    def _publish_identity_world_to_output(self):
        if self.output_frame == self.world_frame:
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.output_frame
        tf_msg.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf_msg)

    def _lookup_static(self, target, source):
        """Static (time-independent) target<-source; returns (R, t) or None."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(strip_slash(target), strip_slash(source), Time())
            return transform_to_Rt(tf_msg)
        except tf2_ros.TransformException:
            return None

    def odom_callback(self, msg: Odometry):
        odom_frame = msg.header.frame_id or self.odom_frame_param
        self.base_frame = msg.child_frame_id or self.base_frame_param

        if self.T_world_odom is None:
            self.T_world_odom = self._lookup_static(self.world_frame, odom_frame)
            if self.T_world_odom is None:
                return

        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        R_ob = quat_to_matrix(q.x, q.y, q.z, q.w)
        t_ob = np.array([p.x, p.y, p.z])
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_hist.append((stamp_sec, R_ob, t_ob))

        R_wb, t_wb = compose(self.T_world_odom, (R_ob, t_ob))
        qx, qy, qz, qw = matrix_to_quat(R_wb)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = strip_slash(self.base_frame)
        out.pose.pose.position.x = float(t_wb[0])
        out.pose.pose.position.y = float(t_wb[1])
        out.pose.pose.position.z = float(t_wb[2])
        out.pose.pose.orientation.x = float(qx)
        out.pose.pose.orientation.y = float(qy)
        out.pose.pose.orientation.z = float(qz)
        out.pose.pose.orientation.w = float(qw)
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def _nearest_odom(self, stamp_sec):
        if not self.odom_hist:
            return None
        best = min(self.odom_hist, key=lambda e: abs(e[0] - stamp_sec))
        if abs(best[0] - stamp_sec) > self.pose_match_tol:
            return None
        return best[1], best[2]

    def _register_and_publish(self, xyz, intensity, lidar_frame, stamp_msg, stamp_sec):
        """xyz: (N,3) in lidar frame; intensity: (N,) or None. Transforms to world."""
        if self.T_world_odom is None:
            return
        if self.T_base_lidar is None or self._cached_lidar_frame != lidar_frame:
            T = self._lookup_static(self.base_frame, lidar_frame)
            if T is None:
                return
            self.T_base_lidar = T
            self._cached_lidar_frame = lidar_frame

        odom_pose = self._nearest_odom(stamp_sec)
        if odom_pose is None:
            if not self._scan_drop_warned:
                self.get_logger().warn("No LIO pose within tolerance for a scan; dropping until odom aligns.")
                self._scan_drop_warned = True
            return
        self._scan_drop_warned = False

        R_wb, t_wb = compose(self.T_world_odom, odom_pose)
        R_wl, t_wl = compose((R_wb, t_wb), self.T_base_lidar)

        xyz_world = xyz @ R_wl.T + t_wl
        if intensity is not None:
            out = np.hstack([xyz_world, intensity.reshape(-1, 1)]).astype(np.float32)
        else:
            out = xyz_world.astype(np.float32)
        self.scan_pub.publish(self._make_cloud(out, stamp_msg, intensity is not None))

    def livox_callback(self, msg):
        lidar_frame = strip_slash(msg.header.frame_id or self.lidar_frame_param)
        pts = msg.points
        if not pts:
            return
        # Build (N,4): x,y,z,reflectivity. (Per-point python access; fine for offline replay.)
        arr = np.array([(p.x, p.y, p.z, p.reflectivity) for p in pts], dtype=np.float32)
        r2 = arr[:, 0] ** 2 + arr[:, 1] ** 2 + arr[:, 2] ** 2
        keep = r2 >= (self.min_range * self.min_range)
        arr = arr[keep]
        if arr.shape[0] == 0:
            return
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._register_and_publish(arr[:, :3], arr[:, 3], lidar_frame, msg.header.stamp, stamp_sec)

    def pointcloud_callback(self, msg: PointCloud2):
        lidar_frame = strip_slash(msg.header.frame_id or self.lidar_frame_param)
        field_names = [f.name for f in msg.fields]
        has_intensity = 'intensity' in field_names
        want = ('x', 'y', 'z', 'intensity') if has_intensity else ('x', 'y', 'z')
        pts = point_cloud2.read_points_numpy(msg, field_names=want, skip_nans=True)
        if pts.shape[0] == 0:
            return
        r2 = pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2
        keep = r2 >= (self.min_range * self.min_range)
        pts = pts[keep]
        if pts.shape[0] == 0:
            return
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        intensity = pts[:, 3] if has_intensity else None
        self._register_and_publish(pts[:, :3], intensity, lidar_frame, msg.header.stamp, stamp_sec)

    def _make_cloud(self, points_f32, stamp, has_intensity):
        out_msg = PointCloud2()
        out_msg.header.stamp = stamp
        out_msg.header.frame_id = self.output_frame
        out_msg.height = 1
        out_msg.width = points_f32.shape[0]
        ftype = PointField.FLOAT32
        fields = [
            PointField(name='x', offset=0, datatype=ftype, count=1),
            PointField(name='y', offset=4, datatype=ftype, count=1),
            PointField(name='z', offset=8, datatype=ftype, count=1),
        ]
        if has_intensity:
            fields.append(PointField(name='intensity', offset=12, datatype=ftype, count=1))
        out_msg.fields = fields
        out_msg.is_bigendian = False
        out_msg.point_step = 4 * (4 if has_intensity else 3)
        out_msg.row_step = out_msg.point_step * out_msg.width
        out_msg.is_dense = True
        out_msg.data = np.ascontiguousarray(points_f32).tobytes()
        return out_msg


def main(args=None):
    rclpy.init(args=args)
    node = BagSlamBridge()
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
