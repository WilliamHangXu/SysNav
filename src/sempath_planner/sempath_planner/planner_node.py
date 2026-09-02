#!/usr/bin/env python3
"""Keyboard-driven GroundPlan planning + waypoint execution on exported SemPathBench maps.

Commands on ``/keyboard_input`` (std_msgs/String, one line per message — the vlm_node keyboard
terminal):

  export               tare_planner + bev_mapper write their dumps (same keyword); this node waits
                       for fresh files, converts them into the embedded SemPathBench checkout
                       (map key ``real/<map_id>``) and keeps the map's grid frame in memory.
  plan <instruction>   run GroundPlan (Gemini, needs $GEMINI_API_KEY) from the robot's current
                       pose; publish the path for RViz. ``replan`` is an alias.
  go                   follow the planned waypoints: Joy autonomy handshake once, then
                       /way_point + /speed until arrival (the local planner does the driving).
  stop                 abort: waypoint at the robot's pose + Joy autonomy off.

The node only plans on a map it converted in this process, so pixel->world stays valid for the
current SLAM session by construction. It never touches scene-graph state.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path as FsPath

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from sempath_planner.annotator_ui import AnnotatorUI
from sempath_planner.path_utils import decimate_path, trajectory_to_world, world_to_pixel

TARE_KEYWORDS = {"reset"}          # keyboard words owned by other nodes (never treated as errors here)
OBJECT_MAPPER_KEYWORDS = {"demo", "resume"}


class SempathPlannerNode(Node):
    def __init__(self):
        super().__init__('sempath_planner')
        self._declare_parameters()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_waypoint = self.create_publisher(PointStamped, '/way_point', 5)
        self.pub_speed = self.create_publisher(Float32, '/speed', 5)
        self.pub_joy = self.create_publisher(Joy, '/joy', 5)
        self.pub_path = self.create_publisher(Path, '/sempath_plan/path', latched)
        self.pub_markers = self.create_publisher(MarkerArray, '/sempath_plan/markers', latched)

        self.create_subscription(String, '/keyboard_input', self._keyboard_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._pose: tuple[float, float, float] | None = None   # x, y, z in the map frame
        self._frame_id = 'map'
        # last converted map: {'map_key', 'json_path', 'frame' (grid_coordinate_frame), 'stamp'}
        self._map: dict | None = None
        # last accepted plan: {'world' (full path), 'waypoints' (decimated), 'instruction'}
        self._plan: dict | None = None
        # follower state: {'waypoints', 'index', 'deadline'} or None
        self._exec: dict | None = None

        self.create_timer(1.0 / max(self.follow_rate_hz, 0.1), self._follow_tick)

        self._ui: AnnotatorUI | None = None
        if self.ui_enabled:
            try:
                self._ui = AnnotatorUI(self._resolve_sysnav_root(), self.ui_host, self.ui_port,
                                       self.get_logger())
                self.get_logger().info(f"annotator UI serving at {self._ui.start()}")
            except Exception as exc:
                self.get_logger().warning(f"annotator UI disabled: {type(exc).__name__}: {exc}")
                self._ui = None

        self.get_logger().info(
            f"sempath_planner ready (map id '{self.map_id}', dumps '{self.dump_dir}'). "
            "Keyboard: export | plan <instruction> | go | stop")

    def _declare_parameters(self):
        p = self.declare_parameter
        self.dump_dir = p('dump_dir', 'output/sempath_export').value
        self.sysnav_root = p('sysnav_root', '').value            # '' = node cwd (teleop scripts cd to the repo root)
        self.map_id = p('map_id', 'live_train').value
        self.odom_topic = p('odom_topic', '/state_estimation').value
        self.grounding_variant = p('grounding_variant', 'direct_cap_repair').value  # upstream's canonical default
        self.llm_timeout_s = p('llm_timeout_s', 90.0).value
        self.dump_wait_timeout_s = p('dump_wait_timeout_s', 20.0).value
        self.make_overview = p('make_overview', False).value     # overview PNG on every save (slower)
        self.speed = p('speed', 1.0).value                       # m/s handed to /speed while following
        self.arrival_radius_m = p('arrival_radius_m', 0.5).value
        self.final_arrival_radius_m = p('final_arrival_radius_m', 0.3).value
        self.waypoint_timeout_s = p('waypoint_timeout_s', 60.0).value
        self.rdp_epsilon_m = p('rdp_epsilon_m', 0.15).value
        self.max_spacing_m = p('max_spacing_m', 1.5).value
        self.follow_rate_hz = p('follow_rate_hz', 2.0).value
        self.ui_enabled = p('ui.enabled', True).value
        self.ui_host = p('ui.host', '127.0.0.1').value
        self.ui_port = p('ui.port', 8010).value
        self.ui_open_browser = p('ui.open_browser', True).value

    # ------------------------------------------------------------------ inputs

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        with self._lock:
            self._pose = (pos.x, pos.y, pos.z)
            if msg.header.frame_id:
                self._frame_id = msg.header.frame_id

    def _keyboard_callback(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        word, _, rest = line.partition(' ')
        word = word.lower()
        if word == 'export':
            self._start_worker(self._do_save, 'export', time.time())
        elif word in ('plan', 'replan'):
            instruction = rest.strip()
            if not instruction:
                self.get_logger().error("usage: plan <instruction text>")
                return
            self._start_worker(self._do_plan, 'plan', instruction)
        elif word == 'go':
            self._start_follow()
        elif word == 'stop':
            self._stop_follow('stopped by user', autonomy_off=True)
        elif word in TARE_KEYWORDS or word in OBJECT_MAPPER_KEYWORDS:
            pass  # other nodes' keywords on the shared channel
        else:
            self.get_logger().info(f"ignoring keyboard input '{line}' (not a sempath_planner command)")

    def _start_worker(self, target, name: str, *args):
        if self._exec is not None:
            self.get_logger().error(f"'{name}' refused: the robot is following a plan (type 'stop' first)")
            return
        if self._worker is not None and self._worker.is_alive():
            self.get_logger().error(f"'{name}' refused: a save/plan is still running")
            return
        self._worker = threading.Thread(target=self._run_worker, args=(target, name) + args, daemon=True)
        self._worker.start()

    def _run_worker(self, target, name, *args):
        try:
            target(*args)
        except Exception as exc:  # worker threads must never die silently
            self.get_logger().error(f"'{name}' failed: {type(exc).__name__}: {exc}")

    # ------------------------------------------------------------------ save (export -> convert)

    def _resolve_sysnav_root(self) -> FsPath:
        root = FsPath(self.sysnav_root) if self.sysnav_root else FsPath.cwd()
        return root.resolve()

    def _do_save(self, cmd_time: float):
        root = self._resolve_sysnav_root()
        dump_dir = (root / self.dump_dir).resolve() if not os.path.isabs(self.dump_dir) else FsPath(self.dump_dir)
        # tare_planner writes latest.json last (after PNG + snapshot JSON); bev_mapper writes bev_latest.npz.
        needed = [dump_dir / 'latest.json', dump_dir / 'bev_latest.npz']
        self.get_logger().info(f"waiting for fresh dumps in {dump_dir} ...")
        deadline = cmd_time + self.dump_wait_timeout_s
        while time.time() < deadline:
            if all(f.exists() and f.stat().st_mtime >= cmd_time - 0.25 for f in needed):
                break
            time.sleep(0.25)
        else:
            missing = [str(f) for f in needed if not (f.exists() and f.stat().st_mtime >= cmd_time - 0.25)]
            self.get_logger().error(
                f"dumps not refreshed within {self.dump_wait_timeout_s:.0f}s: {missing} "
                "(are tare_planner/bev_mapper running with export.enabled?)")
            return

        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        try:
            from tools.sempath_export.convert_sysnav_scene import ConvertOptions, load_sysnav_dump
            from tools.sempath_export.transform_sysnav_to_map import (
                DEFAULT_OUTPUT_DIR, build_output_prefix, transform_sysnav_to_map)
        except ImportError as exc:
            self.get_logger().error(f"converter unavailable: {exc}")
            return

        t0 = time.time()
        dump = load_sysnav_dump(dump_dir)
        prefix = build_output_prefix(DEFAULT_OUTPUT_DIR, self.map_id)
        outputs = transform_sysnav_to_map(
            dump, prefix, self.map_id, ConvertOptions(),
            skip_overview=not self.make_overview, overwrite=True)
        payload = json.loads(outputs['json_path'].read_text(encoding='utf-8'))
        frame = payload['metadata']['grid_coordinate_frame']
        with self._lock:
            self._map = {'map_key': f"real/{self.map_id}", 'json_path': outputs['json_path'],
                         'frame': frame, 'stamp': time.time()}
            self._plan = None
        self.get_logger().info(
            f"map saved: real/{self.map_id} ({payload['grid_size']}x{payload['grid_size']} @ "
            f"{frame['resolution']} m, {len(payload['room_instances'])} rooms, "
            f"{len(payload['object_instances'])} objects, {time.time() - t0:.1f}s) -> {outputs['json_path'].parent}")
        self._open_ui(f"real/{self.map_id}", plan=False)

    # ------------------------------------------------------------------ plan (GroundPlan)

    def _do_plan(self, instruction_text: str):
        with self._lock:
            map_info = self._map
            pose = self._pose
        if map_info is None:
            self.get_logger().error("no map saved this session yet: type 'export' first")
            return
        if pose is None:
            self.get_logger().error(f"no robot pose received yet on {self.odom_topic}")
            return
        if not os.environ.get('GEMINI_API_KEY'):
            self.get_logger().error("GEMINI_API_KEY is not set in this environment (GroundPlan needs it)")
            return

        root = self._resolve_sysnav_root()
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        try:
            from tools.sempath_export import spb  # noqa: F401  (sys.path bootstrap for the embedded checkout)
            from scripts.make_instruction.make_instruction import load_map_state
            from scripts.methods.groundplan.llm_client import GeminiClient
            from scripts.methods.groundplan.pipeline import GroundPlanRunConfig, run_groundplan_pipeline
        except ImportError as exc:
            self.get_logger().error(f"SemPathBench unavailable: {exc}")
            return

        row, col = world_to_pixel(pose[0], pose[1], map_info['frame'])
        instruction = {
            'instruction': instruction_text,
            'start_pose': {'row': row, 'col': col},
            'map_id': map_info['map_key'],
        }
        self.get_logger().info(
            f"planning \"{instruction_text}\" from ({pose[0]:.2f}, {pose[1]:.2f}) = cell ({row}, {col}) "
            f"on {map_info['map_key']} ... (LLM calls, expect 15-60 s)")
        t0 = time.time()
        map_state = load_map_state(map_info['map_key'])
        result = run_groundplan_pipeline(
            map_state, instruction,
            config=GroundPlanRunConfig(grounding_variant=self.grounding_variant),
            llm_client=GeminiClient(cache_root=None, timeout=self.llm_timeout_s))

        status = result.metadata.get('status') if isinstance(result.metadata, dict) else None
        if not result.success or len(result.trajectory) < 2:
            self.get_logger().error(
                f"plan failed after {time.time() - t0:.1f}s: {result.failure_reason or status or 'no path'} "
                f"(trajectory has {len(result.trajectory)} cells); previous plan kept")
            return

        world = trajectory_to_world(result.trajectory, map_info['frame'])
        waypoints = decimate_path(world, self.rdp_epsilon_m, self.max_spacing_m)
        with self._lock:
            self._plan = {'world': world, 'waypoints': waypoints, 'instruction': instruction_text}
        self._publish_plan(world, waypoints)
        if self._ui is not None:
            self._ui.set_live_plan(map_info['map_key'], instruction_text,
                                   instruction['start_pose'], result.trajectory)
        goal = world[-1]
        self.get_logger().info(
            f"plan ready in {time.time() - t0:.1f}s: {len(result.trajectory)} cells -> "
            f"{len(waypoints)} waypoints, goal ({goal[0]:.2f}, {goal[1]:.2f}). Check RViz, then type 'go'.")
        self._open_ui(map_info['map_key'], plan=True)

    def _open_ui(self, map_key: str, *, plan: bool):
        if self._ui is None or not self.ui_open_browser:
            return
        url = self._ui.map_url(map_key, plan=plan)
        if url:
            self.get_logger().info(f"opening {url}")
            self._ui.open_in_browser(url)

    # ------------------------------------------------------------------ visualization

    def _publish_plan(self, world: list[tuple[float, float]], waypoints: list[tuple[float, float]]):
        z = (self._pose[2] if self._pose else 0.0)
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = now
        for x, y in world:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

        markers = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for i, (x, y) in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = now
            m.ns, m.id, m.type, m.action = 'waypoints', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            last = i == len(waypoints) - 1
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.2, 0.2, 0.9) if last else (0.2, 0.8, 0.2, 0.9)
            markers.markers.append(m)
        self.pub_markers.publish(markers)

    # ------------------------------------------------------------------ follower

    def _start_follow(self):
        with self._lock:
            plan = self._plan
        if plan is None:
            self.get_logger().error("'go' refused: no plan yet (type 'plan <instruction>' first)")
            return
        if self._exec is not None:
            self.get_logger().error("'go' refused: already following (type 'stop' to abort)")
            return
        if self._worker is not None and self._worker.is_alive():
            self.get_logger().error("'go' refused: a save/plan is still running")
            return
        waypoints = plan['waypoints'][1:]  # first point is the robot start
        if not waypoints:
            self.get_logger().error("'go' refused: plan has no waypoints beyond the start")
            return
        self._publish_autonomy(True)
        self._exec = {'waypoints': waypoints, 'index': 0,
                      'deadline': time.time() + self.waypoint_timeout_s}
        self.get_logger().info(f"following {len(waypoints)} waypoints at {self.speed:.1f} m/s "
                               f"(\"{plan['instruction']}\")")

    def _follow_tick(self):
        state = self._exec
        if state is None:
            return
        with self._lock:
            pose = self._pose
        if pose is None:
            return
        wx, wy = state['waypoints'][state['index']]
        last = state['index'] == len(state['waypoints']) - 1
        radius = self.final_arrival_radius_m if last else self.arrival_radius_m
        if math.hypot(wx - pose[0], wy - pose[1]) <= radius:
            if last:
                self._exec = None
                self._publish_waypoint(pose[0], pose[1], pose[2])
                self.get_logger().info("goal reached")
                return
            state['index'] += 1
            state['deadline'] = time.time() + self.waypoint_timeout_s
            wx, wy = state['waypoints'][state['index']]
        elif time.time() > state['deadline']:
            self._stop_follow(
                f"waypoint {state['index'] + 1}/{len(state['waypoints'])} not reached within "
                f"{self.waypoint_timeout_s:.0f}s", autonomy_off=False)
            return
        self._publish_waypoint(wx, wy, pose[2])
        self.pub_speed.publish(Float32(data=float(self.speed)))

    def _stop_follow(self, reason: str, *, autonomy_off: bool):
        was_following = self._exec is not None
        self._exec = None
        with self._lock:
            pose = self._pose
        if pose is not None:
            self._publish_waypoint(pose[0], pose[1], pose[2])
        if autonomy_off:
            self._publish_autonomy(False)
        if was_following:
            self.get_logger().warning(f"execution stopped: {reason}")
        elif autonomy_off:
            self.get_logger().info("not following; published a hold waypoint at the robot")

    def _publish_waypoint(self, x: float, y: float, z: float):
        msg = PointStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = x, y, z
        self.pub_waypoint.publish(msg)

    def _publish_autonomy(self, enabled: bool):
        # localPlanner's joystickHandler: axes[2] <= -0.1 -> autonomyMode on; axes[5] > -0.1 keeps
        # obstacle checking on. axes[3]/[4] = 0 so /speed (published while following) sets the speed.
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = 'sempath_planner'
        joy.axes = [0.0, 0.0, -1.0 if enabled else 0.0, 0.0, 0.0, 1.0]
        joy.buttons = [0] * 8
        self.pub_joy.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = SempathPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._ui is not None:
            node._ui.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
