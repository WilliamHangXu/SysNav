#!/usr/bin/env python3
"""Demo-mode control helper for the SysNav scene-graph container.

Drives the request/response handshake the robot uses in `MODE=demo` (see
docker/supervisor.sh). The whole external contract is std_msgs/String on two
global topics:

  /scene_graph_generator/request   robot -> container   start|complete|cancel|received
  /scene_graph_generator/response  container -> robot   the scene-graph JSON

Two subcommands, each a short-lived rclpy node:

  await --topic T --keyword start [--cancel-keyword cancel]
      Block until a String on T equals one of the keywords.
        exit 0 -> keyword          (supervisor: launch the pipeline)
        exit 3 -> cancel-keyword   (supervisor: tear down; nothing was started)

  serve --req REQ --resp RESP --output-root DIR [...]
      Owns the complete -> save -> respond -> received state machine:
        - wait for "complete" or "cancel" on REQ
        - either one triggers a snapshot save (publish --save-keyword on
          --save-topic, i.e. the planner's existing /keyboard_input "ssg" hook)
          and waits for the fresh snapshot file to land under --output-root
        - "complete": stream that JSON on RESP every --interval s until the
          robot replies "received" (or "cancel") on REQ, or --ack-timeout
          elapses (guard against a lost ack)
        - "cancel": just save locally, then terminate (no RESP traffic)
      A snapshot that never lands within --file-timeout is logged and we
      terminate as usual. serve always exits 0; the supervisor tears the whole
      system down once it returns.

Live robot => wall clock; use_sim_time stays false. Logs go to stdout, which the
supervisor surfaces on the container console.
"""
import argparse
import glob
import os
import sys
import time

import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


def _control_qos():
    # Reliable + volatile + keep-last-10, matching the ros1_bridge defaults for
    # the /scene_graph_generator/* control channel. Volatile (not transient_local)
    # on purpose: these are one-shot commands, so a late subscriber must not
    # replay a stale "start"/"complete".
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


def _newest_snapshot(output_root):
    """(path, mtime) of the newest snapshot_*.json under the newest run_* dir, or
    (None, 0.0). Snapshot writes are atomic (temp + rename in the planner), so any
    file seen here is already complete."""
    runs = sorted(glob.glob(os.path.join(output_root, "run_*")))
    if not runs:
        return None, 0.0
    snaps = glob.glob(os.path.join(runs[-1], "snapshot_*.json"))
    if not snaps:
        return None, 0.0
    path = max(snaps, key=os.path.getmtime)
    return path, os.path.getmtime(path)


# --------------------------------------------------------------------------- #
# await
# --------------------------------------------------------------------------- #
def cmd_await(args):
    rclpy.init()
    node = rclpy.create_node("demo_await")
    result = {"code": None}

    def on_msg(msg):
        d = msg.data.strip()
        if d == args.keyword:
            result["code"] = 0
        elif args.cancel_keyword and d == args.cancel_keyword:
            result["code"] = 3

    node.create_subscription(String, args.topic, on_msg, _control_qos())
    node.get_logger().info(
        "waiting for '%s'%s on %s ..." % (
            args.keyword,
            " or '%s'" % args.cancel_keyword if args.cancel_keyword else "",
            args.topic))
    while rclpy.ok() and result["code"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    code = result["code"] if result["code"] is not None else 1
    node.get_logger().info("await done (exit %d)" % code)
    node.destroy_node()
    rclpy.shutdown()
    return code


# --------------------------------------------------------------------------- #
# serve
# --------------------------------------------------------------------------- #
def cmd_serve(args):
    rclpy.init()
    node = rclpy.create_node("demo_serve")
    qos = _control_qos()
    resp_pub = node.create_publisher(String, args.resp, qos)
    # Created now (well before "complete"), so discovery with the planner's
    # /keyboard_input subscriber is done by the time we trigger a save.
    save_pub = node.create_publisher(String, args.save_topic, qos)

    state = {"complete": False, "cancel": False, "received": False}

    def on_req(msg):
        d = msg.data.strip()
        if not (state["complete"] or state["cancel"]):  # WAIT phase
            if d == "complete":
                state["complete"] = True
            elif d == args.cancel:
                state["cancel"] = True
        else:                                            # RESPOND phase
            if d == args.ack:
                state["received"] = True
            elif d == args.cancel:
                state["cancel"] = True

    node.create_subscription(String, args.req, on_req, qos)
    node.get_logger().info(
        "🟢 Scene graph stack started. Waiting for 'complete' or '%s' on %s ..." % (args.cancel, args.req))

    # Phase 1 -- wait for "complete" or "cancel".
    while rclpy.ok() and not state["complete"] and not state["cancel"]:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Both paths save the graph locally first.
    before_path, before_mtime = _newest_snapshot(args.output_root)
    save_pub.publish(String(data=args.save_keyword))
    node.get_logger().info(
        "published '%s' on %s (save)" % (args.save_keyword, args.save_topic))
    fresh = None
    deadline = time.time() + args.file_timeout
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        path, mtime = _newest_snapshot(args.output_root)
        if path is not None and (path != before_path or mtime > before_mtime + 1e-6):
            fresh = path
            break

    # "cancel" -> save locally and terminate; never touches /response.
    if state["cancel"] and not state["complete"]:
        if fresh:
            node.get_logger().info("cancel: saved locally -> %s; terminating" % fresh)
        else:
            node.get_logger().warn(
                "cancel: snapshot not observed within %.0fs; terminating" % args.file_timeout)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # "complete" -> stream the fresh JSON until acked.
    if not fresh:
        node.get_logger().warn(
            "complete: snapshot not observed within %.0fs; terminating without sending"
            % args.file_timeout)
        node.destroy_node()
        rclpy.shutdown()
        return 0
    try:
        with open(fresh, "r", encoding="utf-8") as f:
            payload = f.read()
    except OSError as e:
        node.get_logger().warn("complete: could not read %s: %s; terminating" % (fresh, e))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    node.get_logger().info(
        "complete: streaming %s (%d bytes) on %s every %.0fs until '%s' ..."
        % (fresh, len(payload), args.resp, args.interval, args.ack))
    deadline = time.time() + args.ack_timeout
    last_send = 0.0
    while (rclpy.ok() and not state["received"] and not state["cancel"]
           and time.time() < deadline):
        now = time.time()
        if now - last_send >= args.interval:
            resp_pub.publish(String(data=payload))
            last_send = now
            node.get_logger().info("sent scene graph on %s" % args.resp)
        rclpy.spin_once(node, timeout_sec=0.1)

    if state["received"]:
        node.get_logger().info("🟢 Scene graph received by the robot; terminating")
    elif state["cancel"]:
        node.get_logger().info("cancel during respond; terminating")
    else:
        node.get_logger().warn("no ack within %.0fs; terminating" % args.ack_timeout)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main():
    p = argparse.ArgumentParser(description="SysNav demo-mode control helper")
    sub = p.add_subparsers(dest="cmd", required=True)

    pa = sub.add_parser("await", help="block until a keyword arrives on a topic")
    pa.add_argument("--topic", required=True)
    pa.add_argument("--keyword", required=True)
    pa.add_argument("--cancel-keyword", default="")

    ps = sub.add_parser("serve", help="complete->save->respond->received state machine")
    ps.add_argument("--req", required=True)
    ps.add_argument("--resp", required=True)
    ps.add_argument("--save-topic", default="/keyboard_input")
    ps.add_argument("--save-keyword", default="ssg")
    ps.add_argument("--output-root", required=True)
    ps.add_argument("--interval", type=float, default=5.0)
    ps.add_argument("--ack", default="received")
    ps.add_argument("--cancel", default="cancel")
    ps.add_argument("--file-timeout", type=float, default=15.0)
    ps.add_argument("--ack-timeout", type=float, default=300.0)

    args = p.parse_args()
    sys.exit(cmd_await(args) if args.cmd == "await" else cmd_serve(args))


if __name__ == "__main__":
    main()
