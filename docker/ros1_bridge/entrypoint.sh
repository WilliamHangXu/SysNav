#!/usr/bin/env bash
# Entrypoint for the SysNav ROS 1 <-> ROS 2 bridge container.
#
# Subcommands:
#   bridge [extra dynamic_bridge args]   dynamic_bridge (selective; BRIDGE_ALL=1 = all)
#   param-bridge [extra args]            parameter_bridge over an explicit allowlist
#                                        (BRIDGE_TOPICS_FILE); eager + per-topic QoS,
#                                        removes the startup calibration lag (see README).
#                                        Namespace comes from robot.yaml (ROBOT_CONFIG_FILE),
#                                        rendered into the allowlist's __NS__ token.
#   roscore                              start a ROS 1 (Noetic) roscore
#   play <rosbag play args>              rosbag play (Noetic); pass --clock for sim time
#   shell                                interactive bash
#   <anything else>                      run it with ROS 2 Jazzy sourced
set -e

JAZZY_SETUP=/opt/ros/jazzy/setup.bash
NOETIC_SETUP=/opt/ros/noetic/setup.bash

# The upstream builder leaves the compiled bridge as a colcon workspace; find its
# local_setup.bash wherever it placed it (use local_setup, NOT setup — the bridge
# was compiled in a container with different underlay paths).
find_bridge_setup() {
  local f
  for f in \
      /ros-jazzy-ros1-bridge/install/local_setup.bash \
      "${HOME}/ros-jazzy-ros1-bridge/install/local_setup.bash" \
      /root/ros-jazzy-ros1-bridge/install/local_setup.bash; do
    [ -f "$f" ] && { echo "$f"; return 0; }
  done
  find / -name local_setup.bash -path '*ros-jazzy-ros1-bridge*' 2>/dev/null | head -1
}

cmd="${1:-bridge}"; shift || true
case "$cmd" in
  bridge)
    # shellcheck disable=SC1090
    source "$JAZZY_SETUP"
    BR="$(find_bridge_setup)"
    if [ -z "$BR" ]; then
      echo "ERROR: could not locate ros1_bridge local_setup.bash in this image." >&2
      echo "       Is this built FROM ros-jazzy-ros1-bridge-builder? See README." >&2
      exit 1
    fi
    # shellcheck disable=SC1090
    source "$BR"
    : "${ROS_MASTER_URI:=http://localhost:11311}"; export ROS_MASTER_URI
    # Selective by default: plain dynamic_bridge bridges a topic ONLY when the far
    # side has a subscriber -- i.e. exactly the inputs the ROS 2 pipeline asks for,
    # with correct direction (ROS 1->ROS 2 only) and no /tf feedback loop. Set
    # BRIDGE_ALL=1 to bridge every topic in the bag/graph instead.
    flags=""
    [ "${BRIDGE_ALL:-0}" = "1" ] && flags="--bridge-all-topics"
    echo "[bridge] ROS_MASTER_URI=$ROS_MASTER_URI  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  BRIDGE_ALL=${BRIDGE_ALL:-0}"
    echo "[bridge] sourced: $BR"
    # shellcheck disable=SC2086
    exec ros2 run ros1_bridge dynamic_bridge $flags "$@"
    ;;
  param-bridge)
    # Explicit-allowlist bridge -- eager bridges + per-topic QoS (transient_local
    # /tf_static) remove the live startup calibration lag. The logic lives in the
    # shared helper so the unified container's supervisor reuses it verbatim;
    # robot.yaml stays the single source of the namespace (rendered into __NS__).
    # See start_bridge.sh + bridge_topics.yaml + README.
    exec /usr/local/bin/start_bridge.sh "$@"
    ;;
  roscore)
    # shellcheck disable=SC1090
    source "$NOETIC_SETUP"
    exec roscore "$@"
    ;;
  play)
    # shellcheck disable=SC1090
    source "$NOETIC_SETUP"
    : "${ROS_MASTER_URI:=http://localhost:11311}"; export ROS_MASTER_URI
    exec rosbag play "$@"
    ;;
  shell)
    exec bash
    ;;
  *)
    # shellcheck disable=SC1090
    source "$JAZZY_SETUP"
    exec "$cmd" "$@"
    ;;
esac
