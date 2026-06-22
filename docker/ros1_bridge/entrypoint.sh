#!/usr/bin/env bash
# Entrypoint for the SysNav ROS 1 <-> ROS 2 bridge container.
#
# Subcommands:
#   bridge [extra dynamic_bridge args]   dynamic_bridge (selective; BRIDGE_ALL=1 = all)
#   param-bridge [extra args]            parameter_bridge over an explicit allowlist
#                                        (BRIDGE_TOPICS_FILE); eager + per-topic QoS,
#                                        removes the startup calibration lag (see README)
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
    # Explicit-allowlist bridge: load BRIDGE_TOPICS_FILE onto the ROS 1 master,
    # then run parameter_bridge. Unlike selective dynamic_bridge it creates the
    # bridges EAGERLY (no discovery over the robot's 300+ topic graph) and honours
    # per-topic QoS -- notably durability: transient_local on /tf_static, so latched
    # static transforms (e.g. base->camera) reach the pipeline instantly instead of
    # after the ~30s latched-delivery race. This is what removes the startup
    # calibration lag on the live robot. See bridge_topics.yaml + README.
    : "${ROS_MASTER_URI:=http://localhost:11311}"; export ROS_MASTER_URI
    : "${BRIDGE_TOPICS_FILE:=/bridge_topics.yaml}"
    if [ ! -f "$BRIDGE_TOPICS_FILE" ]; then
      echo "ERROR: topic allowlist not found at $BRIDGE_TOPICS_FILE." >&2
      echo "       Mount it (-v .../bridge_topics.yaml:/bridge_topics.yaml:ro)" >&2
      echo "       or set BRIDGE_TOPICS_FILE to its path." >&2
      exit 1
    fi
    # rosparam is a ROS 1 tool; load the allowlist onto the master as /topics.
    # shellcheck disable=SC1090
    source "$NOETIC_SETUP"
    echo "[param-bridge] loading allowlist $BRIDGE_TOPICS_FILE -> $ROS_MASTER_URI"
    rosparam load "$BRIDGE_TOPICS_FILE"
    # Then the ROS 2 side + the compiled bridge (install local_setup, NOT the build
    # hook -- find_bridge_setup tries the install path first).
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
    echo "[param-bridge] ROS_MASTER_URI=$ROS_MASTER_URI  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  sourced: $BR"
    exec ros2 run ros1_bridge parameter_bridge "$@"
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
