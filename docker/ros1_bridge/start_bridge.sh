#!/usr/bin/env bash
# Canonical ros1_bridge launcher (shared). Renders the robot-agnostic allowlist
# (__NS__ -> robot_namespace, read from robot.yaml), loads it onto the ROS 1
# master, and runs parameter_bridge. Used by BOTH the standalone bridge image
# (docker/ros1_bridge/entrypoint.sh `param-bridge`) and the unified container
# (docker/supervisor.sh) -- one place, so robot.yaml stays the single source of
# the robot name and nothing here is hardcoded.
#
# Eager bridges + per-topic QoS (durability: transient_local on /tf_static) are
# what remove the ~30s live startup calibration lag. See bridge_topics.yaml.
#
# Env (all defaulted):
#   ROS_MASTER_URI       ROS 1 master            (default http://localhost:11311)
#   BRIDGE_TOPICS_FILE   allowlist w/ __NS__      (default /bridge_topics.yaml)
#   ROBOT_CONFIG_FILE    robot.yaml (namespace)   (default /robot.yaml)
#   JAZZY_SETUP / NOETIC_SETUP   ROS setup.bash paths
# Extra args are passed through to `ros2 run ros1_bridge parameter_bridge`.
set -e

JAZZY_SETUP="${JAZZY_SETUP:-/opt/ros/jazzy/setup.bash}"
NOETIC_SETUP="${NOETIC_SETUP:-/opt/ros/noetic/setup.bash}"
: "${ROS_MASTER_URI:=http://localhost:11311}"; export ROS_MASTER_URI
: "${BRIDGE_TOPICS_FILE:=/bridge_topics.yaml}"
: "${ROBOT_CONFIG_FILE:=/robot.yaml}"

# The upstream builder leaves the compiled bridge as a colcon workspace; source
# its INSTALL local_setup.bash (NOT the build hook, which doesn't register the
# package). Try the known install path first.
find_bridge_setup() {
  local f
  for f in /ros-jazzy-ros1-bridge/install/local_setup.bash \
           "${HOME}/ros-jazzy-ros1-bridge/install/local_setup.bash" \
           /root/ros-jazzy-ros1-bridge/install/local_setup.bash; do
    [ -f "$f" ] && { echo "$f"; return 0; }
  done
  find / -name local_setup.bash -path '*ros-jazzy-ros1-bridge*' 2>/dev/null | head -1
}

[ -f "$BRIDGE_TOPICS_FILE" ] || {
  echo "ERROR: topic allowlist not found at $BRIDGE_TOPICS_FILE." >&2
  echo "       Mount it or set BRIDGE_TOPICS_FILE." >&2; exit 1; }
[ -f "$ROBOT_CONFIG_FILE" ] || {
  echo "ERROR: robot config not found at $ROBOT_CONFIG_FILE (robot.yaml)." >&2
  echo "       Mount it or set ROBOT_CONFIG_FILE." >&2; exit 1; }

# robot.yaml is the SINGLE source of the namespace -> render __NS__ from it.
ROBOT_NS="$(grep -E '^[[:space:]]*robot_namespace:' "$ROBOT_CONFIG_FILE" \
             | head -1 \
             | sed -E 's/.*robot_namespace:[[:space:]]*//; s/#.*$//; s/[[:space:]]*$//' \
             || true)"
[ -n "$ROBOT_NS" ] || { echo "ERROR: no 'robot_namespace:' in $ROBOT_CONFIG_FILE." >&2; exit 1; }

rendered="$(mktemp)"
sed "s|__NS__|$ROBOT_NS|g" "$BRIDGE_TOPICS_FILE" > "$rendered"

# rosparam is a ROS 1 tool: load the rendered allowlist onto the master as /topics.
# shellcheck disable=SC1090
source "$NOETIC_SETUP"
echo "[bridge] robot_namespace=$ROBOT_NS (from $ROBOT_CONFIG_FILE)"
echo "[bridge] loading allowlist $BRIDGE_TOPICS_FILE -> $ROS_MASTER_URI"
rosparam load "$rendered"

# ROS 2 side + the compiled bridge.
# shellcheck disable=SC1090
source "$JAZZY_SETUP"
BR="$(find_bridge_setup)"
[ -n "$BR" ] || {
  echo "ERROR: ros1_bridge local_setup.bash not found in this image." >&2
  echo "       Is this built FROM/with ros-jazzy-ros1-bridge-builder? See README." >&2; exit 1; }
# shellcheck disable=SC1090
source "$BR"
echo "[bridge] ROS_MASTER_URI=$ROS_MASTER_URI  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  sourced: $BR"
exec ros2 run ros1_bridge parameter_bridge "$@"
