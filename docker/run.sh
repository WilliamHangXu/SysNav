#!/usr/bin/env bash
# Host launcher for the unified SysNav dev container -- one command, a few knobs.
# Bakes the stable floor (image); MOUNTS the volatile workspace (src/ + the colcon
# build in a named volume), so editing the pipeline needs no image rebuild.
#
# Usage (knobs are env vars):
#   MODE=live ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh
#   MODE=demo ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh  # robot-gated live
#   MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh
#   MODE=bag        BAG=/home/all/AlphaZ/bags/multifloor_test_slam     docker/run.sh
#   docker/run.sh shell          # debug shell in the container (workspace sourced)
#   docker/run.sh build          # just colcon build (incremental), no pipeline run
#   docker/run.sh rebuild        # wipe build/install/log, then build from scratch
#   BUILD=1 ... docker/run.sh     # force a colcon rebuild (e.g. after C++ changes)
#   INS=1 ... docker/run.sh       # also auto-open a 2nd terminal inside the container
#
# Knobs: IMAGE MODE ROS_DOMAIN_ID RVIZ OBJECTS ROBOT_IP LAPTOP_IP BAG BUILD VOLUME NAME INS
#        START_OFFSET DURATION  (bag/bag-direct: seconds to skip / play; default = whole bag)
#        HOLD  (bag/bag-direct/demo: default 1 keeps the stack incl. RViz up after the
#              bag ends / the robot's demo run finishes; HOLD=0 auto-exits. Ctrl-C quits)
#        RECORD  (live/demo: RECORD=1 records a ROS 2 bag of the pipeline inputs to
#              output/recordings/<ts>/, replayable in bag-direct; off by default)
#   The container is named NAME (default 'sysnav'), so a second terminal can attach:
#     docker exec -it sysnav bash      # ROS-ready (~/.bashrc sources ROS 2 + workspace)
#   Set NAME=... to run more than one container at once.
#   INS=1 (default 0) automates that attach: once the container is up, run.sh opens a
#   new host terminal already `docker exec`'d into it, with sample ROS inspection
#   commands printed (commented out). Needs a host terminal emulator + X.
#   bag/bag-direct confine ROS 2 discovery to this host (ROS_AUTOMATIC_DISCOVERY_RANGE
#   =LOCALHOST) so two laptops on the same WiFi don't cross-wire; override with
#   ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET if you ever need cross-host ROS 2 there.
#   RVIZ=1 (default) forwards X; you may need `xhost +local:root` once on the host.
#   OBJECTS=1 adds object detection+mapping; default 0 = rooms + navgraph only.
#   Cloud-VLM keys (GEMINI_API_KEY / DASHSCOPE_API_KEY / VLM_PROVIDER) pass through
#   from your environment if set.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

IMAGE="${IMAGE:-sysnav:latest}"
MODE="${MODE:-live}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RVIZ="${RVIZ:-1}"
OBJECTS="${OBJECTS:-0}"
BUILD="${BUILD:-0}"
VOLUME="${VOLUME:-sysnav-build}"   # named volume holding /app/{build,install,log}
NAME="${NAME:-sysnav}"            # container name; also the `docker exec` / INS target
INS="${INS:-0}"                   # 1 = auto-open a 2nd terminal inside the container

# Bind targets for artifacts must exist as the host user (else docker makes them root).
# output/recordings is pre-created too so RECORD=1 bags land host-owned, not root.
mkdir -p "$REPO/output" "$REPO/output/recordings" "$REPO/runlogs"

run=( docker run --rm -it
  --name "$NAME"                           # so `docker exec -it sysnav bash` is predictable
  --gpus all --network host --ipc host
  -e MODE="$MODE" -e RVIZ="$RVIZ" -e OBJECTS="$OBJECTS" -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e BUILD="$BUILD"
  -e FORCE_ENGINE_REBUILD="${FORCE_ENGINE_REBUILD:-0}"
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)"   # supervisor chowns output/+runlogs/ back to you on exit

  -v "$VOLUME:/app"                       # build/install/log persist here
  -v "$REPO/src:/app/src"                 # the volatile workspace (mounted, not baked)
  -v "$REPO/docker:/app/docker:ro"        # supervisor + scripts (edit-without-rebuild)
  -v "$REPO/output:/app/output"
  -v "$REPO/runlogs:/app/runlogs" )

# Build-only subcommands: compile the workspace into the named volume and exit -- no
# robot, no bag, no pipeline. Handled HERE, before the MODE plumbing below, because a
# build needs neither MODE nor ROBOT_IP/BAG (and no GPU/X). The base `run=(...)` above
# already has the volume + src mounts, which is all colcon needs; the supervisor does
# the actual work and exits.
#   docker/run.sh build     incremental colcon build (same compile BUILD=1 does)
#   docker/run.sh rebuild   wipe /app/{build,install,log} first, then build from scratch
case "${1:-}" in
  build|rebuild)
    run+=( "$IMAGE" "$1" )
    echo "+ ${run[*]}"
    exec "${run[@]}"
    ;;
esac

# Cloud-VLM credentials / provider knobs (only if present in the environment).
for k in GEMINI_API_KEY DASHSCOPE_API_KEY VLM_PROVIDER QWEN_MODEL QWEN_MODEL_LITE; do
  [ -n "${!k:-}" ] && run+=( -e "$k=${!k}" )
done

# Optional demo-mode (MODE=demo) overrides; the supervisor bakes sane defaults.
for k in REQ_TOPIC RESP_TOPIC ACK_TIMEOUT; do
  [ -n "${!k:-}" ] && run+=( -e "$k=${!k}" )
done

# RViz needs the host X server AND a GL stack. The CUDA base only requests the
# `compute,utility` driver capabilities, so the container has no OpenGL -- RViz
# falls back to Mesa, fails to reach a DRM device, and dies ("failed to load
# driver: iris"). NVIDIA_DRIVER_CAPABILITIES=all makes the toolkit inject the
# NVIDIA GL/GLX libs (RViz renders on the dGPU); mounting /dev/dri also lets the
# Mesa/iGPU path work as a fallback. QT_X11_NO_MITSHM avoids a Qt/X SHM crash.
if [ "$RVIZ" = "1" ]; then
  run+=( -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix
         -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all
         -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root )
  [ -d /dev/dri ] && run+=( --device /dev/dri )
fi

case "$MODE" in
  live|demo)
    # demo == live wiring (robot is the ROS 1 master), but the in-container
    # supervisor gates the pipeline on a robot request instead of auto-starting.
    : "${ROBOT_IP:?$MODE mode needs ROBOT_IP=<robot ip on its subnet, e.g. 192.168.123.18>}"
    : "${LAPTOP_IP:?$MODE mode needs LAPTOP_IP=<your ip on the robot subnet, e.g. 192.168.123.190>}"
    # HOLD is demo-only here: default 1 keeps the stack (incl. RViz) up after the
    # robot's run finishes (HOLD=0 auto-exits). Harmless in live (its supervisor
    # ignores it -- live already blocks on the pipeline until Ctrl-C).
    # RECORD=1 records a ROS 2 bag of the pipeline inputs (off by default); see
    # the supervisor header. Records the in-container ROS 2 side, so no WiFi cost.
    run+=( -e ROS_MASTER_URI="http://$ROBOT_IP:11311" -e ROS_IP="$LAPTOP_IP"
           -e HOLD="${HOLD:-1}" -e RECORD="${RECORD:-0}" )
    ;;
  bag|bag-direct)
    : "${BAG:?$MODE mode needs BAG=<bag directory>}"
    [ -d "$BAG" ] || { echo "BAG '$BAG' is not a directory" >&2; exit 1; }
    # START_OFFSET / DURATION (seconds) trim playback; empty = whole bag.
    # HOLD (default 1) keeps the stack (incl. RViz) up after the bag finishes;
    # HOLD=0 auto-exits when the bag ends (scripted/batch runs).
    # Confine ROS 2 discovery to this host: bag modes are a self-contained ROS 2
    # graph (bag -> pipeline, no robot), so they never need to talk off-box. With
    # --network host + the default ROS_DOMAIN_ID, two people on the same LAN/WiFi
    # would otherwise auto-discover each other and cross-wire /clock, /tf, node
    # names and /rosbag2_player -- breaking both runs. LOCALHOST keeps each laptop
    # isolated. (live/demo reach the robot over ROS 1, not ROS 2, so this is moot
    # there -- left unset to preserve their default subnet discovery.)
    run+=( -e BAG_PATH=/app/bag -v "$BAG:/app/bag:ro"
           -e START_OFFSET="${START_OFFSET:-}" -e DURATION="${DURATION:-}"
           -e HOLD="${HOLD:-1}"
           -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" )
    ;;
  *) echo "unknown MODE=$MODE (use live | demo | bag | bag-direct)" >&2; exit 2 ;;
esac

# Pass-through extra args (e.g. `shell`) to the supervisor entrypoint.
run+=( "$IMAGE" "$@" )

# INS=1 -- open a second host terminal already attached to the running container,
# with a few sample ROS inspection commands printed (commented out) for copy/paste.
# This is the scripted form of the README's "attach a second terminal" recipe.
# Runs in the background (the main `docker run` below holds the foreground): it waits
# for the container to come up and, best-effort, for the workspace to be sourced (the
# SYSNAV-EXEC-SHELL marker supervisor.sh appends to /root/.bashrc once ROS 2 + the
# colcon install are sourced), then launches whatever terminal emulator the host has.
open_inspector() {
  local name="$1" ns="$2" i c de launcher

  # 1) wait for the container to exist and be running.
  for i in $(seq 1 120); do
    if [ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" = "true" ]; then break; fi
    sleep 1
  done
  if [ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" != "true" ]; then
    echo "[run.sh] INS: container '$name' never came up; skipping inspection terminal." >&2
    return 0
  fi

  # 2) best-effort: wait until the workspace is sourced in interactive shells, so the
  #    opened terminal is ROS-ready immediately. On the first run this waits out the
  #    one-time colcon build; later runs it's ready in seconds. Open anyway on timeout
  #    -- the shell still works, ROS just may need a manual source.
  for i in $(seq 1 300); do
    if docker exec "$name" grep -q 'SYSNAV-EXEC-SHELL' /root/.bashrc 2>/dev/null; then break; fi
    sleep 2
  done

  # 3) find a terminal emulator on the host.
  for c in x-terminal-emulator gnome-terminal konsole xfce4-terminal tilix xterm; do
    if command -v "$c" >/dev/null 2>&1; then de="$c"; break; fi
  done
  if [ -z "${de:-}" ]; then
    echo "[run.sh] INS: no terminal emulator found. Attach manually:" >&2
    echo "         docker exec -it $name bash" >&2
    return 0
  fi

  # 4) write the launcher the terminal runs: `docker exec` into the container, print
  #    the (commented) sample commands, then drop to an interactive shell (which sources
  #    ~/.bashrc -> ROS 2 + workspace). A single no-space path so every emulator's
  #    -e / -- accepts it without re-quoting headaches; the container-side banner is a
  #    quoted heredoc, so the surrounding bash -c '...' stays intact.
  launcher="$(mktemp)"
  cat > "$launcher" <<EOF
#!/usr/bin/env bash
exec docker exec -it $name bash -c '
cat <<"BANNER"

# ===== SysNav inspection shell (container: $name) =====
# ROS 2 + the workspace are already sourced (~/.bashrc). Sample commands
# (commented out -- copy a line, drop the leading #, and run):
#
#   ros2 topic list                                       # bridged inputs + pipeline topics
#   ros2 topic hz   /$ns/cloud_registered                 # is the registered cloud flowing?
#   ros2 topic hz   /$ns/lio/odometry                     # odometry rate
#   ros2 topic echo /$ns/camera_rect/camera_info --once   # camera intrinsics
#   ros2 node list                                        # running nodes
#   ros2 run tf2_tools view_frames                        # dump the TF tree -> frames.pdf
#
# Raw ROS 1 robot topics (live/demo only) need the Noetic overlay:
#   source /opt/ros/noetic/setup.bash && rostopic hz /$ns/...
# ======================================================
BANNER
exec bash'
EOF
  chmod +x "$launcher"

  echo "[run.sh] INS: opening inspection terminal ($de) on container '$name'."
  case "$de" in
    gnome-terminal) "$de" --title="sysnav inspect"      -- "$launcher" ;;
    konsole)        "$de" -p "tabtitle=sysnav inspect"  -e "$launcher" ;;
    xfce4-terminal) "$de" --title="sysnav inspect"      -e "$launcher" ;;
    tilix)          "$de" -t "sysnav inspect"           -e "$launcher" ;;
    xterm)          "$de" -T "sysnav inspect"           -e "$launcher" ;;
    *)              "$de"                                -e "$launcher" ;;
  esac >/dev/null 2>&1 || \
    echo "[run.sh] INS: failed to open $de; attach manually: docker exec -it $name bash" >&2

  ( sleep 10; rm -f "$launcher" ) >/dev/null 2>&1 &   # reap the temp launcher
}

echo "+ ${run[*]}"

# Host-side cleanup on exit: revoke the X grant (if we made one) and stop the INS
# watcher (if we started one).
INS_PID=""
cleanup_host() {
  [ -n "${INS_PID:-}" ] && kill "$INS_PID" >/dev/null 2>&1 || true
  if [ "$RVIZ" = "1" ] && command -v xhost >/dev/null 2>&1; then
    xhost -local:root >/dev/null 2>&1 || true
  fi
}

# RViz (run as root in the container) can't open the host X server until it's
# authorized -- otherwise it aborts with "Authorization required ... could not
# connect to display :0" and the pipeline runs on, headless. Grant local root
# access for the lifetime of this run and revoke on exit, so we don't leave the
# host X server loosened. Skip silently if xhost isn't installed / no X.
if [ "$RVIZ" = "1" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# INS=1: launch the background watcher that opens the in-container terminal once it's
# up. Namespace for the sample commands comes from robot.yaml (the single source).
if [ "$INS" = "1" ]; then
  ns_ins="$(grep -E '^[[:space:]]*robot_namespace:' \
              "$REPO/src/exploration_planner/tare_planner/config/robot.yaml" 2>/dev/null \
            | head -1 | sed -E 's/.*robot_namespace:[[:space:]]*//; s/#.*$//; s/[[:space:]]*$//' || true)"
  ns_ins="${ns_ins:-<ns>}"
  echo "[run.sh] INS=1: will open an inspection terminal once '$NAME' is up (ns=$ns_ins)."
  open_inspector "$NAME" "$ns_ins" &
  INS_PID=$!
fi

# Use a trap (no exec) when there's host-side state to clean up; otherwise exec for free.
if { [ "$RVIZ" = "1" ] && command -v xhost >/dev/null 2>&1; } || [ "$INS" = "1" ]; then
  trap cleanup_host EXIT
  "${run[@]}"
else
  exec "${run[@]}"
fi
