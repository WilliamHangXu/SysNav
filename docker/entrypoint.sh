#!/usr/bin/env bash
#
# Container entrypoint for AWS (EC2 GPU / AWS Batch).
#
# Lifecycle:
#   1. source ROS + workspace overlays
#   2. pull the rosbag from S3  (BAG_S3_URI -> $BAG_LOCAL)
#   3. run the headless system against the bag (docker/run_system.sh)
#   4. push outputs back to S3  ($OUTPUT_DIRS -> OUTPUT_S3_URI)
#
# Environment variables:
#   BAG_S3_URI      s3://bucket/path/to/rosbag2_dir   (a ros2 bag is a DIRECTORY)
#   BAG_LOCAL       local path to stage the bag        (default: /data/bag)
#   OUTPUT_S3_URI   s3://bucket/path/results            (optional; skip to keep local)
#   GEMINI_API_KEY / DASHSCOPE_API_KEY                  (inject via Secrets Manager / SSM)
#   VLM_PROVIDER, QWEN_MODEL, QWEN_MODEL_LITE           (optional, see README)
#   AUTOPLAY_DELAY  seconds to wait before un-pausing the bag (default: 15)
#
set -euo pipefail

# ROS setup scripts reference unset vars (e.g. AMENT_TRACE_SETUP_FILES) and are
# not `set -u`-safe, so relax nounset just around sourcing the overlays.
set +u
source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash
set -u

: "${BAG_LOCAL:=/data/bag}"

# --- 1. fetch bag from S3 ----------------------------------------------------
if [[ -n "${BAG_S3_URI:-}" ]]; then
  echo "[entrypoint] downloading bag: ${BAG_S3_URI} -> ${BAG_LOCAL}"
  mkdir -p "${BAG_LOCAL}"
  aws s3 cp --recursive "${BAG_S3_URI}" "${BAG_LOCAL}"
else
  echo "[entrypoint] BAG_S3_URI not set; expecting a bag already at ${BAG_LOCAL}"
fi

if [[ ! -e "${BAG_LOCAL}/metadata.yaml" ]]; then
  echo "[entrypoint] ERROR: no metadata.yaml under ${BAG_LOCAL} — not a valid ros2 bag." >&2
  exit 1
fi

# --- 1b. ensure YOLO TensorRT engines exist (built on THIS GPU) --------------
# Engines are GPU-architecture specific and need a live GPU, so they cannot be
# baked at image-build time. Build them here, on the real GPU, only when absent.
# Set FORCE_ENGINE_REBUILD=1 to force a rebuild (e.g. when the deploy GPU differs
# from the host that produced the baked engines — required on AWS).
EXT_DIR="/app/src/semantic_mapping/semantic_mapping/external"
if [[ "${FORCE_ENGINE_REBUILD:-0}" == "1" ]]; then
  echo "[entrypoint] FORCE_ENGINE_REBUILD=1 — removing existing engines"
  rm -f "${EXT_DIR}/yoloe-26x-seg.engine" "${EXT_DIR}/yolov8x-worldv2_cus.engine"
fi
if [[ -f "${EXT_DIR}/yoloe-26x-seg.engine" && -f "${EXT_DIR}/yolov8x-worldv2_cus.engine" ]]; then
  echo "[entrypoint] YOLO TensorRT engines present; skipping export."
else
  echo "[entrypoint] building YOLO TensorRT engines on local GPU (one-time)..."
  ( cd /app && python3 set_yolo_e.py && python3 set_yolo_world.py )
fi

# --- 2. run the system -------------------------------------------------------
export BAG_LOCAL
set +e
BAG_PATH="${BAG_LOCAL}" /app/docker/run_system.sh
run_rc=$?
set -e
echo "[entrypoint] system exited with code ${run_rc}"

# --- 3. push outputs ---------------------------------------------------------
if [[ -n "${OUTPUT_S3_URI:-}" ]]; then
  echo "[entrypoint] uploading outputs -> ${OUTPUT_S3_URI}"
  for d in output debug runlogs; do
    if [[ -d "/app/${d}" ]]; then
      aws s3 cp --recursive "/app/${d}" "${OUTPUT_S3_URI%/}/${d}" || true
    fi
  done
  # top-level map snapshots
  shopt -s nullglob
  for f in /app/*.png; do
    aws s3 cp "$f" "${OUTPUT_S3_URI%/}/maps/$(basename "$f")" || true
  done
  shopt -u nullglob
fi

exit "${run_rc}"
