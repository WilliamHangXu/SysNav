# Pipeline speed profiling

Component rates and per-frame latencies of the scene-graph pipeline, measured 2026-09-14.
Two complementary experiments:

1. **Achieved rates (Hz)** — how often each component actually produced output during real
   robot missions. Mined from the seven `bags/<location>/<name>_exploration` recordings
   (Sep 5–7, 2026) with [`bag_hz.py`](bag_hz.py), which computes per-topic rates from message
   receive timestamps (mean over the topic's active window + median inter-arrival, robust to
   late-starting topics).
2. **Per-frame latency (ms)** — how long each processing stage takes, independent of input
   rate. Collected on a live sim run (see *Reproducing*) via lightweight instrumentation that
   is compiled in but **off by default**: setting `SYSNAV_PROFILE_DIR=<dir>` in the launch
   environment makes `detection_node`, `semantic_mapping_node` and `room_segmentation` append
   per-frame timing CSVs there; unset, they write nothing.

Hardware for both: RTX 4090 Laptop GPU (16 GB), the machine that also runs the robot stack.

## 1. Achieved rates on the real robot

Seven exploration runs, 208–480 s each; values are cross-run means (min–max). The runs are
remarkably consistent.

| Component | Topic(s) measured | Achieved Hz |
|---|---|---|
| YOLO + SAM object mapping | `/annotated_image`, `/obj_points` | **1.89** (1.80–1.94) |
| — object markers | `/obj_boxes`, `/obj_labels` | 2.05 (1.85–2.33) |
| Room segmentation | `/room_mask_vis`, `/walls`, `/room_map_cloud` | **1.22** (1.07–1.42) |
| Scene-graph node cycle | `/keypose_graph_cloud`, `/object_node_markers`, … | **0.90** (0.81–0.94) |
| BEV mapper (global / local) | `/bev_map/grid` / `/bev_map/local` | 1.00 / 4.89 (its 1 Hz / 5 Hz timers) |
| Local planner | `/path`, `/free_paths` | 9.69 (~10 Hz design rate) |
| State estimation TF | `/tf` | 54.7 |

Camera input on the robot: the Theta Z1 driver publishes **10 Hz** at 1920×960
(`src/utilities/receive_theta/readme.txt`), so the semantic side is compute-bound — it
processes roughly 1 in 5 camera frames — while BEV and the local planner hit their design
rates exactly.

## 2. Per-frame latency (live sim, ~8 min autonomous wander)

1837 detection frames / 803 mapping frames / 369 room-segmentation cycles; ~12 detections
per frame throughout the run.

| Stage | median ms | mean | p95 |
|---|---|---|---|
| **YOLO** inference (YOLOE-26x-seg TensorRT engine, 640×1920, fp16) | **60.2** | 66.9 | 95.6 |
| YOLO full callback (+ annotate + publish) | 65.9 | 72.7 | 104.7 |
| **SAM2** (hiera-b+, ~12 boxes) | **73.3** | 78.4 | 100.4 |
| mapping: annotate | 52.6 | 53.9 | 66.7 |
| mapping: **map update** (lidar–mask fusion + object map) | **206.7** | 204.9 | 248.9 |
| mapping node total per frame | **334.3** | 337.3 | 394.4 |
| **Room segmentation** full cycle | **65.5** | 71.6 | 104.4 |

## Analysis

- **Detection is not the bottleneck.** The TensorRT YOLO runs at ~15 Hz capacity (60 ms);
  the robot camera feeds 10 Hz, so detection could keep up with every frame.
- **The object-mapping backend is the bottleneck.** The mapping node spends ~334 ms per
  frame (~3 Hz capacity), which caps the deployed semantic rate at the observed ~1.9 Hz.
  Within it, the CPU-side map update dominates (207 ms) — not SAM2 (73 ms). That stage
  (`semantic_map_new.update_map`) projects the full lidar cloud into the panorama, builds a
  labeled cloud per SAM mask (depth-continuity filtered), associates clouds to persistent
  objects by track id (cKDTree vote merging), and then runs a global optimization over
  *every* object in the map each frame: DBSCAN shape regularization for inactive objects
  and an all-pairs nearest-centroid scan with oriented-3D-bbox IoU for duplicate merging.
  The last phase scales ~quadratically with the number of mapped objects, so real-building
  runs (many more than the ~12 sim objects) can be expected to sit at or above these times.
- **Room segmentation is trigger-limited, not compute-limited.** A full cycle costs only
  66 ms; its ~1.2 Hz in missions is the rate at which segmentation is triggered (100 ms
  timer + `segment_flag_` gating), leaving ample headroom.
- **Caveats.** Latency was measured in sim while Unity shared the same GPU, so the GPU
  numbers (YOLO, SAM2) are mildly pessimistic. SAM2's time depends on box count (~12 here).
  Do not quote sim input rates — the sim camera is jittery (~1–4 Hz); the robot's input
  figure is the Theta's 10 Hz.

## Reproducing

Achieved rates from any set of viz bags (`record_viz_bag.sh` output):

```bash
unset PYTHONPATH && source /opt/ros/jazzy/setup.bash
python3 tools/profiling/bag_hz.py /tmp/bag_hz.csv bags/*/*_exploration
```

Per-frame latency from any live run (sim or real robot — for paper-grade robot numbers,
do this once on the robot):

```bash
export SYSNAV_PROFILE_DIR=$PWD/output/profiling_$(date +%Y%m%d_%H%M%S)
./system_real_robot_teleop.sh          # or ./system_simulation_teleop.sh
# ... run the mission, then:
python3 tools/profiling/aggregate_profile.py "$SYSNAV_PROFILE_DIR"
```

Raw data for the tables above: `output/profiling_20260914_000836/` (latency CSVs).
Instrumentation lives in `src/semantic_mapping/semantic_mapping/detection_node.py`,
`semantic_mapping_node.py`, and
`src/exploration_planner/tare_planner/src/room_segmentation/room_segmentation.cpp`
(the C++ part needs a Release rebuild after edits).
