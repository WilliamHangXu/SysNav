# semantic_mapping

ROS 2 package that builds and maintains a 3D semantic object map from a single RGB camera + a registered LiDAR scan + odometry. Detection (YOLO‑World) and SAM2 segmentation are fused with the cloud to *lift* every detected mask into a per‑instance voxel cloud. A lifecycle manager keeps a persistent list of `SingleObject`s, merging duplicates, growing them with new observations, and deleting stale ones.

Two ROS nodes are shipped:

| Executable | File | Purpose |
| --- | --- | --- |
| `detection_node` | `semantic_mapping/detection_node.py` | YOLO‑World + BoT‑SORT 2D detection/tracking. Publishes `DetectionResult` (bboxes + track ids + RGB). |
| `semantic_mapping_node` | `semantic_mapping/semantic_mapping_node.py` | Time‑syncs detections / odom / cloud, runs SAM2, lifts masks to 3D, and drives the lifecycle (`ObjMapper`). |

---

## Pipeline at a glance

```
                    /camera/image
                          │
                          ▼
                  ┌───────────────┐
                  │ detection_node│  YOLOv8‑World (+ BoT‑SORT track ids)
                  └───────────────┘
                          │  DetectionResult (track_id, xyxy, label, conf, image)
                          ▼
/registered_scan ─►┌──────────────────────────────────┐◄─/state_estimation (Odom)
                   │     semantic_mapping_node        │
/viewpoint_rep_hdr►│  • SAM2 box→mask                 │◄─/target_object_instruction
/keyboard_input ──►│  • temporal sync (SLERP odom,    │   /object_type_answer
                   │     ±0.5s cloud window)          │
                   │  • CloudImageFusion (lidar→img)  │
                   │  • ObjMapper.update_map(...)     │──► /obj_points, /obj_boxes,
                   └──────────────────────────────────┘    /obj_labels,
                                  │                        /object_nodes_list,
                                  ▼                        /object_type_query,
                          ObjMapper.single_obj_list        /annotated_image
                          (persistent SingleObjects)
```

`detection_node` and `semantic_mapping_node` are independent processes; they only meet over `/detection_result`. Either can be re‑run without restarting the other.

---

## Files

```
semantic_mapping/
├── detection_node.py         # YOLO‑World + BoT‑SORT, publishes DetectionResult
├── semantic_mapping_node.py  # Main node: time sync, SAM2, calls ObjMapper
├── semantic_map_new.py       # ObjMapper – manages list of SingleObject, add/update/merge/delete
├── single_object_new.py      # SingleObject + VoxelFeatureManager – per‑instance state
├── cloud_image_fusion.py     # Per‑platform lidar→pixel projection + mask→cloud association
├── utils.py                  # Angle / color / stamp helpers, label→color map
├── visualizer.py             # (optional) Rerun visualizer
├── tools/ros2_bag_utils.py   # Helpers for PointCloud2/Marker/Odom/TF + bag writer
├── config/objects.yaml       # Open‑vocabulary class list (text prompts, is_instance flag)
└── config/botsort.yaml       # Tracker config used by ultralytics
launch/                       # *.launch files (real / sim / bagfile / go2w)
config/                       # ros__parameters yamls per platform
```

External weights are expected at:

- `semantic_mapping/external/sam2/checkpoints/sam2.1_hiera_base_plus.pt` (config `configs/sam2.1/sam2.1_hiera_b+.yaml`)
- `semantic_mapping/external/yolov8x-worldv2_cus.engine` (TensorRT engine; substitute the equivalent `.pt` if no engine)

---

## ROS interface

### `detection_node`
| dir | topic | type | notes |
|---|---|---|---|
| sub | `image_topic` (param, e.g. `/camera/image` or `/go2w_006/camera/image_rect_color`) | `sensor_msgs/Image` | BGR8 |
| pub | `/detection_result` | `tare_planner/DetectionResult` | bboxes + track ids + RGB image |
| pub | `/annotated_image_detection` | `sensor_msgs/Image` | optional debug |

### `semantic_mapping_node`
| dir | topic | type | notes |
|---|---|---|---|
| sub | `/detection_result` | `DetectionResult` | from `detection_node` |
| sub | `/registered_scan` | `PointCloud2` | LiDAR cloud in world (`map`) frame |
| sub | `/state_estimation` (or `/aft_mapped_to_init_incremental` on `mecanum_bagfile`) | `Odometry` | robot pose |
| sub | `/viewpoint_rep_header` | `ViewpointRep` | TARE viewpoint timestamps; trigger one mapping update aligned to that stamp + saves viewpoint image |
| sub | `/target_object_instruction` | `TargetObjectInstruction` | sets `target_object` / `anchor_object`; forces re‑publish of all currently‑labeled targets |
| sub | `/object_type_answer` | `ObjectType` | VLM confirmation; locks a label via `update_target_object` |
| sub | `/keyboard_input` | `String` | `"demo"` freezes map writes, `"resume"` unfreezes |
| pub | `/obj_points` | `PointCloud2` | colored voxels of all persistent objects |
| pub | `/obj_boxes`, `/obj_labels` | `MarkerArray` | wireframe boxes + text labels (with `DELETE` markers for removed objects) |
| pub | `/object_nodes_list` | `ObjectNodeList` | structured per‑object output. `status=True` = upsert, `status=False` = delete |
| pub | `/object_type_query` | `ObjectType` | asks a VLM/captioner about a target candidate (sends best image path + id + label history) |
| pub | `/annotated_image` | `sensor_msgs/Image` | SAM masks + tracker ids overlay |

`tare_planner/ObjectNode` carries: `int32[] object_id` (list, because merges concatenate ids), `label`, `position`, `bbox3d[8]`, `cloud`, `status` (alive/deleted), `img_path` (best masked crop on disk), `is_asked_vlm`, `viewpoint_id`.

---

## How `semantic_mapping_node` ticks

A 0.5 s timer (`mapping_callback`) drives the loop:

1. **Pick a detection frame.** Free‑run mode uses the second‑newest `DetectionResult`. If a `/viewpoint_rep_header` stamp is queued, pick the detection closest to that stamp instead (it is also saved to `output/viewpoint_images/`).
2. **Sync odometry.** Find the two odometry samples bracketing the detection stamp; linearly interpolate position/velocity and SLERP orientation to obtain `camera_odom` at the detection time. If detection is older than the oldest odom, drop and wait.
3. **Gather a cloud window.** Concatenate every `/registered_scan` arriving in `[det_stamp − 0.5 s, det_stamp + 0.1 s]`. Reject the frame if empty.
4. **Mapping processing (`mapping_processing`):**
   - Run **SAM2** with the YOLO bboxes as prompts → per‑detection binary masks (BF16 autocast).
   - Annotate + publish debug image (`/annotated_image`).
   - If not `demo_frozen`: `ObjMapper.update_map(detections, stamp, camera_odom, cloud, image, viewpoint_stamp)`.
   - Publish results via `publish_map` (deleted markers first, then upsert markers + cloud + `ObjectNodeList`).
   - Run `check_target_objects()`: if any object has `label ∈ {target_object, anchor_object}`, is unverified by VLM, and `best_image_score > 500`, fire `/object_type_query` so a downstream VLM can confirm/relabel it.
5. Every 10 calls force `gc.collect()` + `torch.cuda.empty_cache()`.

Locks (`cloud_cbk_lock`, `odom_cbk_lock`, `rgb_cbk_lock`, `detection_result_lock`, `mapping_processing_lock`) protect the per‑topic queues. `self.timestamp` advances after each cycle and the callbacks evict anything older than `timestamp` to bound memory.

---

## `CloudImageFusion` – mask → 3D cloud

`CloudImageFusion(platform)` picks a `scan2pixels_<platform>` function. Supported: `wheelchair`, `mecanum`, `mecanum_bagfile`, `mecanum_sim`, `scannet`, `diablo`, `go2w`. Each one encodes the platform‑specific LiDAR‑to‑camera extrinsics + camera intrinsics (panoramic or pinhole). The `go2w_bag` path (direct LIO) sources its calibration from `camera_info` + tf at runtime and applies no gravity rotation.

`generate_seg_cloud(cloud, masks, labels, confidences, R_b2w, t_b2w, image_src)`:

1. Project the (body‑frame) cloud to pixel indices + depth via `scan2pixels`. The platform projection also Z‑buffers / occlusion‑filters using a 3‑pixel minimum filter so background points behind a foreground surface get dropped.
2. Drop points outside the image.
3. For each detection mask, gather the cloud points whose projection falls inside the mask.
4. **Depth‑jump filter** (anti‑bleeding through the mask onto a wall behind): the cloud is depth‑sorted; if the largest consecutive depth gap exceeds 0.3 m, keep only the near segment before the jump. This is the main mechanism preventing background points from being lifted as part of the object.
5. Transform survivors to world frame: `pts @ R_b2w.T + t_b2w`.
6. Return the per‑object world‑frame point lists.

---

## `SingleObject` – the per‑instance state

```
class_id: dict[label → vote count]
conf_list: dict[label → max conf seen]
points_count: dict[label → cumulative point count]
weighted_class_scores: dict[label → points * max_conf]    # used for dominant label
obj_id: list[int]                                          # tracker ids merged into this object
voxel_manager: VoxelFeatureManager                         # voxels + per‑voxel votes + obs angle bins
robot_poses: list[{R,t}]                                   # unique observation poses
key_frames / key_pose: list[mask], pose                    # for re‑projection
life, inactive_frame, latest_stamp, info_frames_cnt        # lifecycle counters
status, updated, updated_by_vlm, publish_status            # change tracking for publishing
best_image_path, best_image_score, base_image_dir          # best (largest mask area) crop saved on disk
is_asked_vlm                                               # set True once VLM answered
spatial_relations                                          # placeholder dict, currently unused
```

### `VoxelFeatureManager`
A voxelized representation backed by a kd‑tree on voxel centers. Each voxel stores:

- `vote`: integer count of how many times this voxel has been re‑observed.
- `observation_angles`: `[N, num_angle_bin]` binary matrix marking which azimuth bins (relative to the camera at observation time) have seen this voxel. Used as a *diversity* score so points visible from many sides count more.
- `regularized_voxel_mask`: which voxels survived `regularize_shape_v2` (DBSCAN‑based outlier rejection).
- `remove_vote`: votes for *removing* a voxel (used by reprojection‑against‑mask checks).

Key voxel operations:

- `update(voxels, R, t)` – merge a new observation: voxels closer than `voxel_size` (3 cm) to existing voxels just bump the vote + add their angle bin; further voxels are appended; the kd‑tree is rebuilt.
- `update_through_vote_stat(other)` – used during object merging: fuse another `VoxelFeatureManager`'s voxels, votes and observation angles.
- `reproject_obs_angle(R_w2b, t_w2b, mask, projection_func)` – reproject this object's voxels into the current image: for the subset landing inside the latest mask, mark the corresponding azimuth bin as observed.
- `retrieve_valid_voxel_indices(diversity_percentile)` – sort voxels by `sum(obs_angles) * votes` and keep the top `(1 - p)` fraction.

### Shape regularization (`regularize_shape_v2`)
On each update of an existing object:

1. DBSCAN cluster the voxels (`eps = 2.5 * voxel_size`, `min_points` 5 or 10).
2. Compute each cluster's weight = `sum(observation_angles)`. Drop clusters with weight `< 5`.
3. Pick the heaviest cluster as the main; for the others, multiply weight by `exp(-3 * dist_to_main / extent_diag)` so far‑away spurious clusters decay fast.
4. Greedily accept clusters until the accumulated adjusted weight reaches `percentile_thresh = 0.85` of the total. Mark accepted voxels in `regularized_voxel_mask`.

The oriented 3D bbox is built from the regularized voxels via a 2D minimum‑bounding‑rectangle (rotating calipers on the convex hull, in the XY plane) extruded along Z by `[z_min, z_max]`.

---

## `ObjMapper.update_map` – the lifecycle

This is the heart of the package. Called once per accepted frame with all current detections.

### A) Per‑detection ingest (association)

```
for each detection i with confidence ≥ 0.30 (= confidence_thres):
    erode mask by 3x3 kernel ×2          # peel boundary so we don't drag in neighbors
    lift mask to world cloud (CloudImageFusion)
    drop points with ||p - robot_t|| ≥ 8.0 m   (cloud_to_odom_dist_thres)
    skip if cloud has < 2 points (unless this is a viewpoint frame)
    voxel‑downsample to 0.03 m
    obj_id = detection track_id (from BoT‑SORT)

    if obj_id < 0:                       # tracker gave no id → background
        append new SingleObject to background_obj_list
    elif obj_id matches some existing single_obj.obj_id:
        single_obj.update(...)           # update voxels, vote labels, store pose
        single_obj.reproject_obs_angle(...) # update angle coverage from this image
        single_obj.inactive_frame = -1
        single_obj.save_best_image(image, mask, conf, save_queue)   # best crop, by mask area
    else:
        new SingleObject → single_obj_list
        save initial best image
```

`SingleObject.update` only counts the observation toward the class histogram when the new robot pose is *not* similar to a previously stored one (angle Δ ≥ 5° or distance Δ ≥ 0.3 m). This stops a stationary robot from voting one label N times.

### B) Per‑object optimization (merge / delete)

After ingest, iterate `single_obj_list`:

```
for i, obj in single_obj_list:
    obj.life += 1
    if obj.inactive_frame > 20: continue            # marked stale; left alone

    if obj.life ∈ (0, 1200):
        if not obj.updated:                          # not seen this cycle
            obj.inactive_frame += 1
            obj.regularize_shape_v2(0.85)
            # DELETE branch ────────────────────────
            if (valid_regularized_voxels < 15
                and inactive_frame > 50
                and dominant_label ≠ target_object):
                publish_deleted_object(obj)          # batched into deleted_objects_batch
                obj.cleanup_images(save_queue)       # async delete crop file
                remove obj from single_obj_list
            continue

        # obj was updated this cycle
        recompute centroid and oriented bbox
        # Find nearest same‑class neighbor:
        target_same = argmin_{j ≠ i, same label} ||centroid_j - centroid_i||

        # MERGE branch (same‑class) ────────────────
        merge if any of:
          • dist < ||(extent_i/2 + extent_j/2)/2|| · 0.5
          • dist < 0.25 m
          • 3D IoU > 0.20
          • ratio_obj > 0.4  or  ratio_target > 0.4
        merge_object: voxel fusion, vote fusion, dedup poses, concat obj_ids
        publish_deleted_object(target)
        remove target from list

        # Cross‑class merge: currently DISABLED (kept as comments).

    if not merged: regularize_shape_v2 again
```

Same‑class merging is by far the most common map‑shrinking event: it collapses a tracker that fragmented into multiple ids back into one object. Background objects (`obj_id < 0`) live in a separate list and never participate in merging.

`MERGE_PRIMITIVE_GROUPS` (`[chair, sofa]`, `[table, desk]`) lists labels treated as equivalent by `is_merge_allowed`, but the same‑class merge branch above only fires when `dominant_label`s match; the primitive groups are checked elsewhere via `is_merge_allowed` (called by future cross‑class branches — kept for extensibility).

### C) Publishing

`to_ros2_msgs(stamp, viewpoint_id)` walks `single_obj_list` and, for every object that was `updated` since the last publish, emits:

- a voxel cloud, colored by `map_label_to_color(label)` (red if `label == target_object` and `not is_asked_vlm`),
- a wireframe bbox `Marker` (id = first `obj_id`),
- a text `Marker` showing `label(id)` or `Potential Target(id)` for unverified targets,
- an `ObjectNode` entry in a single `ObjectNodeList` published per cycle.

`to_ros2_msgs_deleted(stamp)` flushes `DELETE` markers for objects removed earlier. `flush_deleted_objects_batch` publishes one `ObjectNodeList` with `status=False` for the batch. The `updated` / `updated_by_vlm` / `publish_status` flags are cleared after publishing so the same object isn't re‑sent every tick.

### D) Target‑object loop (VLM verification)

Open‑vocabulary YOLO labels are noisy. When the operator (via `/target_object_instruction`) names a target (e.g. `"refrigerator"`):

1. Any existing object with that label is forced to re‑publish (`updated = True`).
2. After each `update_map`, `check_target_objects()` returns candidates whose label matches `target_object` or `anchor_object`, have not yet been confirmed by the VLM (`is_asked_vlm == False`), and have a sufficiently large best image (`best_image_score > 500`).
3. The node publishes `/object_type_query` with the object id, the candidate label set, and the path to the best masked crop on disk.
4. An external VLM/captioner responds on `/object_type_answer` with `final_label`. `ObjMapper.update_target_object(id, final_label)` bumps that label's vote (+50) and confidence (50.0) so `weighted_class_scores` makes it dominant, then sets `is_asked_vlm = True` and forces a republish.

The `Captioner` import is optional; if `captioner` is not on the Python path the module loads in a "no captioning" fallback.

---

## Object lifecycle states (quick reference)

| Event | Trigger | What changes |
|---|---|---|
| **Add (new instance)** | New tracker id with `obj_id ≥ 0` not seen before | New `SingleObject` appended to `single_obj_list`; initial mask + image saved |
| **Add (background)** | `obj_id < 0` (tracker lost id) | Appended to `background_obj_list`; never merged or VLM‑queried |
| **Update** | Detection id matches existing object | Voxels merged, label histogram bumped, pose stored (if novel), `inactive_frame = -1`, best image possibly replaced |
| **Reproject‑only update** | Object visible but not detected this frame | `reproject_obs_angle` adds angle coverage; `inactive_frame += 1` only happens through the no‑update branch |
| **Merge** | Same‑label neighbor close enough or 3D IoU/ratio over threshold | Voxel managers fused (`update_through_vote_stat`), obj_id lists concatenated, ids/poses deduped, the *other* object is removed and announced via `status=False` `ObjectNode` |
| **VLM relabel** | `/object_type_answer` arrives | `class_id[new_label] += 50`, `conf_list[new_label] = 50`, `is_asked_vlm = True`, force republish |
| **Delete** | Not updated this frame **and** `inactive_frame > 50` **and** `<15` valid regularized voxels **and** label ≠ target | Removed from list, image file deletion queued, `status=False` ObjectNode published in next batch |

Inactive‑without‑deletion (`inactive_frame > 20`) is a soft state: the object stays in the list and continues to be published as long as it has enough voxels — it just won't be re‑optimized.

---

## Configuration

### Per‑platform ROS yamls (`config/`)
| File | Platform |
|---|---|
| `mapping_mecanum_real.yaml` | real mecanum robot |
| `mapping_mecanum_sim.yaml`  | Gazebo mecanum |
| `mapping_mecanum_bagfile.yaml` | replay a bag (different odom topic) |
| `mapping_go2w.yaml` | Unitree Go2‑W (pinhole front camera, image topic override) |

Common keys: `platform` (must be a key recognised by `CloudImageFusion`), `target_object`, `grounding_score_thresh`, `annotate_image`, `object_file` (path to `objects.yaml`), `detection_*_time_bias` (advance/delay detection timestamps relative to odom — useful for sensor latency calibration). `detection_node` extra: `image_topic`.

### Class vocabulary – `config/objects.yaml`
Hierarchical map `meta_label → { prompts: [text...], is_instance: bool }`. The flattened list of `prompts` is what the YOLO‑World text encoder sees; `is_instance` controls whether merging treats each detection as a separate instance.

### Tracker – `config/botsort.yaml`
Standard ultralytics BoT‑SORT config used by `YOLO.track(..., tracker=...)`.

### Tunable mapping constants (`ObjMapper.__init__`)
| Const | Default | Meaning |
|---|---|---|
| `voxel_size` | 0.03 m | downsample + association radius |
| `confidence_thres` | 0.30 | detection confidence floor |
| `cloud_to_odom_dist_thres` | 8.0 m | max distance from robot for a point to count |
| `ground_height` | -0.5 m | (currently not enforced; commented) |
| `num_angle_bin` | 20 | azimuth bins for diversity weighting |
| `percentile_thresh` | 0.85 | weighted percentile kept by `regularize_shape_v2` and `retrieve_valid_voxel_indices` |
| `save_object_image` | True | save best masked crop per object |
| `image_save_interval` | 1 | every N frames |
| `DYNAMIC_CLEARING_VOTE_THRESH` | dict | per‑class threshold for tracker‑driven dynamic clearing (lookup table only, used by older code paths) |
| inactive‑remove rules | `inactive>50`, `<15` voxels | see lifecycle table above |
| same‑class merge | dist<0.25 m / IoU>0.20 / ratio>0.4 | see lifecycle B above |

### Output directories (cleared on startup if `annotate_image=True`)
- `output/debug_mapper/segmentation_results/` – annotated PNGs
- `output/viewpoint_images/` – RGB + transform `.npy` per TARE viewpoint
- `output/object_images/` – best masked crop per object (`{obj_id}.npy` + `{obj_id}_mask.npy`, optionally `.png` if `save_png=True`)

A background `save_worker` thread drains `obj_mapper.save_queue` so disk I/O does not block the mapping callback. Queue items are `(flag, img_path, mask_path, img, mask)`; `flag=1` writes, `flag=0` deletes.

---

## Running

Build the workspace once (`colcon build --packages-select semantic_mapping tare_planner`). Then in two terminals:

```bash
ros2 launch semantic_mapping detection_node.launch use_sim_time:=false
ros2 launch semantic_mapping semantic_mapping_real.launch use_sim_time:=false
```

Launch variants:

| Launch file | When to use |
|---|---|
| `semantic_mapping_real.launch` + `detection_node.launch` | real mecanum |
| `semantic_mapping_sim.launch` + `detection_node.launch` | sim (uses `use_sim_time:=true`) |
| `semantic_mapping_bagfile.launch` + `detection_node.launch` | bag replay (subscribes `/aft_mapped_to_init_incremental` for odom) |
| `semantic_mapping_go2w.launch` + `detection_node_go2w.launch` | Unitree Go2‑W (pinhole camera, gravity‑aligned cloud) |

CLI debugging hooks:
```bash
ros2 topic pub /keyboard_input std_msgs/String "data: 'demo'"     # freeze writes
ros2 topic pub /keyboard_input std_msgs/String "data: 'resume'"   # unfreeze
ros2 topic pub /target_object_instruction tare_planner/TargetObjectInstruction \
  "{target_object: 'refrigerator', anchor_object: ''}"
```

---

## When something breaks – first places to look

- **No objects appear** → check `/detection_result` is being published, `confidence_thres` (0.30) isn't filtering everything, and `platform` matches the actual sensor stack.
- **Objects fly off into the wall / floor** → wrong `scan2pixels_<platform>` extrinsics in `cloud_image_fusion.py`, or the depth‑jump cutoff (0.3 m) is too lax for a cluttered scene.
- **Duplicates of the same object** → merge thresholds are too tight; loosen IoU/ratio in `update_map` § B.
- **Single object keeps splitting** → BoT‑SORT id swapping; check `config/botsort.yaml` and the YOLO confidence threshold.
- **Memory growth** → the cycle calls `gc.collect()` + `torch.cuda.empty_cache()` every 10 ticks. The mask erode in `update_map` *must* stay in `semantic_map_new.py` (see the FIXME comment) — moving it leaked memory historically.
- **Target object never gets confirmed** → check `best_image_score > 500` triggers, that `/object_type_query` has a subscriber, and that `/object_type_answer` is being published back with the same `object_id`.
