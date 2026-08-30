# Room Labeling Pipeline

How a room gets its semantic **type** (e.g. `office room`, `corridor`) in SysNav:
the planner bins the covered LiDAR cloud into rooms, crops the panorama frame
around each room's covered points, asks the VLM node with a closed candidate
list, and accumulates the answers as a coverage-weighted vote on the room node.
Room *segmentation* (which cells belong to which room id) is a separate, purely
geometric node — see [`room_segmentation/README.md`](../room_segmentation/README.md);
labeling only consumes its mask. Parent guide: [`ARCHITECTURE.md`](../../../../../ARCHITECTURE.md).

> **Verified against `rsb_test` @ e9504a4 (2026-08-29).** This is sysnav's (`rsb`)
> original room labeling, restored in Phase 2 of [`RSB_TEST_PLAN.md`](../../../../../RSB_TEST_PLAN.md)
> after deepclean's best-3 view-buffer pipeline was dropped (commit `8bd01df`).
> Navigation-only hooks from `rsb` (early-stop / change-room queries) were not ported.

`cpp:` = `sensor_coverage_planner_ground.cpp` (this dir), `h:` = its header in
`include/sensor_coverage_planner/`, `rep.h:` = `include/representation/representation.h`,
`py:` = `src/vlm_node/vlm_node/vlm_reasoning_node.py`. Line numbers drift; grep the symbol.

## Data flow

```
camera frame ──► CameraImageCallback ──► camera_image_, imageTime
odom ring buffer ─► GetPoseAtTime(imageTime)        │
covered cloud (planning_env_->GetUpdatedCloudInRange) ──┐
room_mask_ (from room_segmentation) ────────────────────┤
                                                        ▼
        execute() ──► UpdateRoomLabel()  ── bins cloud per room, triggers,
                          │                 project_pcl_to_image() crop
                          ▼
                 /room_type_query  (tare_planner/RoomType)
                          ▼
              vlm_reasoning_node.py  (closed candidate list, 2 images)
                          ▼
                 /room_type_answer (same msg, room_type filled)
                          ▼
                 RoomTypeCallback() ── room_mask_[anchor] ──► RoomNodeRep.labels_[type] += voxel_num
                          ▼
     PublishRoomTypeVisualization() ──► /room_type_vis  "id label" at centroid
```

## Stage by stage

### (a) Image intake — `CameraImageCallback` (cpp:466-480, h:208)
Subscribed with depth 5 on `sub_camera_image_topic_` (cpp:390-392). Converts to
`bgr8`, stores `camera_image_` and `imageTime` (header stamp, seconds). Nothing else
happens here; no per-frame gating. `camera_image_` is initialised as a 640x1920
zero image (cpp:326) — the crop stage assumes sysnav's **1920x640 360° panorama**.

### (b) Trigger + query — `UpdateRoomLabel` (cpp:1813-1997, h:233)
Called once per planning loop in `execute()`, immediately before
`SetCurrentRoomId()` (cpp:1632-1633). Returns early if `room_mask_` is empty.
1. **Coverage binning** (cpp:1823-1848): every point of
   `planning_env_->GetUpdatedCloudInRange()` is voxelised with `shift_` /
   `room_resolution_` and looked up in `room_mask_`; per room it accumulates
   `room_counts` (voxels), the covered sub-cloud and the mean point.
2. **First query** (cpp:1855-1919): a room with `room_counts > 0` and
   `!IsLabeled()` → `SetVoxelNum`, **`SetIsLabeled(true)`** (cpp:1870, i.e. at
   query time, before any answer), crop image, publish query.
3. **Re-query** (cpp:1929-1994) for rooms already labeled:
   `room_counts − GetVoxelNum() > 20` **or** `area_ − last_area_ > 5.0` (m²).
   A voxel-growth trigger (`flag1`) makes a **new crop + new anchor**; an
   area-only trigger reuses the stored `image_` / `anchor_point_` (cpp:1958-1977).
4. Per query the node stores: `anchor_point_` = mean covered point at
   `robot_position_.z` (cpp:1897-1901), `image_` (cpp:1895), `voxel_num_`
   (cpp:1869), `last_area_ = area_` (cpp:1902). The covered sub-cloud (+ center
   with intensity 10) is published on `/room_cloud` for RViz (cpp:396-397, 1886).
   **There is no rate limit / minimum interval**; a room can be re-queried every loop
   while the thresholds keep tripping.

### (c) The crop — `GetPoseAtTime` + `project_pcl_to_image` (cpp:1764, 2046-2240)
`GetPoseAtTime(imageTime)` interpolates the lidar pose from the odom ring buffer
at the frame's capture time. `project_pcl_to_image` uses **hardcoded** camera
extrinsics (`camX=-0.12, camY=-0.075, camZ=0.265, roll=-π/2, yaw=-π/2`,
cpp:2053-2054) and `imageWidth=1920, imageHeight=640` (cpp:2055-2056), projects
the room's covered points onto the panorama, rotates the panorama so the room
center is not split at the seam, and crops horizontally to the projected span
**±50 px** (cpp:2220-2221, 2232). **Size guard** (cpp:2057-2066): if the frame is
not 1920x640 it logs `RCLCPP_WARN_ONCE` and returns the **whole frame uncropped**.
On a pinhole front camera (e.g. `go2w_026`, `camera/image_rect_color`) the VLM
therefore sees the full current frame regardless of which room is being asked.

### (d) Message — `tare_planner/msg/RoomType.msg`
```
std_msgs/Header header        # frame_id = world, stamp = now (first query only sets these)
geometry_msgs/Point anchor_point   # mean covered point, z = robot z; the re-ID key
sensor_msgs/Image image        # the crop (bgr8)
sensor_msgs/Image room_mask    # node's bbox-cropped mono8 mask (rep.h:186-210, margin 10)
string room_type               # "" in the query; the answer in the reply
int32 room_id                  # planner room id (log-only on the apply side)
int32 voxel_num                # room_counts at query time = the vote weight
bool in_room                   # room_id == current_room_id_
```
Published on `/room_type_query` (depth 10, cpp:416-417); answers subscribed on
`/room_type_answer` (depth 10, cpp:398-400).

### (e) VLM — `vlm_reasoning_node.py`
- `/room_type_query` → `room_type_callback` appends to a deque (py:115-118, 159).
- A 0.1 s timer runs `vlm_node_callback` (py:147, 314-328): drains the deque
  keeping only the **latest query per `room_id`** (by header stamp), then starts
  one `threading.Thread` per query on `process_room_type_query` (py:165).
- Decodes `msg.image` (bgr8) and `msg.room_mask` (mono8), JPEG+base64 both
  (py:182-187). Prompt = `ROOM_TYPE_PROMPT` (py:73-81, "select one of the options")
  + the closed candidate list `self.room_types` (py:72):
  `Classroom, Laboratory, Office Room, Meeting Room, Computer Lab, Restroom,
  Storage Room, Copy Room, Student Lounge, Reception, Corridor`.
  **`Corridor` is removed when `msg.in_room` is false** (py:194-197).
  `ROOM_TYPE_PROMPT_FREE` (py:83) is defined but unused.
- Call: `beta.chat.completions.parse(model=MODEL_NAME, ...)` with the prompt as
  system message and the two images as user content, structured `Result{room_type}`
  (py:201-214); on parse failure the raw text is used (py:220-223).
- Model/provider from `constants.py`: `VLM_PROVIDER` env (`gemini`|`qwen`, auto
  from which key is set); `MODEL_NAME` = `gemini-2.5-flash` or `QWEN_MODEL`
  (default `qwen3.6-plus`). `config/vlm_config.yaml` only sets `log_dir`.
- Reply: the **incoming message is republished as-is** with `room_type`
  overwritten by the lowercased answer (py:229-231) — `anchor_point`, `voxel_num`,
  `room_id` are echoed back verbatim, which stage (f) relies on.
- Debug dump: `debug/room_type/<room_id>_<type>.jpg`, `..._mask.jpg`, `....txt`
  (py:237-244; dir created at py:152, relative to the node's cwd).
- **Any exception drops the query** (py:246-247): no retry, no re-queue. The room
  stays `is_labeled_ = true` with empty `labels_` until a re-query trigger fires.

### (f) Apply — `RoomTypeCallback` (cpp:764-806, h:209)
1. Voxelise `msg.anchor_point` and sample `room_mask_` (cpp:769-781). Out of
   bounds → `RCLCPP_ERROR "Anchor point is out of room mask bounds"`, drop
   (cpp:777). Sampled id not a known room (incl. 0 = unassigned) →
   `"Room id %d is out of bounds"`, drop (cpp:782-787). `msg.room_id` is **not**
   used for resolution — only logged.
2. `labels_[room_type] += msg.voxel_num` (cpp:792), `SetIsLabeled(true)` (cpp:793).
3. `GetRoomLabel()` (rep.h:223-238) = label with the highest accumulated count.
4. If the winning label changed, every object in the room gets
   `SetIsConsidered(false)` (cpp:798-805) so object reasoning re-evaluates.

### (g) STRIP — `RoomNodeListCallback` (cpp:700-728)
After CREATE/UPDATE/DEATH bookkeeping, every **labeled** room's stored
`anchor_point_` is re-sampled in `room_mask_`. Out of bounds (cpp:713-718) or
`room_mask_[anchor] != room_id` (cpp:720-725) → `RCLCPP_ERROR "Anchor point of
room %d is not in the room, removing labels"` + `ClearRoomLabels()`.
`ClearRoomLabels` (rep.h:246-256) wipes `labels_`, `is_labeled_=false`,
`is_asked_=2`, `last_area_=0`, `voxel_num_=0`, `anchor_point_`, `image_` — the room
is then treated as never asked and re-queried from scratch by (b).2 on the next loop.

### (h) `is_labeled_` semantics
**`is_labeled_` means "a query has been sent", not "an answer landed"** (rep.h:307:
`// Whether the room has been asked`). It is set by `UpdateRoomLabel` at query time
(cpp:1870, 1936) and again by `RoomTypeCallback` (cpp:793); it is cleared only by
`ClearRoomLabels`. A room with `is_labeled_ == true` and empty `labels_` is a room
whose query is in flight or was dropped. `GetRoomLabel()` returns `""` in that case;
the RViz marker then shows just the id.

## Parameters

| Parameter | Where | Default / values | Notes |
|---|---|---|---|
| `sub_camera_image_topic_` | cpp:47-48, 143 | `/camera/image` | used only when `robot_namespace` is empty |
| `robot_namespace` | cpp:151 | `""`; `go2w_026` in `config/robot.yaml:14` | non-empty → camera topic = `/<ns>/<topic_suffix.camera_image>` (cpp:166-168) |
| `topic_suffix.camera_image` | cpp:155-156 | `camera/image_raw`; `camera/image_rect_color` in `config/go2w_bag_direct.yaml:15` | |
| `room_resolution_` / `shift_` | planner globals | — | mask voxelisation used by (b), (f), (g) |
| `log_dir` (vlm_node) | `config/vlm_config.yaml` | `logs/episode_0` | not used by the room-type path |

Everything else is hardcoded: +20 voxels / +5 m² re-query thresholds (cpp:1929-1930),
±50 px crop margin, 1920x640 panorama + extrinsics, the candidate list, the model
name. **No rate limit, minimum interval, evidence threshold or object-count trigger exists.**

## Diagnostics

- `#define ROOM_DBG_ENABLED 1` (cpp:33) gates all `[room_dbg]` lines (INFO):
  - `CREATE id= show_id= centroid= area= neighbors=` (cpp:677)
  - `DEATH id= labeled= label= objects=` (cpp:689)
  - `MASK_UPDATE dims= nonzero_cells=` (cpp:753) — `/room_mask` cadence
  - `QUERY_PUB id= first=1|0 voxels= [new_image=] anchor= in_room=` (cpp:1918, 1993)
  - `ANSWER_APPLY msg_room_id= resolved_id= (by anchor) type= label='old'->'new'` (cpp:795)
- STRIP is logged as `RCLCPP_ERROR "Anchor point of room %d is not in the room, removing labels"`
  (cpp:723) / `"... is out of room mask bounds"` (cpp:716); apply-side drops at cpp:777, 784.
- Crop fallback: `RCLCPP_WARN_ONCE "[project_pcl_to_image] camera image is WxH, not the 1920x640 panorama"` (cpp:2062).
- RViz: `/room_type_vis` (cpp:418-419), `PublishRoomTypeVisualization` (cpp:1999-2044):
  `TEXT_VIEW_FACING` marker at `centroid_` (cpp:2023-2025) with text `"<id>"`, plus
  `" <label>"` once `IsLabeled()` and `GetRoomLabel()` is non-empty (cpp:2036-2040).
  `/room_cloud` shows the covered points that produced the last query.
- VLM node: `debug/room_type/` dumps (see (e)); per-query INFO lines
  `Processing room type query for room N`, `Determined room type: ...`, `... processed in X s`.

## Known gotchas

- **Panorama assumption.** The crop only works on a 1920x640 360° image with the
  hardcoded extrinsics; any other camera silently (one WARN) sends the full frame,
  so labels on a front pinhole camera reflect wherever the robot is looking, not the
  queried room. Real validation of the crop is the Unity sim.
- **No rate limit.** A growing room re-queries every loop that adds >20 voxels; the
  VLM node only coalesces what is queued between two 0.1 s ticks.
- **Vote semantics.** `labels_[type] += voxel_num`: an early wrong answer with a
  large `voxel_num` can dominate later correct ones; the label never "resets"
  except via STRIP.
- **STRIP is coarse.** Any re-segmentation that moves the anchor's cell to another
  id (or 0) wipes the vote and image; the room is re-asked from scratch.
- **Anchor re-ID lag.** Apply samples the *current* `room_mask_` at the anchor;
  if the mask changed while the VLM was busy, the vote lands on whatever room now
  owns that cell (`ANSWER_APPLY msg_room_id != resolved_id`).
- **Stale in-code comments.** cpp:2020-2022 says the marker sits on the "interior
  point"; it sits on `centroid_`. cpp:1807-1812 describes a yaw helper that no
  longer exists (the comment sits above `UpdateRoomLabel`).

## Key files

| File | Role |
|---|---|
| `src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | `CameraImageCallback`, `UpdateRoomLabel`, `project_pcl_to_image`, `RoomTypeCallback`, STRIP in `RoomNodeListCallback`, `PublishRoomTypeVisualization` |
| `include/sensor_coverage_planner/sensor_coverage_planner_ground.h` | declarations (h:206-233), `camera_image_` (h:291), `imageTime` (h:306) |
| `include/representation/representation.h` | `RoomNodeRep`: `labels_`, `is_labeled_`, `is_asked_`, `voxel_num_`, `last_area_`, `anchor_point_`, `image_`, `room_mask_` (rep.h:297-314); `GetRoomLabel`, `ClearRoomLabels` |
| `msg/RoomType.msg` | query/answer message (stage d) |
| `src/vlm_node/vlm_node/vlm_reasoning_node.py` | stage (e) |
| `src/vlm_node/vlm_node/constants.py` | provider / model / base-URL selection |
| `src/vlm_node/config/vlm_config.yaml` | `log_dir` only |
