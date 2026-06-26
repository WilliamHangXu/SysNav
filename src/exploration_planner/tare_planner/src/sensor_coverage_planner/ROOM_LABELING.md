# Room Labeling Pipeline

> **What this is.** A handoff/onboarding map for **how rooms get a *type* label**
> (e.g. "kitchen", "office room") in SysNav. It is the path
> *camera frame → view-image admission → VLM query → answer applied to the room
> node*. Start here if your task touches room **types**, view-image selection, the
> `/room_type_query`↔`/room_type_answer` round-trip, or why a room ends up
> unlabeled / mislabeled.
>
> **Not the same as room *segmentation*.** The geometric mask (which pixels are
> which room id) is produced by a separate, purely geometric node — see
> [`room_segmentation/README.md`](../room_segmentation/README.md). Segmentation has
> **no** knowledge of labels and **never** reads label state; labeling is a pure
> downstream consumer of the mask. Don't conflate the two.
>
> Parent guide: [`ARCHITECTURE.md`](../../../../../ARCHITECTURE.md).
> Last verified against code: **2026-06-26**.

All `file:line` anchors below are in
`sensor_coverage_planner/sensor_coverage_planner_ground.cpp` unless prefixed with
`vlm_reasoning_node.py` (in `src/vlm_node/vlm_node/`). Line numbers drift — search
the named function/symbol if they don't match.

---

## TL;DR — the four stages

```
camera frame ─►[1] UpdateRoomViews()      ─► best_views_ (≤3 jpgs/room) + views_dirty_
                   (planner C++, :4432)                       │
objects (planner) ────────────────────────────────────────────┤
                                                               ▼
              [2] PublishRoomTypeQueries() ─/room_type_query─►[3] vlm_reasoning_node.py
                   (planner loop, :4030)                          (Gemini, IMAGE-ONLY today)
                                                                     │
              [4] RoomTypeCallback()  ◄────────/room_type_answer─────┘
                   apply by room_id (:1636, "Fix #1")
```

A room is **typed** only if a camera frame clears the Stage-1 gates, that triggers
a Stage-2 query, the VLM answers, and Stage-4 applies it without a later strip.

---

## ⚠️ Current status / gotchas (read before editing)

| Item | State | Where |
|---|---|---|
| **Fix #1 — apply answer by `room_id`** | ✅ **DONE** (committed `7158fd9`) | `RoomTypeCallback` `:1643-1684` |
| **Fix #2 — decouple `is_labeled_` from navigation** | ❌ **deferred / not applied** | nav writes at `:4912`, `:4962` |
| **Fix #3 — neuter the mask-flicker STRIP** | ❌ **deferred / not applied** | `RoomNodeListCallback` `:1500-1524` |
| **Objects in the VLM prompt** | ⚠️ **commented out** — labeling is image-only | `vlm_reasoning_node.py:446-447` |
| **VLM prompt** | open-vocabulary `ROOM_TYPE_PROMPT_FREE` (not the closed candidate set) | `vlm_reasoning_node.py:443` |
| **Diagnostic logging** | ON (`#define ROOM_DBG_ENABLED 1`) — `[room_dbg]` / `[room_views]` / `LAG` / `PROJ` | top of the `.cpp` |

**The biggest "looks half-finished" thing:** the planner builds and ships the
per-room object inventory in the query, but the VLM node ignores it (the line that
adds objects to the prompt is commented out). The "objects-first, objects are the
primary signal" redesign is wired end-to-end on the planner side but **disabled at
the last step**, so room typing currently runs on the ≤3 images alone.

---

## Stage 1 — View-image collection (`UpdateRoomViews`, `:4432`)

Camera frames arrive on `CameraImageCallback` (`:1089`; subscription `:926`, topic
default `/camera/image`, `sensor_msgs/Image`, QoS depth 5). The callback only
stores `camera_image_` + `imageTime`, then calls `UpdateRoomViews()` synchronously.
**All gating is inside `UpdateRoomViews`**, run per frame, in this order:

| # | Gate | Param (default) | Reject when | Line |
|---|---|---|---|---|
| 1 | Camera pose lookup | — | uses `GetPoseAtTime(imageTime)` (pose at *capture* time, interpolated from the odom ring buffer) | `:4455` |
| 2 | Yaw-rate / blur | `room_view.max_yaw_rate_deg_s` (**30**) | turning faster than threshold | `:4471` |
| 3 | Motion intake | `room_view.motion_dist_m` (**0.2 m**) **&** `room_view.motion_yaw_deg` (**5°**) | moved < dist **and** turned < yaw since last evaluated frame | `:4486` |
| 4 | Projection + range | `room_view.max_range_m` (**8 m**) | project **instantaneous `registered_cloud_`** via `PointToCameraView` (`:4361`, pinhole + plumb-bob, 1280×720); drop points beyond range / out of FOV / behind camera | `:4509` |
| 5 | Room attribution | — | bin in-FOV points to `room_mask_` cells (deduped per room); frame → room with **most observed cells** (argmax); reject if none | `:4539` |
| 6 | **Coverage** | `room_view.min_coverage_m2` (**1.0 m²**) | `cells × room_resolution² < threshold` (wall-facing / pass-through slivers) | `:4555` |
| 7 | Best-3 / pose-diversity | `room_view.pose_dist_m` (**1.5 m**), `room_view.pose_yaw_deg` (**40°**) | within dist+yaw of an existing view and not higher coverage; or all 3 slots full+distinct and doesn't beat the weakest | `:4562` |
| 8 | Save | — | writes `room_views/room_<id>_slot_<k>.jpg`, sets `views_dirty_ = true` | `:4614`, `:4633` |

Coverage uses **actual LiDAR returns**, so a wall yields no returns past it →
cheap occlusion for free (don't switch to geometric cells; that regressed before).
Use the **instantaneous** `registered_cloud_`, never the accumulated stack.
All rejections return silently; an admitted view logs `[room_views]` (`:4655`).

> **Known weak spot:** the camera pose is from `imageTime` but the cloud is the
> *latest* sweep — if the image topic lags, a fresh cloud is paired with a stale
> pose and the coverage count (gate 6) is under-measured, starving small rooms of
> views. Diagnostic `LAG`/`PROJ` logs were added to quantify this (image transport
> of the raw 2.8 MB frames was the lag source; compute was ruled out).

---

## Stage 2 — Query emission (`PublishRoomTypeQueries`, called `:4030`)

Once per planning loop, per room:

- **Trigger** (`:4698`): `views_dirty_` **OR** object count changed
  (`object_count != objects_at_last_query_`), and ≥1 image or ≥1 object to send.
- **Rate limit** (`:4706`): skip if `now − last_query_time_ < room_type_query.min_interval_s` (**3.0 s**, `:391`).
- **Payload** (`tare_planner/RoomType`, built `:4722`): `room_id`, `anchor_point`
  (= centroid), ≤3 `image_paths`, `objects` string, `room_mask`,
  `room_type = GetRoomLabel()` (the room's **current** label, for stability),
  `in_room`, `voxel_num`. Published on `/room_type_query`.
- **`objects` string** (`:4669`): room's object indices, confidence-filtered
  (`room_view.object_conf_min` = **0.3**), deduped by label → `"chair x3, desk x1"`.

---

## Stage 3 — VLM inference (`vlm_reasoning_node.py`)

- Subscribes `/room_type_query` (`:202`), queues, processes in
  `process_room_type_query` (`:401`).
- Reads ≤3 images from disk + the room-mask image, base64-encodes (`:416`).
- **Prompt = `ROOM_TYPE_PROMPT_FREE`** (`:443`, defined `:98-108`): **open
  vocabulary**, single word / short phrase, "no unknown/area/room".
- **Label-stability instruction** (`:452`): if a current label exists, "return the
  exact same label unless clearly a different room; no synonyms."
- **Objects: NOT used** — the line appending `msg.objects` to the prompt is
  commented out (`:446-447`). The string is received and ignored.
- **Provider/model:** Gemini `gemini-2.5-flash` via the OpenAI-compatible client
  (provider selectable via `VLM_PROVIDER`; see `vlm_node` `constants.py`),
  structured `Result` output.
- **Answer** (`:492`): re-publishes the **entire incoming message** on
  `/room_type_answer` with only `room_type` overwritten (lowercased). So
  **`room_id` is echoed back verbatim** — Stage 4 depends on this.

---

## Stage 4 — Apply the answer (`RoomTypeCallback`, `:1636`) — *Fix #1*

- Target room = `room_id = room_type_msg->room_id` (`:1648`) — the stable handle
  the query was issued for (monotonic, never reused). Only if that node is gone
  does it fall back to re-sampling `room_mask_` at the anchor (`:1654-1678`).
- **Latest-answer-wins** (`:1691`): `labels.clear(); labels[type] = 1; SetIsLabeled(true)` (`:1694`).
- **Why Fix #1 exists:** the legacy path re-derived the room id by sampling
  `room_mask_` at the anchor *every time*; that raster flickers per
  re-segmentation cycle (a valid cell reads 0 some cycles), which dropped correct
  answers. Applying by `room_id` is robust to that flicker.

---

## What determines whether a room gets labeled

A room earns a **real VLM type** only if it clears this whole chain. Things that
break it — including code *outside* the labeling pipeline:

- **A. Stage-1 gates.** Imaged only while turning fast (gate 2), only from
  near-duplicate poses (gate 3), beyond 8 m (gate 4), or with `<1.0 m²` of floor in
  frame (gate 6) → no view accrues → never queried. Small pass-through rooms and
  **image-topic lag** both bite at the coverage gate.
- **B. Stage-2 trigger/throttle.** No `views_dirty_` change and no object-count
  change → no query; the 3 s rate-limit coalesces bursts.
- **C. Objects ignored (Stage 3).** A room with strong object evidence but weak
  imagery can't lean on objects — typing is purely what the ≤3 frames show.
- **D. Mask-flicker STRIP — `RoomNodeListCallback` `:1500-1524` (Fix #3 not
  applied).** For every *labeled* room it re-samples `room_mask_` at the stored
  anchor; on out-of-bounds or id mismatch it calls `ClearRoomLabels()`, wiping a
  correct label whenever the segmentation raster flickers. Can un-label a room you
  already typed.
- **E. `is_labeled_` conflation (Fix #2 not applied).** `is_labeled_` is
  overloaded. Navigation bookkeeping in `UpdateRoomLabel` sets `SetIsLabeled(true)`
  at `:4912` (first time a room accrues points) and `:4962` (room grew), **before
  any VLM answer**. Consequences:
  - a room can be `is_labeled_ == true` with **empty `labels_`** → exports as
    `unknown` yet counts as labeled;
  - that same flag gates navigation: `CheckDoorCloudInRange:2464` skips
    `IsLabeled()` rooms, so the planner stops routing the robot to a room's door —
    potentially *before* it was imaged enough to label, starving it of views.
- **F. Room lifecycle.** Rooms are erased on re-segmentation (`DEATH` in
  `RoomNodeListCallback`); their `best_views_` go to an orphan pool and are
  re-homed by anchor→mask, but background-mapped ones are dropped. A label dies
  with its node unless re-derived.

> **Two meanings of "labeled."** `is_labeled_` (a bool, co-written by nav + VLM) vs
> a populated `labels_` (the actual type string from the VLM). The exporter gates
> on `is_labeled_` and reads `GetRoomLabel()` — which is exactly why an
> `is_labeled_`-true / `labels_`-empty room shows up as `unknown`. The scope
> intent (see memory `scene-graph-only-scope`) is that **only `RoomTypeCallback`
> should ever write label state**; navigation should get its own separate flag.

---

## Pending work (the next session's worklist)

1. **Re-enable objects in the VLM** — uncomment `vlm_reasoning_node.py:446-447`.
   The planner already sends a deduped, confidence-filtered inventory. Lowest-risk,
   highest-impact; restores the intended objects-first signal.
2. **Fix #2 — decouple `is_labeled_`.** Give navigation its own flag (e.g.
   `is_anchored_`/`is_asked_`); stop `UpdateRoomLabel` (`:4912`, `:4962`) from
   writing `SetIsLabeled`. Label state owned solely by `RoomTypeCallback`. Update
   the exporter and `CheckDoorCloudInRange:2464` to read the right flag.
3. **Fix #3 — neuter the mask-flicker STRIP** (`:1500-1524`). Stop
   `ClearRoomLabels()` from firing on transient single-cycle mask flicker (e.g.
   require N consecutive mismatched cycles, or re-resolve via a small neighborhood
   instead of the exact anchor cell).
4. *(optional)* a recovery pass for imaged-but-unlabeled rooms.

---

## Diagnostics & artifacts

- **Toggle:** `#define ROOM_DBG_ENABLED 1` near the top of the `.cpp` (search
  `ROOM_DBG_ENABLED`); set to `0` to silence all `[room_dbg]` lines in one place.
- **Tags in `pipeline.log`:**
  - `[room_views]` — a view was admitted (Stage 1 success).
  - `[room_dbg] CREATE / DEATH / VIEW_ORPHAN_DROP / VIEW_REHOME` — room lifecycle.
  - `[room_dbg] VALIDATE / STRIP` — the anchor→mask check and label strips (gate D).
  - `[room_dbg] ANSWER_RECV / ANSWER_APPLY / ANSWER_APPLY_FALLBACK / ANSWER_DROP` —
    Stage 4 apply path.
  - `[room_dbg] MASK_UPDATE` — `/room_mask` cadence/extent.
  - `LAG` / `PROJ` — image-vs-cloud lag and projection timing (the lag
    investigation; transport-bound on the raw image stream).
- **Outputs:** `output/scene_graph/run_<ts>/room_views/room_<id>_slot_<k>.jpg`
  (the admitted view images) + the exported scene-graph JSON.
  `runlogs/<ts>/pipeline.log` is the combined planner+VLM log. Filenames use the
  **runtime `room.id_`** (= exported `sgid`), not `show_id`.

---

## Key files

| File | Role |
|---|---|
| `sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | Owns Stages 1, 2, 4 + the STRIP and `is_labeled_` writes. The hub. |
| `vlm_node/vlm_node/vlm_reasoning_node.py` | Stage 3: prompt, VLM call, `/room_type_answer`. |
| `vlm_node/vlm_node/constants.py` | VLM provider / model / base-URL selection. |
| `tare_planner/msg/RoomType.msg` | Query/answer message (image_paths, objects, room_id, anchor, room_type, room_mask, …). |
| `representation/representation.h` | `RoomNodeRep`: `labels_`, `is_labeled_`, `best_views_`, `views_dirty_`, `objects_at_last_query_`. |
| `room_segmentation/` | Produces the mask this pipeline *consumes* (separate, geometric, label-blind). |
