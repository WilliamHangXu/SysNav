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
> Last verified against code: **2026-06-27**.

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
| **Fix #3 — mask-flicker STRIP** | ✅ **REMOVED** — label lifecycle now follows room lifecycle | deleted from `RoomNodeListCallback` |
| **Interior point (pole of inaccessibility)** | ✅ **NEW** — single stable point for marker / `wp_0` / re-ID | `room_segmentation` PIA → `RoomNode.interior_point` |
| **Objects in the VLM prompt** | ⚠️ **commented out** — labeling is image-only | `vlm_reasoning_node.py:446-447` |
| **VLM prompt** | open-vocabulary `ROOM_TYPE_PROMPT_FREE` (not the closed candidate set) | `vlm_reasoning_node.py:443` |
| **VLM 429 / no retry** | ⚠️ drops query on any exception, no backoff/re-queue | `vlm_reasoning_node.py:508` |
| **Diagnostic logging** | ON (`#define ROOM_DBG_ENABLED 1`) — `[room_dbg]` / `[room_views]` / `[room_pia]` / `LAG` / `PROJ` | top of `.cpp` + `room_segmentation` |

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
  (= centroid, **unchanged**, nav-only), `interior_point` (= the room's
  pole-of-inaccessibility; the snapshot Stage 4 re-IDs against), ≤3 `image_paths`,
  `objects` string, `room_mask`,
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
  does it fall back to re-sampling `room_mask_` at the **`interior_point`
  snapshot** (`:1654-1678`) — a deep-interior cell, so the sample is robust to
  mask growth/churn (the old anchor sample mis-resolved when the anchor drifted).
- **Latest-answer-wins** (`:1691`): `labels.clear(); labels[type] = 1; SetIsLabeled(true)` (`:1694`).
- **Why Fix #1 exists:** the legacy path re-derived the room id by sampling
  `room_mask_` at the anchor *every time*; that raster flickers per
  re-segmentation cycle (a valid cell reads 0 some cycles), which dropped correct
  answers. Applying by `room_id` is robust to that flicker.

---

## Room reference points — centroid vs interior point vs anchor

Three distinct per-room points exist; keep them straight (all on `RoomNodeRep`):

| Point | Computed | Meaning | Consumers |
|---|---|---|---|
| `centroid_` | segmentation, area-mean of mask cells | geometric center of footprint; can fall **outside** a non-convex room | reserved for geometry (future room-area subdivision); still shipped as `RoomNode.centroid` |
| `interior_point_` | segmentation, **pole of inaccessibility** | a guaranteed-**inside**, deepest-clearance cell — the stable "where is this room" handle | RViz label **marker**, exporter **`wp_0`**, async **re-ID** snapshot (`RoomType.interior_point`) |
| `anchor_point_` | **legacy, no writer remains** | was the drifting nav anchor written by the (deleted) `UpdateRoomLabel` | none — answers re-resolve rooms via `interior_point_` |

**Interior point (pole of inaccessibility).** Computed in `room_segmentation`'s
per-room centroid loop: bbox-crop the room's binary mask (+2 px margin so the
surrounding non-id cells form the zero boundary), `cv::distanceTransform`
(`DIST_L2`), then take the **medoid of the max-clearance ridge** — *not* a bare
`minMaxLoc` argmax. The argmax is degenerate: for a rectangle the maximum clearance
is the whole **medial axis** (a line segment), so argmax lands at one **end**
(~1.5 m off-center for a 1.5×4.5 m room, verified) and hops between ends as the
mask shifts cycle-to-cycle. The medoid (ridge centroid snapped to the nearest real
max-clearance cell) is centered for a rectangle, stable across cycles, and still a
genuine deepest cell → **always inside** (a plain region centroid can fall in a gap
for a U-shape). Cost ~tens of µs/room; timed under `[room_pia]`.

**Why this replaced the old anchor-based design.** The marker used to sit on
`anchor_point_` (the drifting mean) → it visibly yo-yo'd toward the mask's growing
frontier and snapped back. Re-ID used to sample the mask at the anchor → it
mis-resolved whenever the anchor drifted onto a foreign/background cell. Both now
key off the stable interior point; `anchor_point_` is left untouched as nav's own
target. This — together with removing the STRIP (gate D) — is what fixed the
marker yo-yo, the disappear/relabel flicker, and the duplicate view-image jpgs.

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
- **D. ~~Mask-flicker STRIP~~ — REMOVED (Fix #3 done).** This used to re-sample
  `room_mask_` at the stored anchor for every labeled room each cycle and
  `ClearRoomLabels()` on a mismatch, wiping correct labels (and their on-disk
  views) on transient raster flicker — the source of the disappear/relabel and
  duplicate-jpg symptoms. It is gone; **label lifecycle now follows room
  lifecycle** (genuine split/merge/death is still handled by the `DEATH`/re-home
  path below). No per-cycle label stripping remains.
- **E. ~~`is_labeled_` conflation~~ — FIXED (Fix #2 done).** `UpdateRoomLabel`
  (and all navigation bookkeeping) was deleted in the scene-graph reduction;
  **`RoomTypeCallback` is now the only writer of `is_labeled_`** — the flag means
  exactly "a real VLM answer landed." The RViz room marker no longer waits for
  it either: the room id shows as soon as the room exists, and the label text is
  appended when the answer arrives.
- **F. Room lifecycle.** Rooms are erased on re-segmentation (`DEATH` in
  `RoomNodeListCallback`); their `best_views_` go to an orphan pool and are
  re-homed by anchor→mask, but background-mapped ones are dropped. A label dies
  with its node unless re-derived.

> **One meaning of "labeled" (since the reduction).** `is_labeled_` is written
> only by `RoomTypeCallback` when a VLM answer lands, so it always coincides with
> a populated `labels_`. The historical `unknown`-export failure mode (nav code
> setting the flag before any answer) is gone with the nav code.

---

## Pending work (the next session's worklist)

1. ~~**Re-enable objects in the VLM**~~ — **done**: the VLM prompt consumes the
   deduped, confidence-filtered object inventory (the objects-first signal).
2. ~~**Fix #2 — decouple `is_labeled_`.**~~ — **done** via the scene-graph
   reduction: `UpdateRoomLabel` and all nav writers were deleted; label state is
   owned solely by `RoomTypeCallback`.
3. **VLM 429 / no retry** (`vlm_reasoning_node.py`, `process_room_type_query`). Any exception drops the
   query — no retry, backoff, or re-queue. Gemini free-tier (20 req/day/model)
   exhaustion (HTTP 429) silently left many rooms unlabeled in
   `runlogs/20260626_193759` (12×429, 2 success). Add 429-aware retry/backoff or
   re-queue so a transient quota/network blip doesn't permanently un-label a room.
4. **Stage-1 image-topic lag.** Raw ~2.8 MB frames lag the cloud, so a fresh cloud
   is paired with a stale pose and coverage (gate 6) is under-measured, starving
   small rooms of views (the `LAG`/`PROJ` finding — transport-bound, not compute).
5. *(optional)* recovery pass for imaged-but-unlabeled rooms; *(optional)* reconcile
   on-disk slot jpgs with live `best_views_` (was masked by the now-removed STRIP,
   so a much smaller concern).

---

## Diagnostics & artifacts

- **Toggle:** `#define ROOM_DBG_ENABLED 1` near the top of the `.cpp` (search
  `ROOM_DBG_ENABLED`); set to `0` to silence all `[room_dbg]` lines in one place.
- **Tags in `pipeline.log`:**
  - `[room_views]` — a view was admitted (Stage 1 success).
  - `[room_dbg] CREATE / DEATH / VIEW_ORPHAN_DROP / VIEW_REHOME` — room lifecycle.
  - `[room_pia]` — interior-point (pole-of-inaccessibility) compute: an INFO
    per-cycle aggregate (`computed interior points for N room(s) in X ms`) plus a
    per-room DEBUG line (`room <id> interior=(x,y) clearance=<m>m took <us>us`).
    Emitted by `room_segmentation`. (The old `VALIDATE`/`STRIP` tags are gone with
    the STRIP.)
  - `[room_dbg] ANSWER_RECV / ANSWER_APPLY / ANSWER_APPLY_FALLBACK / ANSWER_DROP` —
    Stage 4 apply path; `ANSWER_RECV` now logs the `interior=` re-ID key.
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
| `sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | Owns Stages 1, 2, 4 + the `is_labeled_` writes. The hub. (STRIP removed.) |
| `vlm_node/vlm_node/vlm_reasoning_node.py` | Stage 3: prompt, VLM call, `/room_type_answer`. |
| `vlm_node/vlm_node/constants.py` | VLM provider / model / base-URL selection. |
| `tare_planner/msg/RoomType.msg` | Query/answer message (image_paths, objects, room_id, anchor_point, **interior_point**, room_type, room_mask, …). |
| `tare_planner/msg/RoomNode.msg` | Segmentation→planner room geometry; carries `centroid` + **`interior_point`** (PIA). |
| `representation/representation.h` | `RoomNodeRep`: `labels_`, `is_labeled_`, `best_views_`, `views_dirty_`, `objects_at_last_query_`, `centroid_`, **`interior_point_`**, `anchor_point_`. |
| `room_segmentation/room_segmentation.cpp` | Produces the mask this pipeline *consumes* + computes the **interior point (PIA)**. Label-blind otherwise. |
| `scene_graph_exporter/scene_graph_exporter.cpp` | Exports rooms; `wp_0` = the room's **interior point**. |
