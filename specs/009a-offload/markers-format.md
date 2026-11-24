# 009a: format of `/detections_markers` consumed by the 2025S2 semantic node (T903)

Records the exact format of the `visualization_msgs/MarkerArray` that the 2025S2
semantic segmentation node reads, so the `projector` (T908) can be written
(009a-R13, D-16). Source: third-party IA368II repository, pinned commit
`51a5fa2c816c7ee712c963843e5d747b3ed2a98f` (`deps.repos`), read-only (Art. II).
Verified by cloning that commit outside the working tree and reading it.

All paths below are relative to `final_project/final_project_ws/src/my_robot_pkg/`
in that repository.

## 1. Where it is in the source

| What | File:lines |
|---|---|
| Consumer subscription (`MarkerArray`, topic parameter `detections_topic`, default `/detections_markers`, depth 10) | `my_robot/my_robot_semantic_segmentation_node.py:37`, `:86-88` |
| Consumer callback `_detections_cb` | `my_robot/my_robot_semantic_segmentation_node.py:125-153` |
| Pose to room id lookup `_lookup_room_id` | `my_robot/my_robot_semantic_segmentation_node.py:353-364` |
| Label to room mapping loader (lowercases and strips keys) | `my_robot/my_robot_semantic_segmentation_node.py:99-111` |
| Label to room table | `config/semantic_room_seg_classes.json` |
| Original producer (`my_robot_detection_node`): publisher, depth 10, republish timer 2.0 s | `my_robot/my_robot_detection_node.py:99-100` |
| Original producer: marker construction | `my_robot/my_robot_detection_node.py:627-672` |
| Original producer: pre-filters (confidence, allowed labels, mapped area) | `my_robot/my_robot_detection_node.py:346-358`, `:697` |

## 2. What the consumer reads

Only `_detections_cb` consumes the topic. For each `marker` in `msg.markers`:

```python
if marker.ns != 'detections_text' or not marker.text:   # :131
    continue
label = marker.text.split('(')[0].strip().lower()       # :133
room_name = self.label_to_room.get(label)               # :136 (unknown label: skipped)
room_id = self._lookup_room_id(marker.pose)             # :139
```

| Field | Used? | How |
|---|---|---|
| `ns` | yes | must equal exactly `detections_text`; any other value (including the empty default) is skipped |
| `text` | yes | carries the class label (see section 3); empty text is skipped |
| `pose.position.x`, `pose.position.y` | yes | metres in the frame of `/topologic_map`; converted to a cell with `int((p - origin) / resolution)` (:360-361) |
| `pose.position.z`, `pose.orientation` | no | ignored |
| `header.frame_id` | no | never read. The pose is used as if it were in the `/topologic_map` frame. The original producer writes `map` |
| `header.stamp` | no | never read; there is no age filtering in the consumer |
| `id` | no | never read (the producer uses `det_id + 100000` for text, `det_id` for the cube) |
| `type` | no | not checked (the producer uses `TEXT_VIEW_FACING`) |
| `action` | no | not checked; `DELETE`/`DELETEALL` are not honored, a marker with any action is treated as an addition |
| `scale`, `color`, `lifetime`, `frame_locked`, `points`, `colors`, `mesh_*` | no | ignored |
| Confidence | no | not carried. The original producer filters it before publishing (`confidence_threshold`, default 0.7, :52, :71, :349) |
| Markers with other `ns` (for example the cube in `detections`) | no | skipped; not needed by the consumer |

Consumer behaviour:

- It is stateless per message and per marker: each valid marker sets `latest_assignments[room_id] = room_name`
  (last writer wins per room id, :142-146). Duplicates are harmless.
- It also needs `/topologic_map` first. If no `OccupancyGrid` has been received yet,
  the whole callback returns early (:127-128) and the message is lost (no queue, no retry).
- Subscription QoS: `rclpy` integer depth 10 means RELIABLE, VOLATILE, KEEP_LAST 10. A publisher with the
  default profile (also RELIABLE/VOLATILE) matches.
- Topic and consumer node are not configured by us: the topic name is the parameter default.

## 3. How the label is carried

In `marker.text`, with `ns == "detections_text"`, as `"<label> (<id>)"`, for example `cup (3)`.
The consumer takes everything before the first `(`, strips whitespace and lowercases it. The
text `cup` without a suffix works equally well. The label must be a key of
`config/semantic_room_seg_classes.json` (lowercase, may contain spaces, for example `dining table`,
`cell phone`, `potted plant`) or the marker is silently ignored (:136-138). The label
therefore is the detector class name (COCO names for `yolo11n`/`yolo11x`; the JSON also has two
non-COCO entries, `kitchen drawer` and `hood`, so the original producer used a custom model
`my_robot.pt`; this is a note for the detector model, not for the projector).
The label must not contain `(`.

## 4. Unknown or missing fields

No crash path was found for message content. Every bad case is skipped silently, with no log:

| Case | Result |
|---|---|
| `ns` missing/different, or `text` empty | marker skipped (:131) |
| label empty after split/strip | skipped (:134-135) |
| label not in the JSON | skipped (:136-138) |
| pose outside the grid (negative or beyond width/height) | `_lookup_room_id` returns `None`, skipped (:139-141, :362-363) |
| pose over a wall/unknown or a cell with id <= 0 | skipped (:140) |
| extra or unused fields (`id`, `color`, `lifetime`, ...) | ignored |
| NaN in `pose.position`: `int(nan)` raises `ValueError` in `_lookup_room_id` | this one DOES raise inside the callback (rclpy logs the exception, the node survives, the rest of the message is lost). The projector must never publish NaN or inf (inference from reading the code, not run) |
| `/topologic_map` with `resolution == 0` | division error, not related to our message |

## 5. Frame and timestamp semantics

- Pose frame: the same frame as `/topologic_map` (`map` in practice; the consumer does not
  check `header.frame_id`, so a wrong frame would be accepted silently and produce wrong rooms).
  The projector must therefore transform to `map` itself and set `header.frame_id = "map"`.
- Stamp: unused by the consumer. For traceability the projector should still set it. Original
  producer: `get_clock().now()` at publish time. Recommendation: use the image timestamp
  (the detection `header.stamp`, which TF was queried with, D-16) when the marker is first
  created. Either is accepted.
- Lifetime: original producer leaves `lifetime` at 0 (forever); the consumer ignores it.
  The original producer republishes the **whole persistent set every 2.0 s** (timer, :100) and
  never sends DELETE.

## 6. Minimal valid message

```yaml
markers:
- header: {frame_id: map, stamp: {sec: 1700000000, nanosec: 0}}
  ns: detections_text
  id: 100003
  type: 9            # TEXT_VIEW_FACING
  action: 0          # ADD
  pose: {position: {x: 2.35, y: -1.10, z: 0.25}, orientation: {w: 1.0}}
  scale: {z: 0.2}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 0.9}
  text: "cup (3)"
```

Strictly required by the consumer: `ns`, `text`, `pose.position.x/y`. Everything else is for RViz parity.

## 7. Checklist: what the projector must publish (009a-R13, D-16)

- [ ] Topic `/detections_markers`, type `visualization_msgs/msg/MarkerArray`, QoS default (RELIABLE, VOLATILE, depth 10).
- [ ] One text marker per accepted detection with `ns = "detections_text"` exactly.
- [ ] `text = "<class name> (<id>)"`, class name equal to a key of `semantic_room_seg_classes.json` (lowercase, no `(`).
- [ ] `pose.position.x/y` in metres in the `map` frame (TF at the photo timestamp), finite, inside the mapped area.
- [ ] `header.frame_id = "map"`; stamp set (image stamp recommended; not used by the consumer).
- [ ] `type = TEXT_VIEW_FACING`, `action = ADD`, `orientation.w = 1.0`, `pose.position.z = 0.25`, `scale.z = 0.2`, white color `a = 0.9`, `lifetime = 0`, for parity with the original producer (RViz config `rviz/my_robot.rviz:153` uses the topic).
- [ ] Unique `id` per object (original: `det_id + 100000`); optional companion `ns = "detections"` CUBE marker with `id = det_id` for RViz only.
- [ ] Re-publish the full persistent set periodically (original: every 2 s), because the consumer drops everything received before it has `/topologic_map` and keeps no history. Publishing only once per detection is NOT enough.
- [ ] Apply the original filters before publishing: confidence threshold (default 0.7), label in the JSON, position inside the known map. Dedup by label and proximity (`find_matching_detection`, :534) keeps one marker per object. Where each filter lives (`detection_selector` vs `projector`) is a T908 design point; the consumer itself does none of this except label and room lookup.
- [ ] Contract test (T908): a fixture `MarkerArray` built per section 6 fed to a copy of the `_detections_cb` logic, or a replica of the parsing rule `ns == 'detections_text'` and `text.split('(')[0].strip().lower()`, must yield the expected room assignment. Mark as unverified: the consumer was read, not executed.

## 8. Proposed follow-ups

| # | Follow-up | Owner | Depends on |
|---|---|---|---|
| 1 | Update the `/detections_markers` `note` in `contracts/topics.yaml` (and `interfaces.md` if it lists the topic) with: `ns=detections_text`, `text="<label> (<id>)"`, frame `map`, periodic republish, link to this file | architect | PR #21 merged (contract changes are serialized) |
| 2 | Tick T903 in `specs/009a-offload/tasks.md`; replace the open item in `plan.md` ("exact format ... to be confirmed (T903)") with a link to this file | architect | PR #18 merged |
| 3 | Unblock T908: implement `projector` publishing per section 7, with the parsing contract test | ros-dev | 1, T904/T905 (core already merged per tasks.md), 2 |
| 4 | Decide where the confidence/label/mapped-area filters and the persistence/dedup set live (`projector` vs `detection_selector`) | architect (design), then ros-dev | T908 start |
| 5 | Confirm the detector model class names match the JSON keys (`yolo11n`/`yolo11x` COCO vs custom `my_robot.pt`; `kitchen drawer`, `hood` are not COCO) | ros-dev | T907 |
