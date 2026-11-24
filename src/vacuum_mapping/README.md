# vacuum_mapping

Spec 003 (mapping). This first slice holds only the **pure-Python cores and file I/O**: no ROS nodes, no
launch, no simulator, no SLAM. The nodes (`odom_tf_bridge`, `room_segmenter`, `exploration_server`) and
`mapping.launch.py` come with their own tasks and will wrap these cores (D-18 to D-20).

Dependencies: `numpy`, `pyyaml` (both `exec_depend` in `package.xml`). No scipy or OpenCV (Q-16 is
unresolved); the segmenter uses only numpy and plain breadth-first searches.

## Modules

| Module | Role | Task / requirement |
|---|---|---|
| `core/types.py` | `Raster` (cells + resolution + origin), cell codes, `Room`, `LabelSource`, logger type | shared |
| `core/map_diff.py` | `MapDiff.compare(committed, new)` -> `MapDiffReport`; `format_report()` -> Markdown | 003-R34 |
| `core/room_segmenter.py` | `RoomSegmenter.segment(raster)` -> raw label raster (erosion, seeds, nearest-seed growth) | 003-R16, R17 |
| `core/room_tracker.py` | `RoomTracker.track(previous, raw_labels, ...)` -> `TrackedMap` with stable indexes | 003-R19, R20 |
| `core/room_labeler.py` | `RoomLabeler.label_rooms(...)` weighted vote of observations -> rooms | 003-R21 |
| `map_io.py` | nav2 `map.yaml` + `.pgm` <-> `Raster` | 003-R33, R34 |
| `rooms_io.py` | `<map>_rooms.yaml` and `room_rules.yaml` <-> dataclasses | 003-R21, R23, R30 |
| `map_diff_cli.py` | `ros2 run vacuum_mapping map_diff <committed.yaml> <new.yaml> [-o report.md]` | 003-R34 |

Rules followed: the `core/` modules never read files, topics or clocks, hold no global state, and log
through an injected `log(str)` callable (default: drop). Every result is an explicit frozen dataclass.

## Data flow

```
map.yaml/.pgm --map_io.read_map--> Raster --RoomSegmenter--> raw labels (0 = no room, 1..N scan order)
                                                                 |
   previous TrackedMap (or None) ------------------------------> RoomTracker --> TrackedMap(labels = Room.index,
                                                                                        rooms, next_index)
   observations (class, x, y) + rules --------------------------> RoomLabeler --> rooms with rule labels
   TrackedMap.rooms/next_index <--> rooms_io.read_rooms / write_rooms   (user labels survive a restart)

committed Raster + new Raster --MapDiff--> MapDiffReport --format_report--> report text
```

Notes for the future `room_segmenter` node:

- `RoomSegmenter` returns 0 for occupied, unknown and unreachable cells. The published `room_labels`
  grid needs -1 on unknown cells (003-R18): the node applies that with `raster.cells == UNKNOWN`.
- After a restart the previous label raster is not stored. Segmentation is deterministic, so segment the
  loaded map again, build a `TrackedMap` from it with the rooms read from `_rooms.yaml` (matching by
  index), and pass that as `previous`.
- `RoomLabeler` never changes rooms with `label_source == USER`, so the node can call it on every batch
  of detections.

## Run the tests

In Docker, both distros (Art. X), from the repository root:

```bash
scripts/test-docker.sh            # jazzy and lyrical
scripts/test-docker.sh lyrical
```

A single file on any machine with numpy, pyyaml and pytest:

```bash
cd src/vacuum_mapping && python3 -m pytest -q test/test_room_segmenter.py -k R17
```

Test names carry the requirement id (`test_003_R17_...`). Test grids are tiny synthetic arrays built in
`test/grid_fixtures.py`.

## Add a segmentation method

1. Write a class with `segment(self, raster: Raster) -> np.ndarray` in a new `core/<name>.py`
   (the `SegmentationMethod` protocol in `core/room_segmenter.py`): int32 labels, same shape as the
   raster, 0 for no room, rooms numbered 1..N consecutively in scan order, deterministic. Take its
   parameters in the constructor and a `log` callable; no I/O.
2. Copy the requirement-named tests of `test/test_room_segmenter.py` (two rooms, door width, open plan,
   no label on occupied or unknown, determinism, timing on 300 x 300) and make them pass.
3. `RoomTracker` and `RoomLabeler` need no change: they only see the label raster.

## Known limits and open questions

- `map_io.read_map` follows the thresholds of the file, except that pixel 205 (the `map_saver` unknown grey)
  is always unknown in non-negated maps; `write_map` writes the standard 0.65 / 0.25 thresholds.
- Requirement gap for the room tracker node: `RoomTracker` can only match rooms when the previous and the new label
  rasters have the same shape. When the map grows (new shape) it logs a WARNING and every room gets a new
  index, losing labels. Architect to rule whether to match by world-coordinate centroid; not implemented.
- `MapDiff` distance is O(N * M) in occupied cells (see its docstring).

### Open questions (architect)

- `RoomLabeler`: the tie rule (keep current label), `min_votes` counting observations of the winning label,
  and vote inflation when the same object is detected repeatedly across frames (no de-duplication).
- `MapDiff`: which resolution to compare at when the two resolutions differ (now: the finer one).

## Not here yet

`FrontierFinder` and `GridView` (they belong in the shared `vacuum_grid` package), all nodes,
`config/room_rules.yaml`, `config/mapping.yaml`, `maps/`. The interfaces (`Room`, `RoomArray`, ...) are
mirrored by the plain dataclasses in `core/types.py` until the contract change is applied.
