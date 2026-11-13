# 003: Technical plan

## Where the code goes

New package `src/vacuum_mapping/` (ament_python), own launch, runnable alone (Art. III.1):

```
src/vacuum_mapping/
├── vacuum_mapping/
│   ├── core/                       # pure Python, no rclpy imports (D-18, D-19)
│   │   ├── frontier_finder.py      # FrontierFinder
│   │   ├── room_segmenter.py       # RoomSegmenter
│   │   ├── room_tracker.py         # RoomTracker (stable ids across segmentations)
│   │   └── room_labeler.py         # RoomLabeler
│   ├── odom_tf_bridge_node.py      # thin LifecycleNode
│   ├── exploration_server_node.py  # thin LifecycleNode (action server + Nav2 client)
│   └── room_segmenter_node.py      # thin LifecycleNode (segmentation, tracking, labels, services)
├── launch/mapping.launch.py
├── config/{slam_toolbox.yaml, room_rules.yaml, mapping.yaml}
├── test/
└── maps/                           # house.yaml, house.pgm, house_rooms.yaml (small, versioned; "maps/" in the requirements means this directory)
```

Grid helpers (`GridView`: world/cell conversion, inflation) are shared with spec 005 through a tiny package `src/vacuum_grid/` (pure Python, no ROS, no dependency on `vacuum_mapping` or `vacuum_coverage`), created in T309; this avoids coupling and a circular dependency between the two modules (Art. III.1). Fallback if rejected: two copies pinned by identical unit tests.

## Data flow

```
CoppeliaSim ─ZeroMQ─▶ 2025S2 bridge ─▶ /scan, /myrobot/odom
/myrobot/odom ─▶ odom_tf_bridge ─▶ TF odom→base_link         (only if 2025S2 does not broadcast it, Q-14)
static TF base_link→laser/camera (launch, parameters from T301)
/scan + TF ─▶ slam_toolbox ─▶ /map, TF map→odom               (or map_server + static map→odom with map_file)
/map ─▶ exploration_server ─(NavigateToPose)─▶ Nav2 ─▶ mux/nav          [Nav2 from spec 005]
exploration_server ─▶ /myRobot/map/explored (latched)
explored=true | service segment ─▶ room_segmenter ─▶ /myRobot/map/room_labels, /myRobot/map/rooms (latched)
/detections_markers ─▶ RoomLabeler (inside room_segmenter) ─▶ labels;  set_label service ─▶ user labels (persisted)
supervisor ─(Explore action)─▶ exploration_server           supervisor ◀─ explored
coverage_server (spec 005) ◀─ room_labels, rooms
```

Only `controller_server` moves the robot during exploration (D-24 of spec 005); `exploration_server` computes and sends goals (a service in the sense of Art. III.4).

## Odometry decision (D-23, decided by the project owner, 2026-11-13)

| Option | Release on Jazzy and Lyrical (`ros/rosdistro` `distribution.yaml`, 2026-11-13) | Cost | Verdict |
|---|---|---|---|
| `sim`: `/myrobot/odom` from the 2025S2 bridge (simulator pose) + `odom_tf_bridge` | not applicable (our node) | small; frames and origin to confirm (Q-14) | **default** |
| `laser`: `rf2o_laser_odometry` | **no entry** on either distribution | source build in `deps.repos`, port to Lyrical unverified | optional alternative, second in the cut order after the automatic labels (T321) |
| `robot_localization` EKF | entry on both | needs a second, noisy source; no gain here | not used |

The 2025S2 code and the House scene are not read in this change; everything about their frames is marked unverified (Q-14, T302). If T302 finds that the 2025S2 node already broadcasts `odom -> base_link`, `odom_tf_bridge` is not launched (003-R02), and if it publishes pose in the world frame instead of a start-relative frame, the bridge subtracts the pose at the first message so that `odom` starts at the origin. Which of these holds is decided by T302 and T301.

Reproducibility note: with ground-truth odometry the SLAM correction `map -> odom` is near identity; the map-quality numbers (003-R08) are optimistic. The report states this as a limitation. A noisy-odometry test is optional, only if time remains.

## SLAM Toolbox

- Online asynchronous, mapping mode, `resolution: 0.05`, explicit frames and `scan_topic` (003-R05); `use_sim_time` per Q-13.
- Released on both distros: 2.8.5 (Jazzy) and 2.10.0 (Lyrical), from D-12 (source: `ros/rosdistro`, 2026-11-05; the entries were re-checked on 2026-11-13).
- Whether a lifecycle variant exists in each release is unverified (Q-15); T311 checks and records. The map is published latched (`/map`, profile `latched`).
- Persistence: `nav2_map_server` `map_saver` through `/myRobot/map/save`; posegraph serialization is not used in the minimal version.

## Toolbox classes (pure Python, D-19)

| Class | Input | Output | Notes |
|---|---|---|---|
| `FrontierFinder` | grid, resolution, origin, robot position, blacklist, parameters | ordered list of `Frontier(centroid, size)` | Only computes. Caller picks, sends and blacklists. |
| `RoomSegmenter` | grid, `door_width_m`, `min_room_area_m2` | integer label raster | Erosion by `door_width_m / 2`, components as seeds, nearest-seed propagation. Uses `scipy.ndimage` (Q-16) or the fallback listed there. |
| `RoomTracker` | previous rooms, new label raster | rooms with persistent ids | Overlap matching at 50 % (003-R19); keeps `label` and `label_source`. |
| `RoomLabeler` | rooms (masks), observations `(class, x, y)`, rule table, `min_votes` | label per room | Never overrides `label_source = user`. |

Pure logic receives a logger and a metrics callback (D-20).

## Nodes

| Node | Type | Role | Publishes | Consumes |
|---|---|---|---|---|
| `odom_tf_bridge` | LifecycleNode, service | `/myrobot/odom` to TF | TF `odom -> base_link`, telemetry | `/myrobot/odom` |
| `slam_toolbox` | third party (Art. II: configuration) | SLAM | `/map`, TF `map -> odom` | `/scan`, TF |
| `exploration_server` | LifecycleNode, service | action `/myRobot/explore`; `NavigateToPose` client; `FrontierFinder` | `/myRobot/map/explored`, `Notification`, telemetry; no `Twist` | `/map`, TF, `/navigate_to_pose` |
| `room_segmenter` | LifecycleNode, service | `RoomSegmenter`, `RoomTracker`, `RoomLabeler`; services `segment`, `set_label`, `save` | `/myRobot/map/room_labels`, `/myRobot/map/rooms`, telemetry | `/map`, `explored`, `observations_topic` |
| `supervisor` | existing | new rows below | existing | `explored`, action `/myRobot/explore` |

## Supervisor: new rows, active only with `explore_on_start:=true` (ros-dev edits `SubmodeMachine`, T315; plan 001 is not edited, the architect merges these rows when accepted)

`exploration_done` is an internal flag, set when an `Explore` goal ends (success or failure); it prevents a loop on failure and is cleared only by a restart or a new map.

| Current state | Event | Action | Next |
|---|---|---|---|
| NONE | mode = CLEANING, `explored` false, `explore_on_start` true (default false), `exploration_done` false, lock off | sends `explore` instead of `cover_room`; this changes 001-R06 and needs the amendment A1 of the spec before the default becomes true | EXPLORING |
| EXPLORING | `explore` succeeded | sends `cover_room`; `exploration_done = true` | COVERING |
| EXPLORING | `explore` failed | sends `cover_room` with the partial map; `exploration_done = true` | COVERING |
| EXPLORING | `StuckEvent` | cancels `explore`; `resume_submode = EXPLORING`; sends `escape` | ESCAPING |
| ESCAPING | `escape` succeeded, `resume_submode = EXPLORING` | sends `explore` again | EXPLORING |
| EXPLORING | mode leaves CLEANING | cancels `explore` (does not touch the lock, 001-R22) | NONE |
| ESCAPING | mode changes while `resume_submode = EXPLORING` | as in 001-R21: after the escape, COVERING for CLEANING, NONE otherwise | ESCAPING |

`escape` failure from EXPLORING follows the 001 row (lock, ESCAPE_FAILED).

## Proposed contract changes (NOT applied; the architect applies them in one serialized change, after PRs #9 and #10)

`interfaces.md` (new types in `vacuum_interfaces`, T303):

```
# msg/Room.msg
uint8 DEFAULT=0
uint8 RULE=1
uint8 USER=2
uint16 index                    # value in room_labels, from 1, stable (003-R19)
string id                       # room_%02d of index
string label                    # default = id
uint8 label_source
geometry_msgs/Point centroid    # frame map
float32 area_m2

# msg/RoomArray.msg
std_msgs/Header header          # frame map, stamp of the map used
vacuum_interfaces/Room[] rooms

# action/Explore.action
float32 timeout_s               # 0 = parameter explore_timeout_s
---
bool success
uint32 frontiers_done
uint32 frontiers_blacklisted
float32 duration_s
---
uint32 frontiers_remaining
float32 elapsed_s

# srv/SetRoomLabel.srv
string id
string label
---
bool success
string message
```

`topics.yaml`:

```yaml
topics:
  - {name: /map, type: nav_msgs/msg/OccupancyGrid, qos: latched, owner: [slam_toolbox, map_server], since: "003", note: "exactly one of them runs (map_file selects map_server); the contract test shall accept either owner"}
  - {name: /myRobot/map/explored,    type: std_msgs/msg/Bool, qos: latched, owner: exploration_server, max_publishers: 1, since: "003"}
  - {name: /myRobot/map/room_labels, type: nav_msgs/msg/OccupancyGrid, qos: latched, owner: room_segmenter, since: "003", note: "value = room index from 1, 0 = no room, -1 = unknown"}
  - {name: /myRobot/map/rooms,       type: vacuum_interfaces/msg/RoomArray, qos: latched, owner: room_segmenter, since: "003"}
  - {name: /myRobot/telemetry/odom_tf_bridge,    type: diagnostic_msgs/msg/DiagnosticArray, qos: default, owner: odom_tf_bridge, since: "003"}
  - {name: /myRobot/telemetry/exploration_server, type: diagnostic_msgs/msg/DiagnosticArray, qos: default, owner: exploration_server, since: "003"}
  - {name: /myRobot/telemetry/room_segmenter,    type: diagnostic_msgs/msg/DiagnosticArray, qos: default, owner: room_segmenter, since: "003"}
services:
  - {name: /myRobot/map/save,         type: std_srvs/srv/Trigger, owner: room_segmenter, since: "003"}
  - {name: /myRobot/map/segment,      type: std_srvs/srv/Trigger, owner: room_segmenter, since: "003"}
  - {name: /myRobot/rooms/set_label,  type: vacuum_interfaces/srv/SetRoomLabel, owner: room_segmenter, since: "003"}
actions:
  - {name: /myRobot/explore, type: vacuum_interfaces/action/Explore, owner: exploration_server, since: "003"}
lifecycle:
  lifecycle_manager_core: [supervisor, stop_guard, stuck_detector, escape_server, coverage_server, odom_tf_bridge, exploration_server, room_segmenter]
parameters:
  supervisor:
    explore_on_start: {type: bool, default: false, requirement: 003-R14, note: "true needs amendment A1 of spec 001; false until exploration exists"}
    explore_action: {type: string, default: /myRobot/explore}
  odom_tf_bridge:
    odom_topic: {type: string, default: /myrobot/odom, note: "D-07"}
    odom_frame: {type: string, default: odom}
    base_frame: {type: string, default: base_link}
    zero_at_first_message: {type: bool, default: false, note: "true if /myrobot/odom is in the world frame (Q-14)"}
  mapping:                          # launch arguments and static TF parameters
    odom_source: {type: string, default: sim, values: [sim, laser], requirement: 003-R01}
    map_file: {type: string, default: "", requirement: 003-R06}
    map_name: {type: string, default: house}
    start_pose_xyyaw: {type: double_array, default: [0, 0, 0], requirement: 003-R30, note: "read from maps/<map_name>_rooms.yaml with map_file"}
    scan_frame: {type: string, default: laser_link, note: "to be measured, T301"}
    laser_to_base_xyz_rpy: {type: double_array, default: [0, 0, 0, 0, 0, 0], note: "to be measured, T301"}
    camera_to_base_xyz_rpy: {type: double_array, default: [0, 0, 0, 0, 0, 0], note: "to be measured, T301"}
  exploration_server:
    min_frontier_cells: {type: int, default: 8, requirement: 003-R09}
    gain_weight: {type: double, default: 0.05}
    robot_radius_m: {type: double, default: 0.16}
    frontier_timeout_s: {type: double, default: 60.0, requirement: 003-R10}
    explore_timeout_s: {type: double, default: 900.0, requirement: 003-R11}
    map_loaded: {type: bool, default: false, requirement: 003-R06, note: "set by the launch with map_file; explored = true"}
    max_blacklist_ratio: {type: double, default: 0.5, requirement: 003-R11}
    nav_action: {type: string, default: /navigate_to_pose}
  room_segmenter:
    door_width_m: {type: double, default: 0.9, requirement: 003-R16, note: "to be measured, T301"}
    min_room_area_m2: {type: double, default: 2.0, requirement: 003-R16}
    min_votes: {type: int, default: 2, requirement: 003-R21}
    observations_topic: {type: string, default: /detections_markers, requirement: 003-R22}
    rules_file: {type: string, default: config/room_rules.yaml}
```

Diagrams: when the proposal is applied, update `docs/diagrams/a3-perception-offload.mmd` (SLAM, `/map`, rooms, labels; the `/topologic_map` of 2025S2 is not used by this spec) and `a1-overview.mmd` (mapping box), and re-render with `scripts/render-diagrams.sh`. Not touched now because the contract is not changing yet.

## Tests

- Unit (pytest, no ROS): `FrontierFinder` (empty room, door, blacklist, determinism), `RoomSegmenter` (two rooms, open plan, noise, 300 x 300 timing), `RoomTracker`, `RoomLabeler`, `SubmodeMachine` new rows; the requirement id in each test name.
- Node tests (rclpy, localhost discovery and random domain, as in T020 of spec 001): `odom_tf_bridge` (stamp and pose to TF; single broadcaster), `room_segmenter` (services, latched topics, persistence), `exploration_server` against a fake `NavigateToPose` server (retry, blacklist, timeout, cancel).
- `launch_testing` (Art. V.2): `mapping.launch.py` with `map_file` and fakes (003-R27).
- Simulator (local, marked, Art. V.3): R08, R15, R24, and the measurements of T301.

## Risks

- Time: exploration depends on Nav2 (spec 005, T510). Mitigation: `map_file` mode, so segmentation and labels are done without exploration; a map recorded by teleoperation is the fallback.
- Door width below the erosion radius merges rooms; above, it splits large rooms. `door_width_m` is measured on the scene (T301) and the result of 003-R24 decides the value.
- Ground-truth odometry makes SLAM easy and hides drift; stated as a limitation (D-23).
- Stamps and TF: if the bridge does not use simulation time consistently, SLAM and Nav2 reject messages (Q-13).
- Detection class names are unknown until spec 004 and T903 of spec 009a; `room_rules.yaml` starts with a few examples (for instance refrigerator -> kitchen, bed -> bedroom) marked as placeholders.
