# 003: Mapping

Status: draft for review · Target: 2026-11-22 (map and TF first, by 2026-11-18; they unblock spec 005) · Depends on: 001 · Used by: 004 (labels), 005 (map, TF, rooms), 007 (rooms in the interface)
Assignment module: 2 (exploration, SLAM, segmentation, labels). Core module: never cut (Art. IX.3). Inside the module, the cut candidates are listed under "Minimal version".

## Goal

Build, from the robot's own sensors in the *House* scene, an occupancy map; explore the house autonomously until no
reachable frontier is left; split the map into rooms with stable identifiers; and give each room a label, by rule from
detected objects or by the user. Decide which odometry feeds SLAM (Q-03). All of it on Jazzy and Lyrical.

## Out of scope

- Detection and the object ontology (spec 004); this spec only consumes object observations.
- The interface that shows or edits rooms (spec 007); this spec only provides the topics and the rename service.
- Navigation, coverage and escape (spec 005). Exploration reuses its Nav2 configuration (T510 of spec 005 owns `nav.launch.py`).
- Long-term localization on a saved map (AMCL). SLAM Toolbox stays in mapping mode (online, asynchronous).

## Minimal version and cut order inside the module (Art. IX.1)

1. Must have: odometry and TF, SLAM Toolbox, a map saved to disk, room segmentation, default labels, `map_file` mode.
2. Next: autonomous frontier exploration. Fallback if it is late: the user drives the robot with teleoperation to build the map and the launch is started with `map_file:=...` (003-R06).
3. Next: automatic labels by object rules and the `set_label` service. Fallback: `room_<n>` only.
4. Cut first: the alternative odometry `odom_source:=laser` (003-R04).

## Decision proposed by this spec (to be moved into spec 000 by the architect): odometry for SLAM (closes Q-03)

| ID | Proposal | Evidence and rationale |
|---|---|---|
| D-23 (proposed) | The default odometry source is the simulator pose published by the 2025S2 bridge on `/myrobot/odom` (`odom_source:=sim`). A thin node, `odom_tf_bridge`, turns it into the TF `odom -> base_link` if the 2025S2 node does not already broadcast it (Q-14). The alternative, laser odometry (`rf2o`), is selectable by launch (`odom_source:=laser`) and is a cut candidate. | (1) `rf2o_laser_odometry` has **no entry** in `ros/rosdistro` `jazzy/distribution.yaml` and `lyrical/distribution.yaml` (checked 2026-11-13 on the `master` branch). Using it means a source build through `deps.repos` and an unverified port to Lyrical (Python 3.14, rclcpp of Lyrical). (2) `slam_toolbox` (2.8.5 / 2.10.0), `navigation2` and `robot_localization` do have entries on both. (3) Ground-truth pose is deterministic, which helps reproducible measurements and the training of spec 008. **Limit:** ground truth has no drift, so the map quality measured in the simulator is optimistic and says nothing about a real robot; the report shall say so. Not verified yet: the frame and origin of `/myrobot/odom` (world or start pose), covariance and rate (T302, T301). |

## Usage scenarios

1. **First run.** The user starts the system with `explore_on_start:=true` and no saved map. Mode CLEANING: the supervisor publishes EXPLORING, the robot explores, the map is saved, the rooms are segmented, `explored` becomes `true`, and the supervisor goes to COVERING (spec 005).
2. **Known map.** The system starts with `map_file:=maps/house.yaml`. `explored` is `true` at once, rooms are segmented from the file and no exploration happens.
3. **Stuck while exploring.** A `StuckEvent` during EXPLORING: escape, then back to EXPLORING.
4. **Relabel.** The user renames `room_02` to "kitchen" by service; the name survives a new segmentation.

## Requirements (EARS)

Odometry and TF.

| ID | Requirement | Verification |
|---|---|---|
| 003-R01 | The system shall use a single odometry source for SLAM and Nav2, chosen by the launch argument `odom_source` (`sim` default, `laser`), with the same TF names in both: `map -> odom -> base_link -> <sensor frames>`. | launch_testing (TF tree) |
| 003-R02 | Where `odom_source:=sim` and the 2025S2 node does not broadcast `odom -> base_link`, `odom_tf_bridge` shall broadcast it from the pose and the `header.stamp` of each `/myrobot/odom` message. There shall be exactly one broadcaster of `odom -> base_link` at any time. | unit + launch_testing |
| 003-R03 | The launch shall publish the static transforms `base_link -> <laser frame>` and `base_link -> <camera frame>` from parameters whose values come from the scene (T301). The frame of `/scan` shall be the one declared in the parameter `scan_frame`. | launch_testing |
| 003-R04 | Where `odom_source:=laser`, the launch shall start `rf2o` (built from source, `deps.repos`) instead of `odom_tf_bridge`, without any change in the other nodes (Art. III.5). Cut candidate. | launch_testing + sim |

SLAM and map file.

| ID | Requirement | Verification |
|---|---|---|
| 003-R05 | The system shall run `slam_toolbox` online asynchronous in mapping mode with explicit parameters (Art. X.3): `odom_frame: odom`, `map_frame: map`, `base_frame: base_link`, `scan_topic: /scan`, `resolution: 0.05`, `max_laser_range` equal to the laser range measured in T301, `use_sim_time` per Q-13; it publishes `/map` (latched, `nav_msgs/OccupancyGrid`) and the TF `map -> odom`. | launch_testing |
| 003-R06 | When the argument `map_file` is set, the launch shall load that map with `nav2_map_server`, shall not start `slam_toolbox`, shall broadcast a static identity `map -> odom` (valid only for `odom_source:=sim`), and shall publish `explored = true`. | launch_testing |
| 003-R07 | When `/myRobot/map/save` (service `std_srvs/Trigger`) is called, and when exploration ends, the system shall save the map to `maps/<map_name>.yaml` + `.pgm` with `nav2_map_server` and emit an INFO `Notification` `MAP_SAVED`. | node |
| 003-R08 | Needs simulator: after the exploration of 003-R10 and 003-R11, the map shall match a reference raster of the scene (exported by T301): median distance from each occupied cell of the map to the nearest occupied cell of the reference `<= 0.10 m`, and at least 90 % of the reference free area reachable by the robot marked free. The thresholds are initial. | sim (local) |

Exploration.

| ID | Requirement | Verification |
|---|---|---|
| 003-R09 | `FrontierFinder` shall return, from an occupancy grid and a robot position, the frontier clusters (known free cells with an unknown 8-neighbour, 8-connected clusters of at least `min_frontier_cells`, default 8), each with a centroid snapped to the nearest free cell with clearance `>= robot_radius_m`, ordered by a deterministic cost (distance to the robot minus `gain_weight` times cluster size), skipping the blacklisted ones. | unit |
| 003-R10 | When it accepts an `Explore` goal, `exploration_server` shall repeat: take the best frontier, send it as a `NavigateToPose` goal, and on abort or timeout (`frontier_timeout_s`, default 60) retry once and then blacklist it. | unit + node |
| 003-R11 | The goal shall succeed when no frontier remains; it shall return `success=false` with the partial map at `explore_timeout_s` (default 900 s) or when more than `max_blacklist_ratio` (default 0.5) of the frontiers seen are blacklisted; in that case it shall emit a WARN `Notification` `EXPLORATION_INCOMPLETE`. | unit |
| 003-R12 | When the goal is cancelled, the server shall cancel the Nav2 goal within 0.5 s and return. The server shall publish no velocity (Art. III.4; motion is on `mux/nav`). | node |
| 003-R13 | The server shall publish `/myRobot/map/explored` (`std_msgs/Bool`, latched): `false` at startup without `map_file`, `true` when an `Explore` goal succeeds or a map is loaded (003-R06). | node |
| 003-R14 | While the mode is CLEANING, `explored` is `false` and the parameter `explore_on_start` is `true`, the supervisor shall publish EXPLORING and send `/myRobot/explore`; on success it shall go to COVERING; on failure it shall go to COVERING with the partial map; on a `StuckEvent` it shall run the escape and return to EXPLORING (rows in the plan). | unit (`SubmodeMachine`) |
| 003-R15 | Needs simulator: on the *House* scene the exploration shall finish in at most 900 s (initial target) with at least 90 % of the reachable free area known, and the number of `StuckEvent`s and the time shall be recorded. | sim (local) |

Segmentation and labels.

| ID | Requirement | Verification |
|---|---|---|
| 003-R16 | `RoomSegmenter` shall turn an occupancy grid into a label raster: free space is eroded by `door_width_m / 2` (default door width 0.9 m, to be measured in T301), connected components with at least `min_room_area_m2` (default 2.0) become seeds, and every free cell takes the label of the nearest seed (multi-source breadth-first). It is deterministic. | unit |
| 003-R17 | On synthetic maps, `RoomSegmenter` shall return 2 rooms for two rectangles joined by a 0.9 m opening, 1 room for an open rectangle, and no label on occupied or unknown cells. | unit |
| 003-R18 | `room_segmenter` shall run when `explored` becomes `true`, and when `/myRobot/map/segment` (`std_srvs/Trigger`) is called, and shall publish `/myRobot/map/room_labels` (`nav_msgs/OccupancyGrid`, latched, same geometry as `/map`, value = room index from 1, 0 = no room, -1 = unknown) and `/myRobot/map/rooms` (`vacuum_interfaces/RoomArray`, latched). | node |
| 003-R19 | After a new segmentation, a room whose area overlaps a previous room by at least 50 % shall keep that room's `id` and `label`; other rooms get the next free id `room_NN`. | unit (`RoomTracker`) |
| 003-R20 | Every `Room` shall have `id`, `label` (default equal to `id`), centroid, area in m2, and `label_source` (`default`, `rule`, `user`). | unit |
| 003-R21 | `RoomLabeler` shall choose each room's label by weighted vote of object observations (class, x, y) located in that room, using a rule table `class -> label, weight` (file `config/room_rules.yaml`), with at least `min_votes` (default 2); otherwise it keeps the default label. | unit |
| 003-R22 | The node shall feed `RoomLabeler` from the topic in `observations_topic` (default `/detections_markers`; adapter written after T903 of spec 009a and the ontology of spec 004), and only while the label is not set by the user. | node |
| 003-R23 | The service `/myRobot/rooms/set_label` (`SetRoomLabel`: `id`, `label`) shall set `label_source = user`, shall be persisted in `maps/<map_name>_rooms.yaml` and shall survive a new segmentation and a restart. An unknown `id` shall be rejected with `success=false`. | node |
| 003-R24 | Needs simulator: on the *House* scene, the number of rooms shall equal the number in the hand-labelled reference (T301), and each reference room shall be matched by a segmented room with IoU `>= 0.7` (initial). | sim (local) |

Cross-cutting.

| ID | Requirement | Verification |
|---|---|---|
| 003-R25 | The room identifiers of 003-R18 are the values accepted by `CoverRoom.room_id` (spec 005, 005-R16). | node (with spec 005) |
| 003-R26 | Each node shall be a `LifecycleNode` under `lifecycle_manager_core` (Q-15 for `slam_toolbox`), shall log every transition and event, and shall publish telemetry at 1 Hz on `/myRobot/telemetry/<node>` (D-20). Logic shall live in pure Python classes without ROS imports (D-18, D-19). | unit + launch_testing |
| 003-R27 | `mapping.launch.py` shall run alone (Art. III.1), fed by a rosbag or by fakes (a fake `/scan` and `/myrobot/odom` publisher plus a saved map), and shall have a `launch_testing` test: with `map_file`, the rooms and labels topics appear and the `set_label` service works. | launch_testing |
| 003-R28 | `RoomSegmenter` shall process a 300 x 300 cell grid (15 m x 15 m at 0.05 m) in less than 2 s on the CI machine. | unit |
| 003-R29 | Requirements 003-R01 to R03, R05 to R07, R09 to R14, R16 to R23 and R25 to R28 shall hold on Jazzy and on Lyrical, with the same code and configuration. | CI (matrix) |

## Definition of done

- Unit, node and `launch_testing` tests of the requirements listed in 003-R29 passing on both distributions in CI.
- Simulator measurements recorded for R08, R15 and R24, with the thresholds used. The values marked "initial" are adjusted against them.
- D-23 and the answers to Q-13 to Q-16 recorded in spec 000 by the architect after review.
- A saved map of the House scene and its rooms file committed under `maps/` (small files only; the scene file itself is not versioned).

## Open questions (proposed, to be moved into spec 000 section 6)

- Q-13: time source. Does the 2025S2 bridge publish `/clock` and are its stamps simulation time? It decides `use_sim_time` for SLAM Toolbox, Nav2 and TF. Needs T301 (simulator) and T302 (code).
- Q-14: does the 2025S2 odometry node already broadcast `odom -> base_link`? Frame names, parent/child, pose in world or in the start frame, and message rate. Needs T302 (read `src/deps`, read-only) and T301.
- Q-15: does the `slam_toolbox` release on each distro offer a lifecycle variant of its node that `lifecycle_manager_core` can manage (Art. III.2)? If not, it is accepted as a third-party plain node and documented as an exception. Not verified; T311 checks.
- Q-16: availability of `python3-scipy` (used by `RoomSegmenter`) through rosdep on Ubuntu 24.04 and 26.04 with Python 3.12 and 3.14. Not verified; T304 checks. Fallback: `python3-opencv`, or a numpy-only distance transform.
