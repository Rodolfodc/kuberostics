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
3. Next: the `set_label` service (manual label editing, 003-R23), default labels `room_<n>`.
4. Cut order inside the module, decided by the project owner, 2026-11-13: first the automatic labels by object rules (003-R21, R22, tasks T307 and T314); then the alternative odometry `odom_source:=laser` (003-R04, T321 to T323). Segmentation, `set_label` and the rest stay in scope. Without automatic labels, labels are `room_<n>` or user-set.

## Decision of this spec (DECIDED by the project owner, 2026-11-13; to be recorded in spec 000 by the architect): odometry for SLAM (closes Q-03)

| ID | Proposal | Evidence and rationale |
|---|---|---|
| D-23 (**DECIDED**, project owner, 2026-11-13) | The default odometry source is the simulator pose published by the 2025S2 bridge on `/myrobot/odom` (`odom_source:=sim`). A thin node, `odom_tf_bridge`, turns it into the TF `odom -> base_link` if the 2025S2 node does not already broadcast it (Q-14). The alternative, laser odometry (`rf2o`), is selectable by launch (`odom_source:=laser`) and is a cut candidate. | (1) `rf2o_laser_odometry` has **no entry** in `ros/rosdistro` `jazzy/distribution.yaml` and `lyrical/distribution.yaml` (checked 2026-11-13 on the `master` branch). Using it means a source build through `deps.repos` and an unverified port to Lyrical (Python 3.14, rclcpp of Lyrical). (2) `slam_toolbox` (2.8.5 / 2.10.0), `navigation2` and `robot_localization` do have entries on both. (3) Ground-truth pose is deterministic, which helps reproducible measurements and the training of spec 008. **Limit:** ground truth has no drift, so the map quality measured in the simulator is optimistic and says nothing about a real robot; the report shall say so. Not verified yet: the frame and origin of `/myrobot/odom` (world or start pose), covariance and rate (T302, T301). |

## Amendments to spec 001 required by this spec (not applied; for the architect)

`explore_on_start` defaults to `false`, so spec 001 holds unchanged until exploration exists. When the default becomes `true`, the architect must amend, in spec 001 itself and not by precedence: 001-R06 (an exception: with `explore_on_start` and `explored` false, CLEANING publishes EXPLORING instead of COVERING within 0.5 s); 001-R07 (stuck events in EXPLORING, not only COVERING and AUTODOCKING); 001-R08 (return to EXPLORING after the escape); 001-R21 (resume submode after an escape across a mode change). Until then the rows of the plan are exercised only with `explore_on_start:=true` in tests of spec 003. This is architect action A1; it is not a task of the four agents.

## Usage scenarios

1. **First run.** The user starts the system with `explore_on_start:=true` (default is false, so spec 001 scenarios keep working) and no saved map. Mode CLEANING: the supervisor publishes EXPLORING, the robot explores, the map is saved, the rooms are segmented, `explored` becomes `true`, and the supervisor goes to COVERING (spec 005).
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
| 003-R06 | When the argument `map_file` is set, the launch shall load that map with `nav2_map_server`, shall not start `slam_toolbox`, and shall broadcast a static `map -> odom` computed from the start pose recorded with the map (003-R30), valid only for `odom_source:=sim`. In that mode the launch shall pass `map_loaded:=true` to `exploration_server`, which is the only publisher of `explored` (003-R13). | launch_testing |
| 003-R07 | When `/myRobot/map/save` (service `std_srvs/Trigger`) is called, and when exploration ends, the system shall save the map to `maps/<map_name>.yaml` + `.pgm` with `nav2_map_server` and emit an INFO `Notification` `MAP_SAVED`. | node |
| 003-R08 | Needs simulator: after the exploration of 003-R10 and 003-R11, the map shall match a reference raster of the scene (exported by T301): median distance from each occupied cell of the map to the nearest occupied cell of the reference `<= 0.10 m`, and at least 90 % of the reference free area reachable by the robot marked free. The thresholds are initial. | sim (local) |

Exploration.

| ID | Requirement | Verification |
|---|---|---|
| 003-R09 | `FrontierFinder` shall return, from an occupancy grid and a robot position, the frontier clusters (known free cells with an unknown 8-neighbour, 8-connected clusters of at least `min_frontier_cells`, default 8), each with a centroid snapped to the nearest free cell with clearance `>= robot_radius_m`, ordered by a deterministic cost (distance to the robot minus `gain_weight` times cluster size), skipping the blacklisted ones. | unit |
| 003-R10 | When it accepts an `Explore` goal, `exploration_server` shall repeat: take the best frontier, send it as a `NavigateToPose` goal, and on abort or timeout (`frontier_timeout_s`, default 60) retry once and then blacklist it. | unit + node |
| 003-R11 | The goal shall succeed when no frontier remains; it shall return `success=false` with the partial map at `explore_timeout_s` (default 900 s) or when more than `max_blacklist_ratio` (default 0.5) of the frontiers seen are blacklisted; in that case it shall emit a WARN `Notification` `EXPLORATION_INCOMPLETE`. | unit |
| 003-R12 | When the goal is cancelled, the server shall cancel the Nav2 goal within 0.5 s and return. The server shall publish no velocity (Art. III.4; motion is on `mux/nav`). | node |
| 003-R13 | `exploration_server` shall be the only publisher of `/myRobot/map/explored` (`std_msgs/Bool`, latched, `max_publishers: 1` in the contract): `false` at startup, `true` when an `Explore` goal succeeds or when its parameter `map_loaded` is `true` (003-R06). | node |
| 003-R14 | While the mode is CLEANING, `explored` is `false`, the supervisor parameter `explore_on_start` is `true` (default **false** until the real exploration exists), `exploration_done` is false and the lock is off, the supervisor shall publish EXPLORING and send `/myRobot/explore`; on success it shall go to COVERING; on failure it shall go to COVERING with the partial map; on a `StuckEvent` it shall run the escape and return to EXPLORING (rows in the plan). With `explore_on_start` false the supervisor behaves exactly as in spec 001. | unit (`SubmodeMachine`) |
| 003-R15 | Needs simulator: on the *House* scene the exploration shall finish in at most 900 s (initial target) with at least 90 % of the reachable free area known, and the number of `StuckEvent`s and the time shall be recorded. | sim (local) |

Segmentation and labels.

| ID | Requirement | Verification |
|---|---|---|
| 003-R16 | `RoomSegmenter` shall turn an occupancy grid into a label raster: free space is eroded by `door_width_m / 2` (default door width 0.9 m, to be measured in T301), connected components with at least `min_room_area_m2` (default 2.0) become seeds, and every free cell takes the label of the nearest seed (multi-source breadth-first). It is deterministic. | unit |
| 003-R17 | On synthetic maps with `door_width_m` = 0.9 and the erosion radius 0.45 m, `RoomSegmenter` shall return 2 rooms for two rectangles joined by a 0.80 m opening (narrower than `door_width_m`), 1 room when the opening is 1.00 m, 1 room for an open rectangle, and no label on occupied or unknown cells. | unit |
| 003-R18 | `room_segmenter` shall run when `explored` becomes `true`, and when `/myRobot/map/segment` (`std_srvs/Trigger`) is called, and shall publish `/myRobot/map/room_labels` (`nav_msgs/OccupancyGrid`, latched, same geometry as `/map`, value = `Room.index` of the room that owns the cell, 0 = no room, -1 = unknown) and `/myRobot/map/rooms` (`vacuum_interfaces/RoomArray`, latched). | node |
| 003-R19 | After a new segmentation, a new room shall keep the `index` (and so the `id`), `label` and `label_source` of the previous room with which it has the largest overlap, if that overlap is at least 50 % of the area of the new room (one-to-one matching, largest overlap first); other rooms get the next unused index. Indexes are never reused while the map is the same. | unit (`RoomTracker`) |
| 003-R20 | Every `Room` shall have `index` (integer from 1, the value in `room_labels`), `id` (`room_%02d` of the index), `label` (default equal to `id`), centroid, area in m2, and `label_source` (`default`, `rule`, `user`). | unit |
| 003-R21 | (Cuttable, first in the cut order.) `RoomLabeler` shall choose each room's label by weighted vote of object observations (class, x, y) located in that room, using a rule table `class -> label, weight` (file `config/room_rules.yaml`), with at least `min_votes` (default 2); otherwise it keeps the default label. | unit |
| 003-R22 | (Cuttable, first in the cut order.) The node shall feed `RoomLabeler` from the topic in `observations_topic` (default `/detections_markers`; adapter written after T903 of spec 009a and the ontology of spec 004), and only while the label is not set by the user. | node |
| 003-R23 | The service `/myRobot/rooms/set_label` (`SetRoomLabel`: `id`, `label`) shall set `label_source = user`, shall be persisted in `maps/<map_name>_rooms.yaml` and shall survive a new segmentation and a restart. An unknown `id` shall be rejected with `success=false`. | node |
| 003-R24 | Needs simulator: on the *House* scene, the number of rooms shall equal the number in the hand-labelled reference (T301), and each reference room shall be matched by a segmented room with IoU `>= 0.7` (initial). | sim (local) |

Cross-cutting.

| ID | Requirement | Verification |
|---|---|---|
| 003-R25 | `CoverRoom.room_id` (spec 005, 005-R16) shall be the `Room.id` of 003-R20; the cells of that room are those of `room_labels` equal to its `Room.index`. An empty `room_id` means the room under the robot (spec 005, 005-R16; consistent with `interfaces.md`). | node (with spec 005) |
| 003-R26 | Each node shall be a `LifecycleNode` under `lifecycle_manager_core` (Q-15 for `slam_toolbox`), shall log every transition and event, and shall publish telemetry at 1 Hz on `/myRobot/telemetry/<node>` (D-20). Logic shall live in pure Python classes without ROS imports (D-18, D-19). | unit + launch_testing |
| 003-R27 | `mapping.launch.py` shall run alone (Art. III.1), fed by a rosbag or by fakes (a fake `/scan` and `/myrobot/odom` publisher plus a saved map), and shall have a `launch_testing` test: with `map_file`, the rooms and labels topics appear and the `set_label` service works. | launch_testing |
| 003-R28 | `RoomSegmenter` shall process a 300 x 300 cell grid (15 m x 15 m at 0.05 m) in less than 2 s on the CI machine. | unit |
| 003-R30 | The map files shall record the start pose of the robot in frame `map` (x, y, yaw, in `maps/<map_name>_rooms.yaml`). When `map_file` is set, the launch shall build the static `map -> odom` from that pose and shall check that the first `/myrobot/odom` (after `zero_at_first_message` if used) is consistent with the scene being reset to the same start pose; if the check cannot be done, it shall log a WARN that the static transform is valid only if the robot starts at the recorded pose. | unit (transform computation) + sim |
| 003-R31 | When `exploration_server` is deactivated, shut down or terminated while it has a Nav2 goal in flight, it shall cancel that goal before it stops (best effort on a crash, then the 0.5 s command timeout of the `mux/nav` input applies); `frontier_timeout_s` bounds every frontier goal, which is cancelled on timeout. | node |
| 003-R32 | When the supervisor cancels `explore` (for an escape) and sends it again, `exploration_server` shall keep its blacklist and the frontier counters, and shall not restart the `explore_timeout_s` clock, until the goal ends with success or failure. | unit + node |
| 003-R29 | Requirements 003-R01 to R03, R05 to R07, R09 to R14, R16 to R23, R25 to R28 and R30 to R32 shall hold on Jazzy and on Lyrical, with the same code and configuration. | CI (matrix) |

## Definition of done

- Unit, node and `launch_testing` tests of the requirements listed in 003-R29 passing on both distributions in CI.
- Simulator measurements recorded for R08, R15 and R24, with the thresholds used. The values marked "initial" are adjusted against them.
- D-23 and the answers to Q-13 to Q-16 recorded in spec 000 by the architect after review.
- A saved map of the House scene and its rooms file under `maps/` (small files only; the scene file itself is not versioned). Committing them is a proposal for the user; fallback: they stay in `.sim/` (not versioned) and the `map_file` tests use a synthetic map.

## Open questions (proposed, to be moved into spec 000 section 6)

- Timeouts in this spec (R10, R11, R31, R32) use the node clock: wall time unless Q-13 sets `use_sim_time`.
- Contract change order: this spec's changes are applied first, then those of spec 005; one serialized change each.
- Q-13: time source. Does the 2025S2 bridge publish `/clock` and are its stamps simulation time? It decides `use_sim_time` for SLAM Toolbox, Nav2 and TF. Needs T301 (simulator) and T302 (code).
- Q-14: does the 2025S2 odometry node already broadcast `odom -> base_link`? Frame names, parent/child, pose in world or in the start frame, and message rate. Needs T302 (read `src/deps`, read-only) and T301.
- Q-15: does the `slam_toolbox` release on each distro offer a lifecycle variant of its node that `lifecycle_manager_core` can manage (Art. III.2)? If not, it is accepted as a third-party plain node and documented as an exception. Not verified; T311 checks.
- Q-16: availability of `python3-scipy` (used by `RoomSegmenter`) through rosdep on Ubuntu 24.04 and 26.04 with Python 3.12 and 3.14. Not verified; T304 checks. Fallback: `python3-opencv`, or a numpy-only distance transform.
