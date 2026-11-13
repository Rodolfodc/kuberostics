# 005: Coverage and escape

Status: draft for review · Target: 2026-11-27 (stuck detection and escape first, by 2026-11-22) · Depends on: 001 (stubs and contracts), 003 (map, TF, rooms) · Used by: 008 (swaps the escape)
Assignment module: 5 (global path, coverage, avoidance, stuck detection, scripted escape). Core module: never cut (Art. IX.3); only the RL policy of spec 008 is a cut candidate, and this spec is the fallback that stays.

## Goal

Replace the three stubs of spec 001 (`coverage_server`, `escape_server`, `stuck_detector`) with real implementations
behind the same node names, actions, topics and test hooks (Art. IV). The robot covers a room with parallel lanes, avoids
obstacles, notices when it is stuck and frees itself with a deterministic script, with a timeout and a stop as the last
resort (Art. VI.3). All of it on Jazzy and Lyrical.

## Out of scope

- Mapping, SLAM, room segmentation and labels (spec 003; this spec only consumes `/map`, TF and the room topics).
- The RL escape policy and the Gymnasium environment (spec 008). This spec keeps `strategy: "scripted"` in the result so 008 can swap it.
- Docking, schedule and go-to-room (specs 006 and later).
- Nav2 recovery behaviours (spin, back up, wait): not launched. Recovery is the escape of this spec.

## Decisions of this spec (D-24 decided; D-25 proposed; to be recorded in spec 000 by the architect)

| ID | Proposal | Rationale |
|---|---|---|
| D-24 (**DECIDED**, project owner, 2026-11-13) | Coverage motion goes through Nav2: `coverage_server` plans lanes and sends `NavigateToPose` goals; Nav2 `controller_server` is the only coverage actuator and publishes on `/myRobot/mux/nav` (priority 60, already declared in `topics.yaml`). `coverage_server` publishes no `Twist`. `/myRobot/mux/coverage` stays declared for the stub and, in the contract change, becomes `reserved` without an owner (Q-10). **Consequences:** (a) the motion chain in `CLAUDE.md` and spec 000 changes (coverage no longer goes through `mux/coverage`); the architect applies that in a later change, not in this one; (b) the Nav2 bring-up spike T510 moves as early as possible in the schedule (start first, by 2026-11-20); (c) the per-waypoint timeout (005-R20), the cancel of the Nav2 goal on deactivate and shutdown (005-R31) and the supervisor watchdog for a crashed server (005-R34) are mandatory. | Reuses the global planner and the local costmap for avoidance instead of writing a follower. Art. III.3 and III.4 hold: the actuator is `controller_server`; `coverage_server` only computes and sends goals. |
| D-25 (proposed) | Escape priority (80) stays above navigation (60) and coverage (50). The supervisor cancels `cover_room` before sending `escape` (001-R07), so only one of them is active; the priorities are the safety net. | Art. VI.1, D-03. |

## Minimal version and cut list (Art. IX.1)

1. Must have, first: the Nav2 bring-up spike (T510 early: `NavigateToPose` on a saved map moves the robot on `mux/nav` in CoppeliaSim), real `stuck_detector` and `escape_server`.
2. Next: `coverage_server` with the empty-`room_id` region (no spec 003 rooms needed), single pass, retry/skip.
3. Next: room selection by `room_id`, resume after escape (R32), `passes > 1`.
4. Cut first if late: `passes > 1`, `coverage/path` and `coverage/covered` topics, the supervisor rows of R30 (keep the 001 behaviour), the avoidance box test (R24).
5. If the Nav2 spike (T510) fails or cannot meet the schedule, escalate to the project owner; no alternative is pre-approved.

## Amendments to spec 001 required by this spec (not applied; for the architect)

Amendment A2, from Q-12 (005-R30). The `coverage_blocked` flag changes the 001 behaviour in CLEANING, so the architect amends spec 001 itself: (1) 001-R06 gets an exception: when `coverage_blocked` is true, the mode CLEANING does not publish COVERING and does not send `cover_room`; (2) the row of plan 001 "NONE, mode = CLEANING: sends `cover_room`" is suppressed while `coverage_blocked` is true; (3) 001-R05 stays true as written (NONE for modes other than CLEANING and AUTODOCKING), but NONE with mode CLEANING now exists and is described in the plan rows; (4) 001-R21 is unchanged (the result row ends in NONE, so there is no coverage left to cancel). Interplay with A1 of spec 003: the order inside one CLEANING session is EXPLORING (only with `explore_on_start`), then COVERING, then `coverage_blocked` after the `cover_room` result; `exploration_done` and `coverage_blocked` are independent flags. Until A2 is applied, the real supervisor rows are active only behind the supervisor parameter `dock_after_coverage` (default false; proposed, 005-R30), so the 001 tests and scenarios hold unchanged.

## Usage scenarios

1. **Clean a room.** The mode becomes CLEANING, the supervisor publishes COVERING and sends `cover_room`. `coverage_server` builds the lanes, drives Nav2 lane by lane and reports `covered_ratio`. At 90 % (or when the lanes are exhausted) the goal succeeds and the supervisor asks for docking (005-R30).
2. **Wedged against furniture.** Nav2 keeps commanding motion and the robot does not move. `stuck_detector` emits `StuckEvent(NO_PROGRESS)`. The supervisor cancels the coverage and calls `escape`: the robot backs up, turns toward the freer side and the supervisor resumes the coverage.
3. **Escape does not work.** The script fails all attempts and the fallback sweep fails too, or the timeout expires. The goal returns `success=false`, the supervisor locks the robot (001-R09) and notifies.
4. **Unknown room.** `room_id` empty (or spec 003 not running): the region is the free area reachable from the robot.

## Requirements (EARS)

Stuck detection (`stuck_detector`, a service in the sense of Art. III.4: it perceives and never moves the robot).

| ID | Requirement | Verification |
|---|---|---|
| 005-R01 | While the command on `/myRobot/cmd_vel` has `abs(linear.x) >= cmd_linear_min_m_s` (default 0.05) or `abs(angular.z) >= cmd_angular_min_rad_s` (default 0.20), the detector shall integrate the measured motion from the odometry (`odom_topic`, default `/myrobot/odom`); when over `window_s` (default 3.0 s) of continuous command the translation is below `min_translation_m` (default 0.03) and the yaw change is below `min_rotation_rad` (default 0.10), it shall publish `StuckEvent` with cause `NO_PROGRESS`. | unit (`StuckCore`) |
| 005-R02 | When the command drops below both thresholds, the detector shall reset the window and shall not publish. In particular a zero `Twist` from the stop lock (D-22) never produces an event. | unit |
| 005-R03 | After publishing an event, the detector shall publish no other for `cooldown_s` (default 5.0 s), and the window restarts when the cooldown ends. | unit |
| 005-R04 | Each `StuckEvent` shall carry the detection stamp, the pose in frame `map` from TF (`map -> base_link`; if the transform is not available, the odometry pose, with a WARN log), and `scan_ranges` copied from the last `/scan` received less than 0.5 s earlier (empty array if none). | unit + node |
| 005-R05 | Where the parameter `bumper_topic` is not empty, the detector shall publish `StuckEvent` with cause `BUMPER` on a press of that topic. The default is empty (disabled): the minimal version has no bumper source (Q-11). | unit |
| 005-R06 | The service `/myRobot/stuck_detector/trigger` shall keep publishing one `NO_PROGRESS` event per call, as a test hook (001-R07, Art. IV). | node |
| 005-R07 | Needs simulator: with the robot wedged against a wall under constant forward command, the detector shall emit the event within `window_s + 1 s`; during a full coverage run in free space it shall emit zero events. | sim (local) |

Scripted escape (`escape_server`, an actuator: publishes only on `/myRobot/mux/escape`).

| ID | Requirement | Verification |
|---|---|---|
| 005-R08 | When it accepts an `Escape` goal, the server shall run the script `EscapeScript`, attempt by attempt: (1) back up up to `reverse_m` (default 0.20 m) at `reverse_speed_m_s` (default 0.15), measured on the odometry, stopping early if the scan covers the rear and the rear range falls below `rear_clearance_m` (default 0.15); (2) turn in place toward the side with the larger mean range in `event.scan_ranges` by `turn_deg_base + attempt * turn_deg_step` (defaults 90 and 45) at `turn_speed_rad_s` (default 1.0); (3) check front clearance. The speeds shall not exceed the wheel-bridge limits measured in T503. | unit (`EscapeScript`) |
| 005-R09 | The script shall be deterministic: the same `StuckEvent` and the same sensor sequence produce the same command sequence (ties between left and right go to the left). | unit |
| 005-R10 | When after the turn the front range is `>= front_clearance_m` (default 0.40) the attempt succeeds and the goal returns `success=true`; otherwise it runs the next attempt, up to `max_attempts` (default 3) or until `primary_budget_s` (default 5 s, counted from the goal start; an attempt in progress when it expires is abandoned) is used up, whichever comes first. | unit |
| 005-R11 | When the attempts end without success (attempts exhausted or `primary_budget_s` reached), the server shall run the fallback once: rotate 360 degrees in place at `turn_speed_rad_s` recording the scan, turn to face the heading of the largest range (0 to 180 degrees), advance `fallback_forward_m` (default 0.30) at `reverse_speed_m_s`; the turn and the advance are judged by the same front clearance. With the defaults the fallback takes 6.28 s (sweep) + up to 3.14 s (facing, worst case 180 degrees) + 2.0 s (advance) = 11.42 s at most, which fits `fallback_budget_s` (default 11.5 s). `primary_budget_s + fallback_budget_s` (16.5 s) shall be below `default_timeout_s` (17 s), which leaves 3 s to the supervisor's `escape_timeout_s` (20 s). If the fallback fails or does not finish within `fallback_budget_s`, the goal returns `success=false`. Unit tests with a simulated clock: (a) every attempt failing reaches the fallback before `default_timeout_s`; (b) the fallback with the worst-case facing turn (180 degrees) completes within `fallback_budget_s`; (c) the speeds are validated at start-up against the limits measured in T503 (reject a configuration whose worst case does not fit). | unit |
| 005-R12 | The server shall finish the goal within `goal.timeout_s` (when 0, parameter `default_timeout_s`, default 17 s, below the supervisor's `escape_timeout_s` of 20 s). On timeout, cancel, failure, deactivation or shutdown it shall stop publishing on `mux/escape` within 0.2 s and return `success=false`. Timeouts are measured on the node clock (wall time unless Q-13 sets `use_sim_time`). | unit + node |
| 005-R13 | The result shall carry `duration_s` and `strategy = "scripted"`; the feedback `elapsed_s` shall be published at 2 Hz or faster. | node |
| 005-R14 | The parameter `outcome` shall keep the values `succeed`, `fail`, `hang` (stub behaviour, 001-R16) and add `auto` (default of the real node: the script above). | node |
| 005-R15 | Needs simulator: from at least 20 distinct stuck poses (corner, furniture leg, between two obstacles, against a wall), the success rate and median duration of the script shall be recorded. Initial target: success rate `>= 0.8` (to be adjusted after the first run); this is the baseline of spec 008. | sim (local) |

Coverage and avoidance (`coverage_server` plans; Nav2 moves the robot).

| ID | Requirement | Verification |
|---|---|---|
| 005-R16 | When it accepts a `CoverRoom` goal, the server shall build the region to clean: if `room_id` is not empty, the cells of `/myRobot/map/room_labels` whose value equals `Room.index` of the `Room` with that `id` in `/myRobot/map/rooms`; if `room_id` is empty and the room topics have been received, the room under the robot (the label at the robot pose; if the robot is on a cell with no room, the nearest room); if the room topics have not been received, the free cells connected to the robot position, limited to `max_region_area_m2` (default 150), logged as a WARN. An unknown non-empty `room_id` shall be rejected. | unit + node |
| 005-R17 | `CoveragePlanner` shall produce waypoints in boustrophedon order (parallel lanes of spacing `lane_spacing_m`, default 0.25, alternating direction) over the region free of obstacles inflated by `robot_radius_m` (default 0.16) + `margin_m` (default 0.05), skipping lane segments shorter than `min_segment_m` (default 0.30). With `passes > 1`, each extra pass rotates the lane direction by 90 degrees. The output is deterministic. | unit |
| 005-R18 | On synthetic maps (empty rectangle, rectangle with a central obstacle, two rooms), simulating the robot passing through every waypoint with `tool_radius_m` (default 0.15) shall cover at least 95 % of the coverable cells. | unit |
| 005-R19 | The server shall send each waypoint as a `NavigateToPose` goal (`/navigate_to_pose`) in order and shall publish no velocity. Global path and obstacle avoidance are done by Nav2 (`planner_server`, `controller_server` with local costmap fed by `/scan`). `controller_server` and `bt_navigator` shall take the odometry from `/myrobot/odom` (`nav_msgs/Odometry` with a usable twist; to confirm in T510 and Q-14). | node + sim |
| 005-R20 | If Nav2 aborts a waypoint, or the waypoint is not reached within `waypoint_timeout_s` (default 120 s; the goal is then cancelled), the server shall retry it once and then skip it, incrementing `skipped_waypoints`; if skipped waypoints exceed `max_skip_ratio` (default 0.3) of the total, or the goal exceeds `coverage_timeout_s` (default 1800 s), the goal returns `success=false`. | unit |
| 005-R21 | The server shall mark as covered the cells within `tool_radius_m` of the robot pose (TF) and publish `covered_ratio = covered / coverable` as feedback at 1 Hz or faster. The goal succeeds when `covered_ratio >= target_ratio` (default 0.90) after the requested `passes`, or when the waypoints are exhausted with `skipped_waypoints` within the limit of 005-R20. | unit + node |
| 005-R22 | When the goal is cancelled, the server shall cancel the ongoing Nav2 goal within 0.5 s and return. | node |
| 005-R23 | Nav2 shall publish velocity only through `controller_server`, remapped to `/myRobot/mux/nav`; every Nav2 node that publishes velocity shall declare `enable_stamped_cmd_vel: false` explicitly (D-13, Art. X.3); `twist_mux` shall have the `nav` input with priority 60 and timeout 0.5 s, between `escape` (80) and `coverage` (50). | config test + contract |
| 005-R24 | Needs simulator: with a box placed in a lane, the robot shall reach the following waypoint with at most one `StuckEvent` and no collision recorded by the bridge; the box shall appear in the local costmap. | sim (local) |
| 005-R25 | Needs simulator: on the *House* scene, a full CLEANING cycle (all rooms, one pass) shall reach `covered_ratio >= 0.85` of the reachable area; the time, the number of `StuckEvent`s and the number of failed escapes shall be recorded for the report. The 0.85 target is initial. | sim (local) |

Cross-cutting.

| ID | Requirement | Verification |
|---|---|---|
| 005-R26 | The real nodes shall keep the names `coverage_server`, `escape_server` and `stuck_detector`, the actions `/myRobot/cover_room` and `/myRobot/escape`, the topic `/myRobot/stuck` and the telemetry topics of spec 001, and shall pass the contract tests of spec 001 (T005/T022) with only the contract changes listed in the plan. The one proposed change is the entry `/myRobot/mux/coverage`: marked `reserved: true` with no owner (it keeps the `twist_mux` subscription) in the contract change, which shall be applied before T511; until then the stub `coverage_server` remains the default and no stub leaves the default launch (T508, T509, T519 depend on T005 and T022). | contract (T005/T022) |
| 005-R27 | Each node shall be a `LifecycleNode` managed by its lifecycle manager (`lifecycle_manager_nav` for the Nav2 nodes, Q-09), shall log every state transition and event and shall publish telemetry at 1 Hz on its `/myRobot/telemetry/<node>` (D-20). The logic shall live in pure Python classes without ROS imports (D-18, D-19). | unit + launch_testing |
| 005-R28 | Each module shall have its own launch file that runs it alone with fakes (Art. III.1) and a `launch_testing` test: `stuck_detector` + `escape_server` with fake odometry and scan; `coverage_server` with a static map and a fake `NavigateToPose` server. | launch_testing |
| 005-R29 | Requirements 005-R01 to 005-R06, 005-R08 to 005-R14, 005-R16 to 005-R23, 005-R26 to 005-R28 and 005-R30 to 005-R38 shall hold on Jazzy and on Lyrical, with the same code and configuration. | CI (matrix) |
| 005-R30 | When the `cover_room` result arrives (success or failure) while the mode is CLEANING, the supervisor shall publish an INFO `Notification` `COVERAGE_DONE` (success) or a WARN `Notification` `COVERAGE_FAILED` (failure), publish NONE, request docking from the IA368II orchestrator through the mechanism confirmed in T520 without extending the `Mode` enum, and send no further `cover_room` goal while the mode stays CLEANING (flag `coverage_blocked`, cleared when the mode leaves CLEANING). If the mode has not become AUTODOCKING within `dock_request_timeout_s` (default 30 s), it shall publish an ERROR `Notification` `DOCK_REQUEST_FAILED` and stay in NONE (Art. VI.3). Decided Q-12, project owner, 2026-11-13; the rows are a proposal for plan 001. | unit (`SubmodeMachine`) + node (fake `start_stop` service) |
| 005-R31 | When `coverage_server` (or `exploration_server`, spec 003) is deactivated or shut down gracefully while it has a Nav2 goal in flight, it shall cancel that goal before it stops, so `controller_server` stops publishing on `/myRobot/mux/nav`. (`escape_server` has no Nav2 goal and is not covered here; its stop is 005-R12.) | node |
| 005-R32 | When the supervisor cancels `cover_room` (for an escape) and sends it again with the same `room_id` within `resume_window_s` (default 120 s), `coverage_server` shall keep the covered mask and the list of remaining waypoints and resume from the first waypoint not yet reached, instead of rebuilding the lanes; the window and the state are cleared when a goal ends with success or failure. | unit + node |
| 005-R33 | The launch file shall set the `escape_server` parameter `outcome` per implementation: `succeed` for the stub (as in spec 001) and `auto` for the real node. | launch_testing |
| 005-R34 | When the `cover_room` or `explore` action server stops answering while its goal is active (crash, kill, loss of the node), the supervisor shall, within `nav_cancel_s` (default 1.0 s), cancel all goals of `/navigate_to_pose` (cancel request with zero goal id and zero stamp), treat the action as failed, and the velocity published by `controller_server` shall stop. Residual bound for the case where the supervisor is also lost: the BT of `nav_bt.xml` bounds each navigation goal with a timeout `nav_goal_timeout_s` (default 180 s; the `Timeout` decorator of BT.CPP, unverified in the Nav2 of both distros, T510 checks), and `stop_guard` plus `lock_stop` stay available to the user. The `twist_mux` 0.5 s input timeout alone does not cover this case, because a live `controller_server` keeps publishing fresh velocity. Test: kill the server mid-goal with a fake Nav2 and a fake velocity publisher on `mux/nav`, and assert that no non-zero command arrives on `mux/nav` within 2.0 s (0.5 s detection, 1.0 s cancel, 0.5 s margin). | launch_testing |
| 005-R35 | The system shall start its lifecycle managers in a defined order: `lifecycle_manager_core` first, then `lifecycle_manager_nav`, then `lifecycle_manager_optional` (when present). A sequencer (`ManagerSequencer`, pure logic, run by the thin node `lifecycle_sequencer`) shall start each manager only when the previous one reports `is_active` true, or has been declared failed after `manager_startup_timeout_s` (default 30 s). | unit + launch_testing |
| 005-R36 | If a node of a manager fails to activate, the sequencer shall apply these rules and publish the state and a `Notification` on `/myRobot/notifications`: core failure: `lifecycle_manager_nav` and the optional manager are not started (state FAILED_CORE, ERROR `CORE_START_FAILED`); nav failure (the manager or any of `planner_server`, `controller_server`, `bt_navigator`): the core stays active, the nav nodes are left inactive, `cover_room` and `explore` goals are rejected, and no automatic motion exists (state DEGRADED_NAV, WARN `NAV_UNAVAILABLE`); optional failure: core and nav are unaffected (state DEGRADED_OPTIONAL, Art. VII). No node drives while a dependency is not active: `controller_server` is active only when `planner_server` and `bt_navigator` are, and `coverage_server` rejects goals while `/navigate_to_pose` is not available. Teleoperation and the stop lock stay available in every state. | unit + launch_testing |
| 005-R37 | On shutdown, the sequencer shall stop the managers in the reverse order (optional, nav, core), so that `controller_server` stops publishing before the supervisor and `stop_guard`, and `stop_guard` is the last to go down; a failure in one stop shall not prevent the next, and the sequence shall end within `manager_shutdown_timeout_s` (default 15 s). | unit + launch_testing |
| 005-R38 | A `launch_testing` test shall bring up all managers (the optional one with a dummy lifecycle node) and assert that every node of every manager reaches `active` deterministically (the test repeats the start 5 times without a flake); that, for each manager in turn with one node made to fail its activation, the state of 005-R36 is reached and no non-zero command appears on `/myRobot/cmd_vel`; and that the shutdown order of 005-R37 is observed. Decided Q-09 condition: the synchronization among all nodes is guaranteed (project owner, 2026-11-13). | launch_testing |

## Definition of done

- Unit and node tests of the requirements listed in 005-R29 passing on both distributions in CI.
- Simulator measurements recorded (R07, R15, R24, R25) in the report, with the parameter values used. The values marked "initial" are tuned against them.
- Scenarios 1 to 3 recorded in a short video.
- The stubs remain in the repository as `with_stubs:=true` launch options (Art. III.5) and the contract tests pass with either.

## Open questions (proposed, to be moved into spec 000 section 6)

- Q-09 (**DECIDED, CONDITIONAL**, project owner, 2026-11-13): a third lifecycle manager, `lifecycle_manager_nav`, is allowed, "contanto que a sincronia entre todos os nós seja garantida" (provided the synchronization among all nodes is guaranteed). It amends D-09 (decision of spec 000, not an article); the amendment is applied later by an architect change to spec 000. The condition is made testable by 005-R35 to R38.
- The Nav2 spike (T510) deadline is 2026-11-20 (decided, project owner, 2026-11-13). If it is not working by then, escalate to the project owner, who decides the coverage scope cut; no alternative is pre-approved.
- Q-10: `/myRobot/mux/coverage` becomes `reserved` without an owner in the contract change, applied together with D-24 (decided); the test tolerance for a topic with no publisher and a `twist_mux` subscriber is confirmed in T005/T022.
- Q-11: source of the bumper signal. There is no bumper topic in the contract and the model `models/myRobot.ttm` may or may not have a bumper sensor (to be checked by T503). Without a source, `BUMPER` stays disabled.
- Q-12 (**DECIDED**, project owner, 2026-11-13): when `cover_room` ends, successfully or not, the supervisor asks the IA368II orchestrator for autodocking and the robot returns to the base, without extending the `Mode` enum (Art. II). The mechanism is confirmed in T520 (candidate, unverified: `/myRobot/start_stop` with `"stop"`). The transition rows stay proposals for plan 001 (005-R30), which this change does not edit.
- Timeouts in this spec (R12, R20, R32, window of R01) use the node clock: wall time unless Q-13 (spec 003) sets `use_sim_time`.
- Note on `enable_stamped_cmd_vel` (R23): the parameter may not exist in Nav2 1.3.x (Jazzy); declaring it is harmless and the config test accepts both (unverified, T510 records it).
- Contract change order: the changes of spec 003 are applied first, then those of spec 005 (005 consumes the room messages); one serialized change each.
