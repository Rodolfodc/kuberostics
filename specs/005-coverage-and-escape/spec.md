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

## Decisions proposed by this spec (to be moved into spec 000 by the architect)

| ID | Proposal | Rationale |
|---|---|---|
| D-24 (proposed) | Coverage motion goes through Nav2: `coverage_server` plans lanes and sends `NavigateToPose` goals; Nav2 `controller_server` is the actuator and publishes on `/myRobot/mux/nav` (priority 60, already declared in `topics.yaml`). `coverage_server` publishes no `Twist`. `/myRobot/mux/coverage` stays declared for the stub and is retired when the real node replaces it (Q-10). | Reuses the global planner and the local costmap for avoidance instead of writing a follower. Art. III.3 and III.4 hold: the actuator is `controller_server`; `coverage_server` only computes and sends goals. |
| D-25 (proposed) | Escape priority (80) stays above navigation (60) and coverage (50). The supervisor cancels `cover_room` before sending `escape` (001-R07), so only one of them is active; the priorities are the safety net. | Art. VI.1, D-03. |

## Usage scenarios

1. **Clean a room.** The mode becomes CLEANING, the supervisor publishes COVERING and sends `cover_room`. `coverage_server` builds the lanes, drives Nav2 lane by lane and reports `covered_ratio`. At 90 % (or when the lanes are exhausted) the goal succeeds.
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
| 005-R08 | When it accepts an `Escape` goal, the server shall run the script `EscapeScript`, attempt by attempt: (1) back up up to `reverse_m` (default 0.20 m) measured on the odometry, stopping early if the scan covers the rear and the rear range falls below `rear_clearance_m` (default 0.15); (2) turn in place toward the side with the larger mean range in `event.scan_ranges` by `turn_deg_base + attempt * turn_deg_step` (defaults 90 and 45); (3) check front clearance. | unit (`EscapeScript`) |
| 005-R09 | The script shall be deterministic: the same `StuckEvent` and the same sensor sequence produce the same command sequence (ties between left and right go to the left). | unit |
| 005-R10 | When after the turn the front range is `>= front_clearance_m` (default 0.40) the attempt succeeds and the goal returns `success=true`; otherwise it runs the next attempt, up to `max_attempts` (default 3). | unit |
| 005-R11 | After `max_attempts` failed attempts, the server shall run the fallback once (a deterministic sweep: rotate 360 degrees in place recording the scan, face the heading of the largest range, advance `fallback_forward_m`, default 0.30) and judge it by the same front clearance. If it fails, the goal returns `success=false`. | unit |
| 005-R12 | The server shall finish the goal within `goal.timeout_s` (when 0, parameter `default_timeout_s`, default 15 s, below the supervisor's `escape_timeout_s` of 20 s). On timeout, cancel or failure it shall stop publishing on `mux/escape` within 0.2 s and return `success=false`. | unit + node |
| 005-R13 | The result shall carry `duration_s` and `strategy = "scripted"`; the feedback `elapsed_s` shall be published at 2 Hz or faster. | node |
| 005-R14 | The parameter `outcome` shall keep the values `succeed`, `fail`, `hang` (stub behaviour, 001-R16) and add `auto` (default of the real node: the script above). | node |
| 005-R15 | Needs simulator: from at least 20 distinct stuck poses (corner, furniture leg, between two obstacles, against a wall), the success rate and median duration of the script shall be recorded. Initial target: success rate `>= 0.8` (to be adjusted after the first run); this is the baseline of spec 008. | sim (local) |

Coverage and avoidance (`coverage_server` plans; Nav2 moves the robot).

| ID | Requirement | Verification |
|---|---|---|
| 005-R16 | When it accepts a `CoverRoom` goal, the server shall build the region to clean: if `room_id` is not empty, the cells of that room in `/myRobot/map/room_labels`; if it is empty, or no room topic has been received, the free cells connected to the robot position, limited to `max_region_area_m2` (default 150). An unknown non-empty `room_id` shall be rejected. | unit + node |
| 005-R17 | `CoveragePlanner` shall produce waypoints in boustrophedon order (parallel lanes of spacing `lane_spacing_m`, default 0.25, alternating direction) over the region free of obstacles inflated by `robot_radius_m` (default 0.16) + `margin_m` (default 0.05), skipping lane segments shorter than `min_segment_m` (default 0.30). With `passes > 1`, each extra pass rotates the lane direction by 90 degrees. The output is deterministic. | unit |
| 005-R18 | On synthetic maps (empty rectangle, rectangle with a central obstacle, two rooms), simulating the robot passing through every waypoint with `tool_radius_m` (default 0.15) shall cover at least 95 % of the coverable cells. | unit |
| 005-R19 | The server shall send each waypoint as a `NavigateToPose` goal (`/navigate_to_pose`) in order and shall publish no velocity. Global path and obstacle avoidance are done by Nav2 (`planner_server`, `controller_server` with local costmap fed by `/scan`). | node + sim |
| 005-R20 | If Nav2 aborts a waypoint, the server shall retry it once and then skip it, incrementing `skipped_waypoints`; if skipped waypoints exceed `max_skip_ratio` (default 0.3) of the total, or the goal exceeds `coverage_timeout_s` (default 1800 s), the goal returns `success=false`. | unit |
| 005-R21 | The server shall mark as covered the cells within `tool_radius_m` of the robot pose (TF) and publish `covered_ratio = covered / coverable` as feedback at 1 Hz or faster. The goal succeeds when `covered_ratio >= target_ratio` (default 0.90) after the requested `passes`, or when the waypoints are exhausted with `skipped_waypoints` within the limit of 005-R20. | unit + node |
| 005-R22 | When the goal is cancelled, the server shall cancel the ongoing Nav2 goal within 0.5 s and return. | node |
| 005-R23 | Nav2 shall publish velocity only through `controller_server`, remapped to `/myRobot/mux/nav`; every Nav2 node that publishes velocity shall declare `enable_stamped_cmd_vel: false` explicitly (D-13, Art. X.3); `twist_mux` shall have the `nav` input with priority 60 and timeout 0.5 s, between `escape` (80) and `coverage` (50). | config test + contract |
| 005-R24 | Needs simulator: with a box placed in a lane, the robot shall reach the following waypoint with at most one `StuckEvent` and no collision recorded by the bridge; the box shall appear in the local costmap. | sim (local) |
| 005-R25 | Needs simulator: on the *House* scene, a full CLEANING cycle (all rooms, one pass) shall reach `covered_ratio >= 0.85` of the reachable area; the time, the number of `StuckEvent`s and the number of failed escapes shall be recorded for the report. The 0.85 target is initial. | sim (local) |

Cross-cutting.

| ID | Requirement | Verification |
|---|---|---|
| 005-R26 | The real nodes shall keep the names `coverage_server`, `escape_server` and `stuck_detector`, the actions `/myRobot/cover_room` and `/myRobot/escape`, the topic `/myRobot/stuck` and the telemetry topics of spec 001, and shall pass the contract tests of spec 001 without changes (Art. IV.2). | contract (T005/T022) |
| 005-R27 | Each node shall be a `LifecycleNode` managed by the lifecycle manager (Q-09 for the Nav2 nodes), shall log every state transition and event and shall publish telemetry at 1 Hz on its `/myRobot/telemetry/<node>` (D-20). The logic shall live in pure Python classes without ROS imports (D-18, D-19). | unit + launch_testing |
| 005-R28 | Each module shall have its own launch file that runs it alone with fakes (Art. III.1) and a `launch_testing` test: `stuck_detector` + `escape_server` with fake odometry and scan; `coverage_server` with a static map and a fake `NavigateToPose` server. | launch_testing |
| 005-R29 | Requirements 005-R01 to 005-R06, 005-R08 to 005-R14, 005-R16 to 005-R23 and 005-R26 to 005-R28 shall hold on Jazzy and on Lyrical, with the same code and configuration. | CI (matrix) |
| 005-R30 | When `cover_room` ends, the supervisor shall behave as defined in the plan, section "Supervisor: new rows" (proposal, Q-12): success sends the next pass while the mode stays CLEANING; failure notifies and, after one retry, stops. | unit (`SubmodeMachine`) |

## Definition of done

- Unit and node tests of 005-R01 to 005-R06, R08 to R14, R16 to R23, R26 to R28 passing on both distributions in CI.
- Simulator measurements recorded (R07, R15, R24, R25) in the report, with the parameter values used. The values marked "initial" are tuned against them.
- Scenarios 1 to 3 recorded in a short video.
- The stubs remain in the repository as `with_stubs:=true` launch options (Art. III.5) and the contract tests pass with either.

## Open questions (proposed, to be moved into spec 000 section 6)

- Q-09: which lifecycle manager owns the Nav2 nodes (`planner_server`, `controller_server`, `bt_navigator`)? D-09 lists two of our own managers. Proposal: a third, `lifecycle_manager_nav`, so a Nav2 failure does not block `stop_guard` and the supervisor. This amends D-09 (decision of spec 000, not an article).
- Q-10: retire `/myRobot/mux/coverage` (owner `coverage_server`) once the real node replaces the stub? It would have no publisher. The contract test counts a topic with a subscriber (`twist_mux`) as existing; to confirm in T005/T022.
- Q-11: source of the bumper signal. There is no bumper topic in the contract and the model `models/myRobot.ttm` may or may not have a bumper sensor (to be checked by T503). Without a source, `BUMPER` stays disabled.
- Q-12: policy after `cover_room` ends. Proposal in the plan; the user decides.
