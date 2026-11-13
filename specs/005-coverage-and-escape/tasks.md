# 005: Tasks

`[P]` = can be done in parallel with other `[P]` tasks of the same phase. Each task cites the requirements it meets.
A task is only done when the test that verifies the requirement passes. Task IDs T3xx belong to spec 003.

Pre-condition (architect, not a task of the four agents): apply the "Proposed contract changes" of `plan.md` to `contracts/topics.yaml` in one serialized change, after PRs #9 and #10 are merged. Code tasks that add an interface (T510, T511) start after it.

Order of delivery (Art. IX.1): stuck detection and escape first (T504, T505, T508, T509, T514, T515), because spec 008 needs the baseline; coverage second.

## Phase 0: preparation

- [ ] **T501** [P] Add the `nav` input (`/myRobot/mux/nav`, timeout 0.5 s, priority 60) to `twist_mux.yaml` and extend the priority-order config test: `stop 250 > lock 200 > teleop 100 > escape 80 > nav 60 > coverage 50 > docking 40`. Owner: ros-dev. (005-R23)
- [ ] **T502** [P] Check that rosdep resolves `navigation2`, `nav2_bringup`, `nav2_map_server`, `nav2_regulated_pure_pursuit_controller` and `nav2_navfn_planner` on both images (`osrf/ros:jazzy-desktop-full`, `ros:lyrical-ros-base`) and that the CI time with them stays acceptable; update `scripts/test-docker.sh` and the CI cache if needed. Owner: devops. Depends on T003. (005-R29)
- [ ] **T503** [P] Measure in the House scene with the robot model and record in `.sim/`: laser field of view, range, frame and mounting offset; robot footprint radius; maximum wheel-bridge speeds and its command timeout (leftover of Q-08); whether `models/myRobot.ttm` has a bumper sensor and its handle (Q-11); cleaning tool width. Owner: sim-operator. (005-R01, R08, R17, R21; Q-11)

## Phase 1: tests first

- [ ] **T504** [P] pytest tests of `StuckCore`: motion above the thresholds with no displacement -> event after `window_s`; pure rotation with yaw change -> no event; zero command and zero `Twist` of the stop lock -> no event and window reset; cooldown; simulated clock. Owner: ros-dev. (005-R01 to R03)
- [ ] **T505** [P] pytest tests of `EscapeScript`: every state, turn toward the freer side with the left tie-break, determinism (same input, same output), growing turn per attempt, fallback sweep, result after `max_attempts`; simulated clock. Owner: ros-dev. (005-R08 to R11)
- [ ] **T506** [P] pytest tests of `CoveragePlanner`, `RegionBuilder` and `CoverageTracker` on synthetic grids: empty rectangle, central obstacle, two rooms joined by a door, empty `room_id` with a region capped by `max_region_area_m2`, simulated pass >= 95 % coverage, second pass rotated 90 degrees, determinism. Owner: ros-dev. (005-R16 to R18, R21)
- [ ] **T507** [P] pytest tests of `SubmodeMachine` for the three new rows of the plan (success -> next pass, first failure -> retry, second failure -> NONE with `coverage_blocked`). Skip if Q-12 is rejected. Owner: ros-dev. (005-R30)

## Phase 2: implementation

- [ ] **T508** Implement `StuckCore` until T504 passes; `stuck_detector` real `LifecycleNode` (command from `/myRobot/cmd_vel`, odometry, TF pose, last scan, optional bumper, `~/trigger` hook kept, telemetry); stub kept behind `with_stubs`. Owner: ros-dev. Depends on T504. (005-R01 to R06, R26, R27)
- [ ] **T509** Implement `EscapeScript` until T505 passes; `escape_server` real `LifecycleNode` (action goal, `mux/escape` only, timeout, cancel, feedback 2 Hz, `outcome` parameter with `auto`, `strategy: "scripted"`). Owner: ros-dev. Depends on T505. (005-R08 to R14, R26, R27)
- [ ] **T510** Nav2 configuration and launch: `nav2_params.yaml`, `nav_bt.xml` (no recovery nodes), `nav.launch.py` with `planner_server`, `controller_server` (remap `cmd_vel` -> `/myRobot/mux/nav`), `bt_navigator`, `lifecycle_manager_nav` (Q-09); `enable_stamped_cmd_vel: false` declared; check the plugin and parameter names on Nav2 1.3.14 and 1.5.3 and record differences. Config test for R23. Owner: ros-dev. Depends on T501, T502 and the contract change. (005-R19, R23, R29)
- [ ] **T511** Implement `RegionBuilder`, `CoveragePlanner`, `CoverageTracker` until T506 passes; `coverage_server` real `LifecycleNode`: `CoverRoom` server, `NavigateToPose` client with retry-once-then-skip, feedback at 1 Hz, cancel within 0.5 s, `coverage/path` and `coverage/covered`, room topics optional (empty `room_id` path first, non-empty after T303 of spec 003). `GridView` comes from T309 of spec 003 (a local copy until it is done). Owner: ros-dev. Depends on T506, T510. (005-R16 to R22, R26, R27)
- [ ] **T512** Implement the new `SubmodeMachine` rows and their wiring in the `supervisor` (result handler of `cover_room`, `coverage_blocked`, notifications), until T507 passes. Owner: ros-dev. Depends on T507 and the user's decision on Q-12. (005-R30)
- [ ] **T513** Launch files `stuck_escape.launch.py` and `coverage.launch.py`, `with_stubs` argument in `skeleton.launch.py`, and the `launch_testing` tests described in the plan (fake odometry and scan; `map_server` + fake `NavigateToPose` server). Isolation as in T014/T020 (localhost discovery, random domain). Owner: ros-dev. Depends on T508, T509, T511. (005-R28, R26)

## Phase 3: verification

- [ ] **T514** Wedge and false-positive runs in CoppeliaSim: robot against a wall with constant forward command (time to event) and a coverage run in free space (events must be zero); tune `window_s`, `min_translation_m`, `min_rotation_rad`; record the values. Owner: sim-operator. Depends on T508 and T503. (005-R07)
- [ ] **T515** Escape success rate: at least 20 stuck poses (corner, furniture leg, between two obstacles, against a wall), record success rate, median duration and failures; this is the baseline for spec 008. Owner: sim-operator. Depends on T509, T514. (005-R15)
- [ ] **T516** Avoidance test: box placed in a lane; verify the local costmap and the number of `StuckEvent`s. Owner: sim-operator. Depends on T510, T511 and T310/T311 of spec 003 (or a saved map). (005-R24)
- [ ] **T517** Full CLEANING cycle on the House scene (all rooms, one pass): `covered_ratio`, time, `StuckEvent` count, failed escapes; tune `lane_spacing_m`, `desired_linear_vel`, inflation; record the video of scenarios 1 to 3. Owner: sim-operator. Depends on T511, T512, T515 and T311, T313 of spec 003. (005-R25)
- [ ] **T518** CI: the new tests run on the full matrix `[jazzy, lyrical]`, with simulator tests excluded and marked; record the result. Owner: devops. Depends on T513, T003. (005-R29)
- [ ] **T519** Review spec 000: move D-24, D-25 and Q-09 to Q-12 into spec 000 after the user's decisions (architect, outside the four-agent list), then retire the stubs from the default launch. Owner: ros-dev (launch default only). Depends on T517. (005-R26)
