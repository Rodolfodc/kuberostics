# 003: Tasks

`[P]` = can be done in parallel with other `[P]` tasks of the same phase. Each task cites the requirements it meets.
A task is only done when the test that verifies the requirement passes. Task IDs T5xx belong to spec 005.

Pre-condition (architect, not a task of the four agents): apply the "Proposed contract changes" of `plan.md` to `contracts/topics.yaml` and `interfaces.md` in one serialized change, after PRs #9 and #10 are merged. T303 and every task that adds an interface start after it.

Order of delivery (Art. IX.1): odometry/TF and SLAM first (T301, T302, T310, T311), because spec 005 needs the map and the TF; then segmentation (T306, T313); exploration and labels last.

## Phase 0: preparation

- [ ] **T301** [P] Measure in the House scene with the robot model and record in `.sim/`: whether `/clock` is published and whether message stamps are simulation time (Q-13); frame, origin (world or start pose), rate and covariance of `/myrobot/odom` (Q-14); laser frame, field of view, range and mounting offset, camera frame and offset (values for 003-R03 and 003-R05); door widths and the number and extent of rooms (hand-labelled reference raster for 003-R24); a reference occupancy raster of the scene for 003-R08. Owner: sim-operator. (003-R03, R05, R08, R16, R24; Q-13, Q-14)
- [ ] **T302** [P] Read the 2025S2 odometry and bridge nodes in `src/deps/` (read-only) and report: topics, frames, whether `odom -> base_link` is broadcast, pose frame, time source. The architect closes Q-14 and Q-13 with the report. Owner: ros-dev. (003-R02; Q-13, Q-14)
- [ ] **T303** Add `Room`, `RoomArray`, `Explore` and `SetRoomLabel` to `vacuum_interfaces`, copied from `interfaces.md`; verify with `colcon build` and `ros2 interface show` on both distros. Owner: ros-dev. Depends on the contract change. (003-R12, R18, R20, R23)
- [ ] **T304** [P] Check that rosdep resolves `slam_toolbox`, `nav2_map_server`, `python3-scipy` (Q-16) on both images and that Python 3.14 on Lyrical imports `scipy.ndimage`; update `scripts/test-docker.sh` and CI if needed; report to the architect for Q-16. Owner: devops. Depends on T003. (003-R16, R29; Q-16)

## Phase 1: tests first

- [ ] **T305** [P] pytest tests of `GridView` (world/cell conversion, inflation) and `FrontierFinder`: empty room, door, unknown pockets smaller than `min_frontier_cells`, snap to a free cell with clearance, blacklist, determinism. Owner: ros-dev. (003-R09)
- [ ] **T306** [P] pytest tests of `RoomSegmenter` (two rooms joined by a 0.9 m opening, open rectangle, noise, no label on occupied or unknown cells, 300 x 300 timing) and `RoomTracker` (50 % overlap keeps id and label, new id otherwise). Owner: ros-dev. (003-R16, R17, R19, R28)
- [ ] **T307** [P] pytest tests of `RoomLabeler`: weighted vote, `min_votes`, ties, never overrides `label_source = user`. Owner: ros-dev. (003-R21)
- [ ] **T308** [P] pytest tests of `SubmodeMachine` for the EXPLORING rows of the plan (success, failure, stuck and resume, mode change, `exploration_done`, lock on). Owner: ros-dev. (003-R14)

## Phase 2: implementation

- [ ] **T309** Implement `GridView` and `FrontierFinder` until T305 passes. Owner: ros-dev. Depends on T305. (003-R09)
- [ ] **T310** Implement `odom_tf_bridge` (skipped if T302 shows the 2025S2 node already broadcasts the TF; then keep only the single-broadcaster check) and the static TF and `odom_source` handling of `mapping.launch.py`. Owner: ros-dev. Depends on T301, T302. (003-R01 to R03)
- [ ] **T311** `config/slam_toolbox.yaml` (explicit parameters), `mapping.launch.py` (SLAM, `map_file` mode with `map_server` and static `map -> odom`, `explored` handling), `/myRobot/map/save`; check and record whether a lifecycle variant of `slam_toolbox` exists on each distro (Q-15) and the `use_sim_time` setting (Q-13). Owner: ros-dev. Depends on T310, T301. (003-R05 to R07, R26)
- [ ] **T312** Implement `exploration_server` (`Explore` action, `NavigateToPose` client, retry, blacklist, timeout, cancel, `explored`, notifications, telemetry); node tests against a fake `NavigateToPose` server. Owner: ros-dev. Depends on T303, T309, T311 and T510 of spec 005 (for the live graph; the node tests use the fake). (003-R10 to R13, R26)
- [ ] **T313** Implement `RoomSegmenter` and `RoomTracker` until T306 passes; `room_segmenter` node (`room_labels`, `rooms`, services `segment` and `set_label`, persistence in `maps/<map_name>_rooms.yaml`). Owner: ros-dev. Depends on T303, T306. (003-R16 to R20, R23, R25, R26)
- [ ] **T314** Implement `RoomLabeler` until T307 passes, `config/room_rules.yaml` (placeholder classes) and the adapter for `observations_topic` in the format found by T903 of spec 009a. Owner: ros-dev. Depends on T307, T313, T903 of spec 009a; the class names depend on spec 004. (003-R21, R22)
- [ ] **T315** Implement the EXPLORING rows in `SubmodeMachine` until T308 passes and wire the `supervisor` (action client of `/myRobot/explore`, subscription to `explored`, parameter `explore_on_start`). Owner: ros-dev. Depends on T308, T312 and the contract change. (003-R14)
- [ ] **T316** `launch_testing` test of `mapping.launch.py` with `map_file` and fakes (rooms and labels topics appear; `set_label` works and persists), isolation as in T014/T020 of spec 001; add the three nodes to `lifecycle_core.yaml` and to the skeleton launch. Owner: ros-dev. Depends on T311, T313. (003-R26, R27)

## Phase 3: verification

- [ ] **T317** Map quality on the House scene: run SLAM while the robot is teleoperated along a fixed route, compare with the reference raster of T301, record median wall distance and reachable free area known; tune SLAM parameters. Owner: sim-operator. Depends on T311, T301. (003-R08)
- [ ] **T318** Autonomous exploration on the House scene: time, fraction of reachable free area, `StuckEvent`s, blacklisted frontiers; save the map and its rooms file and hand them to ros-dev for `maps/`. Owner: sim-operator. Depends on T312, T315, T510 of spec 005, T508 of spec 005 (a real stuck detector avoids stalls). (003-R07, R15)
- [ ] **T319** Segmentation against the hand-labelled reference: number of rooms and IoU per room; adjust `door_width_m` and `min_room_area_m2`; record the values. Owner: sim-operator. Depends on T313, T318 (or a map from T317). (003-R24)
- [ ] **T320** CI: the new tests run on the full matrix `[jazzy, lyrical]`, with simulator tests excluded and marked; record the result. Owner: devops. Depends on T316, T003. (003-R29)
- [ ] **T321** [P] Cut candidate. Add `rf2o_laser_odometry` to `deps.repos` pinned by commit and build it in both containers; record whether it builds on Lyrical. Owner: devops. Depends on T003. (003-R04)
- [ ] **T322** Cut candidate. `odom_source:=laser` launch switch in `mapping.launch.py` (rf2o instead of `odom_tf_bridge`, nothing else changes). Owner: ros-dev. Depends on T321, T311. (003-R04)
- [ ] **T323** Cut candidate. Compare `odom_source:=laser` with `sim` in the simulator (drift and map quality, same route as T317). Owner: sim-operator. Depends on T322, T317. (003-R04)
