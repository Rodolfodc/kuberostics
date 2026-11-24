# vacuum_coverage

Real coverage of spec 005. Contains the pure-Python cores only (no ROS imports, D-18/D-19):

| Class | File | Role |
|---|---|---|
| `RegionBuilder` | `core/region_builder.py` | which cells a `CoverRoom` goal cleans (005-R16) |
| `CoveragePlanner` | `core/coverage_planner.py` | boustrophedon waypoints (005-R17) |
| `CoverageTracker` | `core/coverage_tracker.py` | covered mask and `covered_ratio` (005-R21) |
| `CoverageSession`, `ResumeStore` | `core/coverage_session.py` | retry-once-then-skip, timeouts, early success, resume (005-R20, R21, R32; `target_ratio` 0.95, degraded success from `min_acceptable_ratio` 0.70) |

Grid code is reused from `vacuum_mapping` (`Raster`, `erode`); `core/grid_ops.py` adds `dilate` and the
reachable/coverable masks. A shared `vacuum_grid` package is planned by T309 of spec 003.

| `CoverageRun` | `core/coverage_run.py` | one goal from region to result: builds region, lanes, tracker and session, keeps the resume store, validates the configuration (005-R16, R21, R32) |

## `coverage_server` node (T511, T513)

`coverage_server` (`vacuum_coverage.coverage_server_node`, executable `coverage_server`) is the real node behind
`/myRobot/cover_room`. It sends each lane waypoint to Nav2 as a `NavigateToPose` goal and **publishes no
velocity** (D-24, 005-R19): Nav2's `controller_server` drives on `/myRobot/mux/nav`, and `/myRobot/mux/coverage`
stays reserved for the stub. It publishes `/myRobot/coverage/path`, `/myRobot/coverage/covered` and
`/myRobot/telemetry/coverage_server`.

- Parameters: exactly the `coverage_server` block of `contracts/topics.yaml` (the "stub only" ones are not declared).
  Robot radius, tool radius, lane spacing and `margin_m` are placeholders until the robot profile exists; the
  speeds belong to the Nav2 controller configuration, not to this node.
- Constants, not parameters (the contract lists none yet): TF frames `map` and `base_link`.
- Goals are rejected while there is no `/map`, no TF, no `/navigate_to_pose` server (005-R36), an unknown
  `room_id` (the room topics wait for the Room messages of spec 003, so any non-empty `room_id` is unknown for now)
  or another goal is running. `configure` and `activate` never wait for Nav2 (core starts before nav, 005-R35).
- Result: success at `covered_ratio >= 0.95`; degraded success (`success=true`, `covered_ratio` below 0.95) from
  0.70; failure below that, on too many skipped waypoints or on `coverage_timeout_s`.
- Run alone: `ros2 launch vacuum_coverage coverage.launch.py` (autostart configure + activate); needs `/map`,
  TF `map -> base_link` and a `NavigateToPose` server (the tests use fakes).
- Tests: `test_coverage_run.py` (pure), `test_coverage_server_node.py` (in-process rclpy with a fake Nav2) and
  `test_coverage_launch.py` (launch_testing). ROS_DOMAIN_ID is drawn from 215-232 by `test/conftest.py`
  (the range where the default DDS port mapping avoids the Linux ephemeral ports; other packages use 10-189).

Test: `cd src/vacuum_coverage && PYTHONPATH=../vacuum_mapping python3 -m pytest test`.
