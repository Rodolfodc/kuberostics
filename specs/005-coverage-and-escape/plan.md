# 005: Technical plan

## Where the code goes

The three stubs live in `src/vacuum_stubs/` (pure `*_core.py` plus a `*_node.py` each). The real implementations go into a new package, `src/vacuum_coverage/` (ament_python), so the stubs stay as the swappable alternative (Art. III.5, Art. IV):

```
src/vacuum_coverage/
├── vacuum_coverage/
│   ├── core/                       # pure Python, no rclpy imports (D-18, D-19)
│   │   ├── stuck_core.py           # StuckCore
│   │   ├── escape_script.py        # EscapeScript
│   │   └── coverage_planner.py     # CoveragePlanner, CoverageTracker, RegionBuilder
│   │                               # GridView comes from vacuum_mapping.core.grid (spec 003; local copy until T309 of spec 003 is done)
│   ├── stuck_detector_node.py      # thin LifecycleNode
│   ├── escape_server_node.py       # thin LifecycleNode (actuator: mux/escape only)
│   └── coverage_server_node.py     # thin LifecycleNode (action server + Nav2 client)
├── launch/{stuck_escape.launch.py, coverage.launch.py}
├── config/{stuck_escape.yaml, coverage.yaml}
└── test/
src/vacuum_bringup/config/{nav2_params.yaml, nav_bt.xml, twist_mux.yaml (nav input)}
src/vacuum_bringup/launch/nav.launch.py
```

The skeleton launch gets the argument `with_stubs` (default `false` once T511 passes; `true` reproduces spec 001). The real nodes keep the stub node names, so `lifecycle_core.yaml` does not change.

## Toolbox classes (pure Python, D-19)

Each class only computes; the caller decides what to do with the result and when to call.

| Class | Input | Output | Notes |
|---|---|---|---|
| `StuckCore` | `update(t, cmd_lin, cmd_ang, pose_xy_yaw)` | `None` or `cause` | Sliding window; thresholds in the constructor; logger/metrics callback injected (D-20). Does not read topics, does not publish. |
| `EscapeScript` | `start(event_ranges, pose)`, then `step(t, pose, front_range, rear_range) -> Command(linear, angular) or Result` | the next velocity or the final outcome | Finite state machine: `REVERSE -> TURN -> CHECK -> (next attempt) -> FALLBACK_SWEEP -> FALLBACK_ADVANCE -> DONE`. Owns no clock: the node passes `t` (testable with a simulated clock). Timeout is the caller's job (005-R12). |
| `GridView` | `OccupancyGrid` arrays + metadata | `world_to_cell`, `cell_to_world`, `inflate(radius)`, `connected_component(cell)` | Shared by the planner and the tracker. |
| `RegionBuilder` | grid, room labels (optional), `room_id`, robot position | boolean mask of the region | Empty room: component connected to the robot, capped by `max_region_area_m2`. |
| `CoveragePlanner` | region mask, inflated grid, `lane_spacing_m`, `passes` | ordered list of waypoints `(x, y, yaw)` | Boustrophedon. Deterministic. Lane direction along the room's longer axis (pass 1), rotated 90 degrees on the next pass. Waypoints are the two ends of every free lane segment; no intermediate points (Nav2 plans between them). |
| `CoverageTracker` | pose stream, mask, `tool_radius_m` | `covered_ratio`, covered mask | Marks cells within the tool radius. |

Why Nav2 between waypoints and not an own follower: Nav2 already gives the global path (`planner_server`) and avoidance (local costmap + controller). Writing a follower plus costmap is the larger risk with 3 weeks left (Art. IX.1).

## Nodes

| Node | Type | Role | Publishes | Consumes |
|---|---|---|---|---|
| `stuck_detector` | LifecycleNode, service | `StuckCore` glue | `/myRobot/stuck`, telemetry | `/myRobot/cmd_vel`, `odom_topic`, `/scan`, TF, optional `bumper_topic`; service `~/trigger` |
| `escape_server` | LifecycleNode, actuator | `EscapeScript` glue; action `/myRobot/escape` | `/myRobot/mux/escape` (10 Hz+ while running), telemetry | `odom_topic`, `/scan` |
| `coverage_server` | LifecycleNode, service | action `/myRobot/cover_room`; `NavigateToPose` client | `/myRobot/coverage/path`, `/myRobot/coverage/covered`, telemetry; no `Twist` | `/map`, `/myRobot/map/room_labels`, `/myRobot/map/rooms` (optional until 003), TF |
| `planner_server`, `controller_server`, `bt_navigator` | Nav2 (third party, Art. II: configuration only) | global path, local avoidance, `NavigateToPose` | `controller_server`: `/myRobot/mux/nav` (remapped from `cmd_vel`) | `/map`, `/scan`, TF |

Nav2 minimal configuration (initial values, to tune in the simulator):

- Planner `nav2_navfn_planner::NavfnPlanner`; controller `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`, `desired_linear_vel: 0.25` (below the 0.30 m/s limit of spec 009a), `robot_radius: 0.16`, inflation 0.30 m. Plugin names unverified here: T510 checks them on both distros.
- `bt_navigator` with an own tree `nav_bt.xml`: `ComputePathToPose` + `FollowPath`, replanning at 1 Hz, **no recovery nodes**. `behavior_server`, `velocity_smoother` and `collision_monitor` are not launched, so `controller_server` is the only Nav2 node that publishes velocity.
- `enable_stamped_cmd_vel: false` on `controller_server` (and on any other velocity publisher; D-13). A config test checks it (005-R23).
- `use_sim_time` follows the outcome of Q-13 of spec 003.
- Costmaps: local costmap `obstacle_layer` on `/scan`; global costmap `static_layer` on `/map` + inflation.

## Escape: timeout and fallback (Art. VI.3)

```
supervisor sends Escape(timeout_s)
  attempt 1..max_attempts:  REVERSE -> TURN(toward the freer side) -> CHECK front clearance
        fail -> next attempt with a larger turn
  fallback (once):          SWEEP 360 -> face largest range -> ADVANCE -> CHECK
  timeout / cancel / fallback failed -> stop publishing on mux/escape, result success=false
supervisor: success=false or its escape_timeout_s (20 s) -> lock_stop + ESCAPE_FAILED notification (001-R09)
```

`default_timeout_s` (15 s) is below the supervisor's 20 s, so the server normally fails cleanly before the supervisor's clock does. While the server is not publishing, `twist_mux` times out the `escape` input after 0.5 s, and the lower inputs take over; after a failed escape the supervisor locks the robot, and `stop_guard` keeps it at zero (D-22).

## Supervisor: new rows (proposal, Q-12; ros-dev edits `SubmodeMachine`, T512)

The table of plan 001 has no row for the result of `cover_room`. Proposed additions, to be merged into that table by the architect when accepted:

| Current state | Event | Action | Next |
|---|---|---|---|
| COVERING | `cover_room` succeeded, mode still CLEANING | INFO `Notification` `COVERAGE_DONE`; sends `cover_room` again (next pass) | COVERING |
| COVERING | `cover_room` failed, first failure | WARN `Notification` `COVERAGE_FAILED`; sends `cover_room` once more | COVERING |
| COVERING | `cover_room` failed, second consecutive failure | ERROR `Notification` `COVERAGE_FAILED`; no new goal | NONE |

The failure counter resets on success. NONE with mode CLEANING after the second failure would send `cover_room` again under the 001 table; the machine therefore keeps a `coverage_blocked` flag, cleared when the mode leaves CLEANING. If the user rejects Q-12, the machine keeps the 001 behaviour and 005-R30 is dropped.

## Dependencies on spec 003

005 needs from 003: `/map`, TF `map -> odom -> base_link` and `/scan` frame, and later the room topics. Until then the module runs on a saved map from `map_server` with a static `map -> odom` identity (valid for the simulator ground-truth odometry, see spec 003) so that 005 is not blocked. `room_id` resolution (005-R16, non-empty case) depends on message `Room`/`RoomArray` from T303.

## Proposed contract changes (NOT applied; the architect applies them in one serialized change, after PRs #9 and #10)

`topics.yaml`:

```yaml
topics:
  # already declared, no change: /myRobot/mux/nav (owner controller_server, priority 60, since "005")
  - {name: /myRobot/coverage/path,    type: nav_msgs/msg/Path,          qos: default, owner: coverage_server, since: "005", note: "lane waypoints of the current pass, for the interface (spec 007); published once per pass"}
  - {name: /myRobot/coverage/covered, type: nav_msgs/msg/OccupancyGrid, qos: latched, owner: coverage_server, since: "005", note: "100 = covered, 0 = not covered, -1 = outside the region; 1 Hz"}
  # Q-10 (proposed): /myRobot/mux/coverage gets "reserved: true" and loses its owner when the stub is retired

  # consumed Nav2 interfaces, declared so the contract tests see them (owner = third party, frozen: true)
actions:
  - {name: /navigate_to_pose, type: nav2_msgs/action/NavigateToPose, owner: bt_navigator, frozen: true, since: "005"}

lifecycle:
  lifecycle_manager_nav: [planner_server, controller_server, bt_navigator]   # Q-09 (proposed; amends D-09)
  lifecycle_manager_core: [supervisor, stop_guard, stuck_detector, escape_server, coverage_server]   # unchanged

parameters:
  stuck_detector:
    cmd_linear_min_m_s: {type: double, default: 0.05, requirement: 005-R01}
    cmd_angular_min_rad_s: {type: double, default: 0.20, requirement: 005-R01}
    window_s: {type: double, default: 3.0, requirement: 005-R01}
    min_translation_m: {type: double, default: 0.03, requirement: 005-R01}
    min_rotation_rad: {type: double, default: 0.10, requirement: 005-R01}
    cooldown_s: {type: double, default: 5.0, requirement: 005-R03}
    odom_topic: {type: string, default: /myrobot/odom, note: "D-07"}
    scan_topic: {type: string, default: /scan}
    bumper_topic: {type: string, default: "", requirement: 005-R05, note: "empty = BUMPER disabled (Q-11)"}
  escape_server:
    outcome: {type: string, default: auto, values: [auto, succeed, fail, hang], requirement: 005-R14, note: "extends the 001 values; stub default stays succeed"}
    reverse_m: {type: double, default: 0.20}
    rear_clearance_m: {type: double, default: 0.15}
    turn_deg_base: {type: double, default: 90.0}
    turn_deg_step: {type: double, default: 45.0}
    front_clearance_m: {type: double, default: 0.40}
    max_attempts: {type: int, default: 3}
    fallback_forward_m: {type: double, default: 0.30}
    default_timeout_s: {type: double, default: 15.0, requirement: 005-R12}
    odom_topic: {type: string, default: /myrobot/odom}
    scan_topic: {type: string, default: /scan}
  coverage_server:
    lane_spacing_m: {type: double, default: 0.25, requirement: 005-R17}
    robot_radius_m: {type: double, default: 0.16}
    margin_m: {type: double, default: 0.05}
    min_segment_m: {type: double, default: 0.30}
    tool_radius_m: {type: double, default: 0.15}
    target_ratio: {type: double, default: 0.90, requirement: 005-R21}
    max_skip_ratio: {type: double, default: 0.3, requirement: 005-R20}
    coverage_timeout_s: {type: double, default: 1800.0, requirement: 005-R20}
    max_region_area_m2: {type: double, default: 150.0, requirement: 005-R16}
    map_topic: {type: string, default: /map}
    nav_action: {type: string, default: /navigate_to_pose}
```

`interfaces.md`: no new `.msg`, `.srv` or `.action` in this spec. `CoverRoom`, `Escape` and `StuckEvent` are unchanged (`BUMPER` stays reserved). The room messages are defined by spec 003 (`Room`, `RoomArray`).

`twist_mux.yaml` (ros-dev, T501, not a contract file): add `nav: {topic: /myRobot/mux/nav, timeout: 0.5, priority: 60}`.

Diagrams: when the proposal is applied, update `docs/diagrams/a2-control-and-motion.mmd` (add `controller_server -> mux/nav`; `coverage_server -> NavigateToPose`) and re-render with `scripts/render-diagrams.sh`. Not touched in this change because the contract is not changing yet.

## Tests

- Unit (pytest, no ROS): `StuckCore`, `EscapeScript` (simulated clock; every state and the fallback), `CoveragePlanner`, `CoverageTracker`, `RegionBuilder`, each requirement id in the test name.
- Node tests (rclpy, localhost discovery and random domain, as in T020): thin nodes against fakes.
- `launch_testing` (Art. V.2): `stuck_escape.launch.py` with fake odometry and scan (wedge -> event -> escape -> success; fake never frees -> `success=false` before the timeout); `coverage.launch.py` with `map_server` + fake `NavigateToPose` action server (lane order, skip and retry, cancel).
- Config test: every Nav2 node that publishes velocity declares `enable_stamped_cmd_vel: false`; `twist_mux.yaml` priorities `stop 250 > lock 200 > teleop 100 > escape 80 > nav 60 > coverage 50 > docking 40`.
- Simulator (local, marked, Art. V.3): R07, R15, R24, R25.

## Risks

- Nav2 plugin names or parameter names differ between Nav2 1.3.14 (Jazzy) and 1.5.3 (Lyrical): config test on both distros before any tuning (T510).
- Nav2 without recovery behaviours can loop on a blocked waypoint: bounded by R20 (retry once, then skip) and by the stuck detector.
- `NO_PROGRESS` false positives when Nav2 rotates in place against a wall: handled by the yaw criterion (R01); tune in T514.
- Laser field of view: rear clearance (R08) needs a scan that covers the rear; if the laser is front-only the check is skipped (T503 measures it).
- Ground-truth odometry (spec 003) makes stuck detection ideal; with a noisy source the thresholds would need retuning.
- Time: coverage depends on 003 (map). Mitigation: saved map + static TF, see "Dependencies on spec 003".
