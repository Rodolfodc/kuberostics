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
│   │                               # GridView comes from the tiny shared package vacuum_grid (no ROS, no other dependency; created by T309 of spec 003)
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

Shutdown and failure handling (005-R31, R34): on `on_deactivate` and `on_shutdown`, `coverage_server` cancels its Nav2 goal first and then releases the action; if it crashes, the Nav2 goal is NOT cancelled and `controller_server` keeps publishing fresh velocity, so the 0.5 s `twist_mux` input timeout does not help. The supervisor therefore watches the action servers: when `cover_room` or `explore` stops answering during an active goal, it cancels all `/navigate_to_pose` goals within `nav_cancel_s` (1 s) and fails the action (005-R34). Residual risk if the supervisor is lost as well: the BT goal timeout `nav_goal_timeout_s` (180 s) and, for the user, the stop lock. The same pattern applies to `exploration_server` (spec 003). Timeouts use the node clock (wall time unless Q-13 sets `use_sim_time`).

Nav2 minimal configuration (initial values, to tune in the simulator):

- Planner `nav2_navfn_planner::NavfnPlanner`; controller `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`, `desired_linear_vel: 0.25` (below the 0.30 m/s limit of spec 009a), `robot_radius: 0.16`, inflation 0.30 m. Plugin names unverified here: T510 checks them on both distros.
- `bt_navigator` with an own tree `nav_bt.xml`: `ComputePathToPose` + `FollowPath`, replanning at 1 Hz, **no recovery nodes**. `behavior_server`, `velocity_smoother` and `collision_monitor` are not launched, so `controller_server` is the only Nav2 node that publishes velocity.
- `controller_server` and `bt_navigator` take odometry from `/myrobot/odom` (`odom_topic`); T510 checks that it is `nav_msgs/Odometry` with a usable twist (Q-14).
- `enable_stamped_cmd_vel: false` on `controller_server` (and on any other velocity publisher; D-13). A config test checks it (005-R23); the parameter may not exist in Nav2 1.3.x (Jazzy), where declaring it is harmless: the test accepts both (unverified, T510 records it).
- Startup order: `lifecycle_manager_core` (stop_guard, supervisor, stubs or real nodes) comes up first; `lifecycle_manager_nav` starts after it and a failure there leaves the core active (Q-09 decided, conditional on guaranteed synchronization; see "Lifecycle synchronization"). `coverage_server` waits for `/navigate_to_pose` with a timeout and reports `COVERAGE_FAILED` instead of blocking.
- `use_sim_time` follows the outcome of Q-13 of spec 003.
- Costmaps: local costmap `obstacle_layer` on `/scan`; global costmap `static_layer` on `/map` + inflation.

## Lifecycle synchronization (Q-09 condition, 005-R35 to R38)

```
start:    core (stop_guard, supervisor, stuck_detector, escape_server, coverage_server, ...)  -> is_active
          -> nav (planner_server, controller_server, bt_navigator)                              -> is_active
          -> optional (MQTT bridge etc., empty until spec 009)                                   -> is_active
shutdown: optional -> nav -> core   (reverse; stop_guard last)
```

- `ManagerSequencer` (pure Python, injected clock and a `call_manager(name, command)` callback) holds the state machine: `STARTING_CORE`, `STARTING_NAV`, `STARTING_OPTIONAL`, `ALL_ACTIVE`, `FAILED_CORE`, `DEGRADED_NAV`, `DEGRADED_OPTIONAL`, `SHUTTING_DOWN`. It only decides; the thin node `lifecycle_sequencer` calls the Nav2 manager services (`~/manage_nodes` and `~/is_active`, names to confirm in T521) with `autostart: false` on the nav and optional managers and publishes the state in `/myRobot/telemetry/lifecycle_sequencer` and `Notification`s. The sequencer is a plain bootstrap node, not under a manager (it manages the managers): an exception to Art. III.2, to be stated in the contract entry.
- Failure rules are in 005-R36. In every state teleoperation (`mux/teleop`) and the stop lock keep working because they depend only on `twist_mux`, `stop_guard` and `teleop`.
- Proposed contract entries (not applied): node `lifecycle_sequencer` and `/myRobot/telemetry/lifecycle_sequencer` (`diagnostic_msgs/DiagnosticArray`, `default`); parameters `manager_startup_timeout_s` 30.0, `manager_shutdown_timeout_s` 15.0, `managers` (ordered list: `[lifecycle_manager_core, lifecycle_manager_nav, lifecycle_manager_optional]`).
- The D-09 amendment (three managers, the order above) is applied by the architect to spec 000 later; until then this spec is the proposal.

## Escape: timeout and fallback (Art. VI.3)

```
supervisor sends Escape(timeout_s)
  attempt 1..max_attempts:  REVERSE -> TURN(toward the freer side) -> CHECK front clearance
        fail -> next attempt with a larger turn
  primary ends at max_attempts or primary_budget_s (5 s), whichever comes first
  fallback (once, <= 11.5 s): SWEEP 360 -> FACE largest range (0..180) -> ADVANCE -> CHECK
  timeout / cancel / fallback failed -> stop publishing on mux/escape, result success=false
supervisor: success=false or its escape_timeout_s (20 s) -> lock_stop + ESCAPE_FAILED notification (001-R09)
```

Time budget (005-R10 to R12), with the defaults `reverse_speed_m_s` 0.15 and `turn_speed_rad_s` 1.0 (the stub's 0.1 m/s and 0.8 rad/s would leave no time): one attempt is about 1.3 s of reverse plus 1.6 to 3.1 s of turn, so the primary phase (`primary_budget_s` 5 s) fits one or two attempts and ends at 5 s. The fallback needs 6.28 s (sweep) + 0 to 3.14 s (facing) + 2.0 s (advance) = 8.3 s best case and 11.42 s worst case, so `fallback_budget_s` is 11.5 s. 5 + 11.5 = 16.5 s, below `default_timeout_s` (17 s). Unit tests (T505): the worst case reaches the fallback; the fallback with a 180-degree facing turn completes inside its budget. Speeds may be raised only within the limits measured in T503.

`default_timeout_s` (17 s) is below the supervisor's 20 s, so the server normally fails cleanly before the supervisor's clock does. While the server is not publishing, `twist_mux` times out the `escape` input after 0.5 s, and the lower inputs take over; after a failed escape the supervisor locks the robot, and `stop_guard` keeps it at zero (D-22).

## Supervisor: new rows (Q-12 decided by the project owner, 2026-11-13; the rows are a proposal for plan 001; ros-dev edits `SubmodeMachine`, T512)

The table of plan 001 has no row for the result of `cover_room`. Proposed additions, to be merged into that table by the architect when accepted:

| Current state | Event | Action | Next |
|---|---|---|---|
| COVERING | `cover_room` ended (success or failure), mode CLEANING | INFO `COVERAGE_DONE` or WARN `COVERAGE_FAILED` `Notification`; requests docking (call to the orchestrator user-command service, mechanism confirmed in T520); `coverage_blocked = true`; starts the dock timer | NONE |
| NONE | mode becomes AUTODOCKING | as in 001 (NONE; the IA368II `autodocking_server` drives, D-05) | NONE |
| NONE | dock timer expires (`dock_request_timeout_s`) and mode still CLEANING | ERROR `Notification` `DOCK_REQUEST_FAILED`; no new goal (bounded, Art. VI.3) | NONE |
| NONE | mode leaves CLEANING | clears `coverage_blocked` | NONE |

The 001 row "NONE, mode CLEANING: sends `cover_room`" is suppressed while `coverage_blocked` is true; that is amendment A2 of spec 001 (listed in the spec; architect, later). Until A2 is applied the rows are active only with `dock_after_coverage:=true` (default false), and the 001 tests pass unchanged with `coverage_blocked` false. The supervisor gains a client of the orchestrator command service (the frozen `/myRobot/start_stop`, to be confirmed). The `Mode` enum is not extended and `BehaviourManager` is not touched (Art. II). If T520 finds no way to request docking without breaking Art. II, escalate to the project owner. This is the formal EARS text of 005-R30.

## Dependencies on spec 003

005 needs from 003: `/map`, TF `map -> odom -> base_link` and `/scan` frame, and later the room topics. Until then the module runs on a saved map from `map_server` with a static `map -> odom` identity (valid for the simulator ground-truth odometry, see spec 003) so that 005 is not blocked. `room_id` resolution (005-R16) depends on `Room`/`RoomArray` from T303: `Room.index` is the value in `room_labels`, `Room.id` is `room_NN` with `NN = index`. Shared grid code lives in the tiny package `vacuum_grid` (no ROS, created in T309), so `vacuum_coverage` and `vacuum_mapping` do not depend on each other (Art. III.1); fallback if that is rejected: two copies pinned by identical unit tests.

## Proposed contract changes (NOT applied; the architect applies them in one serialized change, after PRs #9 and #10)

`topics.yaml`:

```yaml
topics:
  # already declared, no change: /myRobot/mux/nav (owner controller_server, priority 60, since "005")
  - {name: /myRobot/coverage/path,    type: nav_msgs/msg/Path,          qos: default, owner: coverage_server, since: "005", note: "lane waypoints of the current pass, for the interface (spec 007); published once per pass"}
  - {name: /myRobot/coverage/covered, type: nav_msgs/msg/OccupancyGrid, qos: latched, owner: coverage_server, since: "005", note: "100 = covered, 0 = not covered, -1 = outside the region; 1 Hz"}
  # Q-10 (applied with D-24, before T511; test tolerance confirmed in T005/T022): /myRobot/mux/coverage gets "reserved: true" and no owner (twist_mux still subscribes);
  #   the stub owns it only with with_stubs:=true.
  # /map: spec 003 lists owner [slam_toolbox, map_server]; the contract test must tolerate either.

  # consumed Nav2 interfaces, declared so the contract tests see them (owner = third party, frozen: true)
actions:
  - {name: /navigate_to_pose, type: nav2_msgs/action/NavigateToPose, owner: bt_navigator, frozen: true, since: "005"}

lifecycle:
  lifecycle_manager_nav: [planner_server, controller_server, bt_navigator]   # Q-09 (decided, conditional; amends D-09, applied by the architect later)
  lifecycle_manager_core: [supervisor, stop_guard, stuck_detector, escape_server, coverage_server]   # unchanged

parameters:
  lifecycle_sequencer:
    manager_startup_timeout_s: {type: double, default: 30.0, requirement: 005-R35}
    manager_shutdown_timeout_s: {type: double, default: 15.0, requirement: 005-R37}
    managers: {type: string_array, default: [lifecycle_manager_core, lifecycle_manager_nav, lifecycle_manager_optional], requirement: 005-R35}
  supervisor:
    dock_after_coverage: {type: bool, default: false, requirement: 005-R30, note: "true needs amendment A2 of spec 001"}
    nav_cancel_s: {type: double, default: 1.0, requirement: 005-R34}
    nav_cancel_action: {type: string, default: /navigate_to_pose}
    dock_request_timeout_s: {type: double, default: 30.0, requirement: 005-R30}
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
    outcome: {type: string, default: auto, values: [auto, succeed, fail, hang], requirement: 005-R14, note: "extends the 001 values; the launch file sets it per implementation: succeed for the stub, auto for the real node (005-R33)"}
    reverse_m: {type: double, default: 0.20}
    rear_clearance_m: {type: double, default: 0.15}
    turn_deg_base: {type: double, default: 90.0}
    turn_deg_step: {type: double, default: 45.0}
    front_clearance_m: {type: double, default: 0.40}
    max_attempts: {type: int, default: 3}
    fallback_forward_m: {type: double, default: 0.30}
    default_timeout_s: {type: double, default: 17.0, requirement: 005-R12}
    reverse_speed_m_s: {type: double, default: 0.15, requirement: 005-R08, note: "stub script used 0.1; limit from T503"}
    turn_speed_rad_s: {type: double, default: 1.0, requirement: 005-R08, note: "stub script used 0.8; limit from T503"}
    primary_budget_s: {type: double, default: 5.0, requirement: 005-R10}
    fallback_budget_s: {type: double, default: 11.5, requirement: 005-R11}
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
    waypoint_timeout_s: {type: double, default: 120.0, requirement: 005-R20}
    nav_goal_timeout_s: {type: double, default: 180.0, requirement: 005-R34, note: "BT-level bound (Nav2 config nav_bt.xml, not a node parameter)"}
    resume_window_s: {type: double, default: 120.0, requirement: 005-R32}
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
