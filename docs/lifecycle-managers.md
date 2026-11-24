# Lifecycle managers and node classification (D-26, 005-R35 to R39)

Diagram: `docs/diagrams/lifecycle-managers.mmd` (rendered in `docs/diagrams/render/lifecycle-managers.svg`, shown in
`docs/architecture.md`, section 10).

## The idea in a few lines

A ROS 2 lifecycle node has states (unconfigured, inactive, active, finalized). A **lifecycle manager** drives many nodes
through those states in a fixed order, so the robot never has half of its nodes active. We use that for every node that
holds something a manager must control: drivers, publishers that must stop, timers, action servers with goals in
flight, the Nav2 servers.

We keep three managers of our own, plus the unchanged IA368II one (D-09, Q-09):

1. `lifecycle_manager_core`: the safety and decision nodes. `stop_guard` comes up first (the safety net exists before
   anything can move), the actuators next, the `supervisor` last (it sends goals as soon as it is active).
2. `lifecycle_manager_nav` (`autostart: false`): `planner_server`, `controller_server`, `bt_navigator`. It starts only
   after the core is active.
3. `lifecycle_manager_optional` (`autostart: false`): cloud and MQTT, empty until spec 009. Its failure never affects
   the first two (Art. VII).

Who starts the managers in that order? The `lifecycle_sequencer`. It starts each manager only when the previous one
reports it is active (or after a timeout, 005-R35), applies the failure rules (core failure: stop, nav failure:
degraded, no automatic motion; optional failure: ignore), and on shutdown goes in reverse (optional, nav, core), so
`controller_server` stops publishing before the supervisor and `stop_guard` go down and `stop_guard` is last (005-R37).

## Which nodes are lifecycle and which are plain, and why (D-26)

| Node | Kind | Why |
|---|---|---|
| `stop_guard`, `supervisor`, `stuck_detector`, `escape_server`, `coverage_server`, `room_segmenter`, `odom_tf_bridge`, `exploration_server` | lifecycle, `lifecycle_manager_core` | They hold timers, publishers and action servers that must be configured and stopped in order |
| `planner_server`, `controller_server`, `bt_navigator` | lifecycle, `lifecycle_manager_nav` | Nav2 nodes; `controller_server` publishes velocity and must stop before the core goes down |
| MQTT bridge and other cloud nodes | lifecycle, `lifecycle_manager_optional` | Network resources that may fail without affecting the robot (Art. VII) |
| `behaviour_server`, `myrobot`, charging and docking servers | lifecycle, IA368II `lifecycle_manager` | Third-party, unchanged (Art. II) |
| `lifecycle_sequencer` | **plain** | It only orchestrates the managers and has nothing to configure, activate or clean up. Making it a lifecycle node would need a manager for the manager |
| `twist_mux`, `my_robot_cmd_vel_node` (wheel bridge), `teleop_twist_keyboard` | **plain**, third-party | Not ours to change (Art. II); started by the launch file |
| one-shot bootstrap glue, tools | **plain** | No controllable state |

Rule (D-26): a node is a lifecycle node when a manager must configure, activate, deactivate or clean up something it
holds; it may be a plain node when it only orchestrates managers or is pure glue with no controllable state. Any other
plain node must be added to this table with its reason (005-R39).

## Order

- Startup: plain third-party nodes and the IA368II group (launch) -> sequencer -> core (`stop_guard` first,
  `supervisor` last) -> nav -> optional.
- Shutdown: optional -> nav -> core (`stop_guard` last).
- If the sequencer dies after start-up the robot keeps running (the managers are independent); the ordered shutdown is
  then no longer guaranteed (accepted residual, 005-R37).
