# vacuum_bringup

Launch files and configuration of the system, plus the lifecycle sequencer (spec 005, D-26).

## The lifecycle sequencer

Three Nav2 lifecycle managers bring the nodes up: `lifecycle_manager_core` (stop_guard first, supervisor last),
`lifecycle_manager_nav` and `lifecycle_manager_optional` (the last is not launched while it has no nodes). The `lifecycle_sequencer` starts them in that order, each
one only when the previous one reports `is_active` (or after `manager_startup_timeout_s`), and stops them in
reverse order on shutdown, so `stop_guard` is the last to go down. It is a **plain node on purpose** (D-26): it
only orchestrates managers.

| Piece | File | Role |
|---|---|---|
| `ManagerSequencer` | `vacuum_bringup/manager_sequencer.py` | Pure Python, no ROS imports, never sleeps. Ordered steps, injected `call`, `wait_active`, `clock`, `log`, `notify`. Every step returns a `StepResult` (`OK`, `FAILED`, `TIMEOUT`, `SKIPPED`). |
| `lifecycle_sequencer` | `vacuum_bringup/sequencer_node.py` | Thin node: maps `call` to `/<manager>/manage_nodes` (`nav2_msgs/srv/ManageLifecycleNodes`), `wait_active` to `/<manager>/is_active` (`std_srvs/srv/Trigger`), publishes Notifications and telemetry `/myRobot/telemetry/lifecycle_sequencer`, watches `/stop_guard/transition_event`. A worker thread runs the sequence; SIGINT/SIGTERM trigger the ordered shutdown. |
| `dummy_lifecycle_node` | `vacuum_bringup/dummy_lifecycle_node.py` | Test helper with the parameter `fail_on_activate` (default false). |
| Parameters | `config/lifecycle_sequencer.yaml` | `managers` (order), `manager_startup_timeout_s` (30), `manager_shutdown_timeout_s` (15). Declared in `contracts/topics.yaml`. |
| Node classification | `config/node_classification.yaml` | Machine-readable copy of the table in `docs/lifecycle-managers.md` (005-R39). Keep both in step. |

States: `NOT_STARTED`, `STARTING_CORE`, `STARTING_NAV`, `STARTING_OPTIONAL`, `ALL_ACTIVE`, `FAILED_CORE`,
`DEGRADED_NAV`, `DEGRADED_OPTIONAL`, `SHUTTING_DOWN`. Failure rules (005-R36):

- core fails: nav and optional are not started, ERROR `CORE_START_FAILED`, state `FAILED_CORE`;
- nav fails: the nav manager is reset and every node it left active is then deactivated one by one (the Nav2 manager of Jazzy aborts RESET/PAUSE at the first node that cannot make the transition, so `planner_server` could stay active), WARN `NAV_UNAVAILABLE`, `DEGRADED_NAV`, optional is still tried;
- optional fails: WARN `OPTIONAL_UNAVAILABLE`, `DEGRADED_OPTIONAL`, nothing else is touched;
- `stop_guard` leaves `active` while running (confirmed with `get_state` on every transition event and every `stop_guard_poll_period_s`, because a volatile event can be missed; two unreachable samples in a row also count): nav is paused, ERROR `STOP_GUARD_LOST`, `DEGRADED_NAV`.

Shutdown: reverse order of the managers that were started; one failing stop does not prevent the next; the whole
sequence stays within `manager_shutdown_timeout_s` (each step gets an equal share of what is left).

### Using it

```bash
ros2 launch vacuum_bringup skeleton.launch.py with_sequencer:=true   # default false: lifecycle_manager_core autostarts alone
```

With `with_sequencer:=true` the launch adds the sequencer and, with `nav:=true`, `lifecycle_manager_nav` (`autostart: false`, the three Nav2 nodes); `lifecycle_manager_optional` is not launched until the cloud/MQTT nodes (spec 009) give it nodes (a Nav2 manager with an empty `node_names` aborts), and
`lifecycle_manager_core` starts with `autostart: false`.

### How to add a manager

1. Add its name and role to `MANAGER_ROLES` in `manager_sequencer.py` (a new kind of failure rule goes in
   `ManagerSequencer._on_failure`, with its test).
2. Create `config/lifecycle_<x>.yaml` (`autostart: false`, `bond_timeout: 0.0`, `node_names`).
3. Add it to `managers` in `config/lifecycle_sequencer.yaml`, to the launch (`sequencer_nodes()`), to
   `config/node_classification.yaml` and to `docs/lifecycle-managers.md` (architect).
4. Add a line to `MANAGER_CONFIGS` in `test/test_node_classification.py`.

### Tests

```bash
scripts/test-docker.sh lyrical        # or jazzy; builds everything and runs every test of src/
# the pure core alone, no ROS needed:
cd src/vacuum_bringup && python3 -m pytest test/test_manager_sequencer.py
```

- `test/test_manager_sequencer.py`: the pure core (success path, a manager that never becomes active, nav and
  optional failures, shutdown order, double shutdown, timeouts) with a simulated clock.
- `test/test_node_classification.py`: 005-R39, every node of the launch is on the plain list or managed.
- `test/test_sequencer_graph.py`: live graph with the real Nav2 managers, the real sequencer and dummy nodes: five
  clean starts, one failing node per manager, `stop_guard` lost, shutdown order (read from the monotonic times the dummy nodes print, not from event arrival). Condition waits only; domain IDs 130 to 159.

### Nav2 (spec 005)

`launch/nav.launch.py` starts `planner_server`, `controller_server` (`cmd_vel` remapped to `/myRobot/mux/nav`),
`bt_navigator` and `lifecycle_manager_nav` with `config/nav2_params.yaml` and `config/nav_bt.xml` (no recovery
nodes; the whole goal sits under a `Timeout` of 180 s, 005-R34). One YAML serves Jazzy (Nav2 1.3.x) and Lyrical
(1.5.x): Nav2 ignores undeclared parameters, and the differences are listed in the launch file docstring.
Arguments: `autostart` (default true; false when `lifecycle_sequencer` starts the manager), `with_manager`,
`params_file`, `bt_xml`, `sim_slowdown` (below). `skeleton.launch.py nav:=true` includes it (default `false`, nothing changes without it); with
`with_sequencer:=true` the include starts no manager of its own and `lifecycle_manager_nav` (`config/lifecycle_nav.yaml`:
`controller_server`, `planner_server`, `bt_navigator`) is started by the sequencer after the core manager; without
`nav` no nav manager is launched and the sequencer lists only `lifecycle_manager_core`. A Nav2 `lifecycle_manager` with `node_names: []` aborts at startup (exit -6), so no manager without nodes is launched (`lifecycle_manager_optional` neither, until spec 009); `ManagerSequencer` additionally skips (log `X: no nodes configured, skipped`, telemetry `startup_X_outcome=SKIPPED`, no state change) a listed manager that reports no nodes. `/map` and the `map -> odom` TF come from
outside (`map_tf:=true` on `skeleton.launch.py`, or `map_tf.launch.py`, provides the static identity `map -> my_robot_odom`, dead reckoning only; `map_source:=slam` is a placeholder that fails). `bond_timeout` stays 0.0 on purpose: the sequencer checks the managers only at startup (005-R36), so a bond
break at run time would reset Nav2 while the sequencer still says ALL_ACTIVE, and a 4 s bond can break spuriously on a
loaded runner or the slow simulator. A dead Nav2 node is safe anyway (no goal; the mux input times out).
`real_coverage:=true` swaps the coverage stub for the real `coverage_server` (`autostart` false, the core manager starts it).
`real_nodes:=true` also swaps the `escape_server` and `stuck_detector` stubs for the real ones of `vacuum_coverage` (`stuck_escape.launch.py`, `autostart` false, same node names) and implies `real_coverage`.

#### Slow simulator: `sim_slowdown`

With the laser model CoppeliaSim ran at 1.5 to 1.9 steps/s (about 0.09x real time) and every wall-clock timeout
(`use_sim_time` stays false, there is no `/clock`) aborted valid goals. `sim_slowdown:=<factor>` (default `1.0`,
real time: the shipped files are used untouched) multiplies `progress_checker.movement_time_allowance`, the
`costmap_update_timeout` of `controller_server` and `planner_server` and the BT `Timeout` (005-R34) by the
factor. The launch writes overlay copies of the params and of the BT to a temporary directory
(`vacuum_bringup/slowdown.py`, pure and unit-tested), so nothing is edited in two places:

```bash
ros2 launch vacuum_bringup nav.launch.py sim_slowdown:=11     # 15 s -> 165 s, 180 s -> 1980 s, 1 s -> 11 s
ros2 launch vacuum_coverage coverage.launch.py sim_slowdown:=11   # waypoint_timeout_s 120 -> 1320 s
```

`skeleton.launch.py sim_slowdown:=<factor>` forwards it to both (nav and real_coverage). The overlay directory is removed when the launch exits; a custom `params_file` or `bt_xml` with nothing to scale logs a warning. Use the same factor on both launches (factor about 1 / real-time factor). The supervisor and skeleton timeouts
(`escape_timeout_s`, `dock_request_timeout_s`, `feedback_stale_s`) are not scaled: the skeleton launches only
the stubs, and the real escape and stuck nodes do not exist yet.

- `test/test_slowdown.py`, `test/test_nav_slowdown_graph.py`: the helper, the launch argument and a live Nav2 started
  with `sim_slowdown:=4` (parameters read back from the nodes; domain from `graph_support.domain_for(22)`).
- `test/test_nav_config.py`: YAML and BT checks, no ROS graph (005-R19, R23, R34).
- `test/test_nav_graph.py`, `test/test_nav_timeout.py`: live graph with real Nav2, real `map_server` and
  `test/fake_robot.py` (odometry, TF, empty scan); goal reached, cancel silences `mux/nav`, the `Timeout`
  decorator aborts a long goal. Domains come from the vacuum_bringup range 100 to 129 (`graph_support.domain_for(20)` and `(21)`).
- `test/test_nav_sim_live.py`: the same against CoppeliaSim, `VACUUM_SIM_LIVE=1`, run by the sim-operator only.
