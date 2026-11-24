# vacuum_bringup

Launch files and configuration of the system, plus the lifecycle sequencer (T521, spec 005, D-26).

## The lifecycle sequencer

Three Nav2 lifecycle managers bring the nodes up: `lifecycle_manager_core` (stop_guard first, supervisor last),
`lifecycle_manager_nav` and `lifecycle_manager_optional`. The `lifecycle_sequencer` starts them in that order, each
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
- `stop_guard` leaves `active` while running: nav is paused, ERROR `STOP_GUARD_LOST`, `DEGRADED_NAV`.

Shutdown: reverse order of the managers that were started; one failing stop does not prevent the next; the whole
sequence stays within `manager_shutdown_timeout_s` (each step gets an equal share of what is left).

### Using it

```bash
ros2 launch vacuum_bringup skeleton.launch.py with_sequencer:=true   # default false: lifecycle_manager_core autostarts alone
```

With `with_sequencer:=true` the launch adds `lifecycle_manager_nav`, `lifecycle_manager_optional` (both
`autostart: false`, empty node lists until specs 005/T510 and 009 fill them in) and the sequencer, and
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
