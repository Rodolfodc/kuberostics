# Proposal: a Nav2 lifecycle manager that hangs during start-up (005-R35 to R39, D-26)

Status: **PROPOSAL, not applied.** It changes no requirement, no contract file and no code. If approved, the architect
merges section 7 into `spec.md` and `tasks.md`, and the contract entries of section 8 are added by the contract change
in flight at that time (Art. I: one at a time; PR #27 adds `stop_guard_poll_period_s` to `topics.yaml`, PR #29 edits the
notification registry of `interfaces.md`; this document edits neither).

Verdict (details in section 6): **adopt L1 + L2 + better diagnostics; do not adopt L3 for this delivery.** L3 is
documented as a residual risk with a manual remedy and with the shape it would take if it is ever reopened (section 4).

## 1. The problem, with the source evidence

PR #27 (branch `fix/sequencer-graph-test-flake`) found it while reproducing a flaky live-graph test at `--cpus=1`: a node's
reply to the manager is dropped (`failed to send response (timeout): client will not receive response`), the manager
stays in `Configuring <node>`, a retried `manage_nodes` or reset is not served, and the sequencer only reaches its own
timeout (`DEGRADED_NAV` or `FAILED_CORE`). A longer timeout does not help because the reply is lost, not slow.

Why the manager never comes back (read from the Nav2 sources on GitHub, branches `jazzy` and `lyrical`,
`nav2_lifecycle_manager/src/lifecycle_manager.cpp`, `nav2_util/src/lifecycle_service_client.cpp`, `service_client.hpp`;
**not** from the installed binaries, see section 9):

1. **The wait is unbounded on both distros.**
   - Jazzy: `changeStateForNode` calls `change_state(transition)`, which ends in `ServiceClient::invoke(request, response)`
     and `spin_until_future_complete(future)` with no timeout.
   - Lyrical: `changeStateForNode` passes `milliseconds(-1)` as the transition timeout, which takes the same
     `invoke(request, response)` branch. The new `service_timeout` parameter (default 5 s) only bounds `wait_for_service`
     and `get_state`, never the wait for the reply of a transition.
   - A lost reply therefore wedges the manager permanently, on both distros.
2. **`manage_nodes` and `is_active` share one `MutuallyExclusive` callback group** (`callback_group_`, served by one
   `service_thread_`). While `startup()` is blocked, `is_active` is not answered either (it times out; it does not return
   false). This is the cheap discriminating signal for L2.
3. **Nav2's own bond and respawn mechanism covers none of it.**
   - Both of our configs set `bond_timeout: 0.0`: no bond and no bond timer exist.
   - Even with bonds, `checkBondConnections` and `checkBondRespawnConnection` run on the same blocked callback group, and
     `attempt_respawn_reconnection` handles a managed *node* that died, not the *manager* wedged. The bond protects the
     robot from a dead server; it does not detect a hung manager.
4. **A restarted manager cannot resume.** A new manager starts with `managed_nodes_state_ = UNCONFIGURED` and its `startup()`
   issues CONFIGURE for every node. A node that executed `configure` and only lost the reply is already `inactive`; the
   CONFIGURE transition from `inactive` is invalid, the node answers failure, and `startup()` aborts. The group must be
   driven back to `unconfigured` first. The manager-level RESET does not help: it is non-hard and aborts at the first
   invalid transition (`changeStateForAllNodes(..., hard_reset=false)`).

The sequencer (`manager_sequencer.py`, `sequencer_node.py`) behaves as specified. Two weaknesses of the current code
show up in the same evidence and are cheap to fix (they belong to L2):

- On a nav TIMEOUT, `_on_failure` calls `_deactivate_nav(RESET)`, which blocks for a second full `startup_timeout_s`
  because a wedged manager never serves the RESET. Degrading takes about 2 x 30 s and delays the optional manager.
- `run_shutdown` sends SHUTDOWN to a wedged nav manager, which just burns its share of `manager_shutdown_timeout_s`;
  nothing deactivates `controller_server` directly, so 005-R37 (nav off before `stop_guard`) is not honoured in that case.

## 2. L1: readiness before starting a group

### 2.1 Is it implementable in the sequencer, and valid?

Yes, as one more injected callable of `ManagerSequencer` (D-18): `ready(manager, timeout_s) -> Outcome`, called before
`call(STARTUP)`. The ROS node implements it with calls the sequencer already knows how to make:

1. `/<manager>/manage_nodes` and `/<manager>/is_active` are visible, and `is_active` answers (true or false).
2. `node_names` read from `/<manager>/get_parameters` (already done by `_deactivate_nodes`).
3. For every node of the group, `/<node>/get_state` answers from the sequencer **and** the state is `unconfigured`. A node
   that is already `inactive` or `active` makes the later STARTUP fail with an invalid transition: report it as not ready
   with the states in the detail text (see 4.3).
4. A settle margin `discovery_settle_s` (default 1.0 s) after the last positive check, to let the participants finish
   matching.
5. All within `manager_ready_timeout_s` (default 30 s, separate from `manager_startup_timeout_s`).

A group that is not ready after the timeout is handled by the unchanged rule of 005-R36 for its role, **without sending
STARTUP**: the wedge is never created, and the notification says "not ready" instead of "did not become active".

Apply it to the core group too: the mechanism is identical and a wedged core manager is the worse case.

### 2.2 What it guarantees, and what it does not

It does **not** guarantee the manager's reply path. The dropped hop is the node's response writer to the **manager's**
response reader, in the manager's own DDS participant. Everything the sequencer observes is the sequencer's own
participant (its clients to the node, its client to the manager). Visibility on the sequencer's clients is a heuristic
(a node that answers `get_state` to us has finished its own start-up and its executor is spinning), not proof.

One piece of manager-side evidence exists, and it differs by distro (read from the diffs of the two `lifecycle_manager.cpp`
files):

- **Lyrical:** `manage_nodes` and `is_active` are created in `createLifecycleServiceServers()`, called from the init timer
  **after** `createLifecycleServiceClients()`, and the `LifecycleServiceClient` constructor blocks until the node's
  `get_state` service is visible to the manager. So on Lyrical, a visible `is_active` implies the manager has already
  discovered the request side of every node of its group.
- **Jazzy:** the two services are created in the constructor, so their visibility says nothing about the manager's clients.

Even on Lyrical this proves the request direction only. Residual risk after L1: the race can still occur, at a much lower
probability, because on the robot the nodes are up long before the manager is asked to start (a nav group is started only
after the core is `active`), while in the test everything started at once. L2 bounds what is left.

## 3. L2: detect a stuck manager, and stop waiting for it

### 3.1 Classification (pure logic in `ManagerSequencer`)

After a TIMEOUT of `call(STARTUP)`, `wait_active` or any other manager command, the sequencer probes `is_active` with
`manager_probe_timeout_s` (default 2.0 s, a few attempts):

| Probe | Meaning | Handling |
|---|---|---|
| answers `true` | the start-up finished late: treat as OK | continue |
| answers `false` | the manager is alive and its start-up failed or is still running | rule of 005-R36 as today, manager-level RESET allowed |
| **no answer** | the manager is **unresponsive** (wedged, or dead): any further manager command would only burn a timeout | rule of 005-R36, plus `MANAGER_UNRESPONSIVE`; **no manager-level command** is sent to it any more in this run |

This rests on fact 2 of section 1 (shared callback group), not on a heuristic about timing.

### 3.2 What the sequencer does instead of manager commands

- Direct per-node deactivate (`DEACTIVATE_NODES`, which exists and already reads `node_names`) for the nav group, right
  away: this is the existing safety path and it takes seconds, not a second timeout.
- The same path in `run_shutdown` for an unresponsive manager, **before** the core is shut down: `controller_server` is off
  before `stop_guard`, and `stop_guard` stays last (005-R37, Art. VI). The core manager's SHUTDOWN is unchanged.
- No new state: the sequencer stays in `DEGRADED_NAV`, `DEGRADED_OPTIONAL` or `FAILED_CORE`. "Unresponsive" is a flag per
  manager in the telemetry, so the contract of the states does not change.

### 3.3 Diagnostics

The notification and the telemetry must tell the operator what is stuck, without a `/rosout` subscription (cheaper and
pure): when a manager is classified unresponsive the sequencer reads `get_state` of every node of the group once and
reports the states, e.g. `lifecycle_manager_nav unresponsive; nodes: planner_server=inactive, controller_server=unconfigured`.
A node that is one step ahead of the rest while the manager waits is the signature of a lost reply. Telemetry keys (no
contract change, they are values of the existing `DiagnosticArray`): `unresponsive_<manager>` (0/1),
`ready_<manager>_s`, `ready_<manager>_outcome`, `node_states_<manager>`.

## 4. L3: restart the stuck manager. What it would be, and why it is expensive

### 4.1 Who detects, who restarts

- **Detects:** the sequencer, with the L2 classification (nothing else is needed).
- **Restarts:** nobody can today. `respawn=True` in `launch` fires when the process **exits**, and a wedged manager never
  exits. The sequencer is plain and must not launch or kill processes (D-26); the robot side runs in Docker Compose
  (no restart-on-unhealthy) and K3s is used for cloud workloads only, so a `livenessProbe` does not reach it. Whatever
  restarts the manager is therefore **a new component of ours**. Three shapes, in order of preference if it is ever built:
  1. **Same-name replacement manager** (Art. II allows it): a small Python node that serves `manage_nodes` and `is_active`
     with the same names, types and `node_names`/`autostart` parameters, and bounds every transition with a timeout and a
     retry that first reads the node state (never re-sends an already done transition). It removes the cause (no
     unbounded invoke) and needs no kill. Cost: it duplicates a third-party component for the nav group (bond features
     are unused here, `bond_timeout: 0.0`), and "managed by a Nav2 lifecycle manager" (D-09, Art. III.2 wording) then
     needs a sentence in D-26.
  2. **Wrapper process of ours** that is the parent of the Nav2 manager (`subprocess`), restarts it on request or on exit,
     and is itself under `respawn=True`. The request arrives from the sequencer on a new topic or service (a contract
     change). Needs the PID tracking and a clean kill of a C++ process in a container.
  3. A reaper node that kills by name or PID: rejected (it needs the PID of a process it did not start, and killing by name
     is blocked by project policy).

### 4.2 What happens to the nodes the manager already configured

Nothing, because they are separate processes: they keep their lifecycle state (and `controller_server` keeps publishing
if it was active). A restarted manager cannot take them over (fact 4 of section 1). Before the new manager is asked to
start, the sequencer would have to drive the group to `unconfigured` itself: direct `deactivate` then a new direct
`cleanup` step (`CLEANUP_NODES`, which does not exist yet; `DEACTIVATE_NODES` only deactivates). Only then can STARTUP run.
So L3 includes the L2 direct path and a new one, plus the restart component, plus a retry state of the sequence.

### 4.3 New states, codes, parameters, telemetry (if built)

States: `RESTARTING_<ROLE>` or a sub-step of `STARTING_NAV`; codes `MANAGER_RESTARTED` (INFO),
`MANAGER_RESTART_EXHAUSTED` (ERROR for nav: it stays degraded); parameters `max_manager_restarts` (default 1) and
`manager_restart_backoff_s` (default 5.0); telemetry `restarts_<manager>`; a request channel to the restart component.
The interaction with 005-R37 is the same as in section 3.2, plus: shutdown while a restart is in progress must cancel it
first, and the nav group is shut down directly if no manager exists at that moment.

## 5. Failure modes L3 would add, and the bound

| Failure mode | Why it is real | Bound |
|---|---|---|
| Restart loop | the new manager hits the same race (the cause is environmental: a loaded host) | at most `max_manager_restarts` = 1 per manager per boot, backoff, then stay degraded with `MANAGER_RESTART_EXHAUSTED` |
| Second hang | same unbounded invoke in the new manager | the L2 classification applies again, the restart budget counts it |
| Restart while motion is active | `controller_server` keeps publishing after the manager is killed (no bond) | restart only in `STARTING_NAV` or `DEGRADED_NAV`, only after the direct deactivate has been confirmed by `get_state`; never for the core manager; `stop_guard` is outside the restarted group |
| Restart of the core manager | would leave `stop_guard`, the actuators and the `supervisor` unmanaged (Art. VI) | forbidden: a wedged core stays `FAILED_CORE`, the operator restarts the stack |
| Restart racing the ordered shutdown (005-R37) | two controllers for the same nodes | shutdown cancels the restart first |
| Nodes with bonds enabled | what a broken bond does to a node when its manager is killed is **unverified** | keep `bond_timeout: 0.0` for the nav group, state it in the config comment |

## 6. Cost, value, verdict

The race is a **start-up-only** event, observed in a test that spawned everything at once on a `--cpus=1` runner and not
yet on the robot. The cure for the remaining probability is cheap and already within the operator's reach: the whole
bring-up takes under a minute, so a manual re-run of the launch is a complete remedy (and the sequencer already degrades
safely: no automatic motion, notification, `stop_guard` untouched). L3 needs a new killer or replacement component, a
cleanup path, a retry state, a new request channel (contract), and tests with process kill on both distros. Estimate
(ros-dev, tests included): L1 about 0.5 day, L2 about 1 day, L3 about 3 days, against a window of 10 days to
2026-12-04 in which the coverage and escape verification (T510 to T517) and the end-to-end scenario still need the same
agent. The six core modules are never cut (Art. IX), and the sequencer is part of the skeleton, but a robust sequencer does
not require self-healing of a third-party manager: bounded degradation is enough for the demo.

- **Adopt L1** (readiness gating, for core and nav): removes the race on the robot to a low residual and makes the failure
  message accurate.
- **Adopt L2** (classification, no manager command after unresponsive, direct node path in degrade and in shutdown, state
  report): the part of L3's value that is free. It also fixes the 2 x timeout degrade and the R37 gap of section 1.
- **Do not adopt L3.** Record the unrecoverable wedge as an accepted residual in 005-R37's neighbourhood, with the manual
  remedy (restart the bring-up) and a reopening criterion: the wedge is seen on the real robot, **after** L1, in at least
  one of 20 consecutive cold starts. If reopened, build the same-name replacement manager (4.1, option 1), not a killer.
- No amendment of the constitution is needed (D-26 and Art. II are respected: no process is launched by the sequencer and
  no third-party code is edited).

## 7. Spec amendment text (for `spec.md`; IDs continue after 005-R39)

| ID | Requirement (EARS) | Verification |
|---|---|---|
| 005-R40 | Before sending STARTUP to a lifecycle manager (core, nav or optional), the sequencer shall verify that the group is ready: `/<manager>/manage_nodes` and `/<manager>/is_active` are visible and `is_active` answers, and every node of the manager's `node_names` answers `get_state` with state `unconfigured`, followed by a settle margin `discovery_settle_s` (default 1.0 s). The wait is bounded by `manager_ready_timeout_s` (default 30 s, separate from `manager_startup_timeout_s`). | unit (simulated clock) + launch_testing |
| 005-R41 | If a group is not ready within `manager_ready_timeout_s`, the sequencer shall not send STARTUP and shall apply the rule of 005-R36 for the role of the manager; the log and the notification shall name the nodes that were not ready and their states. | unit + launch_testing |
| 005-R42 | When a command to a manager times out, the sequencer shall probe `is_active` for at most `manager_probe_timeout_s` (default 2.0 s). If the probe is not answered, the manager shall be classified unresponsive: the sequencer shall publish a `Notification` with code `MANAGER_UNRESPONSIVE` (WARN for nav and optional, ERROR for core) that lists the states of the nodes of its group, shall publish `unresponsive_<manager>` = 1 in `/myRobot/telemetry/lifecycle_sequencer`, and shall send no further manager-level command to it in that run. The state remains the one given by 005-R36. | unit + launch_testing with a test double that never answers |
| 005-R43 | When the nav manager is unresponsive, the sequencer shall deactivate the nodes of its group directly through their `change_state` services (reverse order of `node_names`), without waiting for a manager-level RESET, and shall do the same in the ordered shutdown before the core manager is shut down; `stop_guard` shall remain the last node to go down (005-R37, Art. VI). | unit + launch_testing (order observed, no non-zero `/myRobot/cmd_vel`) |
| 005-R44 | The sequencer shall not restart, launch or kill any process. A manager that is classified unresponsive stays so until the bring-up is restarted; this shall be stated in `docs/lifecycle-managers.md` as an accepted residual with its manual remedy, and the Nav2 manager shall keep `bond_timeout: 0.0` for the nav group. | inspection |

Edit of 005-R37 (residual, to be appended): "A lifecycle manager that hangs after a lost service reply (unbounded wait in
the Nav2 manager, Jazzy and Lyrical) is detected and bypassed (005-R42, 005-R43) but not restarted (005-R44)."
Requirement 005-R29 (both distros) shall list 005-R40 to 005-R43. Q to record for the owner: reopening criterion of
section 6 (one wedge in 20 cold starts on the robot).

## 8. Proposed contract entries (NOT applied; to go in the contract change in flight when the other two are merged)

`specs/000-system/contracts/topics.yaml`, `parameters.lifecycle_sequencer` (three lines, no new topic):

```yaml
    manager_ready_timeout_s: {type: double, default: 30.0, requirement: 005-R40}
    discovery_settle_s: {type: double, default: 1.0, requirement: 005-R40}
    manager_probe_timeout_s: {type: double, default: 2.0, requirement: 005-R42}
```

`specs/000-system/contracts/interfaces.md`, notification registry (after the PR #29 layout is merged): `MANAGER_UNRESPONSIVE`,
source `lifecycle_sequencer`, severity WARN (nav, optional) or ERROR (core), text lists the node states, requirement
005-R42. No new `.msg/.srv`: the sequencer only uses `ManageLifecycleNodes`, `Trigger`, `GetState`, `ChangeState`,
`GetParameters`, already used.

Diagram: `docs/diagrams/lifecycle-managers.mmd` gains a "ready?" step before each manager start and a "no answer to
`is_active`: direct node path" branch; the architect updates it and re-renders with the task T526 below.

## 9. Tasks (to add to `tasks.md`, numbered after T521)

Tests first (Art. V, Art. VIII). Dependency common to all: PR #27 merged (`GuardWatch`, worker polling, staged live-graph
test) and the contract entries of section 8 applied.

- [ ] **T522** (ros-dev) Pure tests in `test_manager_sequencer.py` (simulated clock, fake callables): not ready -> no STARTUP and
  the role rule of 005-R36 for each role; ready after a delay -> STARTUP; probe answers true/false/none ->
  OK / R36 as today / unresponsive; unresponsive nav -> no RESET sent, direct `DEACTIVATE_NODES` called, `MANAGER_UNRESPONSIVE`
  once; shutdown with an unresponsive nav -> direct deactivate before the core SHUTDOWN, core last; total degrade time below
  `manager_ready_timeout_s + startup_timeout_s + probe`. (005-R40 to R43)
- [ ] **T523** (ros-dev) Implement `ready` and the probe in `ManagerSequencer` until T522 passes. Depends on T522.
- [ ] **T524** (ros-dev) Thin glue in `sequencer_node.py`: the `ready` callable (get_parameters, get_state, is_active, settle),
  the `is_active` probe, node states in the notification text and telemetry keys; parameters declared and in
  `config/lifecycle_sequencer.yaml`. Depends on T523.
- [ ] **T525** (ros-dev) Live-graph test, deterministic (no race needed): a test double that serves `/lifecycle_manager_nav/manage_nodes`
  and `/is_active` on one single-threaded group and blocks forever in `manage_nodes` (the same shape as the Nav2 wedge);
  assert `DEGRADED_NAV`, `MANAGER_UNRESPONSIVE`, the group's nodes deactivated, no non-zero `cmd_vel`, ordered shutdown with
  `stop_guard` last; both distros, `--cpus=1`, 5 runs. Depends on T524. (005-R42, R43, R38)
- [ ] **T526** (architect) After approval: merge section 7 into `spec.md` (R40 to R44, edit of R37, R29), update
  `docs/lifecycle-managers.md` (residual and manual remedy, plain-node list unchanged) and `docs/diagrams/lifecycle-managers.mmd`
  (+ render with `scripts/render-diagrams.sh`), apply the contract entries of section 8 when the contract slot is free.
- [ ] **T527** (devops) Check that `scripts/test-docker.sh` (or a loop around it) can repeat the live-graph tests N times at
  `--cpus=1` per distro, and add the option if it is missing; record the cold-start count needed by the reopening criterion
  (one wedge in 20) in the CI notes. No change to the Nav2 manager, to `launch` respawn or to the images.

If the schedule slips, cut order within this proposal: T527, then the node-state text of 3.3, then L1 for the optional
manager. Do not cut T522 to T525 for the nav and core managers.

## 10. What is not verified

- The Nav2 sources were read from GitHub (`jazzy` and `lyrical` branches), **not** from the installed binaries: the
  `vacuum-ci` images do not contain Nav2 (rosdep installs it at test time), so the shipped patch versions are not checked.
- The RMW in use is not pinned in the repo. `librmw_fastrtps_cpp` is present in the Jazzy image; Lyrical was not
  inspected. The "failed to send response (timeout)" string is the rclcpp server-side warning when no client reader is
  matched yet (the string is present in the rclcpp-based libraries of the Jazzy image; the exact rmw return path is not
  re-read here).
- Whether `rmw_service_server_is_available` on the manager's side also checks the reply path matching (it would make the
  Lyrical argument of 2.2 stronger) was not checked.
- What a Nav2 server node does when its bond to a killed manager breaks (relevant only if bonds were enabled).
- The probability of the race on the real robot is unmeasured; the only data are the PR #27 runs (1 failure in the first
  run before staging, 1 in 5 with staging only, none in 10 Jazzy and 5 Lyrical runs with the staged start and the sequencer fix).
- The effort estimates are the architect's estimates, not measured.
