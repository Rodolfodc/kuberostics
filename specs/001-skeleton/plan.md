# 001: Technical plan

## Repository structure

```
kuberostics/
├── .specify/memory/constitution.md
├── specs/
│   ├── 000-system/{spec.md, contracts/}
│   └── 001-skeleton/{spec.md, plan.md, tasks.md}
├── deps.repos                       # IA368II pinned by commit (D-10)
├── src/
│   ├── vacuum_interfaces/        # .msg and .action, no code (Art. I)
│   ├── vacuum_bringup/           # launch, twist_mux.yaml, lifecycle_core.yaml
│   ├── vacuum_supervisor/        # submode state machine
│   ├── vacuum_safety/            # stop_guard (D-22)
│   ├── vacuum_stubs/             # coverage_server, escape_server, stuck_detector, teleop_fake
│   └── vacuum_contract_tests/    # tests generated from topics.yaml
├── tools/sim_bench/                 # steps/s measurement
├── tools/probes/                    # checks of third-party nodes (mux_lock_probe.sh)
├── docker/{jazzy,lyrical}/Dockerfile  # one container per distribution (D-12)
├── compose.yaml                     # starts the chosen container with network_mode: host
└── .github/workflows/ci.yaml        # matrix distro: [jazzy, lyrical]
```

The IA368II code comes in through `vcs import src/deps < deps.repos` and lives in `src/deps/`, which git ignores.

## Dependencies between nodes

```
behaviour_server (IA368II) ──mode──▶ supervisor ──submode──▶ (observation)
                                        │  ├──action──▶ coverage_server ──▶ mux/coverage ─┐
stuck_detector ──StuckEvent──▶──────────┘  ├──action──▶ escape_server   ──▶ mux/escape   ─┤
                                           └──Bool────▶ mux/lock_stop                      ├─▶ twist_mux ─▶ /myRobot/cmd_vel ─▶ wheel bridge (2025S2) ─▶ CoppeliaSim
autodocking_server (IA368II) ──remap──▶ mux/docking ─────────────────────────────────────────┤
teleop ──────────────────────────────▶ mux/teleop ──────────────────────────────────────────┘
cleaning_server (IA368II) ──remap──▶ /myRobot/legacy/cleaning_cmd_vel (no subscriber, D-04)
```

## Supervisor: state machine

Inputs: orchestrator mode, `StuckEvent`, action results.
Internal state: `submode`, `resume_submode` (where to return after the escape) and `locked` (the stop lock, independent of the submode).

| Current state | Event | Action | Next |
|---|---|---|---|
| any | mode ∉ {CLEANING, AUTODOCKING} | cancels ongoing actions; **does not touch the lock** (001-R22) | NONE |
| NONE | mode = CLEANING | sends `cover_room`, unless the lock is on (001-R22) | COVERING |
| COVERING | mode = AUTODOCKING | cancels `cover_room` (001-R21) | NONE |
| COVERING | `StuckEvent` | cancels `cover_room`; `resume_submode = COVERING`; sends `escape` | ESCAPING |
| NONE with mode AUTODOCKING | `StuckEvent` | `resume_submode = NONE`; sends `escape` | ESCAPING |
| ESCAPING | `escape` succeeded | resends the action of `resume_submode`, if any | `resume_submode` |
| ESCAPING | `escape` failed or timed out | `lock_stop = true`; ERROR `Notification` | ESCAPE_FAILED |
| any, lock on | service `/myRobot/supervisor/reset` (from the user) | `lock_stop = false` (001-R22) | NONE if it was ESCAPE_FAILED, else unchanged |
| ESCAPING | mode changes | the escape continues; `resume_submode` = COVERING for CLEANING, NONE for AUTODOCKING (001-R21) | ESCAPING |

EXPLORING and GOTO_ROOM exist in the type but only come into use in specs 003 and 005.

The logic lives in a pure Python class (`SubmodeMachine`), without ROS, tested with pytest.
The `LifecycleNode` only wires that class to the topics and actions. This way the transition rules are testable without the simulator.

## twist_mux

```yaml
twist_mux:
  ros__parameters:
    use_stamped: false        # D-13: the twist_mux 4.5 default is true on both distributions
    topics:
      teleop:   {topic: /myRobot/mux/teleop,   timeout: 0.5, priority: 100}
      escape:   {topic: /myRobot/mux/escape,   timeout: 0.5, priority: 80}
      coverage: {topic: /myRobot/mux/coverage, timeout: 0.5, priority: 50}
      docking:  {topic: /myRobot/mux/docking,  timeout: 0.5, priority: 40}
      stop:     {topic: /myRobot/mux/stop,     timeout: 0.5, priority: 250}   # D-22: zero velocity from stop_guard
    locks:
      stop:     {topic: /myRobot/mux/lock_stop, timeout: 0.0, priority: 200}  # blocks every input below 200
```

While a lock is active `twist_mux` publishes **nothing**, so `stop_guard` (D-22, `docs/stop-guard.md`) publishes a zero `Twist` on the `stop` input, which outranks the lock. The `cmd_vel_out` output is remapped to `/myRobot/cmd_vel`. With `use_stamped: false`, the inputs and the output are `Twist`.

## Two distributions

- **Containers:** `docker/jazzy` starts from the official `ros:jazzy` image and `docker/lyrical` from the official Lyrical image
  (confirm the tag in T003). Both install the same dependencies through `rosdep`.
- **Network:** CoppeliaSim runs on the host. The container uses `network_mode: host` to reach the ZeroMQ port
  (23000) and for DDS to discover the nodes.
- **CI:** the same sequence of steps in `strategy.matrix.distro: [jazzy, lyrical]`.
- **Contract:** the contract test runs on both and explicitly checks the `Twist` type on `/myRobot/cmd_vel` (001-R18).

## Contract test

`vacuum_contract_tests` reads `specs/000-system/contracts/topics.yaml`, brings up the skeleton launch and, for
each interface, queries `get_publishers_info_by_topic` and `get_subscriptions_info_by_topic`. The test fails if:

- the topic does not exist;
- the type differs from the declared one;
- the QoS differs from the declared one;
- a topic with `max_publishers` has too many publishers;
- a `sink` topic has subscribers.

This test is the executable form of spec 000.

Where it runs: the library and unit tests run in the default `colcon test`. The live checks are opt-in locally (`VACUUM_CONTRACT_LIVE=1`) and run automatically, with no variable, as a `launch_testing` test against `skeleton.launch.py` with `with_ia368ii:=false`, `with_wheel_bridge:=false`, `with_teleop:=false` (T022). The IA368II-dependent checks (001-R12, IA368II manager) need the IA368II checkout and are covered only when T023 is done; until then they are open (T003b, T014).

### Domain ID allocation

Tests that bring up a graph use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and a random `ROS_DOMAIN_ID` from a range reserved per package, so parallel `colcon test` runs do not see each other:

| Package | Range |
|---|---|
| `vacuum_stubs` | 10-39 |
| `vacuum_safety` | 40-69 |
| `vacuum_supervisor` | 70-99 |
| `vacuum_bringup` | 100-129 |
| `vacuum_contract_tests` | 130-159 |

## Steps/s measurement (D-11)

`tools/sim_bench/bench.py` connects through the ZeroMQ Remote API, turns on `sim.setStepping(True)`, runs N calls of
`sim.step()` with the robot driving in a straight line and prints steps/s. It must run with CoppeliaSim started without
a graphical interface (`-h`).

Decision rule of spec 008:
- **200 steps/s or more:** train directly in CoppeliaSim;
- **less than that:** train in the 2D environment, with validation in CoppeliaSim.

## Risks

- `twist_mux` publishes nothing (not zero) while the stop lock is active; solved by `stop_guard` (D-22). If `stop_guard` is not running, a locked robot keeps its last command: it must be in `lifecycle_manager_core` and in the launch file, and the T014 test must assert zero on `/myRobot/cmd_vel`, not only silence from the inputs.
- Object names in the *House* scene differ from the 2025S2 scene (Q-01). Mitigation: the names become parameters in the wheel bridge, through remapping or parameters of the 2025S2 node.
- Start-up order between the IA368II lifecycle manager and `lifecycle_manager_core`. Mitigation: the new nodes do not depend on its nodes to activate, only to receive the mode.
