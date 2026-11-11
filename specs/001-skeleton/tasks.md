# 001: Tasks

`[P]` = can be done in parallel with other `[P]` tasks of the same phase. Each task cites the requirements it meets.
A task is only done when the test that verifies the requirement passes.

## Phase 0: preparation

- [x] **T001** Create `deps.repos` with `IA368II` at commit `51a5fa2c…`. Write `scripts/setup.sh`, which runs `vcs import` and `rosdep install`. (D-10)
- [x] **T002** [P] Create `vacuum_interfaces` with `Submode`, `StuckEvent`, `Notification`, `Escape` and `CoverRoom`, copied from `contracts/interfaces.md`. Verify with `colcon build` and `ros2 interface show`. (001-R04)
- [ ] **T003** [P] Create `docker/jazzy` and `docker/lyrical` (confirm the tag of the official Lyrical image) and `.github/workflows/ci.yaml` with matrix `[jazzy, lyrical]`: setup, `colcon build`, `colcon test`. (001-R15, 001-R17)
- [ ] **T003b** Build and bring up the IA368II `myrobot_bringup` and `my_robot_pkg` in the Lyrical container and record the result in Q-04 of spec 000. Depends on T001 (`deps.repos`) and T003 (Lyrical image and tag). Not parallel with T003. (001-R17)
- [ ] **T004** [P] Write `tools/sim_bench/bench.py`, run it on the *House* scene and record D-11. Answer Q-01 along the way. (001-R14)

## Phase 1: tests first

- [ ] **T005** Write `vacuum_contract_tests`: a library that reads `topics.yaml` (including the `since` semantics of the contract header), unit tests of that library, and opt-in live tests (`VACUUM_CONTRACT_LIVE=1`, needing a running graph). Under default `colcon test` only the library and unit tests run; the live tests are skipped, so T005 alone does NOT check 001-R03, 001-R04 or 001-R12 against a graph. That is done by T022. (001-R03, 001-R04, 001-R12)
- [x] **T006** [P] Write the pytest tests of `SubmodeMachine`, one row of the transition table per case. (001-R05 to 001-R09)

## Phase 2: implementation

- [x] **T007** Create `vacuum_bringup/config/twist_mux.yaml` with `use_stamped: false` (D-13). (001-R10, 001-R11, 001-R18)
- [x] **T008** Implement `SubmodeMachine` until T006 passes. (001-R05 to 001-R09)
- [x] **T009** Implement the `supervisor` `LifecycleNode`: subscribes to the mode, publishes the submode at 5 Hz, action client, lock and notifications. (001-R05 to 001-R09)
- [x] **T017** [P] Write the pytest tests of `StopGuard`. (001-R19)
- [x] **T018** Implement `StopGuard` and the `stop_guard` `LifecycleNode`; add the `stop` input and the lock priority 200 to `twist_mux.yaml`. (001-R10, 001-R19, 001-R20)
- [x] **T019** [P] `tools/probes/mux_lock_probe.sh`: probe of the real `twist_mux` with our config. (001-R10, 001-R19)
- [x] **T010** [P] `coverage_server` stub: `CoverRoom` server that publishes a constant `Twist` on `mux/coverage` while the goal is active. (001-R06)
- [x] **T011** [P] `escape_server` stub with the parameter `outcome` (`succeed`, `fail`, `hang`): reverses and turns for 2 s on `mux/escape`. (001-R07, 001-R16)
- [x] **T012** [P] `stuck_detector` stub: publishes a `StuckEvent` when the `~/trigger` service is called. Real detection is left to spec 005. (001-R07)
- [x] **T013** Create `skeleton.launch.py`, with:
  - the six IA368II nodes and its lifecycle manager, with the remappings of D-04 and D-05;
  - `lifecycle_manager_core`, managing `stop_guard`, `coverage_server`, `escape_server`, `stuck_detector` and `supervisor` (in this order, the supervisor last);
  - `twist_mux`;
  - the 2025S2 wheel bridge, with the remapping of D-06;
  - `teleop_twist_keyboard` with output on `mux/teleop`.

  Launch arguments: `with_ia368ii` (default true), `with_wheel_bridge` (default true, only effective when `with_ia368ii` is true; the bridge connects to CoppeliaSim at startup) and `with_teleop` (default true; teleop needs a terminal, so headless and Docker runs pass `with_teleop:=false`).
  Done: the launch description is unit-tested without launching (remaps, parameters, only `twist_mux` outputs `/myRobot/cmd_vel`); running the graph is T014. (001-R01, 001-R02, 001-R12, 001-R13)

## Phase 3: verification

- [ ] **T014** Write `launch_testing` tests for scenarios 1 to 4, using the escape stub with each `outcome`. Launch with `with_teleop:=false` (and without the wheel bridge when no simulator is running). Also check on the live graph that `/myRobot/cmd_vel` has `twist_mux` as its only publisher, that both lifecycle managers report their nodes active (001-R02), and that each core node's real name matches `lifecycle_core.yaml`. (001-R06 to 001-R11)
- [ ] **T022** Wire the live contract checks of T005 into `launch_testing` against `skeleton.launch.py` with `with_ia368ii:=false with_wheel_bridge:=false with_teleop:=false`, on both distros, with no environment variable (the test sets up its own graph). Isolation as in T014 and PR #7: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and a random `ROS_DOMAIN_ID` inside the contract-test range (130-159, see plan 001 "Domain ID allocation"). The test must fail while the launch does not exist or while `/myRobot/cmd_vel` has a publisher other than `twist_mux`. Covers 001-R03 and 001-R04 for the interfaces that do not need IA368II (the stubs, supervisor, `stop_guard`, `twist_mux` topics). Open, explicitly: 001-R12 (`legacy/cleaning_cmd_vel` sink, `mux/docking` from `autodocking_server`) and the IA368II lifecycle manager are NOT covered here because they need the IA368II nodes; they stay open under T003b/T014 until T023 provides a checkout path. Owner: ros-dev. Depends on T005 and T014. (001-R03, 001-R04, 001-R17)
- [ ] **T023** CI/Docker path with the IA368II checkout (`vcs import` of `deps.repos` inside the container, no CoppeliaSim, wheel bridge off), so that the IA368II-dependent checks run in CI on both distros: 001-R12 (sink and docking remap), the IA368II lifecycle manager (001-R02) and the live contract tests with `with_ia368ii:=true`. Owner: devops. Depends on T003 and T003b (if T003b finds that IA368II does not build on Lyrical, this task runs on Jazzy only and Q-04 records the gap). (001-R02, 001-R12, 001-R15, 001-R17)
- [ ] **T015** Run on CoppeliaSim and record the short video of scenarios 1 to 4. (001-R13)
- [ ] **T016** Review spec 000: close Q-01, record D-11 and close Q-04 with the result of T003b (Q-02 is already closed by D-13). Depends on T004 (bench result, Q-01, D-11) and T003b (Q-04). (001-R14, 001-R17)
