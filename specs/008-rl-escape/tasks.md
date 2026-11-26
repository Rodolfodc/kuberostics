# 008: Tasks

`[P]` marks a task that can run in parallel with other `[P]` tasks of the same phase. Each task cites the requirements it meets.
A task is done only when the test or report that verifies its requirements exists and passes.

**Pre-conditions (architect, not a task of the agents).**
- The project owner confirms D-29, the 2D environment.
- The "Contract needs" of the spec go into the next serialized contract batch before T812.

**Schedule.**

| Date | Tasks |
|---|---|
| 2026-11-25 to 26 | T801, T802, T803, T804, T807 |
| 2026-11-27 | T805, T806, T808 |
| 2026-11-28 to 29 | T809, T810 |
| 2026-11-30 | T811 |
| **2026-12-01** | **Go/no-go (008-R23)** |
| 2026-12-01 to 02 | T812, T813 |
| 2026-12-02 to 03 | T814 |
| 2026-12-04 | Freeze |

If the go/no-go fails, T812 to T814 are dropped, `strategy` stays `scripted`, and the result is reported (Art. IX.3).

**External dependencies.**
- Spec 005: T509 (the real `escape_server` node; only `EscapeScript` exists today) and T515 (the 20-pose baseline in the simulator).
- T503 for the footprint and the speed limits (Q-18, Q-20).

## Phase 0: setup

- [ ] **T801** [P] Write `vacuum_coverage/core/escape_policy.py` (`ObservationBuilder` v1, `ACTIONS` v1, `PolicyInference`,
  `LoadGate`, all pure Python with numpy). Write `test/test_escape_policy.py` with fixed scans, a committed synthetic
  weights file, a golden pair, and the rejection cases. Add the `EscapeScript.start_fallback` entry point and its test. Both
  distros. Owner: ros-dev. Depends on: none. (008-R08, R09, R13, R16, R22)
- [ ] **T802** [P] Create the `rl/` skeleton:
  - `requirements.txt` (pinned versions of the plan);
  - `run.sh` (Docker `python:3.12-slim`, `--cpus=4`, repo read-only, `rl/runs` read-write, `PYTHONPATH=src/vacuum_coverage`);
  - `configs/env_v1.yaml` and `configs/ppo_v1.yaml`;
  - `README.md`;
  - a pytest smoke test that imports `vacuum_coverage.core.escape_script`.

  Owner: rl-trainer. Depends on: none. (008-R11)
- [ ] **T803** [P] Track the training image `python:3.12-slim` in `IMAGES_ALL` of `scripts/setup_env.sh` and add the `rl/`
  row to the setup table. The `CLAUDE.md` row is proposed to the orchestrator. Owner: devops. Depends on: T802. (008-R11, R22)

## Phase 1: 2D environment

- [ ] **T804** [P] `geometry.py`, `kinematics.py`, `laser2d.py` and `env2d.py`: frames, wheel model, sub-steps, contact,
  laser with noise, dropout and delay, `check_env`, determinism by seed. Owner: rl-trainer. Depends on: T802; it imports T801 for
  the observation (until T801 merges, a local stub with the same signature, deleted before T809). (008-R01 to R04, R07)
- [ ] **T805** `scenarios.py` (corner, leg, gap, wall, distractors, seed splits) and the reward of R06, with tests: each
  class starts in contact with no overlap, and the reward terms are checked on hand cases. Owner: rl-trainer. Depends on: T804. (008-R05, R06)
- [ ] **T806** `scripted_agent.py`: run `EscapeScript` in the env (primary only, and full) on validation seeds. This is the first
  2D baseline table. Owner: rl-trainer. Depends on: T805. (008-R10, R20)

## Phase 2: calibration (needs the simulator, local only)

- [ ] **T807** [P] Simulator runs for motion, laser and contact, from the plan's calibration table: the combined robot model,
  `use_sim_time`, `tools/sim.sh`, a scripted layout of primitives. Report the turn rate at 1.0 rad/s (Q-20) and the
  footprint centre (Q-18). Raw logs go in `.sim/` and the summary numbers in the hand-back. Owner: sim-operator. Depends on: the sim
  bridge (present). (008-R19 a to c)
- [ ] **T808** Fit `tau_s`, `k_lin`, `k_ang` and the contact mode into `env_v1.yaml`. Report the replay and laser errors. Rebuild the
  T515 poses as primitives (Q-19 option c) and report the sim-to-sim gap of `EscapeScript` when T515 exists. Owner:
  rl-trainer. Depends on: T804, T807; for the gap, T515. (008-R19)

## Phase 3: training and export

- [ ] **T809** Train PPO with `ppo_v1.yaml`. Run one seed end to end first, then seeds 1 to 3. Keep the best checkpoint on the validation seeds. Training
  runs in the background with a CPU limit. Owner: rl-trainer. Depends on: T801, T805 (T808 if it is ready; otherwise train with the
  wide ranges and retrain after T808). (008-R11)
- [ ] **T810** `export.py`: weights JSON, `.sha256` sidecar and `golden.json`. Add a parity test that reloads with `PolicyInference`
  (logits within 1e-4; 100 % argmax on rows with a top-2 margin of at least 1e-4). Owner: rl-trainer. Depends on: T809, T801. (008-R12, R13)
- [ ] **T811** `evaluate.py` on the test seeds with all four configurations of the plan. Paste the table into `rl/README.md`
  and give the go/no-go verdict of R23 with its numbers. Owner: rl-trainer. Depends on: T806, T810. (008-R20, R23)

## Phase 4: integration (only if T811 passes the go/no-go)

- [ ] **T812** `escape_server` adds `strategy` and `policy_path` with the policy-then-fallback sequence, the load gate on CHARGED
  and the mode subscription. Add the telemetry keys and the launch arguments in the 005 escape launch. Run unit and node tests on Jazzy
  and Lyrical. Owner: ros-dev. Depends on: T509, T801, the contract batch. (008-R14 to R18, R22)
- [ ] **T813** `launch_testing`: run `strategy: policy` with the synthetic weights of T801 and a fake mode topic. Check that there is no load
  outside CHARGED, that it loads on CHARGED, that a rejected file falls back to `scripted`, and that only `mux/escape` is
  published. Owner: ros-dev. Depends on: T812. (008-R15, R16, Art. V.2)

## Phase 5: validation in the simulator (local only)

- [ ] **T814** Run the T515 poses with `strategy: scripted` and `strategy: policy` (the weights of T810), `use_sim_time`, once per pose
  and strategy. Record success, duration, final phase, collisions and any new `StuckEvent` within 10 s. Put the summary in
  `rl/README.md`, written by rl-trainer from the sim-operator's numbers. Owner: sim-operator. Depends on: T515, T812, T810. (008-R21)

## Traceability

| Requirement | Tasks |
|---|---|
| 008-R01 to R04, R07 | T804 |
| 008-R05, R06 | T805 |
| 008-R08, R09 | T801 |
| 008-R10 | T806 |
| 008-R11 | T802, T803, T809 |
| 008-R12 | T810 |
| 008-R13 | T801, T810 |
| 008-R14 to R18 | T812, T813 (R16: also T801) |
| 008-R19 | T807, T808 |
| 008-R20 | T806, T811 |
| 008-R21 | T814 |
| 008-R22 | T801, T803, T812 |
| 008-R23 | T811 |
