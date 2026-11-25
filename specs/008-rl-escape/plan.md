# 008: Plan

Minimal version first (Art. IX.1): one observation version, one action table, PPO with the default MLP, four
scenario classes made of primitives, and comparison against `EscapeScript`. Nothing from the cloud.

## Layout

```
src/vacuum_coverage/vacuum_coverage/core/
  escape_policy.py        # ros-dev: ObservationBuilder, ACTIONS (v1), PolicyInference, LoadGate (pure, numpy only)
  escape_script.py        # spec 005, unchanged (the env and the node reuse it)
  scan_utils.py           # spec 005, unchanged (sector_min is the success test for both strategies)
src/vacuum_coverage/test/
  test_escape_policy.py   # fixed vectors; synthetic weights + golden committed under test/data/ (tiny, not a trained policy)
rl/                       # rl-trainer only
  README.md               # how to build, train, evaluate; results table (R20) and go/no-go (R23)
  requirements.txt        # pinned: gymnasium==1.4.0, stable-baselines3==2.9.2, torch==2.15.0 (CPU), numpy, pyyaml, pytest
  run.sh                  # docker run python:3.12-slim --cpus=4, repo mounted read-only, rl/runs mounted rw,
                          #   PYTHONPATH=src/vacuum_coverage (imports escape_policy, escape_script, scan_utils)
  configs/env_v1.yaml     # geometry, randomization ranges, reward weights, scenario mix, seed splits
  configs/ppo_v1.yaml     # PPO hyper-parameters, total steps, n_envs, seeds
  escape_rl/
    geometry.py           # segments, discs, boxes; ray casting; disc overlap
    kinematics.py         # diff drive about the axle centre, wheel lag, caps, sub-steps (R02)
    laser2d.py            # 452 beams from base_link, noise, dropout, delay (R03)
    scenarios.py          # corner, leg, gap, wall + distractors; seed splits (R05)
    env2d.py              # EscapeEnv2D (R01, R04, R06, R07)
    scripted_agent.py     # EscapeScript adapter (R10)
  scripts/
    train.py  evaluate.py  export.py  calibrate.py
  tests/                  # pytest, run inside run.sh
  runs/                   # gitignored: checkpoints, tensorboard, CSV, weights JSON, golden.json
```

The import direction is one way: `rl/` imports `vacuum_coverage.core.*` through `PYTHONPATH`. No package in `src/`
imports from `rl/`. `vacuum_coverage/core` has empty `__init__.py` files and needs no ROS to import (checked
2026-11-25).

## Observation, action and inference (D-29)

`ObservationBuilder(n_sectors=30, range_cap_m=2.0, budget_s, n_actions=8)` takes:
- `reset(start_pose)`
- `build(ranges, pose, elapsed_s, prev_action) -> list[float]` (43 values, layout 008-R08)

The env and the node call it with the same arguments. The env passes its simulated scan and the true pose with odometry noise; the
node passes `LaserScan.ranges` and the odometry pose. Neither does any other preprocessing.

`PolicyInference.load(path) -> (ok, reason)`; `act(obs) -> int`; `logits(obs) -> list[float]`. It is a plain
matrix product with `tanh`. The PPO actor is `obs -> Linear(43,64) -> tanh -> Linear(64,64) -> tanh -> Linear(64,8)`, the
SB3 default `pi=[64,64]` (`mlp_extractor.policy_net` plus `action_net`). The export reads these modules from the PPO
policy's `state_dict`. No `VecNormalize` is used: the observation is already in [0, 1], so no running statistics need
exporting.

`LoadGate` (pure) decides whether a load may happen. Its inputs are the last mode, whether a goal is running, the current sha and the file sha.
It returns `load`, `skip` or `reject(reason)` (008-R16).

## 2D environment

- **Frames.** The pose is that of the axle centre `(x, y, yaw)`. `base_link` and the laser are 0.07 m behind it. The footprint
  disc is centred at `footprint_offset_x_m` from `base_link` (default 0.07, Q-18).
- **Step.** Map the action to `(v, w)` with the table, apply the wheel model (lag `tau_s`, gains `k_lin` and `k_ang`, caps), and
  integrate 10 sub-steps of 0.01 s. A sub-step that overlaps an obstacle is not applied and is counted as contact. Then
  cast the laser (or return the previous scan when the delay is 1), build the observation and compute the reward.
- **Speed.** Use vectorised numpy ray casting against at most about 10 primitives, with 452 rays. The aim is at least 1,000
  env steps/s per core, so 1 M steps take about 4 min on 4 workers (`SubprocVecEnv`). This figure is an estimate; the first run measures it.
- **Randomization per episode** (initial ranges in `env_v1.yaml`, narrowed after calibration): `tau_s` 0.05 to 0.3,
  `k_lin` 0.8 to 1.0, `k_ang` 0.55 to 0.8, `sigma_m` 0 to 0.02, `p_drop` 0 to 0.03, scan delay 0 or 1 step, odometry noise
  0 to 2 % of the distance travelled.
- **Seeds.** Train seeds 0 to 9,999, validation 10,000 to 10,999 and test 20,000 to 20,999. The class mix is 25 % each.

## Training (PPO, SB3 2.9.2)

`PPO("MlpPolicy", env, n_steps=512, batch_size=256, n_epochs=10, gamma=0.99, gae_lambda=0.95, ent_coef=0.01,
learning_rate=3e-4, seed=s)` with 8 envs, 1 M steps per run, and seeds `s` in {1, 2, 3}. These are starting values, not
tuned. Every 50 k steps, evaluate on the validation seeds and keep the best checkpoint by success rate. Log to TensorBoard
under `rl/runs/<run_id>/`. The order is to get one seed working end to end (train, export, parity) before running three.

## Evaluation

- **2D (R20).** The same test seeds for:
  - the policy (5 s, policy phase only);
  - the policy followed by the scripted fallback (16.5 s, what the node runs);
  - `EscapeScript` primary only (5 s);
  - `EscapeScript` full (16.5 s).

  `scripts/evaluate.py` writes a CSV to `rl/runs/` and prints a Markdown table. The owner pastes the table into `rl/README.md`.
- **Simulator (R21).** The sim-operator reuses the T515 harness and poses: for each pose, reset the scene, place the
  robot, trigger the stuck event (the `~/trigger` hook or a forward push), and call `escape` with
  `strategy: scripted` and then `strategy: policy`. Every node runs with `use_sim_time: true`. At about 4 steps/s the
  simulator runs at about 0.2 of real time, so wall-clock budgets would expire after about 3.4 s of simulated time (Q-13 of
  spec 003). Cost: 20 poses x 2 strategies x up to 17 s of simulated time (340 steps) is about 85 s of wall time each,
  about 1 hour in total.

## Calibration (R19)

| Check | Simulator run (sim-operator, T807) | 2D fit (rl-trainer, T808) |
|---|---|---|
| Motion | Empty area. For each action of the table: hold it 3 s, then stop 1 s. Log the ground-truth pose (`/myRobot/Ground_truth`) and the odometry every step with sim time. Also turn at `turn_speed_rad_s` 1.0 for 90 deg (Q-20). | Fit `tau_s`, `k_lin` and `k_ang` by least squares on the pose; report the 3 s replay error. |
| Laser | At least 10 static poses at 0.2 to 1.5 m from a wall, a box and a 0.03 m cylinder placed by script. Log the raw `distSignal` and the ground-truth pose. | Ray-cast the same layout and compare beam by beam: median error and miss rate. |
| Contact | Drive forward at 0.15 m/s into a wall at 0, 30 and 60 deg incidence for 3 s; log the pose. | Choose between "block the sub-step" (default) and "slide along the wall". |
| Gap | T515 poses rebuilt as 2D primitives (Q-19 option c) | Run `EscapeScript` in 2D on the same poses and compare with T515. |

The scenes are built by script over ZeroMQ on a copy of the scene (`tools/sim.sh`). Raw logs stay in `.sim/`; only the
fitted numbers and the summary go into `rl/configs/env_v1.yaml` and `rl/README.md`.

## escape_server changes (ros-dev, after T509 and the contract batch)

- New parameters: `strategy`, `policy_path`, `policy_check_period_s`, `policy_range_cap_m`, `mode_topic`.
- New subscription to the mode topic (QoS `ia368ii`). A timer runs every `policy_check_period_s` and calls `LoadGate`.
- Goal flow with `policy` and weights loaded:

  ```
  start -> POLICY (10 Hz: obs -> act -> mux/escape; front clear -> success)
             | primary_budget_s spent or inference error
             v
           EscapeScript fallback (FALLBACK_SWEEP -> FACE -> ADVANCE) -> success / failure
  005-R12 timeout, cancel and deactivate apply to the whole goal
  ```

  `EscapeScript` gets an entry point that starts directly in the fallback (for example
  `start_fallback(event_ranges, pose, ...)`). It is a small addition in `escape_script.py` with its own tests. The
  primary logic does not change.
- The publication path, the timeout and the stop of 005-R12 are shared with the scripted strategy. Only the source of the command
  changes.
- Launch: `strategy` and `policy_path` are launch arguments of `stuck_escape.launch.py`, defaults `scripted` and `""`.
  Swapping is done in the launch alone (Art. III.5).

## Tests

| Requirement | Test |
|---|---|
| R08, R09, R13 | `vacuum_coverage/test/test_escape_policy.py`: fixed scans (all `inf`, all NaN, a 452-beam wall at 0.3 m, a scan with fewer beams); committed synthetic weights with known logits; rejection for each bad field; sha mismatch; tie goes to the lowest index |
| R16 | `LoadGate` truth table (mode x goal running x sha changed x file valid) |
| R14, R15, R17 | Core test of the policy-then-fallback sequencer with a fake policy (always "forward"; raises an error; yields NaN). The node test uses a fake mode topic, fake scan and odometry, and asserts that only `mux/escape` is published, the result is `rl` or `scripted`, and that nothing loads outside CHARGED. |
| R01 to R07, R10 | `rl/tests/`: `check_env`; determinism by seed; the kinematics of each action in free space; the laser against analytic distances; that contact never ends an episode; that each class starts in contact; the scripted adapter runs to a result |
| R12, R13 parity | `rl/tests/test_export.py`: export then reload with `PolicyInference`, 100 % argmax match on `golden.json` |

## Risks

- **Reality gap at corners (Q-18).** If the real footprint is not centred on the axle, turning in place scrapes, and the 2D env
  is optimistic. The calibration contact check and the R19 d gap figure measure this. The fallback stays scripted either way.
- **Turn rate (Q-20).** If the robot turns at only about 0.65 of the command, both strategies lose time. The policy learns it
  through `k_ang`, and the script does not adapt. This is reported, not hidden.
- **Schedule.** Nine days remain. T509 (the real `escape_server`) and T515 (the baseline) are not done, and the R21 simulator
  validation depends on both. If they slip, 008 stops at the 2D result (R20, R23), which is still a valid deliverable.
- **Training throughput.** If the env runs below about 300 steps/s per core, cut the beams cast to the 30 sector
  centres plus their neighbours. This is allowed only if the laser check of R19 b still passes on the down-sampled set.
