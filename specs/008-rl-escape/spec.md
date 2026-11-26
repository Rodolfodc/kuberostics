# 008: Learned escape policy (RL)

Status: draft for review · 2026-11-25 · Depends on: 005 (`EscapeScript`, real `escape_server` T509, baseline T515), D-11 · Used by: 009b (retraining, cut first)
Assignment module: 5 (innovation). **Cut candidate (Art. IX.3, third in the cut order):** if it is late, the scripted escape of spec 005 stays the default and nothing else changes. The LLM agent (010) and the cloud retraining (009b) are cut before this spec.

## Goal

Train a small escape policy with Stable-Baselines3 in a 2D Gymnasium environment, compare it with the scripted
`EscapeScript` of spec 005 on the same stuck poses, and make it selectable in the existing `escape_server` with the
parameter `strategy: policy`. The policy replaces only the primary phase of the escape: the deterministic fallback
sweep of 005-R11, the timeouts of 005-R12 and the mux input `/myRobot/mux/escape` stay as they are (Art. VI.3).
Weights are loaded only in mode CHARGED (Art. VII.3) by a pure-Python inference class that imports neither ROS nor torch.

## Out of scope

- Training in CoppeliaSim (see D-29). CoppeliaSim is used only to calibrate the 2D model and to validate the policy.
- Cloud retraining, MQTT, the digital twin and weight approval (009b). In this spec the weights file is copied by hand.
- The Kinect. The policy uses only the laser and the odometry (this is the answer to the question D-11 leaves to 008).
- Changing `Escape.action`, `StuckEvent.msg` or the mux priorities.
- Making `policy` the default strategy. That is the project owner's decision after the evaluation (008-R21).

## Decisions of this spec

| ID | Status | Decision | Rationale |
|---|---|---|---|
| D-29 | **Proposed** (architect, 2026-11-25); the project owner must confirm, D-11 leaves this choice to them | The escape policy is trained in a 2D Gymnasium environment (`rl/`) calibrated against CoppeliaSim, and validated in CoppeliaSim on the poses of T515. It is not trained in CoppeliaSim. | The policy needs the laser. On the combined model (`myRobotLaser`, GUI mode, `.sim/laser_sweep_bench.out`) one sweep of the 452-beam laser takes about 0.22 s of wall time: 3.96 steps/s with a sweep every step (N=1), 7.0 with N=2, 10.9 with N=4, and 21.8 with the sweep switched off (N=1000). Some starts stall at about 0.95 steps/s and are discarded. The 319 steps/s of D-11 was measured headless on a robot **without the laser and without the Kinect**, so it does not apply. Because the sweep costs about 0.22 s, a laser-equipped robot cannot go above about 4.5 steps/s at N=1 headless either. That figure is inferred and was not measured; headless with the Kinect crashes (D-11). The plan 001 rule (200 steps/s or more trains in the simulator) is therefore not met for any robot that has a laser. |
| D-30 | Proposed | The policy replaces only the **primary phase** of the escape and uses its budget, `primary_budget_s`. When that budget ends, the same fallback sweep of `EscapeScript` runs (005-R11). The time budgets of 005-R10 to R12 do not change. | Art. VI.3 requires a deterministic fallback. A learned policy is a deterministic function, but its behaviour cannot be checked by hand. Reusing the budgets keeps the 005-R11 c check valid with no new numbers. |
| D-31 | Proposed | The observation builder, the action table and the inference class live in **one** pure-Python module, `vacuum_coverage/core/escape_policy.py`. It needs numpy and imports neither ROS nor torch. ros-dev owns it. The 2D environment in `rl/` imports that module; it does not copy it. | A mismatch between the observations seen in training and at run time is the most likely silent failure. One implementation removes it. rl-trainer writes only in `rl/` and ros-dev only in `src/vacuum_*` (CLAUDE.md), so the module goes to the package that runs it. |

## Facts this spec relies on (checked 2026-11-25)

| Fact | Value | Source |
|---|---|---|
| Robot drive | wheel radius 0.05 m, separation 0.20 m, caps 0.5 m/s and 90 deg/s, command timeout 0.5 s, axle centre 0.07 m ahead of `base_link` | `vacuum_sim_bridge/profile.py` |
| Laser | 452 beams, `angle_min` -90 deg, increment pi/450 rad (last beam +90.4 deg), range 0.055 to 10 m, a miss written as 0 becomes `inf`, mounted at `base_link` (0, 0, 0.16) with no yaw. **No rear coverage.** | `profile.py`, `scan_core.py`, `.sim/bridge_live_fixed2.out` |
| Footprint radius | 0.18 m (from the brief). Where its centre is (axle centre or `base_link`) is open, see Q-18. `CoveragePlanner` uses 0.16 m, see Q-18. | brief; 005-R17 |
| Measured turn rate | commanded 0.5 rad/s gave a measured 18.2 to 20.5 deg/s (0.32 to 0.36 rad/s, averaged over 4.6 to 5.2 s, acceleration included). The turn centre moved 0.10 to 0.11 m during a 90-degree turn. Measured once per run; the cause is not established. | `.sim/bridge_live_drive2.out`, `.sim/bridge_live_wheels_odom.out` (local, not committed) |
| Escape limits and speeds | `reverse_speed_m_s` 0.15, `turn_speed_rad_s` 1.0, `max_linear_speed_m_s` 0.30 and `max_angular_speed_rad_s` 1.0 (both placeholders until T503), `primary_budget_s` 5, `fallback_budget_s` 11.5, `default_timeout_s` 17, `publish_rate_hz` 10, `front_clearance_m` 0.40, front sector +-0.35 rad | `topics.yaml` (`escape_server`), `escape_script.py` |
| Python packages | `gymnasium` 1.4.0 (py3), `stable-baselines3` 2.9.2 (py3), `torch` 2.15.0 (wheels cp311 to cp315), `numpy` 2.5.5 | PyPI JSON API, 2026-11-25. Q-05 for training: wheels exist for Python 3.14, but training still runs outside ROS (008-R11). |
| SB3 PPO default policy | `net_arch` `pi=[64, 64]`, `vf=[64, 64]`, activation `Tanh` | `stable_baselines3/common/policies.py` at tag v2.9.2 |

## Usage scenarios

1. **Offline training.** The owner runs `rl/scripts/train.py` with a committed config. The scenario sampler places the
   robot in contact in one of four classes, PPO trains, and the run directory (gitignored) holds checkpoints, the
   exported weights JSON and the golden vectors.
2. **Comparison in 2D.** `rl/scripts/evaluate.py` runs the policy and `EscapeScript` on the same held-out seeds and
   reports the success rate, the median time and the timeout rate for each class.
3. **Use on the robot.** The launch sets `strategy: policy` and `policy_path`. The robot docks and the mode becomes
   CHARGED. `escape_server` checks the file and loads it. In CLEANING, a `StuckEvent` makes the supervisor call
   `escape`: the policy drives `mux/escape` for at most `primary_budget_s`, and if the front is still blocked the
   scripted fallback sweep runs.
4. **No weights or bad weights.** `strategy: policy` with a missing, invalid or mismatched file runs the scripted
   escape, logs a WARN and reports this in telemetry. The robot keeps cleaning.

## Requirements (EARS)

2D environment (`rl/`, owner rl-trainer).

| ID | Requirement | Verification |
|---|---|---|
| 008-R01 | The environment `EscapeEnv2D` shall implement the Gymnasium `Env` API with `observation_space = Box(0, 1, (43,), float32)` (layout in 008-R08) and `action_space = Discrete(8)` (table in 008-R09), shall advance 0.1 s of simulated time per `step` (10 Hz, the `publish_rate_hz` of `escape_server`), and with the same `reset(seed=...)` and the same actions shall produce the same trajectory. | unit + `gymnasium.utils.env_checker.check_env` |
| 008-R02 | Kinematic model: differential drive about the **axle centre**, which is `axle_offset_x` (0.07 m) ahead of `base_link`. The commanded `(v, w)` is clipped to the profile caps (0.5 m/s, 90 deg/s) and split into wheel speeds (separation 0.20 m); each wheel follows a first-order lag `tau_s` with gain `k_lin` (linear part) and `k_ang` (angular part). The model integrates in sub-steps of at most 0.01 s and applies the 0.5 s command timeout. `tau_s`, `k_lin` and `k_ang` come from calibration (008-R19). Until then the defaults are `tau_s` 0.1 s (not verified), `k_lin` 1.0 and `k_ang` 0.68 (middle of the measured 0.64 to 0.72), each randomized per episode within the ranges of the env config. | unit |
| 008-R03 | Laser model: 452 beams from -90 deg in steps of pi/450 rad, cast from `base_link` (0.07 m behind the axle centre) against the 2D obstacles. A ray that hits beyond 10 m or nothing, or a true range below 0.055 m, returns `inf`. Per episode the model randomizes Gaussian noise `sigma_m`, a dropout rate `p_drop` (a dropped beam returns `inf`, as the model's 0 does) and a scan delay of 0 or 1 step. The laser sees nothing behind -90 to +90.4 deg. | unit |
| 008-R04 | Contact model: the robot is a disc of radius `footprint_radius_m` (default 0.18) centred at `footprint_offset_x_m` from `base_link` (default 0.07, the axle centre; Q-18). A sub-step whose pose would make the disc overlap an obstacle is not applied; the pose stays and that step counts as a contact step. Contact never ends the episode. | unit |
| 008-R05 | The scenario sampler shall generate the four classes of 005-R15: `corner` (two perpendicular walls, robot touching one or both, heading +-45 deg around the bisector), `leg` (1 to 4 discs of radius 0.015 to 0.04 m, chair or table spacing 0.35 to 0.6 m, robot touching one leg), `gap` (two obstacles with a gap of 0.36 to 0.42 m, robot wedged in it), `wall` (one wall, heading 0 to +-70 deg to its normal, robot touching it). Each class adds 0 to 3 distractor obstacles. The initial pose is in contact (clearance 0 to 0.01 m). Seeds are split into train, validation and test sets, and test seeds are never used in training. | unit (each class generates, initial pose in contact, no overlap) |
| 008-R06 | Reward per step: `-0.01` (time), `-0.1` per contact step, plus potential-based shaping `gamma * Phi(s') - Phi(s)` with `Phi` = front clearance / `range_cap_m`; `+10` on success; `-5` on truncation. These weights are initial and the owner may tune them; every change is recorded in the env config. | unit |
| 008-R07 | The episode shall terminate with success when the front clearance (minimum valid range within +-`front_sector_half_width_rad` of 0, the same `scan_utils.sector_min` call used by `escape_server`) is at least `front_clearance_m` (0.40). The success test is the same for both strategies. It shall be truncated after `primary_budget_s` (5.0 s, 50 steps). The secondary metric `free_after` (in evaluation only, not in the reward) checks geometrically that a straight 0.30 m advance at the end pose touches nothing. | unit |
| 008-R10 | The environment shall be able to run `EscapeScript` (vacuum_coverage, pure Python) as the agent through an adapter that feeds it pose, front range and `rear_range=None` and turns its `Command` into the kinematic input, with the full primary + fallback budget (16.5 s). This gives the scripted baseline in 2D. | unit |
| 008-R11 | Training shall be reproducible. The config and seeds are committed under `rl/configs/`. Runs, checkpoints, CSVs and weights are written under `rl/runs/` (gitignored). Every run records the git sha, the config hash and the package versions. Training runs in a Docker container with Python 3.12 and a CPU limit (default `--cpus=4`), in the background. | inspection |

Shared observation and inference (`vacuum_coverage/core/escape_policy.py`, owner ros-dev, D-31).

| ID | Requirement | Verification |
|---|---|---|
| 008-R08 | `ObservationBuilder` (observation version 1) shall produce 43 float32 values in [0, 1]: (a) 30 sectors. Sector `i` covers beam indices `floor(i*n/30)` to `floor((i+1)*n/30) - 1` of the live scan (`n` beams; 452 expected). Its value is the minimum valid range, where NaN and values <= 0 are dropped and `inf` is kept; an empty sector or `inf` becomes `range_cap_m` (2.0 m). The value is clipped to [0, `range_cap_m`] and divided by `range_cap_m`. (b) The previous action, one-hot over 8 (all zero at the start). (c) The elapsed time of the policy phase divided by `primary_budget_s`, clipped to [0, 1]. (d) The odometry displacement since the goal start, in the start frame: `(dx, dy) / 0.5 m` clipped to [-1, 1] and mapped to [0, 1], then `(sin dyaw + 1) / 2` and `(cos dyaw + 1) / 2`. The builder takes plain lists and floats and no messages. | unit (fixed vectors) |
| 008-R09 | Action table version 1 (`v = reverse_speed_m_s` 0.15, `w = turn_speed_rad_s` 1.0, the same speeds the script uses, so the comparison is fair and the 005-R11 c limit check holds): 0 forward `(+v, 0)`, 1 reverse `(-v, 0)`, 2 left `(0, +w)`, 3 right `(0, -w)`, 4 forward-left `(+v, +w/2)`, 5 forward-right `(+v, -w/2)`, 6 reverse-left `(-v, +w/2)`, 7 reverse-right `(-v, -w/2)`. | unit |
| 008-R12 | The export shall write a JSON weights file with: `format: "vacuum-escape-policy/1"`, `obs_version: 1`, `n_obs: 43`, `n_actions: 8`, `action_table_version: 1`, the actor layers in order (`W` as a list of rows, `b`, `activation` `tanh` or `linear`), `range_cap_m`, `n_sectors`, the training run id, the git sha and the config hash. A sidecar `<file>.sha256` holds the file hash. Weights are not committed (`.gitignore` rule; they are supplied by path, as the run-time models are). The export shall also write `golden.json` with at least 200 observations (taken from test-seed episodes) and the logits and argmax that SB3 computes for each. | unit (rl) |
| 008-R13 | `PolicyInference` shall load the file of 008-R12. It shall reject it, with a reason, on a wrong `format`, `obs_version`, `n_obs`, `n_actions` or layer shapes, on a non-finite value, or on a sha256 that differs from the sidecar. `act(obs)` shall return the argmax of the logits (ties go to the lowest index), so the result is deterministic. It computes in float32. On `golden.json` the logits shall be within 1e-4 of SB3, and the action shall match SB3 on 100 % of the rows whose top-2 logit margin is at least 1e-4; the near-tie rows are counted and reported. It imports numpy and the standard library only. | unit (both sides: rl with SB3, ros-dev with a committed synthetic weights + golden pair) |

`escape_server` with `strategy: policy` (owner ros-dev, in the real node of T509).

| ID | Requirement | Verification |
|---|---|---|
| 008-R14 | Where `outcome` is `auto` and `strategy` is `policy` and weights are loaded, when an `Escape` goal is accepted, the server shall, every `1 / publish_rate_hz` s, build the observation (008-R08) from the latest `/scan` and odometry, publish the action of 008-R09 on `/myRobot/mux/escape` and nowhere else, and stop with success as soon as the front clearance is at least `front_clearance_m`. When `primary_budget_s` ends without success, it shall run the `EscapeScript` fallback sweep (005-R11) within `fallback_budget_s`. The goal timeout, cancel and stop rules of 005-R12 hold unchanged. The result `strategy` shall be `"rl"`, the value defined in `Escape.action`. The parameter value is `policy` and the result string is `rl`: the action file is not changed. | unit (core) + node |
| 008-R15 | Where `strategy` is `policy` and no valid weights are loaded (empty `policy_path`, file missing or rejected by 008-R13), the server shall run the scripted escape of 005-R08 to R11, return `strategy = "scripted"`, log a WARN once per goal and report `policy_loaded = false` in its telemetry. Where `strategy` is `scripted` (default), behaviour is exactly that of spec 005. `strategy` is ignored when `outcome` is not `auto`. | node |
| 008-R16 | The server shall load or reload the weights file only while the last mode received on `mode_topic` (default `/myRobot/behaviour/mode`) is `CHARGED` and no goal is running (Art. VII.3). It shall check the file when the mode changes to CHARGED and then every `policy_check_period_s` (default 10 s) while it stays CHARGED, and shall reload only when the sha256 changed. A rejected file keeps the weights already loaded, if any. Every load, rejection and reason shall be logged and counted in telemetry. | unit (load gate, pure) + node (fake mode topic) |
| 008-R17 | If inference raises an error or yields a non-finite logit during a goal, the server shall abandon the policy phase at once, log a WARN and run the fallback sweep (008-R14). | unit + node |
| 008-R18 | The telemetry of `escape_server` (`/myRobot/telemetry/escape_server`, D-20) shall add these keys: `strategy`, `policy_loaded`, `policy_sha256` (first 12 characters), `policy_run_id`, `policy_loads`, `policy_rejects`, `last_phase` (`policy`, `fallback` or `scripted`), `policy_steps_last_goal`. | node |

Calibration and evaluation.

| ID | Requirement | Verification |
|---|---|---|
| 008-R19 | Needs simulator: the 2D model shall be calibrated against CoppeliaSim (combined model, `use_sim_time`): (a) **motion**: step and turn commands at the action-table speeds, using ground-truth pose. In open-loop replay the 2D model shall stay within 0.05 m and 5 deg of the simulator after 3 s (initial thresholds). (b) **laser**: at least 10 static poses in front of known primitives (wall, box, cylinder of leg size). The median beam-wise error between 2D and simulated ranges shall be at most 0.03 m, and the miss rate per pose shall be recorded. (c) **contact**: driving forward into a wall at 0.15 m/s, record whether the robot stops, slides or yaws, and choose the 2D contact model from that. (d) **sim-to-sim gap**: the 2D success rate of `EscapeScript` on the replicated T515 poses shall be within 0.15 of the T515 success rate (initial threshold). If any check fails, the gap is reported and the randomization ranges are widened. That does not block training. | sim (local) + report |
| 008-R20 | On test seeds (at least 200 episodes per class), the evaluation shall report for the policy (policy phase only, 5 s) and for `EscapeScript` (primary phase alone, and primary + fallback): success rate, median and p90 time to success, truncation rate, contact steps and `free_after`. It shall report these per class and in total, with 3 training seeds and their spread. | script + committed summary table (numbers only; raw CSVs stay local) |
| 008-R21 | Needs simulator: on the same at least 20 stuck poses as T515, each strategy (`scripted`, `policy`) shall run once per pose through the real `escape_server` with `use_sim_time`. The run shall record success, duration, final phase, collisions and whether a new `StuckEvent` follows within 10 s of resuming. The default `strategy` changes to `policy` only on the project owner's decision and only if the policy's success rate is at least the scripted one and it has no more failed escapes. | sim (local) + report |
| 008-R22 | The ROS parts (`escape_policy.py`, the `escape_server` changes and their tests) shall build and pass on Jazzy and Lyrical (Art. X). Training (`rl/`) runs in its own Python 3.12 container, outside ROS (Q-05). The robot never imports torch or SB3. | CI matrix |
| 008-R23 | Go/no-go on **2026-12-01**. If by then the policy's 2D test success rate (008-R20, policy phase) is below the success rate of `EscapeScript`'s primary phase on the same seeds, or its `free_after` rate is below that of `EscapeScript`'s primary phase (so a policy that only turns until the front sector reads 0.40 m, still facing the trap, does not pass), or 008-R13 parity fails, the RL policy is cut (Art. IX.3). `strategy` stays `scripted`, and the negative result is reported with its numbers. | report |

## Contract needs (not applied here; for the next serialized contract batch, Art. I)

New public parameters of `escape_server` (`topics.yaml`, `parameters.escape_server`):

- `strategy`: string, default `scripted`, values `[scripted, policy]`, 008-R14/R15, applies only with `outcome: auto`
- `policy_path`: string, default `""`, 008-R15/R16
- `policy_check_period_s`: double, default 10.0, 008-R16
- `policy_range_cap_m`: double, default 2.0, 008-R08. It must equal the `range_cap_m` in the weights file, or the file is rejected.
- `mode_topic`: string, default `/myRobot/behaviour/mode`, 008-R16

New consumer: `escape_server` subscribes to `/myRobot/behaviour/mode` (frozen IA368II topic, QoS `ia368ii`). Add it to the consumers of that entry.

Unchanged: `Escape.action` (the result `strategy` uses the existing value `"rl"`), `StuckEvent.msg`, `/myRobot/mux/escape`.

## Amendments to spec 005 required by this spec (not applied; for the architect)

Amendment B1 (from 008-R14 and R15). These amendments apply only when the T812 integration happens. If the
go/no-go fails, they are dropped.
1. 005-R08 to R11 apply where `strategy` is `scripted` (the default) or where no valid weights are loaded. Where
   `strategy` is `policy`, the policy phase (008-R14) replaces the attempts of R08 to R10, and R11 (the fallback) runs
   after it.
2. 005-R13: the result `strategy` is `"scripted"` or `"rl"`, as 008-R14 and R15 define.
3. 005-R33: the escape launch gains the arguments `strategy` (default `scripted`) and `policy_path` (default `""`).
4. 005-R12 and 005-R14 are unchanged.

## Follow-ups outside this change (architect)

- Record D-29 to D-31 in spec 000 §5 and amend the D-11 text with the laser figures (after the owner confirms D-29).
- `docs/architecture.md` and its diagram source show a separate node `escape_rl`. With D-31 it is `escape_server` with
  `strategy: policy`. Update the `.mmd` sources of the cloud cycle and dock sync and re-render.
- `CLAUDE.md` setup table: the `rl/requirements.txt` and the training image (devops task T803).

## Definition of done

- 008-R01 to R10 and R12 to R18 tested; the ROS parts green on Jazzy and Lyrical.
- 2D evaluation table (R20) and go/no-go decision (R23) recorded in `rl/README.md`.
- If the go/no-go passes: simulator calibration (R19) and validation (R21) recorded, with the parameters used.
- If not: the negative result recorded and `strategy: scripted` unchanged. That counts as done for the course; Art. IX.3 allows it.

## Open questions

Numbers Q-18 to Q-22 were free on every local branch on 2026-11-25; renumber them at merge if they collide.

- Q-18: footprint. Is the 0.18 m radius centred on the axle centre or on `base_link`, and is the body a disc at all?
  This decides whether turning in place can touch an obstacle, which is what makes corners hard. T503 and T807
  measure it. Until then the 2D default is a disc centred on the axle (turning in place never touches). This makes the
  2D env optimistic for corners. Also reconcile with `robot_radius_m` 0.16 of 005-R17.
- Q-19: source of the 2D geometry for the replicated T515 poses (R19 d, R21). Options: (a) the saved SLAM map of
  spec 003 as an occupancy grid, ray-cast per cell; (b) the bounding boxes of the House shapes that cross the laser
  plane (z = 0.16 m), exported over ZeroMQ; (c) hand-built primitives around each pose. Default: (c) for 20 poses,
  because it is fastest and needs nothing from spec 003.
- Q-20: the measured turn rate is 0.64 to 0.72 of the command (0.5 rad/s). Is that the wheel bridge, joint
  saturation or caster friction? If `turn_speed_rad_s` 1.0 also gives only about 0.65 rad/s, the scripted budgets of
  005 (one attempt in 5 s) shrink further. T515 and T807 must report the real turn rate at 1.0 rad/s. This affects 005, not only 008.
- Q-21: a front-only laser makes reversing blind (`rear_range=None`), for the script and for the policy. Should the
  bumper (`/myRobot/bumper` exists in the model, Q-11) be added to the observation in a version 2? Not in version 1.
- Q-22: where the approved weights file lives for the demo (the repo forbids committed weights). Options: a GitHub
  release asset or a local path on the demo machine. The default is a local path set in the launch argument.
