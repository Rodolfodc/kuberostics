# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## State of the repo

Spec-driven project (final project of IA368II, FEEC/UNICAMP: autonomous vacuum robot in CoppeliaSim's *House* scene, ROS 2). **Implementation has started** (spec 001): `deps.repos`, `scripts/setup.sh`, `scripts/setup_env.sh`, `src/vacuum_interfaces`, `src/vacuum_supervisor` (`SubmodeMachine` + the `supervisor` LifecycleNode, with tests) and `src/vacuum_bringup` (`twist_mux.yaml`, `lifecycle_core.yaml`, `skeleton.launch.py`) and `src/vacuum_safety` (`stop_guard`, D-22, see `docs/stop-guard.md`) and `src/vacuum_stubs` (`coverage_server`, `escape_server`, `stuck_detector`) and `src/vacuum_detection` (pure-Python `SelectorCore` and `ProjectorCore` with tests, from PR #11, spec 009a; the ROS nodes are pending); T001, T002, T006 to T013 and T017 to T019 are done. Dockerfiles, CI, the contract tests and the launch tests are still *planned* in `specs/001-skeleton/tasks.md`.

This is its own git repo, nested inside the outer folder `projeto_final_ia386ii/` (not a repo; it only holds symlinks `.claude` and `CLAUDE.md` to this repository's files, see `scripts/link-workspace.sh`). That folder also holds two files outside version control:
- `scene_house.ttt` (~29 MB): the CoppeliaSim *House* scene; the open question about scene object names (spec 000 §6) is answered by inspecting it.
- `aspirador_specs.zip` and `aspirador-ia368ii-updated.zip`: packaged snapshots of this repo (the second includes `docs/`). They are not sources of truth; never edit them, change the files in this repo instead.

Everything in the repository is written in **English** (D-21): specs, docs, comments, commit messages, file and directory names. The third-party orchestrator is called `IA368II` (its repository name), never by its author's name.

**Keep this file current:** once the tasks in `specs/001-skeleton/tasks.md` produce real code, replace the "Planned commands" section with the actual build, lint, test and single-test commands, and update "State of the repo".

## Reading order (authoritative)

1. `.specify/memory/constitution.md` — Articles I–X. A spec that contradicts an article is invalid until the article is explicitly amended.
2. `specs/000-system/spec.md` — system boundaries, frozen inherited contract, decisions `D-xx`, open questions `Q-xx`, spec map (001…010).
3. `specs/000-system/contracts/topics.yaml` — machine-readable interface contract (name, type, QoS profile, owner, `frozen`, `max_publishers`); `interfaces.md` holds the draft `.msg/.action` definitions.
4. `docs/architecture.md` — Mermaid architecture and communication-flow diagrams (overview, motion control, offload perception, cloud cycle, selector, failure/recovery, sync at base). Sources are `docs/diagrams/*.mmd` (source of truth); `docs/diagrams/render/` is generated with `mmdc` (see the header of `architecture.md`). When a spec changes a topic or flow, update the matching `.mmd` and re-render.
5. `specs/NNN-*/{spec.md, plan.md, tasks.md}` — per feature (layout compatible with GitHub Spec Kit, optional use). Existing: 000, 001-skeleton, 009a-offload.

## Rules that shape every change

- **Contract first (Art. I):** any new topic/service/action/public parameter goes into `contracts/topics.yaml` (+ `.msg/.srv/.action` in `vacuum_interfaces`, which has no implementation) **in the same commit** as the code.
- **Third-party code is read-only (Art. II):** `IA368II` (`myrobot_bringup`, `myrobot_interfaces`, `my_robot_pkg`) is pulled via `vcs import src/deps < deps.repos`, pinned by commit (`51a5fa2c…`), and `src/deps/` is gitignored. Adapt only via, in order: submodes published by our own node, launch remapping, same-name replacement node honoring the contract, targeted inheritance. Never subclass/replace `BehaviourManager` or extend the `Mode` enum (CLEANING, AUTODOCKING, CHARGING, CHARGED, IDLE); new behaviours are *submodes* on `/myRobot/behaviour/submode` (D-01), and behaviours with a result are ROS 2 actions (D-02).
- **Motion safety (Art. VI, D-03, D-22):** `twist_mux` publishes NOTHING while the stop lock is on, so `stop_guard` feeds a zero `Twist` into the `stop` input (priority 250 > lock 200 > driving inputs); see `docs/stop-guard.md`. Also: only `twist_mux` publishes `/myRobot/cmd_vel`. *Actuator* nodes publish only to their own mux input (`/myRobot/mux/{teleop,escape,coverage,docking}`, priorities 100/80/50/40, plus `mux/stop` at 250 from `stop_guard`; the `lock_stop` lock itself is at 200); *service* nodes never publish motion. The IA368II `cleaning_server` is remapped to a sink (`/myRobot/legacy/cleaning_cmd_vel`, D-04).
- **Two ROS distros (Art. X):** every package must build and pass tests on both Jazzy (Ubuntu 24.04, Py 3.12) and Lyrical Luth (Ubuntu 26.04, Py 3.14); one distro per run, no mixed graphs. Third-party defaults that differ across distros must be set explicitly: `use_stamped: false` on `twist_mux` and `enable_stamped_cmd_vel: false` on every Nav2 node that publishes velocity (D-13). Motion contract is unstamped `geometry_msgs/Twist`.
- **Modules are pluggable (Art. III):** each module is a ROS 2 package with its own launch, runnable alone with stubs/rosbag; new nodes are `LifecycleNode`s under our own Nav2-style lifecycle manager `lifecycle_manager_core` (cloud/MQTT under the separate `lifecycle_manager_optional`, D-09). Stubs are replaced only when the real implementation passes the same contract tests (Art. IV).
- **Cloud is optional (Art. VII):** the robot must clean without network; new policy weights load only in mode CHARGED.
- **Traceability (Art. VIII):** requirements are EARS with IDs (e.g. `001-R04`); tasks cite requirements, tests cite the requirement they verify, commit messages cite the task ID (e.g. `T004`).
- **Deadline 2026-12-04 23:59; cut order if late:** LLM agent → cloud retraining → RL policy (keep the scripted escape). The six core modules are never cut.

## Design directives (D-18 to D-20, spec 000)

- Modular code: pure-Python logic with no ROS imports, thin ROS nodes, dependencies injected by the caller.
- Reusable components (controllers, filters, planners, detectors) are toolbox classes: they only compute, with no publishing, topic reads or policy. The caller chooses configuration, limits, timing and what to do with the result (style example: a `PID` that only turns error and `dt` into an output).
- Logs and telemetry everywhere possible: log every state transition and event, publish periodic metrics; pure logic takes a logger/metrics callback.

## Architecture (big picture, per plan 001/009a)

- CoppeliaSim runs on the **host**, reached only through the ZeroMQ Remote API (port 23000) by Python bridge nodes — the simROS2 plugin is not used (D-14). ROS containers (`docker/jazzy`, `docker/lyrical`) use `network_mode: host`.
- Chain: `behaviour_server` (IA368II, publishes mode every 0.2 s) → our `supervisor` (hierarchical state machine → submode) → actions `CoverRoom` / `Escape` on `coverage_server` / `escape_server` → `twist_mux` → `/myRobot/cmd_vel` → 2025S2 wheel bridge (`my_robot_cmd_vel_node`, `/cmd_vel` remapped, D-06) → CoppeliaSim. `stuck_detector` emits `StuckEvent` to the supervisor.
- Sensor topic names keep the 2025S2 ones (`/scan`, `/camera`, `/camera_depth`, `/myrobot/odom` lowercase); new modules take them as parameters (D-07). Battery/charging-base signals are still simulated by the IA368II `myrobot_server` (D-08).
- Transition logic lives in pure-Python classes with no ROS imports (`SubmodeMachine`, `selector_core.py` circuit breaker) tested with pytest; the `LifecycleNode` is thin glue. Follow this pattern for new state logic.
- Spec 009a splits detection: `detector` (YOLO, RGB → `Detection2DArray`, copying the image header; runs local `yolo11n` or in K3s `yolo11x`), `detection_selector` (circuit breaker + age/watermark filters), and `projector` (always on the robot: 2D detection + depth + TF at the photo timestamp → `/detections_markers` for the unchanged 2025S2 semantic node, D-16). Only compressed RGB leaves the robot; robot↔cloud transport (Q-06: `rmw_zenoh` vs `zenoh-bridge-ros2dds`) is undecided and health is measured at the application layer.
- Open questions that affect code are tracked in `specs/000-system/spec.md` §6 (CoppeliaSim version/scene object names, Lyrical compatibility of the IA368II package, pip wheels on Python 3.14, odometry source for SLAM).

## Planned commands (from the plans; not yet runnable)

```bash
vcs import src/deps < deps.repos          # fetch pinned IA368II dependency
colcon build && colcon test               # CI runs these without the simulator, matrix distro: [jazzy, lyrical]
python tools/sim_bench/bench.py           # steps/s via ZeroMQ; run CoppeliaSim headless (-h); ≥200 steps/s ⇒ train RL in-sim (spec 008)
tools/chaos/kill_detector.sh              # 009a fault injection: delete detector Pod 5×, records OffloadStatus to rosbag
```

Contract tests (`vacuum_contract_tests`) are generated from `topics.yaml` and check existence, type, QoS, `max_publishers` and `sink` topics, including `Twist` on `/myRobot/cmd_vel`. Tests needing CoppeliaSim run locally and must be marked as such (Art. V).

## Local LLM for operational chores

For operational, low-stakes work (summarizing logs, drafting boilerplate, reformatting, quick lookups), delegate to the local llama.cpp server, if installed (optional) instead of doing it all in the main session. It is an OpenAI-compatible router at `http://127.0.0.1:11434/v1` (models in `~/models/models.ini`, e.g. `qwen3-8b`, `qwen3.5-9b`, `llama-3.1-8b`). If it is down, start it with the command in `~/models/llama-server-flags.md`. Do not use it for contract, safety or architecture decisions (constitution Articles I, II, VI): those stay with Claude and the specs. Its output is a draft and must be reviewed before it is committed.

## Agent workflow

Subagents live in `.claude/agents/`. The main session is the orchestrator: it reads the task, picks the owner by the paths below, and routes work. Each agent writes only in its own paths.

| Agent | Writes in |
|---|---|
| architect | `specs/`, `docs/`, `.specify/` (never `constitution.md`) |
| ros-dev | `src/vacuum_*` |
| devops | `docker/`, `.github/`, `deps.repos`, `compose.yaml`, `scripts/`, `deploy/`, `infra/`, `tools/chaos/`, `tools/k3s.sh`, `tools/tf.sh` |
| sim-operator | `tools/sim_bench/`, `tools/sim.sh`, `.sim/` |
| rl-trainer | `rl/` (starts only after D-11 and spec 008 exist) |
| reviewer | nothing (read-only) |

**Review gate:** the reviewer runs only on a *completed* deliverable (a finished task group, spec or artifact), not on each iteration. It returns `APPROVED` or `CHANGES_REQUESTED` with findings (`file:line`, article/requirement, owner). On `CHANGES_REQUESTED` the orchestrator sends the findings back to the owning agent, and the re-review checks the previous findings plus the delta. After 3 rounds without approval, escalate to the user. Nothing is merged without `APPROVED`.

**Contract changes** (`topics.yaml`, `interfaces.md`) are serialized: one in flight at a time.

**Git workflow (user rule):** never commit on `main`. Create a branch from `main` (`feat/…`, `docs/…`, `fix/…`, `chore/…`), commit there with the task ID in the message, and when the deliverable is finished and verified, push the branch and open a PR with `gh pr create`. Opening the PR is authorized; merging is left to the user.

**Dangerous tools** go through wrappers, never raw: `tools/sim.sh` (headless CoppeliaSim on a copy of the scene, kills only its own PID), `tools/k3s.sh` (local k3d cluster `kuberostics`, own kubeconfig), `tools/tf.sh` (fixed to `infra/`). `.claude/settings.json` allows read-only verbs and asks for `apply`, `destroy`, `delete`, `docker`, `pip install` and `git push`; `.claude/hooks/protect.sh` blocks writes to `src/deps/`, `scene_house.ttt`, `.git/` and the constitution, force pushes, killing processes by name, `docker.sock` and `--privileged`.


## Where to start Claude Code

Claude Code reads `.claude/` (agents, settings, hooks) and `CLAUDE.md` from the folder where it starts. The easiest place is this repository. If you start from the folder above it (the one that holds the House scene file, which is not under git), run `scripts/link-workspace.sh` once: it creates symlinks `../.claude` and `../CLAUDE.md` to the tracked files here, so the six agents and the protection hook load from either place. The real files stay in this repository. Agents load only when a session starts, so restart Claude Code after creating or changing them. From the parent folder, commands are `aspirador-ia368ii/tools/...`, so the relative allow rules in `.claude/settings.json` (e.g. `Bash(tools/sim.sh *)`) do not match and those commands ask for permission.

## Environment setup

**Docker first (user rule).** The user will not always have access to the machine this runs on, so build, test, probe and render inside Docker containers and never rely on something installed only on the host. ROS 2 on the host is **optional**; Docker is **required**. Do not tell the user to `sudo apt install` a ROS package: put the dependency in a `package.xml` (rosdep installs it in the containers) or in the right script.

**Everything that must be installed is tracked, in exactly these places:**

| What | Where it is tracked |
|---|---|
| ROS package dependencies (e.g. `twist_mux`, `python3-pytest`, `python3-yaml`) | `exec_depend` / `test_depend` in each `src/vacuum_*/package.xml`; `scripts/test-docker.sh` installs them with rosdep in the containers |
| Docker images: `osrf/ros:jazzy-desktop-full`, `ros:lyrical-ros-base`, `minlag/mermaid-cli:12.0.1` | `scripts/setup_env.sh` (list `IMAGES_ALL`; `--no-images` skips the pull) |
| Host tools: Docker, `kubectl`, `k3d`, `terraform`, firewall for ports 23000-23099 | `scripts/setup_env.sh`, `scripts/setup-tools.sh` (needs sudo) |
| CoppeliaSim 4.10.0 (runs on the host, not in Docker) | `scripts/setup_env.sh` |
| Python tools outside ROS (`.venv`: ZMQ client, pyzmq, cbor, numpy, pytest) | `requirements.txt` |
| Third-party ROS code (IA368II) | `deps.repos` |
| apt packages on the host (`jq`, `unzip`, `poppler-utils`, `vcstool`, ...) | `APT_PKGS` in `scripts/setup_env.sh` |

`scripts/setup_env.sh` is the single entry point to reproduce the machine (`--check` only verifies). Whenever you add a system package, image, tool or Python dependency, add it to the table's place **in the same change**. **Not installed by it and optional:** ROS 2 on the host, and llama.cpp (see "Local LLM" below; never make a task depend on it).

## Building and testing here (verified)

Use Docker (see "Docker first" above). The repo is mounted read-only and copied inside the container, so nothing is written to the working tree.

```bash
scripts/test-docker.sh               # both distros: Jazzy (osrf/ros:jazzy-desktop-full) and Lyrical (ros:lyrical-ros-base)
scripts/test-docker.sh lyrical       # one distro; prints "Summary: N tests, 0 errors, 0 failures"
tools/probes/mux_lock_probe.sh lyrical   # the REAL twist_mux with our config (also: jazzy); needs only Docker
scripts/render-diagrams.sh           # docs/diagrams/*.mmd -> docs/diagrams/render/*.svg, via the mermaid-cli image
```

The scripts exist because writing `rm`/`>` next to the third-party checkout path in an inline command trips `.claude/hooks/protect.sh` (false positive); keep such commands inside a script file.

Optional, only if ROS 2 Lyrical is installed on the host (`/opt/ros/lyrical`; the shell is zsh, so use `bash -c 'source ...'`; `~/.local/bin/python3.11` shadows the system Python and breaks `rosidl`, hence `-DPython3_EXECUTABLE=/usr/bin/python3`; build outside the repo with `--build-base/--install-base/--log-base` under `/tmp`):

```bash
# one test: cd src/vacuum_supervisor && /usr/bin/python3 -m pytest test/test_submode_machine.py -k R07
```

`ament_python` packages need `tests_require` **and** `extras_require={'test': ['pytest']}` in `setup.py`, or colcon reports "0 tests" on Lyrical's newer setuptools. `geometry_msgs/Pose2D` does not exist in Lyrical (D-17); check message types on both distros before using them.

The CoppeliaSim model of the robot is `models/myRobot.ttm` (exported from the 2025S2 `pega_banana.ttt`); the House scene has no robot. D-11 (steps/s) is **not** measured yet: the full robot ran at ~3.5 steps/s with the GUI and `-h` crashes on the Kinect render, so use `SIM_HEADLESS=0 tools/sim.sh start`.
