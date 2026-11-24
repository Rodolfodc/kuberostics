# 000: System specification

Status: draft for review · 2026-11-04

This document fixes the system boundaries, the inherited contract and the decisions that apply to all feature
specifications (001 onwards). It does not describe module behaviour in detail; that lives in specifications 002 to 008.

## 1. Context

Differential-drive vacuum robot (about 32 cm) in the *House* scene of CoppeliaSim. Sensors: 2D laser,
RGB-D camera and bumper. The system covers the six modules of the course assignment. The innovation is:

- an escape policy learned by reinforcement learning;
- a cloud service emulated on K3s.

## 2. Actors

| Actor | Interaction |
|---|---|
| User | Teleoperates, clicks goals, edits rooms and schedule, receives notifications. |
| IA368II orchestrator | Decides the operating mode (unchanged). |
| CoppeliaSim | Provides sensors and executes the wheel velocities. |
| Cloud service (optional) | Stores history, retrains the policy and publishes approved weights. |

## 3. Specification map

| Spec | Content | Assignment module |
|---|---|---|
| 001-skeleton | End-to-end integration with stubs | all |
| 002-control | Teleoperation, clicked goal | 1 |
| 003-mapping | Exploration, SLAM, segmentation, labels | 2 |
| 004-recognition | YOLO, 3D position, ontology | 3 |
| 005-coverage-and-escape | Global path, coverage, avoidance, stuck detection, scripted escape | 5 |
| 006-schedule | Manual and autonomous schedule | 4 |
| 007-interface | Map, room editing, notifications | 6 |
| 008-rl-escape | Gymnasium environment, training, validation, implementation swap | 5 (innovation) |
| 009a-offload | Offloading the YOLO detector to K3s, with circuit breaker and local fallback (reference architecture: FogROS2) | 3 + innovation (professor's suggestion) |
| 009b-cloud | MQTT bridge, K3s, retraining cycle | innovation |
| 010-llm-agent | User complaint → training trap, as validated JSON | optional |

## 4. Inherited contract (frozen)

Source: the `IA368II` repository, commit `51a5fa2c816c7ee712c963843e5d747b3ed2a98f`.
The complete machine-readable list is in `contracts/topics.yaml`.

**Orchestrator (`myrobot_bringup`)**

- Managed nodes: `myrobot`, `behaviour_server`, `cleaning_server`, `autodocking_server`,
  `charging_server`, `charged_server`.
- Mode: `/myRobot/behaviour/mode` (`std_msgs/String`), republished every 0.2 s.
  Values: CLEANING, AUTODOCKING, CHARGING, CHARGED, IDLE.
  The IA368II nodes treat any unknown value as IDLE.
- User command: `/myRobot/start_stop` (`myrobot_interfaces/srv/StartStop`, `"start"` or `"stop"`).
- Telemetry: `/myRobot/battery_state`, `/myRobot/charging_base/strengthSignal`,
  `/myRobot/charging_base/relativeAngle`.
- QoS of all topics: depth 1, reliable, volatile.

**2025S2 project (`my_robot_pkg`, reused in modules 2 and 3)**

- Bridge to CoppeliaSim through the ZeroMQ Remote API, in plain (non-lifecycle) nodes.
- Publishes `/scan`, `/camera`, `/camera_depth` and `/myrobot/odom` (lowercase).
- Subscribes to `/cmd_vel`.
- The perception nodes publish `/topologic_map`, `/detections_markers` and `/semantic_map`.

## 5. Architecture decisions

| ID | Decision | Rationale |
|---|---|---|
| D-01 | The top-level modes remain the orchestrator's; the new behaviours are **submodes**, published by the `supervisor` on `/myRobot/behaviour/submode`. | Constitution, Art. II. The IA368II nodes treat an unknown mode as IDLE. Hierarchical state machine (Harel, 1987). |
| D-02 | Behaviours with a result (escape, room coverage, go to a point) are **ROS 2 actions** called by the supervisor. | The mode topic cannot report success or failure; an action has a result, feedback and cancellation. |
| D-03 | Only `twist_mux` publishes on `/myRobot/cmd_vel`, with priorities and a stop lock. | Constitution, Art. VI. Prevents two actuators from commanding the base at the same time. |
| D-04 | The output of the IA368II `cleaning_server` stub is remapped to `/myRobot/legacy/cleaning_cmd_vel`, which no node subscribes to. The node stays up because the IA368II lifecycle manager expects it. | If it went into the mux, even at minimum priority, the stub would spin the robot whenever the real actuators were silent. |
| D-05 | The IA368II `autodocking_server` is remapped to the mux `docking` input. | It is the docking actuator until the docking spec replaces it by name. |
| D-06 | The 2025S2 wheel bridge (`my_robot_cmd_vel_node`) is reused with `/cmd_vel` remapped to `/myRobot/cmd_vel`. | Reuse without code changes. |
| D-07 | Sensor topics keep the 2025S2 names (`/scan`, `/camera`, `/camera_depth`). New modules consume them through a name parameter. | Modules 2 and 3 reuse these nodes unchanged. |
| D-08 | Until the docking spec, the IA368II `myrobot_server` keeps simulating the battery and the base signal. | Keeps the orchestrator cycle working from week 1. Replacing it with a node connected to CoppeliaSim is left for later. |
| D-09 | Two lifecycle managers of our own: `lifecycle_manager_core` (supervisor, stubs, modules) and `lifecycle_manager_optional` (MQTT bridge). The IA368II lifecycle manager stays as configured there. | Constitution, Art. VII. The absence of an optional node does not block the core. |
| D-10 | IA368II is brought in by `vcs import` from `deps.repos`, pinned by commit. | Reproducibility; its code is neither copied nor edited. |
| D-11 | Measured in task T004 (PR #14; raw outputs in `.sim/*.out`, local artefacts, not committed, cited as evidence): CoppeliaSim 4.10.0, House scene, `dt` = 0.05 s, synchronous stepping over ZeroMQ. **GUI mode** (`SIM_HEADLESS=0`): House alone (117 objects) 35.6 steps/s; House with the full robot (146 objects) 3.6 steps/s (an earlier run 3.5); robot without the Kinect `rgb`/`depth` sensors 20.1 steps/s; with the full robot, dynamics off still gives 2.9 and removing the robot scripts still gives 2.9 to 3.0 (`diag_dynamics.out`, `diag_scripts.out`), while removing the Kinect vision sensors gives 20.4 (`diag_scripts.out`); inference: the Kinect rendering is the dominant cost in GUI mode. **Headless mode** (`SIM_HEADLESS=1` via `tools/sim.sh`): House alone **340.9 steps/s** (2000 steps); robot with the Kinect vision sensors removed **319.3 steps/s** (1000 steps); robot **with** the Kinect: no result (`bench_headless_robot_kinect.out` is empty); the simulator log has a stack trace with a QMutex message (`QMutex: destroying locked mutex`, `.sim/sim-23000.log` line 354). The cause is not established here; the wait time before giving up was reported by the sim-operator and is not in the files. Rule of plan 001 (>= 200 steps/s trains RL in CoppeliaSim, otherwise in the 2D environment): the threshold is **met only for the Kinect-free robot in headless mode** (319.3 steps/s measured); it is never met in GUI mode, nor with the Kinect. Consequence: RL in CoppeliaSim requires a **Kinect-free robot variant** (or a replacement of the Kinect for headless runs). Whether the escape policy needs the Kinect is for spec 008 to decide. **Project-owner decision, to be taken in spec 008 (not decided here): train in CoppeliaSim with a Kinect-free variant, or in the 2D environment with validation in CoppeliaSim.** | Caveats: the Kinect-free headless results were measured once, not repeated, under a host load average of 0.7 to 4, reported by the sim-operator and unverified (not in the files). Parallel simulators remain blocked while all instances share the WebSocket port 23050. **001-R14 (synchronous headless measurement): met only for the Kinect-free robot variant** (the full robot does not run headless); the architect must amend R14/T004 in `specs/001-skeleton` when the PRs touching it land. Movement: in the earlier runs the robot barely moved (0.00 to 0.08 m) because two scripts under `/myRobot` (Lua `controler`, hard-coded -10 deg/s; Python `python_controler`, with a simUI joystick) set both wheel targets in every `sysCall_actuation` and overwrote direct `setJointTargetVelocity` calls. Driven through the float signals (see the Q-01 note 5), the robot moved 0.975 m in 5.0 s of simulation time (about 0.195 m/s; the 0.2 m/s per wheel command is inferred from the logged left joint velocity of -4.00 rad/s via the -v/0.05 relation, and the script of that run is not committed, so it is not reproducible from the repository). The steps/s figures do not depend on this. Not verified: the sign of the wheel target against the direction of travel, and whether the 2025S2 wheel bridge writes these signals. |
| D-12 | The system runs on Jazzy and Lyrical. Each distribution has its own container (`docker/jazzy`, `docker/lyrical`) and CI runs both. | Constitution, Art. X. All third-party dependencies already have a release on both: `navigation2` (1.3.14 / 1.5.3), `slam_toolbox` (2.8.5 / 2.10.0), `twist_mux` (4.5.0 / 4.5.1), `teleop_twist_keyboard`, `rmw_cyclonedds`, `rmw_zenoh` and `vision_msgs`. Source: `ros/rosdistro`, 2026-11-05. |
| D-13 | The motion contract is `geometry_msgs/Twist` without a timestamp, as the IA368II base expects. `use_stamped: false` in `twist_mux` and `enable_stamped_cmd_vel: false` in every Nav2 node that publishes velocity are set explicitly. | `twist_mux` 4.5 defaults to `use_stamped = true` on both distributions. Nav2 defaults to `enable_stamped_cmd_vel = false` on Jazzy and `true` on Lyrical. Trusting the defaults would break the base in one case and Nav2 in the other. Closes Q-02. |
| D-14 | CoppeliaSim talks to ROS only through the ZeroMQ Remote API (Python bridge nodes). The simROS2 plugin is not used. | The ZeroMQ bridge does not depend on the ROS distribution. simROS2 would have to be compiled for each distribution and each CoppeliaSim version. The simulator runs on the host and the ROS containers reach it over the network. |
| D-15 | Offloading nodes to the cloud uses native Kubernetes (K3s): one image per distribution, one Deployment and one selector on the robot. FogROS2 (Ichnowski et al., ICRA 2023) and KubeROS (Zhang, Wurll and Hein, ICRA 2023) serve as reference architectures; neither is used as a dependency. | FogROS2 only has a Humble release, creates privileged bare Pods without restart and installs ROS over SSH on every run. Porting it to Jazzy and Lyrical would cost 4 to 7 days. The full port is left as future work, preferably as a contribution to the original repository. |
| D-16 | Detection is split into two nodes: the `detector` (YOLO: RGB image → `vision_msgs/Detection2DArray` in the camera frame, copying the image `header`) and the `projector` (always on the robot: 2D detection + depth + TF at the photo timestamp → 3D position on the map). | Only the compressed RGB image goes up to the cloud; depth and TF stay on the robot. The copied `header` is the timestamp echo. The `projector` queries TF at the photo instant and keeps the `/detections_markers` output that the 2025S2 semantic node consumes, which stays unchanged. |
| D-17 | `StuckEvent.pose` uses `geometry_msgs/Pose`, not `Pose2D`. | `Pose2D` exists in Jazzy and was removed in Lyrical (checked in `geometry_msgs/msg` on both images, 2026-11-06). Art. X requires the same contract on both. The robot is planar: z = 0 and the yaw comes from the quaternion. |
| D-18 | The code is modular and follows good practices: pure logic in Python modules without ROS, thin ROS nodes that only wire the logic to topics, and dependencies injected by the caller. | Makes testing without the simulator easy (Art. V) and modules swappable (Art. III). |
| D-19 | Reusable components (controllers, filters, planners, detectors) are toolboxes: the class only does its computation, without publishing, reading topics or holding policy. Whoever invokes it decides configuration, limits, when to call and what to do with the result. | The caller knows the context (map, mode, safety) and the class stays testable and reusable. Style example: a `PID` that only computes `u` from error and `dt`. |
| D-20 | Every node and module logs and emits telemetry whenever possible: `rclpy` logger with levels, state transitions and events (stuck, escape, lock) always logged, and periodic metrics on `/myRobot/telemetry/<node>` or `diagnostic_msgs/DiagnosticArray`. Pure logic receives a logger and/or a metrics callback and does not import ROS. | Without this there is no way to debug in the simulator, measure D-11 or feed the training (spec 008). |
| D-21 | All repository text is in English: documentation, comments, docstrings, commit messages, spec directory names and file names. The third-party orchestrator is referred to by its repository name, `IA368II`, never by its author's name. | A single language keeps the repository consistent and accessible to external contributors (public repository). |
| D-22 | The stop lock is made effective by a `stop_guard` actuator: while `/myRobot/mux/lock_stop` is true it publishes a zero `Twist` at 10 Hz on `/myRobot/mux/stop` (priority 250); the `twist_mux` lock has priority 200. `twist_mux` stays the sole publisher on `/myRobot/cmd_vel`. | `twist_mux` publishes nothing while locked (verified on the real node, `tools/probes/mux_lock_probe.sh`), so the base would keep its last command. See `docs/stop-guard.md`. Closes Q-08. |
| D-26 | Criterion for lifecycle versus plain node (decided by the project owner, 2026-11-14; full text and rationale in `specs/005-coverage-and-escape/spec.md`, copied here by the architect). Art. III.2 is not absolute law. A node is a `LifecycleNode` under a lifecycle manager when it holds resources or state that a manager must configure, activate, deactivate or clean up: drivers and device handles, publishers that must stop on deactivation, timers, action servers with goals in flight, Nav2 servers. A node may be a plain node when it has no controllable state: it only orchestrates managers (the `lifecycle_sequencer`), is one-shot bootstrap glue, or is a tool. Every plain node is listed with its reason in `docs/lifecycle-managers.md` and `src/vacuum_bringup/config/node_classification.yaml`; third-party plain nodes (`twist_mux`, the wheel bridge, `teleop_twist_keyboard`) stay plain (Art. II). The wording for Art. III.2 is in `docs/constitution-amendment-III2.md`, pending the project owner's edit of the constitution. Closes Q-17 of spec 005. | Art. III.2, D-09. The sequencer manages the managers and has nothing to configure or clean up; forcing it into a lifecycle would need a manager for the manager. |

D-23 (odometry source, spec 003), D-24 (coverage motion through Nav2 on `mux/nav`) and D-25 (escape priority, proposed) are decided or proposed in specs 003 and 005 and are not copied here yet; the amendment of D-09 (third lifecycle manager) is also still to be applied by the architect.

## 6. Open questions

- ~~Q-01~~: closed by D-11 and the answer below (T004/T016; measured on CoppeliaSim 4.10.0 with the House scene and `models/myRobot.ttm`; raw outputs in `.sim/inspect_house.out` and `.sim/robot_objects.out`).
  - Version: CoppeliaSim 4.10.0 (host). The *House* scene has **no robot**; the robot is loaded from `models/myRobot.ttm` (exported from the 2025S2 `pega_banana.ttt`) and appears as `/myRobot`.
  - House (117 objects): root `/house`; room dummies `/house/{bedroom, corridor, entrance_ramp, toilet, entrance, livingroom, diningroom, bathroom, backyard, kitchen}`; `/house/walls`, `/house/floor`, `/house/roof`, `/ResizableFloorLarge`; the furniture list is in `.sim/inspect_house.out`.
  - Doors (joints): `/house/door_{backyard_diningroom, bathroom_corridor, entrance_ramp_corridor, bedroom_corridor, toilet_corridor, livingroom_corridor, entrance_corridor}`; each has a child `.../door` shape.
  - Robot (146 objects with the House): `/myRobot/{leftMotor, rightMotor, leftWheel, rightWheel, Caster}`, `/myRobot/bumper`, `/myRobot/Ground_truth`, `/myRobot/kinect/{rgb, depth, camera_ref, joint, body, glass}`, sensors `proximitySensor`, `poopximity`, `dockingSensor`, `forceSensor`, `battery`, and scripts `controler`, `python_controler`, `odometry`, `encoder`. The names `leftMotor` and `rightMotor` match the 2025S2 scene, so the wheel bridge needs no renaming (the object-name risk in plan 001 does not materialize for the robot).
  - Notes derived from the measurement:
    1. Sim-based tasks in specs 003 and 005 run at about 3.6 steps/s (0.05 s per step) with the full robot, i.e. about 0.18 simulated seconds per wall second; a future wall-time versus sim-time budget must use this factor, so tests against the simulator need sim time or generous wall timeouts. Without the Kinect in GUI mode the factor is about 1.0 (20.1 x 0.05); headless without the Kinect it is about 16 simulated seconds per wall second (319.3 x 0.05 = 15.97; House alone 340.9 x 0.05 = 17.0).
    2. `/myRobot/bumper` exists in the robot model, so a contact signal for stuck/escape (Q-11 in the 005 proposal) can come from the simulator. Whether the 2025S2 bridge publishes it on a ROS topic was not checked.
    3. `/myRobot/Ground_truth` exists (a dummy), a candidate source of the simulator pose, relevant to Q-03. Which pose feeds SLAM stays open (Q-03).
    4. Whether the 2025S2 `my_robot_cmd_vel_node` has its own command timeout (left open by Q-08) was not checked.
    5. Drive interface of the robot model (`.sim/robot_inspect*.out`, `.sim/robot_move.out`): the `python_controler` script reads the float signals `str(robotHandle)+"rightVel"` and `+"leftVel"` (m/s per wheel; cleared after reading, so they must be set at every step), with right = lin + 0.1*rot, left = lin - 0.1*rot, wheel radius 0.05 m, L 0.2, target velocity = -v/0.05; the signal `+"Battery"` equal to 0 forces the wheels to zero. The scripts `controler` and `python_controler` overwrite direct joint-velocity commands. The 2025S2 wheel bridge (`my_robot_cmd_vel_node`) must be checked against this interface (does it write these signals? sign convention? command timeout, see note 4): task for the ros-dev reading it (T302/T503 in the 003/005 proposals, not yet on main). The 0.975 m run (`robot_move.out`) logs only displacement and a left joint velocity of -4.00 rad/s; the commanded 0.2 m/s is inferred from it. Unverified until then.
- ~~Q-02~~: closed by D-13.
- Q-04 (answered partially, T003b, `deploy/measurements/ia368ii-lyrical-jazzy-bringup.md`): *yes for the orchestrator, no for `my_robot_pkg`
  on Lyrical until the logger incompatibility is handled under Art. II.* Pin `51a5fa2c`, Docker, no simulator:
  the three packages (`myrobot_interfaces`, `myrobot_bringup`, `my_robot_pkg`) build on Jazzy and Lyrical; the upstream
  `bringup.launch.py` brings up six nodes plus `lifecycle_manager`, all active and mode `CHARGED`, on both. Of the 11
  `my_robot_pkg` executables, still alive after 10 s: 4 on Jazzy and 0 on Lyrical. Lyrical-only cause: `RcutilsLogger.warn`
  was removed (`warning` replaces it); 38 call sites in 8 files. It is the *only* distro difference and explains only two
  executables: `my_robot_detection_node` and `my_robot_semantic_segmentation_node` (kept unchanged by D-16), which run on
  Jazzy and crash on Lyrical. The others die from missing runtime dependencies, on both distros: `cartographer` and
  `explorer` for `scipy`, the rest for `coppeliasim_zmqremoteapi_client` (four nodes), the `slam_toolbox` Python module
  and `skimage`; `scipy` and `ultralytics` exist in Jazzy `desktop-full` and not in `ros-base` (an image difference).
  Controlled run of the T003b report (Jazzy on `ros-base`, same flavour as Lyrical): 2 of 11 alive on Jazzy
  (`detection`, `semantic_segmentation`) against 0 on Lyrical, which isolates `.warn`. The `rf2o_laser_odometry` rosdep key
  is unresolved on Lyrical, and on Jazzy in the first run but resolved in the re-run (rosdep index changed between runs), so
  it is a Lyrical gap (tied to Q-03). Limits: no simulator, "alive for 10 s" is not "correct", no `colcon test`, Nav2/SLAM launch
  files not run, wheel bridge not started. Still open: the fix under Art. II (wrapper that restores the logger method in our
  own package, or a same-name replacement node, T024) and tracking the runtime dependencies (T025).
  Reviewer input (PR #17): a same-name replacement node is not recommended (400+ lines copied, drift from upstream,
  contradicts D-16); targeted inheritance does not work because the call is `self.get_logger()` on the `Node` base class.
  **Proposed decision for the project owner (draft; D-number to be assigned at merge, Art. II interpretation, not in
  the constitution):** *Logger-compatibility shim.* A small launcher executable in a `vacuum_*` package runs, in the process
  of each upstream node, `if not hasattr(RcutilsLogger, "warn"): RcutilsLogger.warn = RcutilsLogger.warning` and then
  the upstream entry point. Conditions: a no-op when `warn` exists (Jazzy); no upstream file or pin changes; no
  `BehaviourManager` subclass and no `Mode` extension; pytest plus launch test; recorded as an explicit decision, because it
  is not literally on the ordered adaptation list (closest item: launch wrapper) and patches `rclpy` at runtime.
  Alternative to the launcher: a `sitecustomize` on `PYTHONPATH` set by our launch; it patches every Python process on the
  path, including our own nodes, so the launcher is preferable. Option 0: do nothing and use only the orchestrator
  (`myrobot_bringup`) on Lyrical, which keeps Art. II untouched but leaves `my_robot_pkg` (and D-16's semantic node) on
  Jazzy only, which conflicts with Art. X unless the owner accepts that.
  Follow-up tasks T024 to T028 are *proposed, to be added to `specs/001-skeleton/tasks.md` (and 003/005 where they
  belong) by a later architect change*; they are not defined in any task list yet: T024 shim (ros-dev), T025 rosdep
  `exec_depend`s in our own package (ros-dev), T026 pinned pip requirements in `docker/jazzy` and `docker/lyrical` (devops),
  T027 `rf2o` source build or another odometry source (architect and devops, Q-03), T028 `ultralytics` policy on Python 3.14
  (architect and devops, Q-05, D-16).
- Q-05: pip dependencies on Python 3.14, used on Lyrical: `torch`, `ultralytics` and `stable-baselines3`.
  If any of them has no wheel, RL training runs in a Python environment separate from ROS, and only inference goes into the node.
- Q-06: robot ↔ cloud transport. Two options:
  - (a) `rmw_zenoh` across the whole graph, with the robot router connected to the cluster router (0.2.13 on Jazzy, 0.10.8 on Lyrical);
  - (b) `zenoh-bridge-ros2dds` (1.9.0), keeping the local graph on DDS with CycloneDDS and an explicit topic allow-list for what goes up.

  Neither implements the *Deadline* and *Liveliness* QoS events the way we need. Spec 009a therefore implements age, heartbeat and thresholds in the application, independently of the transport.
  Decide after measuring both in week 3.

  **Status: recommendation recorded, awaiting the project owner** (not closed). T902 measured, on Lyrical only and on
  the same host (`rmw_zenoh` 0.10.6, bridge 1.10.2; the versions above were not re-verified), p50 RTT in ms:
  lan 4.05 (`rmw_zenoh`) against 5.15 (bridge); wan (netem 20 ms +/- 5 ms, 1 % loss, uplink) about 150 against 393;
  wan p99 about 366 to 408 against 566 to 580 (the range is the subset and convention: all 607 numeric rows with
  linear interpolation, or the first 600 rows with the bench's index method; computed from raw CSVs kept locally and not versioned, may be updated by the
  final T902 summary, see spec 009a); no loss or reordering. Recommendation: the bridge, for structural
  reasons (allow-list enforces 009a-R03, motion loop stays on DDS, one binary for both distros; Art. VI and X);
  `rmw_zenoh` is better on a lossy link. Warning: the bridge's p99 on the lossy link (0.58 s) is above
  `close_threshold_s` (0.5 s) and near `max_age_s` (0.67 s) before counting inference time. Depends on PR #13 (selector rulings) and PR #16 (T902) being settled. Pending: Jazzy, real Wi-Fi,
  CPU and inference time (T919 to T921 in spec 009a). Details and implications for the selector in spec 009a.
- Q-07 (future): robot ↔ cloud communication protocol, that is, format, versioning, authentication and the policy of what goes up and when. Out of scope for the course deadline. Spec 009a defines only the minimum needed.
- Q-03: odometry. 2025S2 uses `/myrobot/odom` (simulator ground truth) and rf2o (laser odometry).
  Decide in spec 003 which one feeds SLAM.
- ~~Q-08~~: closed by D-22. Still open from it: whether the 2025S2 `my_robot_cmd_vel_node` has its own command timeout (not checked by the Q-01 inspection; the open item is note 4 under Q-01 above).
