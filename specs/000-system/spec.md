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
| D-11 | (reserved for the steps/s measurement, task T004 of spec 001) | |
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

## 6. Open questions

- Q-01: CoppeliaSim version and path of the *House* scene. The object names (`/myRobot/leftMotor`)
  must match the 2025S2 scene.
- ~~Q-02~~: closed by D-13.
- Q-04: the IA368II `my_robot_pkg` was written for Jazzy. We must confirm that it builds and runs on Lyrical
  (rclpy 10, Python 3.14). If it does not, the fix goes in as an overlay in our repository, never as an edit to its code.
- Q-05: pip dependencies on Python 3.14, used on Lyrical: `torch`, `ultralytics` and `stable-baselines3`.
  If any of them has no wheel, RL training runs in a Python environment separate from ROS, and only inference goes into the node.
- Q-06: robot ↔ cloud transport. Two options:
  - (a) `rmw_zenoh` across the whole graph, with the robot router connected to the cluster router (0.2.13 on Jazzy, 0.10.8 on Lyrical);
  - (b) `zenoh-bridge-ros2dds` (1.9.0), keeping the local graph on DDS with CycloneDDS and an explicit topic allow-list for what goes up.

  Neither implements the *Deadline* and *Liveliness* QoS events the way we need. Spec 009a therefore implements age, heartbeat and thresholds in the application, independently of the transport.
  Decide after measuring both in week 3.
- Q-07 (future): robot ↔ cloud communication protocol, that is, format, versioning, authentication and the policy of what goes up and when. Out of scope for the course deadline. Spec 009a defines only the minimum needed.
- Q-03: odometry. 2025S2 uses `/myrobot/odom` (simulator ground truth) and rf2o (laser odometry).
  Decide in spec 003 which one feeds SLAM.
- Q-08 (safety, found while doing T007): with `/myRobot/mux/lock_stop` active, `twist_mux` (4.5, tested on Jazzy)
  publishes **nothing** on `/myRobot/cmd_vel`; it does not publish zero. The wheel base would keep the last command,
  so 001-R10 ("publish zero velocity while locked") is not met by the mux alone. Proposal: a tiny actuator node
  `stop_guard` that publishes a zero `Twist` at 10 Hz on a new input `/myRobot/mux/stop` (priority 250) only while
  `lock_stop` is `true`, with the lock priority lowered to 200 so that it blocks every other input but not `stop`.
  `twist_mux` stays the sole publisher (Art. VI). Needs a decision before T013/T014, and a check of whether the
  2025S2 `my_robot_cmd_vel_node` has its own command timeout (Q-01 inspection).
