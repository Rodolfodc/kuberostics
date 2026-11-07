# Project constitution: autonomous vacuum robot (IA368II 2026S2)

Principles that apply to every specification, plan, task and line of code in this repository.
A specification that contradicts an article is invalid until the article is explicitly amended.

## Article I. The contract comes before the code

1. Every ROS 2 interface (topic, service, action, public parameter) exists first in
   `specs/000-system/contracts/`, with name, type, QoS (*Quality of Service*), who publishes and who consumes.
2. New types are defined in `.msg`, `.srv` and `.action` files of the `vacuum_interfaces` package.
   That package contains no implementation.
3. Changing a contract requires changing the specification in the same commit.

## Article II. Third-party code is not modified

1. `IA368II` (packages `myrobot_bringup`, `myrobot_interfaces` and `my_robot_pkg`) is a read-only
   dependency, pinned by commit in `deps.repos`.
2. Adaptation uses only these mechanisms, in this order of preference:
   1. submodes published by a node of our own;
   2. name remapping in the launch file;
   3. replacing a node with another of the same name that fulfils the same contract;
   4. targeted inheritance.
3. Inheriting from or replacing `BehaviourManager` (the orchestrator) and extending the `Mode` enum are forbidden.
   The orchestrator keeps publishing exactly CLEANING, AUTODOCKING, CHARGING, CHARGED and IDLE.

## Article III. Every module is pluggable

1. Each module is a ROS 2 package with its own launch file. It comes up on its own, fed by stubs or a rosbag.
2. New nodes are `LifecycleNode`s, managed by a Nav2 lifecycle manager.
3. An **actuator** is a node that moves the robot. It publishes only to its own `twist_mux` input.
4. A **service** is a node that perceives, computes or displays. It never publishes a motion command.
5. Two implementations of the same interface must be swappable through the launch file alone.

## Article IV. The system always runs

1. The first delivery is the walking skeleton (Cockburn): all modules present as stubs,
   integrated end to end.
2. A stub is only removed when the real implementation passes the same contract tests.

## Article V. Contract tests are mandatory

1. Each interface of the contract has an automatic test of existence, type and QoS.
2. Each module has at least one `launch_testing` test.
3. Continuous integration runs `colcon build` and `colcon test` without a simulator.
   Tests with CoppeliaSim run locally and are marked as such.

## Article VI. Motion safety

1. Only `twist_mux` publishes on `/myRobot/cmd_vel`.
2. A maximum-priority stop lock exists.
3. Every recovery behaviour has a timeout and a deterministic fallback.
   When both fail, the robot stops and notifies.

## Article VII. The cloud is optional

1. The robot cleans the house without a network.
2. Cloud components (MQTT bridge, K3s services) come up under a separate lifecycle manager.
3. New policy weights are only loaded with the orchestrator in CHARGED mode.

## Article VIII. Traceability

1. Each requirement has an identifier (for example, `001-R04`), written in EARS notation
   (*Easy Approach to Requirements Syntax*, Mavin et al., 2009).
2. Each task cites the requirements it meets, and each test cites the requirement it verifies.
3. Commit messages cite the task identifier.

## Article IX. Simplicity and deadline

1. Every module first arrives in a minimal working version and only later gains depth.
2. Final deadline: December 4, 2026, 23:59.
3. Cut order in case of delay: (1) LLM agent; (2) cloud retraining; (3) RL policy, keeping the scripted escape.
   The six modules of the assignment are not cut.

## Article X. Two distributions: Jazzy and Lyrical

1. Every package in this repository builds, passes its tests and runs on ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)
   and on ROS 2 Lyrical Luth (Ubuntu 26.04, Python 3.14).
2. Continuous integration runs the full matrix. A test that passes on one distribution and fails on the other counts as a failure.
3. No behaviour depends on a third-party parameter default that changes between distributions.
   Such parameters are always declared explicitly in the configuration. Example: `use_stamped` of `twist_mux`
   and `enable_stamped_cmd_vel` of Nav2.
4. Distribution-specific code is only accepted behind a `ROS_DISTRO` check, with a justification in the spec.
5. Each run uses a single distribution. A graph that mixes Jazzy and Lyrical nodes is not supported.

## Technical stack

- ROS 2 Jazzy and ROS 2 Lyrical Luth, Python (`rclpy`), each in its own container.
- CoppeliaSim with the ZeroMQ Remote API, on the host and independent of the ROS distribution.
- Nav2, SLAM Toolbox, `twist_mux`.
- Gymnasium and Stable-Baselines3.
- K3s, Mosquitto, MinIO, DynamoDB Local, Terraform.

**Version:** 1.2 · 2026-11-06. Translated to English (decision D-21 of spec 000). Article X: Jazzy and Lyrical support.
