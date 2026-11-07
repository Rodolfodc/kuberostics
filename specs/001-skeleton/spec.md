# 001: Walking skeleton

Status: draft for review · Deadline: 2026-11-10 · Milestone: complete system running with stubs

## Goal

Bring up, with a single command, the IA368II orchestrator, `twist_mux`, the submode supervisor and a
stub for each module, all integrated through the spec 000 contract. The robot must move in CoppeliaSim
and the stuck → escape → back-to-coverage cycle must work with fake data. Every following spec swaps one
stub for a real implementation without touching the rest.

## Out of scope

- Real navigation, SLAM, YOLO, schedule, graphical interface, RL and cloud.
- Replacing the IA368II `myrobot_server` (D-08).

## Usage scenarios

1. **Happy path.** With the system up, the user calls `start`. The orchestrator goes to CLEANING, the supervisor
   publishes COVERING and the robot moves through the coverage stub.
2. **Stuck.** During coverage, the detector stub publishes a `StuckEvent`. The supervisor switches to
   ESCAPING and calls the escape action. The escape ends successfully and the supervisor returns to COVERING.
3. **Escape fails.** The escape times out. The supervisor publishes ESCAPE_FAILED, activates the stop lock and
   emits an error notification. The robot stays still.
4. **Teleoperation.** In any mode, a teleoperation command takes precedence over the automatic actuators.
   The stop lock still takes precedence over teleoperation.

## Requirements (EARS)

| ID | Requirement | Verification |
|---|---|---|
| 001-R01 | The system shall come up with a single `ros2 launch vacuum_bringup skeleton.launch.py`, containing the IA368II nodes, `twist_mux`, the supervisor and the stubs. | launch_testing |
| 001-R02 | When startup finishes, the IA368II lifecycle manager and `lifecycle_manager_core` shall answer `true` on `is_active`. | launch_testing |
| 001-R03 | The system shall have exactly one publisher on `/myRobot/cmd_vel`, and it shall be `twist_mux`. | contract |
| 001-R04 | Every interface marked `since: "001"` or `frozen: true` in `topics.yaml` shall exist with the declared type and QoS. | contract |
| 001-R05 | While the orchestrator mode is neither CLEANING nor AUTODOCKING, the supervisor shall publish the NONE submode. | unit |
| 001-R06 | When the mode changes to CLEANING, the supervisor shall publish COVERING within 0.5 s and send a goal to `/myRobot/cover_room`. | launch_testing |
| 001-R07 | When it receives a `StuckEvent` with submode COVERING or with mode AUTODOCKING, the supervisor shall cancel the ongoing action, publish ESCAPING within 0.5 s and send a goal to `/myRobot/escape`. | launch_testing |
| 001-R08 | When the escape action ends successfully, the supervisor shall return to the submode it had before the stuck event. | launch_testing |
| 001-R09 | If the escape action fails or exceeds `escape_timeout_s` (default 20 s), the supervisor shall publish ESCAPE_FAILED, publish `true` on `/myRobot/mux/lock_stop` and emit a `Notification` of severity ERROR with code `ESCAPE_FAILED`. | launch_testing |
| 001-R10 | While `/myRobot/mux/lock_stop` is `true`, the velocity on `/myRobot/cmd_vel` shall be exactly zero, whatever the driving inputs, and `twist_mux` shall remain its only publisher. | launch_testing + `tools/probes/mux_lock_probe.sh` |
| 001-R11 | While there are messages on `/myRobot/mux/teleop` within the input timeout, `twist_mux` shall forward the teleoperation and ignore lower-priority inputs. | launch_testing |
| 001-R12 | The system shall remap the IA368II `cleaning_server` output to `/myRobot/legacy/cleaning_cmd_vel` and the `autodocking_server` output to `/myRobot/mux/docking`, without changing the code of those nodes. | contract |
| 001-R13 | Where CoppeliaSim is running, the wheel bridge shall convert `/myRobot/cmd_vel` into wheel velocities of the robot in the scene. | manual (short video) |
| 001-R14 | The repository shall contain a script that measures simulation steps per second of the *House* scene in synchronous, headless mode, and the result shall be recorded in decision D-11. | manual |
| 001-R15 | When there is a push, continuous integration shall run `colcon build` and `colcon test` on a matrix of Jazzy and Lyrical containers, without CoppeliaSim. | CI |
| 001-R17 | The system shall satisfy 001-R01 to 001-R12, 001-R16 and 001-R18 to 001-R20 on both Jazzy and Lyrical, with the same code and the same configuration. | CI (matrix) |
| 001-R18 | The configuration shall explicitly declare `use_stamped: false` on `twist_mux`, and the contract test shall fail if `/myRobot/cmd_vel` has a type other than `geometry_msgs/msg/Twist`. | contract |
| 001-R19 | While `/myRobot/mux/lock_stop` is `true`, the `stop_guard` shall publish a zero `Twist` on `/myRobot/mux/stop` at 10 Hz or faster, and while it is `false` it shall publish nothing. The lock shall be latched: if `lock_stop` stops arriving, the guard keeps publishing zero. | unit + node |
| 001-R20 | The `twist_mux` configuration shall order the priorities `stop input > stop lock > every driving input` and give the `stop` input a timeout of at most 1 s. | contract (config) |
| 001-R16 | The `escape_server` stub shall accept the parameter `outcome` (`succeed`, `fail`, `hang`) so that scenarios 2 and 3 are testable without the simulator. | unit |

## Definition of done

- All automatic tests for 001-R01 to 001-R12, 001-R16 and 001-R18 to 001-R20 passing in CI, on Jazzy and Lyrical.
- Video of up to 1 minute showing scenarios 1 to 4 in CoppeliaSim.
- D-11 recorded in `specs/000-system/spec.md` with the steps/s measurement.
