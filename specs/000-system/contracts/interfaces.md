# New types of the `vacuum_interfaces` package (spec 001)

Draft of the `.msg`, `.srv` and `.action` files. Task T002 of spec 001 copies this content into the package; the room
types of spec 003 are copied by T303.

## msg/Submode.msg

```
# Submode published by the supervisor inside the orchestrator's mode (D-01).
string NONE=NONE
string EXPLORING=EXPLORING
string COVERING=COVERING
string GOTO_ROOM=GOTO_ROOM
string ESCAPING=ESCAPING
string ESCAPE_FAILED=ESCAPE_FAILED

builtin_interfaces/Time stamp
string mode        # copy of the last /myRobot/behaviour/mode received
string submode     # one of the constants above
string reason      # reason for the last transition, for logs and the interface
```

## msg/StuckEvent.msg

```
string NO_PROGRESS=NO_PROGRESS   # motion command without the corresponding displacement
string BUMPER=BUMPER

builtin_interfaces/Time stamp
string cause
geometry_msgs/Pose pose          # pose on the map at the detection instant (2D: z=0, yaw in the quaternion)
float32[] scan_ranges            # copy of the laser at that instant (input for the training in spec 008)
```

## msg/Notification.msg

```
uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2

builtin_interfaces/Time stamp
uint8 severity
string source      # node name
string code        # stable identifier, e.g. ESCAPE_FAILED
string text        # message for the user
```

Codes in use (checked against the code on `main`; the registry is documentation, `code` stays a free string, so adding a row changes no type; contract change per Art. I, same commit as the 005-R36 wording):

| `code` | Severity | `source` | Meaning | Defined in |
|---|---|---|---|---|
| `ESCAPE_FAILED` | ERROR | `supervisor` | the escape failed or timed out; stop lock on (001-R09) | `vacuum_supervisor/submode_machine.py` |
| `CLOUD_DETECTOR_DOWN` | WARN | `detection_selector` | breaker opened, local detector in use (009a-R08) | `vacuum_detection/selector_core.py` |
| `CLOUD_DETECTOR_RECOVERED` | INFO | `detection_selector` | breaker closed after the stability window (009a-R10) | `vacuum_detection/selector_core.py` |
| `CORE_START_FAILED` | ERROR | `lifecycle_sequencer` | `lifecycle_manager_core` did not become active; nothing else is started (005-R36) | `vacuum_bringup/manager_sequencer.py` |
| `NAV_UNAVAILABLE` | WARN | `lifecycle_sequencer` | `lifecycle_manager_nav` did not become active; no automatic motion (005-R36) | `vacuum_bringup/manager_sequencer.py` |
| `OPTIONAL_UNAVAILABLE` | WARN | `lifecycle_sequencer` | `lifecycle_manager_optional` did not become active; running without it, Art. VII (005-R36) | `vacuum_bringup/manager_sequencer.py` |
| `STOP_GUARD_LOST` | ERROR | `lifecycle_sequencer` | `stop_guard` left `active` while running; nav deactivated (005-R36) | `vacuum_bringup/manager_sequencer.py` |
| `COVERAGE_DONE` | INFO | `supervisor` | `cover_room` ended with success while the mode is CLEANING; docking requested. Only with `dock_after_coverage` true (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`, not on `main` yet) |
| `COVERAGE_FAILED` | WARN | `supervisor` | `cover_room` ended with failure while the mode is CLEANING; docking requested. Only with `dock_after_coverage` true (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`, not on `main` yet) |
| `DOCK_REQUEST_FAILED` | ERROR | `supervisor` | mode still CLEANING after the request, `dock_request_timeout_s` later; stays in NONE. Only with `dock_after_coverage` true (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`, not on `main` yet) |

## msg/OffloadStatus.msg (spec 009a)

```
uint8 CLOSED=0      # using the cloud detector
uint8 OPEN=1        # using the local detector
uint8 HALF_OPEN=2   # using the local one while the cloud proves stability

builtin_interfaces/Time stamp
uint8 state
float32 last_cloud_age_s       # age of the last cloud response, on the robot clock; NaN before the first one
uint32 forwarded               # responses forwarded to /myRobot/perception/detections
uint32 dropped_inactive        # responses from the source that is not active in this state (R07)
uint32 dropped_stale           # responses dropped for age (R05)
uint32 dropped_watermark       # dropped for a timestamp already covered: duplicate or out of order (R06)
uint32 transitions             # breaker state changes since startup
```

`dropped_tf` of the first draft was removed: the selector cannot know it. The projector reports it (and `projected`,
`dropped_depth`) in its own `/myRobot/telemetry/projector` DiagnosticArray. `forwarded`, `dropped_inactive` and
`transitions` were added so that `forwarded + dropped_inactive + dropped_stale + dropped_watermark` equals the number of
responses received (spec 009a, counters) and the R15/R16 report can be computed from the rosbag.

## action/Escape.action

```
# Goal
StuckEvent event
float32 timeout_s
---
# Result
bool success
float32 duration_s
string strategy     # "scripted" or "rl", to compare implementations
---
# Feedback
float32 elapsed_s
```

## action/CoverRoom.action

```
# Goal
string room_id      # empty = current room
uint8 passes
---
# Result
bool success          # true: covered_ratio >= target_ratio, or a degraded result >= min_acceptable_ratio (005-R21);
                      # false: below min_acceptable_ratio, skipped waypoints > max_skip_ratio or coverage_timeout_s (005-R20)
float32 covered_ratio # covered / coverable cells of the region (005-R21)
---
# Feedback
float32 covered_ratio
```

## msg/Room.msg (spec 003)

```
uint8 DEFAULT=0
uint8 RULE=1
uint8 USER=2

uint16 index                    # value in room_labels, from 1, stable (003-R19)
string id                       # room_%02d of index (003-R20); CoverRoom.room_id (003-R25)
string label                    # default = id
uint8 label_source              # DEFAULT, RULE or USER
geometry_msgs/Point centroid    # frame map
float32 area_m2
```

## msg/RoomArray.msg (spec 003)

```
std_msgs/Header header          # frame map, stamp of the map used
vacuum_interfaces/Room[] rooms
```

## srv/SetRoomLabel.srv (spec 003)

```
string id           # Room.id
string label
---
bool success        # false for an unknown id (003-R23)
string message
```

## action/Explore.action (spec 003)

```
# Goal
float32 timeout_s               # 0 = parameter explore_timeout_s
---
# Result
bool success
uint32 frontiers_done
uint32 frontiers_blacklisted
float32 duration_s
---
# Feedback (1 Hz or faster, 003-R31)
uint32 frontiers_remaining
float32 elapsed_s
```


## Simulator interface: robot model and bridges (contract batch 2, 001-R23)

Not a ROS type: the signals and properties through which the CoppeliaSim model (`myRobotLaser.ttm`, the combined
`myRobot` + laser model) and the `vacuum_sim_bridge` nodes (D-27) talk over the ZeroMQ Remote API (D-14). A model
variant or a different robot must keep these names, or override them in its profile YAML (`params_file`).

| Direction | Name | Kind | Content | Written / read by |
|---|---|---|---|---|
| bridge -> model | `<h>leftVel`, `<h>rightVel` | float signal (`sim.setFloatSignal`) | left and right wheel speed in m/s; `<h>` is the decimal integer handle of `robot_path` (`/myRobot`), e.g. `"16leftVel"` | written by `my_robot_cmd_vel_node` every `1 / write_rate_hz` s while driving; read by the model script `python_controler` |
| model -> bridge | `signal.distSignal` on `robot_path` | float-array property (`sim.getFloatArrayProperty`) | one range per beam in m, beam 0 at `scan_angle_min` (right side), step `scan_angle_increment`; 0 = no hit; 452 values expected | written by the model's laser script; read by `my_robot_scan_node` |
| model -> bridge | pose of `robot_path` | `sim.getObjectPose(h, sim.handle_world)` | x y z qx qy qz qw | read by `my_robot_odometry_node` (`odom_source: ground_truth`) |
| model -> bridge | `left_motor_path`, `right_motor_path` | joint position | rad | read by `my_robot_odometry_node` (`odom_source: wheels`) |
| model -> bridge | `rgb_sensor_path`, `depth_sensor_path` | vision sensor image / depth buffer | rgb8 640x480 and float32 0..1 (scaled with `camera_near`/`camera_far`) | read by `my_robot_vision_node` |

Rules:

- The stock `python_controler` clears both wheel signals after one actuation, so the bridge rewrites them every cycle
  and the robot moves reliably only with a model variant that holds them, or in stepping mode with one write per step
  (`src/vacuum_sim_bridge/README.md`). The signal `<h>Battery` equal to 0 forces the wheels to zero (Q-01 note 5).
- The two wheel signals are written with two `setFloatSignal` calls (right first). `atomic_wheel_write: true` asks for
  one `executeScriptString` call; that function does not exist on the CoppeliaSim 4.10 `sim` object, so the client falls
  back to two calls after one warning.
- The beam count is taken from the data: a count other than `scan_num_beams` is accepted with one warning, and
  `angle_max = scan_angle_min + (n - 1) * scan_angle_increment`. Fewer than 2 values publish nothing.
- Wheel speed per side follows the model: right = v + (L/2) w, left = v - (L/2) w, with L = `wheel_separation` (0.2 m),
  capped by `max_linear` / `max_angular`, zeroed after `cmd_timeout` without a command.
