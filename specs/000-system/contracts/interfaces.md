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
bool success          # true when covered_ratio >= target_ratio, or a degraded result >= min_acceptable_ratio (005-R21)
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

## Notification codes added by contract batch 1

`code` stays a free string, so these rows change no type. When the registry of PR #29 (table after `Notification.msg`)
is merged, these rows move into it. "Defined in" points at code on unmerged branches.

| `code` | Severity | `source` | Meaning | Defined in |
|---|---|---|---|---|
| `COVERAGE_DONE` | INFO | `supervisor` | `cover_room` ended with success while the mode is CLEANING; docking requested (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`) |
| `COVERAGE_FAILED` | WARN | `supervisor` | `cover_room` ended with failure while the mode is CLEANING; docking requested (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`) |
| `DOCK_REQUEST_FAILED` | ERROR | `supervisor` | the mode did not become AUTODOCKING within `dock_request_timeout_s`; stays in NONE (005-R30) | `vacuum_supervisor/submode_machine.py` (branch `feat/supervisor-coverage-rows`) |
| `MANAGER_UNRESPONSIVE` | WARN (nav, optional), ERROR (core) | `lifecycle_sequencer` | a lifecycle manager did not answer the `is_active` probe after a command timed out; no further manager command in this run; the text lists the node states (005-R42, proposed in PR #32) | `vacuum_bringup/manager_sequencer.py` (to be written, T523) |
