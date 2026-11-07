# New types of the `vacuum_interfaces` package (spec 001)

Draft of the `.msg` and `.action` files. Task T002 of spec 001 copies this content into the package.

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
float32 last_cloud_age_s       # age of the last cloud response, on the robot clock
uint32 dropped_stale           # responses dropped for age
uint32 dropped_watermark       # dropped for a timestamp already covered (duplicate or out of order)
uint32 dropped_tf              # dropped by the projector for lack of TF at the timestamp
```

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
bool success
float32 covered_ratio
---
# Feedback
float32 covered_ratio
```
