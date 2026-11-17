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
