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
