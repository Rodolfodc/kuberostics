string NO_PROGRESS=NO_PROGRESS   # motion command without the corresponding displacement
string BUMPER=BUMPER

builtin_interfaces/Time stamp
string cause
geometry_msgs/Pose pose          # pose on the map at the detection instant (2D: z=0, yaw in the quaternion)
float32[] scan_ranges            # copy of the laser at that instant (input for the training in spec 008)
