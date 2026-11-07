string NO_PROGRESS=NO_PROGRESS   # comando de movimento sem deslocamento correspondente
string BUMPER=BUMPER

builtin_interfaces/Time stamp
string cause
geometry_msgs/Pose pose          # pose no mapa no instante da detecção (2D: z=0, yaw no quaternion)
float32[] scan_ranges            # cópia do laser no instante (insumo para o treino da spec 008)
