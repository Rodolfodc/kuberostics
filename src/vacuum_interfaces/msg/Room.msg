uint8 DEFAULT=0
uint8 RULE=1
uint8 USER=2

uint16 index                    # value in room_labels, from 1, stable (003-R19)
string id                       # room_%02d of index (003-R20); CoverRoom.room_id (003-R25)
string label                    # default = id
uint8 label_source              # DEFAULT, RULE or USER
geometry_msgs/Point centroid    # frame map
float32 area_m2
