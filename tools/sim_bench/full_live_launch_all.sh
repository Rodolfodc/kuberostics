#!/usr/bin/env bash
# Runs INSIDE the fullrobot-jazzy container (docker cp it to /tmp, then docker exec -d fullrobot-jazzy bash /tmp/full_live_launch_all.sh): scan + odometry bridges, map_server (/tmp/rect_map/room.yaml)
# and skeleton.launch.py with nav, map_tf, real_nodes, sequencer and the vacuum wheel bridge, WITHOUT the IA368II nodes (their battery model drains per wall second).
# sim_slowdown 6.0 = about 1/rtf (measured 0.19x). Local-only (Art. V).
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
P=/ws/src/vacuum_sim_bridge/config/sim_bridge.yaml
ros2 run vacuum_sim_bridge my_robot_scan_node --ros-args --params-file $P > /tmp/bridge_scan.log 2>&1 &
ros2 run vacuum_sim_bridge my_robot_odometry_node --ros-args --params-file $P -p odom_source:=ground_truth > /tmp/bridge_odom.log 2>&1 &
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/tmp/rect_map/room.yaml > /tmp/map_server.log 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map -p autostart:=true -p bond_timeout:=0.0 -p "node_names:=[map_server]" > /tmp/lm_map.log 2>&1 &
ros2 launch vacuum_bringup skeleton.launch.py nav:=true map_tf:=true real_nodes:=true with_sequencer:=true with_teleop:=false with_ia368ii:=false wheel_bridge:=vacuum sim_slowdown:=6.0 > /tmp/skeleton.log 2>&1 &
wait
