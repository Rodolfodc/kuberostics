#!/usr/bin/env bash
# (Re)starts the four bridges + twist_mux inside the bridgelive-jazzy container (restarting the container is the only way
# to stop them: no kill by name). Usage: bridge_live_start.sh [odom_source] [fixed_camera]
# fixed_camera=1 runs with camera_flip_horizontal=false and camera_flip_vertical=true (the values the live check supports).
docker restart bridgelive-jazzy >/dev/null
P=/ws/src/vacuum_sim_bridge/config/sim_bridge.yaml
if [ "${2:-0}" = 1 ]; then
  docker exec bridgelive-jazzy bash -c "sed -e 's/camera_flip_horizontal: true/camera_flip_horizontal: false/' -e 's/camera_flip_vertical: false/camera_flip_vertical: true/' $P > /tmp/params.yaml"
  P=/tmp/params.yaml
fi
docker exec -d bridgelive-jazzy bash -c "source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; ros2 launch vacuum_sim_bridge sim_bridges.launch.py odom_source:=${1:-ground_truth} params_file:=$P > /tmp/bridges.log 2>&1"
docker exec -d bridgelive-jazzy bash -c 'source /opt/ros/jazzy/setup.bash; ros2 run twist_mux twist_mux --ros-args --params-file /ws/src/vacuum_bringup/config/twist_mux.yaml -r cmd_vel_out:=/myRobot/cmd_vel > /tmp/mux.log 2>&1'
sleep 10
