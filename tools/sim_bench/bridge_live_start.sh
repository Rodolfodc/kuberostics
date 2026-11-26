#!/usr/bin/env bash
# (Re)starts the four bridges + twist_mux inside the bridgelive-jazzy container (restarting the container is the only way
# to stop them: no kill by name). Usage: bridge_live_start.sh [odom_source] [fixed_camera]
# fixed_camera=1 runs with camera_flip_horizontal=true and camera_flip_vertical=false (the pre-fix values), to compare.
# Local-only. Needs the container from bridge_live_up.sh.
set -euo pipefail
odom="${1:-ground_truth}"; fixed="${2:-0}"
[[ "$odom" =~ ^[A-Za-z0-9_]+$ ]] || { echo "bad odom_source: $odom" >&2; exit 2; }
docker restart bridgelive-jazzy >/dev/null
P=/ws/src/vacuum_sim_bridge/config/sim_bridge.yaml
if [ "$fixed" = 1 ]; then
  docker exec bridgelive-jazzy bash -c 'sed -e "s/camera_flip_horizontal: false/camera_flip_horizontal: true/" -e "s/camera_flip_vertical: true/camera_flip_vertical: false/" "$1" > /tmp/params.yaml' _ "$P"
  P=/tmp/params.yaml
fi
if [ "${NOVISION:-0}" = 1 ]; then   # scan + odometry + wheels only (the Kinect render costs about half of the step time)
  docker exec -d -e ODOM="$odom" -e PARAMS="$P" bridgelive-jazzy bash -c \
    'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
     for n in scan odometry cmd_vel; do
       x=(); [ $n = odometry ] && x=(-p odom_source:="$ODOM"); [ $n = cmd_vel ] && x=(-r /cmd_vel:=/myRobot/cmd_vel)
       ros2 run vacuum_sim_bridge my_robot_${n}_node --ros-args --params-file "$PARAMS" "${x[@]}" >> /tmp/bridges.log 2>&1 &
     done; wait'
  NBRIDGES=3
else
docker exec -d -e ODOM="$odom" -e PARAMS="$P" bridgelive-jazzy bash -c \
  'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; ros2 launch vacuum_sim_bridge sim_bridges.launch.py odom_source:="$ODOM" params_file:="$PARAMS" > /tmp/bridges.log 2>&1'
  NBRIDGES=4
fi
docker exec -d bridgelive-jazzy bash -c 'source /opt/ros/jazzy/setup.bash; ros2 run twist_mux twist_mux --ros-args --params-file /ws/src/vacuum_bringup/config/twist_mux.yaml -r cmd_vel_out:=/myRobot/cmd_vel > /tmp/mux.log 2>&1'
# wait (max 60 s) until all four bridges and the mux report in their logs
for _ in $(seq 60); do
  if docker exec -e NBRIDGES=$NBRIDGES bridgelive-jazzy bash -c '[ "$(grep -c "connected to the simulator" /tmp/bridges.log 2>/dev/null)" -ge $NBRIDGES ] && grep -q "subscribed to topic" /tmp/mux.log 2>/dev/null'; then
    echo "bridges and twist_mux up"; exit 0
  fi
  sleep 1
done
echo "timeout waiting for the bridges (see /tmp/bridges.log in the container)" >&2; exit 1
