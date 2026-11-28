#!/usr/bin/env bash
# Starts the Nav2 live graph in the bridgelive-jazzy container (local-only, Art. V): an all-free 10 x 10 m map around the
# odometry origin (origin -5,-5; the real walls come from the laser), map_server, the identity map -> my_robot_odom
# transform and vacuum_bringup nav.launch.py. Env: MAP_YAML (default the free map), TF_X TF_Y (odometry origin in the map, default 0 0),
# RELAX_PROGRESS=1 (wall-clock progress allowance 240 s). Needs bridge_live_up.sh + bridge_live_start.sh first. Usage: nav_live_up.sh
set -euo pipefail
docker exec bridgelive-jazzy bash -c '
mkdir -p /tmp/navmap && cd /tmp/navmap
python3 - <<PY
n = 200
rows = [bytes(0 if (r < 2 or r >= n - 2 or c < 2 or c >= n - 2) else 254 for c in range(n)) for r in range(n)]
open("free.pgm", "wb").write(b"P5\n%d %d\n255\n" % (n, n) + b"".join(rows))
open("free.yaml", "w").write("image: free.pgm\nmode: trinary\nresolution: 0.05\norigin: [-5.0, -5.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
PY'
NAV_ARGS=""
if [ "${RELAX_PROGRESS:-0}" = 1 ]; then   # the simulator runs at ~0.1-0.2x real time: scale the wall-clock progress allowance
  docker exec bridgelive-jazzy bash -c 'sed "s/movement_time_allowance: 15.0/movement_time_allowance: 240.0/" /ws/src/vacuum_bringup/config/nav2_params.yaml > /tmp/nav2_params_relaxed.yaml'
  NAV_ARGS="params_file:=/tmp/nav2_params_relaxed.yaml"
fi
docker exec -d -e NAV_ARGS="$NAV_ARGS" -e MAP_YAML="${MAP_YAML:-/tmp/navmap/free.yaml}" -e TF_X="${TF_X:-0}" -e TF_Y="${TF_Y:-0}" bridgelive-jazzy bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML > /tmp/map_server.log 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map -p autostart:=true -p bond_timeout:=0.0 -p "node_names:=[map_server]" > /tmp/lm_map.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --x $TF_X --y $TF_Y --frame-id map --child-frame-id my_robot_odom > /tmp/static_tf.log 2>&1 &
ros2 launch vacuum_bringup nav.launch.py $NAV_ARGS > /tmp/nav.log 2>&1
wait'
for _ in $(seq 90); do
  if docker exec bridgelive-jazzy bash -c 'grep -q "Managed nodes are active" /tmp/nav.log'; then echo "nav2 active"; exit 0; fi
  sleep 1
done
echo "nav2 did not become active (see /tmp/nav.log)" >&2; exit 1
