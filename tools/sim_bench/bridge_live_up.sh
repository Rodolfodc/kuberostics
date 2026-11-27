#!/usr/bin/env bash
# Starts a long-lived jazzy container "bridgelive-jazzy" (host network, domain 77, localhost discovery), builds the
# vacuum_sim_bridge package from the worktree given as $1 (read-only mount) and installs twist_mux + the ZMQ client.
# Local-only helper for the bridge live checks. Usage: bridge_live_up.sh WORKTREE
set -euo pipefail
WT="$(cd "$1" && pwd)"; B="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker rm -f bridgelive-jazzy >/dev/null 2>&1 || true
docker run -d --name bridgelive-jazzy --network host --cpus=2 -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$WT":/repo:ro -v "$B":/bench:ro osrf/ros:jazzy-desktop-full sleep infinity >/dev/null
docker exec bridgelive-jazzy bash -c '
set -e
mkdir -p /ws/src && cp -r /repo/src/vacuum_sim_bridge /ws/src/ && cp -r /repo/src/vacuum_bringup /ws/src/
cd /ws; apt-get update -qq
apt-get install -y -qq ros-jazzy-twist-mux python3-pip >/dev/null 2>&1
pip install --break-system-packages -q coppeliasim-zmqremoteapi-client >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/vacuum_sim_bridge --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 4; }
echo BUILD_OK'
