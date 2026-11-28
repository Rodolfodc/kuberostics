#!/usr/bin/env bash
# Starts the long-lived jazzy container "fullrobot-jazzy" (host network, domain 78, localhost discovery, 4 CPUs) with the WHOLE
# workspace built: every src/vacuum_* package of the worktree given as $1 plus the pinned IA368II checkout (given as $2, the
# folder that holds myrobot_bringup/myrobot_interfaces; it is only read). Local-only helper (Art. V) for the full-robot live run.
# Usage: full_live_up.sh WORKTREE IA368II_CHECKOUT_DIR
set -euo pipefail
WT="$(cd "$1" && pwd)"; DEPS="$(cd "$2" && pwd)"; B="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker rm -f fullrobot-jazzy >/dev/null 2>&1 || true
docker run -d --name fullrobot-jazzy --network host --cpus=4 -e ROS_DOMAIN_ID=78 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$WT":/repo:ro -v "$DEPS":/ia368ii:ro -v "$B":/bench:ro osrf/ros:jazzy-desktop-full sleep infinity >/dev/null
docker exec fullrobot-jazzy bash -c '
set -e
mkdir -p /ws/src && cd /ws
for p in /repo/src/vacuum_*; do cp -r "$p" src/; done
for p in myrobot_bringup myrobot_interfaces; do [ -d /ia368ii/$p ] && cp -r /ia368ii/$p src/ || find /ia368ii -maxdepth 3 -name $p -type d -exec cp -r {} src/ \; ; done
apt-get update -qq
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null
rosdep update --rosdistro jazzy >/dev/null
rosdep install --from-paths src --ignore-src -r -y --skip-keys "teleop_twist_keyboard" >/tmp/rosdep.log 2>&1 || { tail -20 /tmp/rosdep.log; exit 3; }
apt-get install -y -qq python3-pip >/dev/null 2>&1 || true
pip install --break-system-packages -q coppeliasim-zmqremoteapi-client >/dev/null 2>&1 || true
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 4; }
echo BUILD_OK'
