#!/usr/bin/env bash
# Probe of the REAL twist_mux with our config (src/vacuum_bringup/config/twist_mux.yaml), in a throwaway container.
# Checks the behaviour that D-22 / 001-R10 / 001-R19 depend on:
#   1. unlocked: teleop passes (x = 0.2)
#   2. locked, no guard: nothing is published (the reason stop_guard exists)
#   3. locked + zero Twist on mux/stop: output is exactly zero
#   4. locked, guard gone: the stop input times out, output goes silent (last command was zero)
#   5. lock released: teleop passes again
# Usage: tools/probes/mux_lock_probe.sh [jazzy|lyrical]   (default jazzy; needs docker and network)
#        ROS_IMAGE=<image> overrides the image (defaults: osrf/ros:jazzy-desktop-full, ros:lyrical-ros-base)
# Exit code 0 only if every check holds. Interim until the launch_testing tests of T014.
set -euo pipefail
DISTRO="${1:-jazzy}"
case "$DISTRO" in
  jazzy)   IMAGE="${ROS_IMAGE:-osrf/ros:jazzy-desktop-full}";;
  lyrical) IMAGE="${ROS_IMAGE:-ros:lyrical-ros-base}";;
  *) echo "unknown distro: $DISTRO (use jazzy or lyrical)" >&2; exit 2;;
esac
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

docker run --rm -v "$REPO/src/vacuum_bringup/config:/cfg:ro" "$IMAGE" bash -c '
source /opt/ros/'"$DISTRO"'/setup.bash
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq ros-'"$DISTRO"'-twist-mux >/dev/null 2>&1 || { echo "FAIL: cannot install twist_mux"; exit 2; }
export ROS_DOMAIN_ID=93
fail=0
check() { if [ "$2" = "$3" ]; then echo "ok   - $1"; else echo "FAIL - $1 (expected $3, got $2)"; fail=1; fi; }
out() { timeout 3 ros2 topic echo --once /myRobot/cmd_vel --field linear.x 2>/dev/null | head -1; }

ros2 run twist_mux twist_mux --ros-args --params-file /cfg/twist_mux.yaml -r cmd_vel_out:=/myRobot/cmd_vel >/tmp/mux.log 2>&1 &
sleep 3
ros2 topic pub -r 10 /myRobot/mux/teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}" >/dev/null 2>&1 &
sleep 2
check "1. unlocked: teleop passes" "$(out)" "0.2"

ros2 topic pub -r 10 /myRobot/mux/lock_stop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 &
LOCK=$!
sleep 2
check "2. locked, no guard: nothing published" "$(out)" ""

ros2 topic pub -r 10 /myRobot/mux/stop geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
GUARD=$!
sleep 2
check "3. locked + guard: exact zero" "$(out)" "0.0"

kill $GUARD; sleep 1.5
check "4. guard gone: input timed out, silent" "$(out)" ""

kill $LOCK; ros2 topic pub --once /myRobot/mux/lock_stop std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
sleep 1
check "5. lock released: teleop passes" "$(out)" "0.2"
exit $fail'
