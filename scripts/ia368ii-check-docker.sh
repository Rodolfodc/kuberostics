#!/usr/bin/env bash
# T003b (001, Q-04): does the pinned IA368II code build and start on Jazzy and on Lyrical, with no simulator and the
# wheel bridge (my_robot_cmd_vel_node) off? Everything runs in a throwaway container; the repo is mounted read-only
# and the third-party checkout (`vcs import` of deps.repos) lives only inside the container (Art. II).
# Usage: scripts/ia368ii-check-docker.sh [jazzy|lyrical|both]   (default both)
#        ROS_IMAGE=<image> scripts/ia368ii-check-docker.sh lyrical     overrides the image
# Output: deploy/measurements/t003b/<distro>/ (build/rosdep logs, per-node run logs, report.txt)
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHICH="${1:-both}"
case "$WHICH" in jazzy|lyrical|both) ;; *) echo "usage: $0 [jazzy|lyrical|both]" >&2; exit 2;; esac
# Same images that scripts/setup_env.sh tracks (IMG_JAZZY / IMG_LYRICAL).
IMG_JAZZY="osrf/ros:jazzy-desktop-full"; IMG_LYRICAL="ros:lyrical-ros-base"

INNER='
set -o pipefail
mkdir -p /ws /out && cd /ws
cp /repo/deps.repos .
mkdir -p src/deps
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq python3-vcstool git >/dev/null 2>&1 || echo "WARN: vcstool/git install failed" | tee -a /out/report.txt
vcs import src/deps < deps.repos >/out/vcs.log 2>&1; echo "vcs import exit=$?" | tee -a /out/report.txt
{ echo "ROS_DISTRO=$DISTRO"; echo "image=$IMAGE"; echo "python=$(python3 --version)"; echo "IA368II=$(git -C src/deps/IA368II rev-parse HEAD)"; } | tee -a /out/report.txt
# Only the two packages under test (and what they need) are built.
PK="$(find src/deps -name package.xml -path "*myrobot_interfaces*" -o -name package.xml -path "*myrobot_bringup*" -o -name package.xml -path "*my_robot_pkg*" | xargs -n1 dirname | tr "\n" " ")"
echo "packages: $PK" | tee -a /out/report.txt
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null 2>&1
rosdep update --rosdistro "$DISTRO" >/dev/null 2>&1
# rosdep with -r keeps going past unresolved keys; the unresolved ones are the interesting result.
rosdep install --from-paths $PK --ignore-src -r -y >/out/rosdep.log 2>&1; echo "rosdep install exit=$?" | tee -a /out/report.txt
grep -i "cannot locate\|ERROR\|unresolved" /out/rosdep.log | sort -u | tee -a /out/report.txt
source /opt/ros/"$DISTRO"/setup.bash
colcon build --base-paths $PK --continue-on-error --event-handlers console_direct- \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/out/build.log 2>&1
echo "colcon build exit=$?" | tee -a /out/report.txt
grep -E "^(Starting|Finished|Failed|Aborted|Summary)|packages? (finished|failed|aborted)" /out/build.log | tee -a /out/report.txt
tail -40 /out/build.log | grep -i "error\|Traceback\|ModuleNotFound\|failed" | head -15 | tee -a /out/report.txt
source /ws/install/setup.bash 2>/dev/null || { echo "no install/setup.bash" | tee -a /out/report.txt; exit 0; }

# --- runtime: each executable alone, 10 s, no simulator. exit 124 = still running when the timeout hit (good). ---
echo "== executables (10 s each) ==" | tee -a /out/report.txt
for pkg in myrobot_bringup my_robot_pkg; do
  for exe in $(ros2 pkg executables "$pkg" 2>/dev/null | awk "{print \$2}"); do
    [ "$exe" = my_robot_cmd_vel_node ] && { echo "$pkg/$exe: SKIPPED (wheel bridge off)" | tee -a /out/report.txt; continue; }
    timeout 10 ros2 run "$pkg" "$exe" >/out/run_"$pkg"_"$exe".log 2>&1; rc=$?
    echo "$pkg/$exe: exit=$rc $( [ $rc = 124 ] && echo alive || echo EXITED )" | tee -a /out/report.txt
    [ $rc != 124 ] && grep -m3 -i "error\|Traceback\|ModuleNotFound\|No module" /out/run_"$pkg"_"$exe".log | sed "s/^/    /" | tee -a /out/report.txt
  done
done

# --- runtime: the upstream bringup launch, 25 s, then inspect the graph ---
echo "== ros2 launch myrobot_bringup bringup.launch.py ==" | tee -a /out/report.txt
ros2 launch myrobot_bringup bringup.launch.py >/out/launch.log 2>&1 &
LP=$!
sleep 20
{ echo "-- node list"; timeout 10 ros2 node list; echo "-- topics"; timeout 10 ros2 topic list -t;
  echo "-- mode (one sample)"; timeout 8 ros2 topic echo --once /myRobot/behaviour/mode;
  for n in /myrobot /behaviour_server /cleaning_server /autodocking_server /charging_server /charged_server; do
    echo "-- lifecycle $n: $(timeout 5 ros2 lifecycle get $n 2>&1 | head -1)"; done; } >/out/graph.txt 2>&1
cat /out/graph.txt >>/out/report.txt
kill -INT $LP 2>/dev/null; sleep 3; kill -9 $LP 2>/dev/null
grep -i "error\|Traceback\|ModuleNotFound\|died\|process has died" /out/launch.log | sort -u | head -15 | tee -a /out/report.txt
echo "report complete" >>/out/report.txt
'

run() {
  local distro="$1" image="${ROS_IMAGE:-$2}" name="t003b-$1" dest="$REPO/deploy/measurements/t003b/$1"
  echo ">> $distro ($image)"
  docker rm -f "$name" >/dev/null 2>&1 || true
  mkdir -p "$dest"
  docker create --name "$name" -e DISTRO="$distro" -e IMAGE="$image" -v "$REPO":/repo:ro "$image" bash -c "$INNER" >/dev/null
  docker start -a "$name" >"$dest/console.log" 2>&1
  docker cp "$name":/out/. "$dest"/ >/dev/null 2>&1 || echo "  (nothing to copy)"
  docker rm -f "$name" >/dev/null 2>&1
  echo "   report: $dest/report.txt"; cat "$dest/report.txt" 2>/dev/null || tail -20 "$dest/console.log"
}

[[ "$WHICH" == jazzy   || "$WHICH" == both ]] && run jazzy "$IMG_JAZZY"
[[ "$WHICH" == lyrical || "$WHICH" == both ]] && run lyrical "$IMG_LYRICAL"
exit 0
