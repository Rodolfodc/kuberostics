#!/usr/bin/env bash
# T003b (001, Q-04): does the pinned IA368II code build and start on Jazzy and on Lyrical, with no simulator and the
# wheel bridge (my_robot_cmd_vel_node) off? Everything runs in a throwaway container; the repo is mounted read-only
# and the third-party checkout (`vcs import` of deps.repos) lives only inside the container (Art. II).
# Usage: scripts/ia368ii-check-docker.sh [jazzy|lyrical|both] [--jazzy-base] [--report-only]   (default both)
#        --jazzy-base   run Jazzy on the base flavour (same flavour as Lyrical) instead of desktop-full;
#                       output goes to local-logs/jazzy-base/ so the only difference to Lyrical is the distro (Art. X)
#        --report-only  always exit 0 (measurement mode); default: exit 1 on a failed build or a pin mismatch
#        ROS_IMAGE=<image> overrides the image, valid only with a single distro (not with `both`)
# Output: deploy/measurements/ia368ii-bringup-check/local-logs/<distro>/ (build/rosdep logs, per-node run logs, graph.txt,
#         report.txt). Generated, gitignored, local machine only; the written results are in
#         deploy/measurements/ia368ii-lyrical-jazzy-bringup.md
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHICH=both JAZZY_BASE=0 REPORT_ONLY=0
for a in "$@"; do case "$a" in
  jazzy|lyrical|both) WHICH="$a";; --jazzy-base) JAZZY_BASE=1;; --report-only) REPORT_ONLY=1;;
  *) echo "usage: $0 [jazzy|lyrical|both] [--jazzy-base] [--report-only]" >&2; exit 2;; esac; done
if [[ -n "${ROS_IMAGE:-}" && "$WHICH" == both ]]; then echo "ROS_IMAGE is valid only with a single distro" >&2; exit 2; fi
# Images come from scripts/setup_env.sh (it cannot be sourced: it runs the installer): single source of truth.
img() { sed -n "s/^$1=\"\([^\"]*\)\".*/\1/p" "$REPO/scripts/setup_env.sh" | head -1; }
IMG_JAZZY="$(img IMG_JAZZY)"; IMG_LYRICAL="$(img IMG_LYRICAL)"; IMG_JAZZY_BASE="$(img IMG_JAZZY_BASE)"
[[ -n "$IMG_JAZZY" && -n "$IMG_LYRICAL" && -n "$IMG_JAZZY_BASE" ]] || { echo "image names not found in setup_env.sh" >&2; exit 2; }
PIN="$(sed -n 's/^ *version: *//p' "$REPO/deps.repos" | head -1)"

INNER='
set -o pipefail
mkdir -p /ws /out && cd /ws
cp /repo/deps.repos .
mkdir -p src/deps
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq python3-vcstool git >/dev/null 2>&1 || echo "WARN: vcstool/git install failed" | tee -a /out/report.txt
vcs import src/deps < deps.repos >/out/vcs.log 2>&1; echo "vcs import exit=$?" | tee -a /out/report.txt
GOT="$(git -C src/deps/IA368II rev-parse HEAD)"
[ "$GOT" = "$PIN" ] && PINOK=ok || PINOK="MISMATCH(got $GOT, pinned $PIN)"
{ echo "ROS_DISTRO=$DISTRO"; echo "image=$IMAGE"; echo "python=$(python3 --version)"; echo "IA368II=$(git -C src/deps/IA368II rev-parse HEAD)"; } | tee -a /out/report.txt
# Only the two packages under test (and what they need) are built.
PK="$(find src/deps -name package.xml -path "*myrobot_interfaces*" -o -name package.xml -path "*myrobot_bringup*" -o -name package.xml -path "*my_robot_pkg*" | xargs -n1 dirname | tr "\n" " ")"
echo "packages: $PK" | tee -a /out/report.txt
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null 2>&1
rosdep update --rosdistro "$DISTRO" >/dev/null 2>&1
# rosdep with -r keeps going past unresolved keys; the unresolved ones are the interesting result.
rosdep install --from-paths $PK --ignore-src -r -y >/out/rosdep.log 2>&1; echo "rosdep install exit=$?" | tee -a /out/report.txt
UNRES="$(sed -n "s/.*Cannot locate rosdep definition for \[\(.*\)\]/\1/p" /out/rosdep.log | sort -u | tr "\n" " ")"
echo "rosdep UNRESOLVED keys (rosdep -r hides them in its exit code): ${UNRES:-none}" | tee -a /out/report.txt
source /opt/ros/"$DISTRO"/setup.bash
colcon build --base-paths $PK --continue-on-error --event-handlers console_direct- \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/out/build.log 2>&1
BRC=$?; echo "colcon build exit=$BRC" | tee -a /out/report.txt
grep -E "^(Starting|Finished|Failed|Aborted|Summary)|packages? (finished|failed|aborted)" /out/build.log | tee -a /out/report.txt
tail -40 /out/build.log | grep -i "error\|Traceback\|ModuleNotFound\|failed" | head -15 | tee -a /out/report.txt
echo "RESULT build_exit=$BRC pin=$PINOK" | tee -a /out/report.txt
source /ws/install/setup.bash 2>/dev/null || { echo "no install/setup.bash" | tee -a /out/report.txt; echo "RESULT build_exit=$BRC pin=$PINOK no-install-setup" >>/out/report.txt; echo "report complete" >>/out/report.txt; exit 0; }

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

NAME=""
trap '[[ -n "$NAME" ]] && docker rm -f "$NAME" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM
FAIL=0
run() {   # $1 distro  $2 image  $3 output label
  local distro="$1" image="${ROS_IMAGE:-$2}" label="$3" dest="$REPO/deploy/measurements/ia368ii-bringup-check/local-logs/$3"
  local name="ia368ii-check-$label-$$"; NAME="$name"     # unique per run; only this container is ever removed
  echo ">> $label ($image)"
  mkdir -p "$dest"
  docker create --name "$name" -e DISTRO="$distro" -e IMAGE="$image" -e PIN="$PIN" -v "$REPO":/repo:ro "$image" bash -c "$INNER" >/dev/null
  docker start -a "$name" >"$dest/console.log" 2>&1
  docker cp "$name":/out/. "$dest"/ >/dev/null 2>&1 || echo "  (nothing to copy)"
  docker rm -f "$name" >/dev/null 2>&1; NAME=""
  { grep -q "^RESULT build_exit=0 pin=ok" "$dest/report.txt" && ! grep -q "^no install/setup.bash" "$dest/report.txt"; } 2>/dev/null || { echo "   FAILED: build or pin check (see $dest/report.txt)"; FAIL=1; }
  echo "   report: $dest/report.txt"; cat "$dest/report.txt" 2>/dev/null || tail -20 "$dest/console.log"
}

if [[ "$WHICH" == jazzy || "$WHICH" == both ]]; then
  if [[ $JAZZY_BASE == 1 ]]; then run jazzy "$IMG_JAZZY_BASE" jazzy-base; else run jazzy "$IMG_JAZZY" jazzy; fi
fi
[[ "$WHICH" == lyrical || "$WHICH" == both ]] && run lyrical "$IMG_LYRICAL" lyrical
[[ $REPORT_ONLY == 1 ]] && exit 0
exit $FAIL
