#!/usr/bin/env bash
# Builds and tests every package of src/ inside a throwaway Docker container, so nothing depends on what is
# installed on the host (Docker-first). Dependencies are installed by rosdep from the package.xml files,
# which is where they are tracked (twist_mux, python3-pytest, python3-yaml, ...).
# Usage: scripts/test-docker.sh [jazzy|lyrical|both]        (default both)
#        ROS_IMAGE=<image> scripts/test-docker.sh lyrical    overrides the image
#        TEST_OUT=<dir> scripts/test-docker.sh lyrical       also copies the XML results and logs to <dir>/<distro>/
# Exit code 0 only if every selected distro passes.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHICH="${1:-both}"
case "$WHICH" in jazzy|lyrical|both) ;; *) echo "usage: $0 [jazzy|lyrical|both]" >&2; exit 2;; esac
fail=0

run() {
  local distro="$1" image="$2"
  echo ">> $distro ($image)"
  # The repo is mounted read-only; the container works on a copy without the third-party checkout,
  # build outputs and .git.
  local outargs=()
  if [ -n "${TEST_OUT:-}" ]; then
    mkdir -p "$TEST_OUT/$distro"
    outargs=(-v "$(cd "$TEST_OUT/$distro" && pwd)":/out)
  fi
  docker run --rm -e DISTRO="$distro" -v "$REPO":/repo:ro "${outargs[@]}" "$image" bash -c '
    set -e
    mkdir -p /ws && cd /repo && tar --exclude=./src/deps --exclude=./build --exclude=./install --exclude=./log \
      --exclude=./.git --exclude=./.sim --exclude=./.venv -cf - . | tar -xf - -C /ws
    cd /ws
    apt-get update -qq
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null
    rosdep update --rosdistro "$DISTRO" >/dev/null
    rosdep install --from-paths src --ignore-src -r -y >/tmp/rosdep.log 2>&1 || { tail -20 /tmp/rosdep.log; exit 3; }
    source /opt/ros/"$DISTRO"/setup.bash
    colcon build --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 4; }
    colcon test --base-paths src >/tmp/test.log 2>&1 || true
    rc=0
    colcon test-result --all --verbose >/tmp/result.log 2>&1 || rc=$?
    python3 -I /ws/scripts/show-test-failures.py build || true
    grep -E "^Summary" /tmp/result.log || tail -3 /tmp/result.log
    if [ -d /out ]; then
      mkdir -p /out/xml && cp /tmp/*.log /tmp/result.log /out/ 2>/dev/null || true
      (cd build && find . \( -name pytest.xml -o -path "*/test_results/*.xml" \) -exec cp --parents {} /out/xml/ \;) || true
      cp -r log/latest_test /out/colcon_test_log 2>/dev/null || true
      chmod -R a+rwX /out || true
    fi
    exit $rc'
}

case "$WHICH" in
  jazzy|both)   run jazzy   "${ROS_IMAGE:-osrf/ros:jazzy-desktop-full}" || fail=1;;
esac
case "$WHICH" in
  lyrical|both) run lyrical "${ROS_IMAGE:-ros:lyrical-ros-base}" || fail=1;;
esac
exit $fail
