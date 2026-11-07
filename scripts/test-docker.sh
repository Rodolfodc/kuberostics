#!/usr/bin/env bash
# Builds and tests every package of src/ inside a throwaway Docker container, so nothing depends on what is
# installed on the host (Docker-first). Dependencies are installed by rosdep from the package.xml files,
# which is where they are tracked (twist_mux, python3-pytest, python3-yaml, ...).
# Usage: scripts/test-docker.sh [jazzy|lyrical|both]        (default both)
#        ROS_IMAGE=<image> scripts/test-docker.sh lyrical    overrides the image
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
  docker run --rm -e DISTRO="$distro" -v "$REPO":/repo:ro "$image" bash -c '
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
    colcon test-result --all | grep -E "fail|error|Summary" | tail -15 || true
    colcon test-result >/dev/null'
}

case "$WHICH" in
  jazzy|both)   run jazzy   "${ROS_IMAGE:-osrf/ros:jazzy-desktop-full}" || fail=1;;
esac
case "$WHICH" in
  lyrical|both) run lyrical "${ROS_IMAGE:-ros:lyrical-ros-base}" || fail=1;;
esac
exit $fail
