#!/usr/bin/env bash
# Runs ONE test N times in Docker with --cpus=1 (the CI-like starved condition that exposes flaky ROS graph tests)
# and keeps every log in a local, gitignored directory (local-logs/, never committed).
# Usage: scripts/repeat-test-docker.sh <jazzy|lyrical> <package> [<pytest -k expr>] [N=10] [cpus=1]
#   e.g. scripts/repeat-test-docker.sh lyrical vacuum_bringup test_sequencer_graph 20
# Output: local-logs/repeat/<UTC timestamp>-<distro>-<package>/ (console.log, summary, per-run logs,
#         colcon_test_log of each failed run, ros_log/). Exit 0 only if all N runs pass.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${1:-}"; PKG="${2:-}"; K="${3:-.}"; N="${4:-10}"; CPUS="${5:-1}"
case "$DISTRO" in jazzy) IMAGE="${ROS_IMAGE:-osrf/ros:jazzy-desktop-full}";; lyrical) IMAGE="${ROS_IMAGE:-ros:lyrical-ros-base}";;
  *) echo "usage: $0 <jazzy|lyrical> <package> [<pytest -k expr>] [N] [cpus]" >&2; exit 2;; esac
[ -n "$PKG" ] || { echo "package required" >&2; exit 2; }
case "$PKG" in *[!A-Za-z0-9_]*) echo "package must match [A-Za-z0-9_]+" >&2; exit 2;; esac
case "$K" in *[!A-Za-z0-9_\ \(\).\[\]-]*) echo "-k expression has unsupported characters" >&2; exit 2;; esac
[ "${N:-0}" -ge 1 ] 2>/dev/null || { echo "N must be >= 1" >&2; exit 2; }
case "$N" in ''|*[!0-9]*) echo "N must be a number" >&2; exit 2;; esac

OUT="$REPO/local-logs/repeat/$(date -u +%Y%m%dT%H%M%SZ)-$DISTRO-$PKG"
mkdir -p "$OUT"
echo ">> $DISTRO/$PKG -k '$K' x$N at --cpus=$CPUS; logs: $OUT"
set +e
docker run --rm --cpus="$CPUS" -e DISTRO="$DISTRO" -e MODE=repeat -e REPEAT="$N" -e TEST_PKG="$PKG" -e TEST_K="$K" \
  -v "$REPO":/repo:ro -v "$OUT":/out "$IMAGE" bash /repo/scripts/container-test.sh 2>&1 \
  | sed -e "s|/home/$(id -un)|/home/<USER>|g" ${HOME:+-e "s|$HOME|<HOME>|g"} -e "s|$(hostname)|<HOST>|g" | tee "$OUT/console.log"
rc=${PIPESTATUS[0]}
exit "$rc"
