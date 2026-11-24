#!/usr/bin/env bash
# Builds and tests every package of src/ inside a throwaway Docker container, so nothing depends on what is
# installed on the host (Docker-first). Dependencies are installed by rosdep from the package.xml files,
# which is where they are tracked (twist_mux, python3-pytest, python3-yaml, ...). The in-container work is
# scripts/container-test.sh.
# Usage: scripts/test-docker.sh [jazzy|lyrical|both] [--cpus=N]       (default both)
#        ROS_IMAGE=<image> scripts/test-docker.sh lyrical    overrides the image
#        TEST_OUT=<dir> scripts/test-docker.sh lyrical       also copies diagnostics to <dir>/<distro>/:
#          summary.txt, console.log, environment.txt, xml/, colcon_log/, ros_log/ (all redacted: no home, host or user names)
#        scripts/test-docker.sh lyrical --cpus=1             CPU-starved run (reproduces CI-like flakiness)
# CI uploads TEST_OUT as an artifact on failure. Exit code 0 only if every selected distro passes.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHICH="${1:-both}"
case "$WHICH" in jazzy|lyrical|both) ;; *) echo "usage: $0 [jazzy|lyrical|both] [--cpus=N]" >&2; exit 2;; esac
CPUS="${2:-}"   # optional docker flag, e.g. --cpus=1
case "$CPUS" in ''|--cpus=[0-9]*) ;; *) echo "second argument must be --cpus=N" >&2; exit 2;; esac
fail=0

redact_host() { sed -e "s|$HOME|<HOME>|g" -e "s|$(hostname)|<HOST>|g" -e "s|/home/$(id -un)|/home/<USER>|g"; }

repro_hint() {  # repro_hint <distro> <image>
  cat <<HINT
== REPRODUCE LOCALLY (needs only Docker) ==
  scripts/test-docker.sh $1                                          # same build + test as CI
  scripts/test-docker.sh $1 --cpus=1                                 # starved CPU: flaky graph tests show up here
  scripts/repeat-test-docker.sh $1 <package> '<pytest -k expr>' 20   # one test x20 at --cpus=1, logs in local-logs/
  (image: $2; CI artifact: test-results-$1)
HINT
}

run() {
  local distro="$1" image="$2"
  echo ">> $distro ($image)"
  # The repo is mounted read-only; the container works on a copy.
  local outargs=() out="" t0 rc=0
  if [ -n "${TEST_OUT:-}" ]; then
    mkdir -p "$TEST_OUT/$distro"
    out="$(cd "$TEST_OUT/$distro" && pwd)"
    outargs=(-v "$out":/out)
  fi
  t0=$(date -u +%FT%TZ)
  set +e
  # shellcheck disable=SC2086
  docker run --rm ${CPUS:+$CPUS} -e DISTRO="$distro" -v "$REPO":/repo:ro ${outargs[@]+"${outargs[@]}"} "$image" \
    bash /repo/scripts/container-test.sh 2>&1 | redact_host | { if [ -n "$out" ]; then tee "$out/console.log"; else cat; fi; }
  rc=${PIPESTATUS[0]}
  set -e
  if [ -n "$out" ]; then
    {
      echo "start_utc: $t0"; echo "end_utc: $(date -u +%FT%TZ)"
      echo "distro: $distro"; echo "image: $image"
      echo "image_id: $(docker image inspect --format '{{.Id}} {{join .RepoDigests ","}}' "$image" 2>/dev/null)"
      echo "exit_code: $rc"
      echo "host_cpus: $(nproc)  docker_cpu_limit: ${CPUS:-none}  host_load: $(cut -d' ' -f1-3 /proc/loadavg)"
      echo "docker: $(docker version --format '{{.Server.Version}}' 2>/dev/null)"
      echo "git_head: $(git -C "$REPO" rev-parse HEAD 2>/dev/null)"
    } | redact_host >"$out/summary.txt"
  fi
  if [ "$rc" -ne 0 ]; then
    echo "!! $distro FAILED (exit $rc)"
    repro_hint "$distro" "$image"
    return 1
  fi
}

case "$WHICH" in
  jazzy|both)   run jazzy   "${ROS_IMAGE:-osrf/ros:jazzy-desktop-full}" || fail=1;;
esac
case "$WHICH" in
  lyrical|both) run lyrical "${ROS_IMAGE:-ros:lyrical-ros-base}" || fail=1;;
esac
exit $fail
