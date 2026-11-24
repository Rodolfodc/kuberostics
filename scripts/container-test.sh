#!/usr/bin/env bash
# Runs INSIDE the container (called by scripts/test-docker.sh and scripts/repeat-test-docker.sh): copies the
# read-only repo to /ws, installs deps, builds, tests, and gathers diagnostics into /out (if mounted).
# Env: DISTRO (required); MODE=full|repeat (default full); for repeat: REPEAT=N, TEST_PKG=<package>, TEST_K=<pytest -k>.
# Exit codes: 0 pass, 1 tests failed or ran zero tests, 3 rosdep failed, 4 build failed, 5 repo copy failed.
# The preamble runs under set -e (a failed copy must never look like a green run); only colcon test/test-result tolerate errors.
set -e
MODE="${MODE:-full}"
OUT=/out
[ -d "$OUT" ] || OUT=""
export ROS_LOG_DIR=/ws/ros_log          # known dir so node logs (launch + rcl) are collectable
mkdir -p /ws "$ROS_LOG_DIR" /tmp/diag

# Redacts what must not leave the container: home dir, hostname, user name (guarded against empty/"/" values).
redact() {
  local h="${HOME:-/root}" n; n="$(hostname 2>/dev/null || true)"
  local args=(-e "s|/home/[A-Za-z0-9._-]*|/home/<USER>|g")
  [ "${#h}" -gt 1 ] && args+=(-e "s|$h|<HOME>|g")
  [ -n "$n" ] && args+=(-e "s|$n|<HOST>|g")
  sed "${args[@]}"
}

# copy_redacted <src-dir> <dst-dir> [find-expr...]: copies regular files (< 20 MB) through redact, keeping the tree.
copy_redacted() {
  local src="$1" dst="$2"; shift 2
  [ -d "$src" ] || return 0
  (cd "$src" && find . -type f -size -20M "$@" | while read -r f; do
    mkdir -p "$dst/$(dirname "$f")"; redact <"$f" >"$dst/$f" 2>/dev/null || true; done)
}

collect() {  # collect <rc> : copy every log/diagnostic to /out, redacted
  [ -n "$OUT" ] || return 0
  if [ "$1" -ne 0 ]; then env_report_heavy >/tmp/diag/environment-failure.txt || true; fi
  for f in /tmp/*.log /tmp/*.result; do [ -f "$f" ] && redact <"$f" >"$OUT/$(basename "$f")"; done
  copy_redacted /tmp/diag "$OUT"
  copy_redacted /ws/build "$OUT/xml" \( -name pytest.xml -o -path "*/test_results/*.xml" \)
  copy_redacted /ws/log "$OUT/colcon_log" \( -name '*.log' -o -name '*.json' -o -name '*.txt' \)
  copy_redacted "$ROS_LOG_DIR" "$OUT/ros_log"
  chmod -R a+rwX "$OUT" 2>/dev/null || true
}

env_report() {  # whitelisted environment only, never the full env (no secrets, no tokens)
  {
    echo "date_utc: $(date -u +%FT%TZ)"
    echo "distro: ${DISTRO}"
    echo "os: $(. /etc/os-release && echo "$PRETTY_NAME")  python: $(python3 --version 2>&1)  kernel: $(uname -r)"
    echo "cpus_nproc: $(nproc)  cgroup_cpu_max: $(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo n/a)"
    echo "loadavg: $(cat /proc/loadavg)"
    grep -E '^(MemTotal|MemAvailable)' /proc/meminfo | tr '\n' ' '; echo
    cat /sys/fs/cgroup/memory.max 2>/dev/null | sed 's/^/cgroup_memory_max: /'
    for v in ROS_DISTRO ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_LOG_DIR ROS_HOME; do
      echo "$v=${!v-<unset>}"
    done
  } 2>&1 | redact
}

env_report_heavy() {  # slow probes, run only on failure
  {
    echo "--- ros2 daemon status"; timeout 20 ros2 daemon status 2>&1 | head -5
    echo "--- ros2 doctor --report"; timeout 60 ros2 doctor --report 2>&1 | head -80
    echo "--- dmesg-lite"; (dmesg 2>/dev/null | tail -20) || true
  } 2>&1 | redact
}

step_fail() {  # step_fail <code> <log> <label>
  echo "!! $3 failed (exit $1); last 40 lines of $2:"; tail -40 "$2" | redact
  collect "$1"; exit "$1"
}

cd /repo && tar --exclude=./src/deps --exclude=./build --exclude=./install --exclude=./log \
  --exclude=./.git --exclude=./.sim --exclude=./.venv --exclude=./local-logs -cf - . | tar -xf - -C /ws
cd /ws
ls src/*/package.xml >/dev/null 2>&1 || { echo "!! no packages under /ws/src: the repo copy failed"; exit 5; }
apt-get update -qq
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null
rosdep update --rosdistro "$DISTRO" >/dev/null
rosdep install --from-paths src --ignore-src -r -y >/tmp/rosdep.log 2>&1 || step_fail 3 /tmp/rosdep.log rosdep
# shellcheck disable=SC1090
source /opt/ros/"$DISTRO"/setup.bash
colcon build --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >/tmp/build.log 2>&1 || step_fail 4 /tmp/build.log "colcon build"
# shellcheck disable=SC1091
source install/setup.bash
env_report >/tmp/diag/environment.txt

# -rA prints captured output of every test; --show-capture=all keeps stdout/stderr/log for failures.
PYTEST_ARGS=(-rA --show-capture=all)

if [ "$MODE" = repeat ]; then
  pass=0; failn=0
  for i in $(seq 1 "${REPEAT:-10}"); do
    tag=$(printf '%03d' "$i")
    set +e
    colcon test --base-paths src --packages-select "$TEST_PKG" --event-handlers console_direct+ \
      --pytest-args -k "${TEST_K:-.}" "${PYTEST_ARGS[@]}" >"/tmp/run-$tag.log" 2>&1
    colcon test-result --all >"/tmp/run-$tag.result" 2>&1; trc=$?
    set -e
    ntests=$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' "/tmp/run-$tag.result" | tail -1)
    if [ "$trc" -eq 0 ] && [ "${ntests:-0}" -ge 1 ]; then
      pass=$((pass+1)); echo "run $tag: PASS"
    else
      failn=$((failn+1)); echo "run $tag: FAIL (test-result rc=$trc, tests run: ${ntests:-0}; 0 tests means a bad TEST_PKG/TEST_K)"
      mkdir -p "/tmp/diag/failed-$tag"; cp -rL log/latest_test "/tmp/diag/failed-$tag/colcon_test_log" 2>/dev/null || true; cp "/tmp/run-$tag.log" "/tmp/diag/failed-$tag/" || true
      python3 -I /ws/scripts/show-test-failures.py build log/latest_test 40 | redact | head -150 || true
    fi
  done
  echo "repeat summary: $pass passed, $failn failed of ${REPEAT:-10} (cpus=$(nproc), cgroup cpu.max=$(cat /sys/fs/cgroup/cpu.max 2>/dev/null))"
  echo "$pass passed, $failn failed" >/tmp/diag/repeat-summary.txt
  collect "$failn"
  [ "$failn" -eq 0 ] && [ "$pass" -ge 1 ]; exit $?
fi

set +e
colcon test --base-paths src --event-handlers console_direct+ --pytest-args "${PYTEST_ARGS[@]}" >/tmp/test.log 2>&1
colcon test-result --all --verbose >/tmp/result.log 2>&1; rc=$?
set -e
ntests=$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' /tmp/result.log | tail -1)
if [ "$rc" -eq 0 ] && [ "${ntests:-0}" -lt 1 ]; then echo "!! zero tests ran: treating as failure"; rc=1; fi
python3 -I /ws/scripts/show-test-failures.py build log/latest_test 60 | redact
grep -E "^Summary" /tmp/result.log || tail -3 /tmp/result.log
collect "$rc"
exit "$rc"
