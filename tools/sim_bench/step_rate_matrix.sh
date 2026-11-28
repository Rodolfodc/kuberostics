#!/usr/bin/env bash
# One fresh simulator start per line of step_rate_probe.py arguments (local-only). Usage: step_rate_matrix.sh MODEL.ttm "ARGS" ["ARGS" ...]
# Each ARGS string is passed to step_rate_probe.py (e.g. "--mode free --poll --kinect-explicit"). Results go to stdout (RESULT lines).
set -uo pipefail
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; model="$1"; shift
for args in "$@"; do
  "$R/tools/sim.sh" stop >/dev/null; sleep 5
  SIM_HEADLESS=0 "$R/tools/sim.sh" start >/dev/null || { echo "start failed" >&2; exit 1; }
  sleep 14
  echo "ARGS: $args"
  # shellcheck disable=SC2086
  timeout 400 "$R/.venv/bin/python" "$R/tools/sim_bench/step_rate_probe.py" "$model" $args 2>&1 | tail -3
done
"$R/tools/sim.sh" stop >/dev/null
