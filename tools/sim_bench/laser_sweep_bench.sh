#!/usr/bin/env bash
# steps/s of a model for each laser sweep period N, one fresh simulator start per N (local-only).
# Some simulator starts are stuck at ~1.05 s per step regardless of the load (cause not found); the python script exits with
# code 3 then and this wrapper restarts the simulator (up to 8 tries per N). Stepping mode only.
# Usage: laser_sweep_bench.sh MODEL.ttm [--kinect-explicit] [N ...]   (default N: 1 2 4; N=1000 means no sweep after the first)
# No `set -e`: the python exit code is inspected (rc=$?) to decide on a retry; -u and pipefail are on.
set -uo pipefail
[ $# -ge 1 ] || { echo "usage: $0 MODEL.ttm [--kinect-explicit] [N ...]" >&2; exit 2; }
B="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; R="$B/../.."
model="$1"; shift
[ -f "$model" ] || { echo "model not found: $model" >&2; exit 2; }
extra=(); [ -n "${DENSITY:-}" ] && extra+=(--density "$DENSITY")
if [ "${1:-}" = "--kinect-explicit" ]; then extra+=(--kinect-explicit); shift; fi
N_LIST=(1 2 4); [ $# -gt 0 ] && N_LIST=("$@")
for N in "${N_LIST[@]}"; do
  for try in 1 2 3 4 5 6 7 8; do
    "$R/tools/sim.sh" stop >/dev/null; sleep 6
    SIM_HEADLESS=0 "$R/tools/sim.sh" start >/dev/null || { echo "simulator start failed" >&2; exit 1; }
    sleep 12
    timeout 300 "$R/.venv/bin/python" "$B/laser_sweep_bench.py" "$model" --n "$N" "${extra[@]}"; rc=$?
    [ "$rc" -ne 3 ] && break
  done
done
"$R/tools/sim.sh" stop
