#!/usr/bin/env bash
# steps/s of a model for each laser sweep period N, one fresh simulator start per N (local-only).
# Some simulator starts are stuck at ~1.05 s per step regardless of the load (cause not found); the python script exits with
# code 3 then and this wrapper restarts the simulator (up to 8 tries per N). Stepping mode only.
# Usage: laser_sweep_bench.sh MODEL.ttm [--kinect-explicit] [N ...]   (default N: 1 2 4; N=1000 means no sweep after the first)
set -u
B="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; R="$B/../.."
model="$1"; shift; extra=(); [ "${1:-}" = "--kinect-explicit" ] && { extra=(--kinect-explicit); shift; }
for N in ${@:-1 2 4}; do
  for try in 1 2 3 4 5 6 7 8; do
    "$R/tools/sim.sh" stop >/dev/null; sleep 6; SIM_HEADLESS=0 "$R/tools/sim.sh" start >/dev/null; sleep 12
    timeout 300 "$R/.venv/bin/python" "$B/laser_sweep_bench.py" "$model" --n "$N" "${extra[@]}"; rc=$?
    [ $rc -ne 3 ] && break
  done
done
"$R/tools/sim.sh" stop
