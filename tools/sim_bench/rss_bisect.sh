#!/usr/bin/env bash
# Runs rss_bisect.py for each config with a fresh tools/sim.sh start (GUI, SIM_HEADLESS=0) and stops the sim after each.
# Usage: rss_bisect.sh [minutes] [model.ttm] CONFIG...   (local-only; results appended to .sim/rss_bisect.csv)
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
min="$1"; model="$2"; shift 2
for c in "$@"; do
  SIM_HEADLESS=0 "$R/tools/sim.sh" start || exit 1
  sleep 15
  "$R/.venv/bin/python" "$R/tools/sim_bench/rss_bisect.py" "$c" --minutes "$min" --model "$model" --out "$R/.sim/rss_bisect.csv" 2>&1 | tee -a "$R/.sim/rss_bisect.log"
  "$R/tools/sim.sh" stop; sleep 5
done
