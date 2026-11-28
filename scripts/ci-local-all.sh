#!/usr/bin/env bash
# Runs scripts/ci-local.sh for every open PR, sequentially, and prints a table. Exit 0 only if all pass.
# Usage: scripts/ci-local-all.sh [--cpus=N] [--no-status]     (flags are passed to ci-local.sh)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE" || exit 1
mapfile -t PRS < <(gh pr list --state open --json number --jq '.[].number' | sort -n)
[ "${#PRS[@]}" -gt 0 ] || { echo "no open PRs"; exit 0; }
rows=(); fail=0
for pr in "${PRS[@]}"; do
  "$HERE/scripts/ci-local.sh" "$pr" "$@" || fail=1
  sum="$(find "$HERE/local-logs/ci" -maxdepth 2 -name summary.txt -path "*/$pr-*" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
  res="$(head -1 "${sum:-/dev/null}" 2>/dev/null)"
  rows+=("#$pr|${res:-FAIL: no result}")
done
echo; printf '%-6s %s\n' PR RESULT
for r in "${rows[@]}"; do printf '%-6s %s\n' "${r%%|*}" "${r#*|}"; done
exit $fail
