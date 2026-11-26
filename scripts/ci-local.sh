#!/usr/bin/env bash
# Local CI: replaces the (slow, free-tier) GitHub Actions run. For one PR it builds a throwaway worktree of
# origin/<base> plus a merge of the PR head (mimics GitHub's refs/pull/N/merge; a conflict is a failure), runs
# scripts/test-docker.sh for jazzy AND lyrical in Docker, and publishes a commit status `ci/local` on the head SHA.
# Usage: scripts/ci-local.sh <pr-number|branch> [--cpus=N] [--no-status]
#   --cpus=N     docker CPU limit per run (default 2, imitates the 2-core GHA runner)
#   --no-status  do not touch GitHub (no status call at all)
#   <branch>     a branch with an open PR resolves to that PR; otherwise a local/origin branch is tested against
#                origin/main (never published, "local" instead of a PR number)
# Output: local-logs/ci/<pr>-<sha12>/{summary.txt,merge.log,<distro>/...} (gitignored, redacted by test-docker.sh).
# Containers are named ci-local-<pr>-<distro>-<pid>; only those are ever removed (on interrupt). The test
# scripts (container-test.sh, show-test-failures.py) of THIS checkout are copied into the throwaway tree, so PRs
# branched before they existed are tested the same way. Exit 0 only if the merge is clean and both distros pass
# with at least one test. Needs: git, gh (for PRs/status), docker.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CPUS=2; PUBLISH=1; ARG=""
for a in "$@"; do
  case "$a" in
    --cpus=[0-9]*) CPUS="${a#--cpus=}";;
    --no-status) PUBLISH=0;;
    -h|--help) sed -n 2,14p "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0;;
    -*) echo "unknown flag: $a" >&2; exit 2;;
    *) if [ -z "$ARG" ]; then ARG="$a"; else echo "one PR or branch only" >&2; exit 2; fi;;
  esac
done
[ -n "$ARG" ] || { echo "usage: $0 <pr-number|branch> [--cpus=N] [--no-status]" >&2; exit 2; }
case "$CPUS" in ''|*[!0-9.]*) echo "--cpus must be a number" >&2; exit 2;; esac
case "$ARG" in *[!A-Za-z0-9._/-]*) echo "invalid PR or branch name" >&2; exit 2;; esac

cd "$HERE"
git fetch -q origin
PR=""; BASE=main; HEAD_REF=""
if [[ "$ARG" =~ ^[0-9]+$ ]]; then PR="$ARG"; else
  PR="$(gh pr list --head "$ARG" --state open --json number --jq '.[0].number // empty' 2>/dev/null || true)"
fi
if [ -n "$PR" ]; then
  BASE="$(gh pr view "$PR" --json baseRefName --jq .baseRefName)"
  git fetch -q origin "pull/$PR/head"
  HEAD_REF=FETCH_HEAD
  PRTAG="$PR"
else
  PRTAG=local; PUBLISH=0
  for r in "refs/remotes/origin/$ARG" "refs/heads/$ARG"; do
    if git rev-parse -q --verify "$r" >/dev/null; then HEAD_REF="$r"; break; fi
  done
  [ -n "$HEAD_REF" ] || { echo "no PR or branch named $ARG" >&2; exit 2; }
fi
SHA="$(git rev-parse "$HEAD_REF")"
TAG="$PRTAG-${SHA:0:12}"
OUT="$HERE/local-logs/ci/$TAG"
rm -rf "$OUT"; mkdir -p "$OUT"
REPOSLUG=""; [ "$PUBLISH" = 1 ] && REPOSLUG="$(gh repo view --json nameWithOwner --jq .nameWithOwner)"

status() {  # status <state> <description>   (description: short, no logs, paths or hostnames)
  [ "$PUBLISH" = 1 ] || return 0
  gh api "repos/$REPOSLUG/statuses/$SHA" -f state="$1" -f context=ci/local -f description="${2:0:140}" >/dev/null \
    || echo "warning: could not publish the status" >&2
}

WT="$(mktemp -d "${TMPDIR:-/tmp}/ci-local-wt.XXXXXX")"
# shellcheck disable=SC2317  # invoked via trap
cleanup() {
  docker rm -f "ci-local-$PRTAG-$$-jazzy" "ci-local-$PRTAG-$$-lyrical" >/dev/null 2>&1 || true
  git -C "$HERE" worktree remove --force "$WT" >/dev/null 2>&1 || true
  rm -rf "$WT"
  git -C "$HERE" worktree prune
}
trap cleanup EXIT
trap 'echo "interrupted" >&2; status error "interrupted"; exit 130' INT TERM

status pending "running ($CPUS cpus)"
echo ">> CI local: $TAG (base origin/$BASE, cpus=$CPUS, status publishing: $([ "$PUBLISH" = 1 ] && echo on || echo off))"

git worktree add -q --detach "$WT" "origin/$BASE"
MERGE=clean
if ! git -C "$WT" -c user.name=ci-local -c user.email=ci-local@localhost merge --no-edit --no-ff "$SHA" >"$OUT/merge.log" 2>&1; then
  MERGE=conflict
  git -C "$WT" merge --abort >/dev/null 2>&1 || true
fi
if [ "$MERGE" = conflict ]; then
  echo "!! merge with origin/$BASE conflicts (see merge.log in $OUT)"
  echo "FAIL: merge conflict with $BASE" >"$OUT/summary.txt"
  status failure "merge conflict with $BASE"
  exit 1
fi
# Use this checkout's test scripts (older PR trees may lack them).
mkdir -p "$WT/scripts"
cp "$HERE/scripts/container-test.sh" "$HERE/scripts/show-test-failures.py" "$WT/scripts/"

declare -A RES NTESTS
rc=0
for distro in jazzy lyrical; do
  d_rc=0
  REPO_DIR="$WT" CONTAINER_NAME="ci-local-$PRTAG-$$" TEST_OUT="$OUT" \
    "$HERE/scripts/test-docker.sh" "$distro" "--cpus=$CPUS" || d_rc=$?
  n="$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' "$OUT/$distro/console.log" 2>/dev/null | tail -1)"
  NTESTS[$distro]="${n:-0}"
  if [ "$d_rc" -ne 0 ]; then RES[$distro]=FAIL; rc=1
  elif [ "${NTESTS[$distro]}" -lt 1 ]; then RES[$distro]="FAIL(0 tests)"; rc=1
  else RES[$distro]=ok; fi
done

DESC="jazzy ${NTESTS[jazzy]} ${RES[jazzy]}, lyrical ${NTESTS[lyrical]} ${RES[lyrical]}, merge clean, $CPUS cpus"
{ [ "$rc" -eq 0 ] && echo "PASS: $DESC" || echo "FAIL: $DESC"; } >"$OUT/summary.txt"
echo "head: $SHA  base: origin/$BASE" >>"$OUT/summary.txt"
echo ">> $DESC"
echo ">> logs: $OUT"
if [ "$rc" -eq 0 ]; then status success "$DESC"; else status failure "$DESC"; fi
exit "$rc"
