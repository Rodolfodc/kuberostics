#!/usr/bin/env bash
# Lets you start Claude Code from the folder ABOVE this repository and still get the project agents,
# settings, hooks and CLAUDE.md. Claude Code reads `.claude/` and `CLAUDE.md` from the folder where it
# starts, and only this repository is under git, so the real files stay here and the parent folder
# gets symlinks to them.
# Usage: scripts/link-workspace.sh [--remove]
# Idempotent. Never overwrites a real file or directory in the parent folder.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARENT="$(dirname "$REPO")"
NAME="$(basename "$REPO")"
ITEMS=(.claude CLAUDE.md)

for item in "${ITEMS[@]}"; do
  link="$PARENT/$item"
  target="$NAME/$item"                      # relative, so the pair can be moved together
  if [[ "${1:-}" == "--remove" ]]; then
    if [[ -L "$link" ]]; then rm "$link"; echo "removed $link"; fi
    continue
  fi
  [[ -e "$REPO/$item" ]] || { echo "missing $REPO/$item" >&2; exit 1; }
  if [[ -L "$link" ]]; then
    [[ "$(readlink "$link")" == "$target" ]] && { echo "ok      $link -> $target"; continue; }
    echo "$link is a symlink to something else ($(readlink "$link")); not touching it" >&2; exit 1
  elif [[ -e "$link" ]]; then
    echo "$link exists and is not a symlink; not touching it" >&2; exit 1
  fi
  ln -s "$target" "$link"
  echo "created $link -> $target"
done
