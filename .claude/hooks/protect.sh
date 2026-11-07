#!/usr/bin/env bash
# PreToolUse hook: blocks edits/commands that touch protected paths or dangerous patterns.
# Exit 2 = block (stderr is shown to the agent). Needs jq.
set -u
in=$(cat)
tool=$(jq -r '.tool_name // ""' <<<"$in")
block() { echo "BLOCKED by .claude/hooks/protect.sh: $1" >&2; exit 2; }

case "$tool" in
  Edit|Write|NotebookEdit)
    f=$(jq -r '.tool_input.file_path // .tool_input.notebook_path // ""' <<<"$in")
    case "$f" in
      */src/deps/*|src/deps/*)            block "src/deps/ is third-party read-only code (Art. II)";;
      *scene_house.ttt*)                  block "the original CoppeliaSim scene is read-only; work on the copy in .sim/";;
      */.git/*|.git/*)                    block "never edit .git directly";;
      */.specify/memory/constitution.md)  block "constitution changes are the user's decision (amend explicitly)";;
    esac
    ;;
  Bash)
    c=$(jq -r '.tool_input.command // ""' <<<"$in")
    grep -Eq 'git +push.*(--force|-f( |$)|--force-with-lease)' <<<"$c" && block "force push is forbidden"
    grep -Eq 'git +(reset +--hard|clean +-[a-z]*f|checkout +--? *\.)' <<<"$c" && block "destructive git command; ask the user"
    grep -Eq '(pkill|killall)' <<<"$c" && block "do not kill processes by name; use tools/sim.sh stop"
    grep -Eq 'docker\.sock|--privileged' <<<"$c" && block "no docker.sock mount or --privileged"
    grep -Eq 'rm +(-[a-zA-Z]*r[a-zA-Z]*f|-[a-zA-Z]*f[a-zA-Z]*r) +(/|~|\$HOME)( |$)' <<<"$c" && block "rm -rf on root/home"
    grep -Eq 'src/deps' <<<"$c" && grep -Eq '(>|rm |mv |cp |sed -i|tee |git +(add|checkout|apply))' <<<"$c" && block "src/deps/ is read-only (Art. II)"
    grep -Eq 'scene_house\.ttt' <<<"$c" && grep -Eq '(>|rm |mv |tee |chmod|sed -i|git +add|truncate)' <<<"$c" && block "scene_house.ttt is read-only and never committed"
    ;;
esac
exit 0
