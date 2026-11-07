#!/usr/bin/env bash
# Controls a headless CoppeliaSim on a COPY of the House scene. Tracks its own PID:
# `stop` only ever kills the process started by `start` on that port.
# Usage: tools/sim.sh start [port] | stop [port] | status [port]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COPPELIA_DIR="${COPPELIA_DIR:-$HOME/Tools/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04}"
SCENE_SRC="${SCENE_SRC:-$REPO/../scene_house.ttt}"
WORK="$REPO/.sim"
MEM_MAX="${SIM_MEM_MAX:-6G}"

cmd="${1:-status}"; port="${2:-23000}"
pidf="$WORK/sim-$port.pid"; holdf="$WORK/hold-$port.pid"; logf="$WORK/sim-$port.log"; scene="$WORK/scene-$port.ttt"
mkdir -p "$WORK"

alive() { [[ -f "$pidf" ]] && kill -0 "$(cat "$pidf")" 2>/dev/null; }

case "$cmd" in
  start)
    alive && { echo "already running on port $port (pid $(cat "$pidf"))"; exit 0; }
    ss -ltn "sport = :$port" 2>/dev/null | grep -q LISTEN && { echo "port $port is in use by another process; refusing" >&2; exit 1; }
    [[ -x "$COPPELIA_DIR/coppeliaSim.sh" ]] || { echo "coppeliaSim.sh not found in $COPPELIA_DIR" >&2; exit 127; }
    [[ -f "$SCENE_SRC" ]] || { echo "scene not found: $SCENE_SRC" >&2; exit 1; }
    cp -f "$SCENE_SRC" "$scene"; chmod u+w "$scene"          # work on a copy, never on the original
    # -h headless; -G sets a named parameter (VERIFY on 4.10: zmqRemoteApi.rpcPort).
    # SIM_HEADLESS=0 opens the GUI window (needed by models with simUI scripts / vision sensors that crash in -h).
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    hl=(-h); [[ "${SIM_HEADLESS:-1}" == "0" ]] && hl=()
    run=(setsid "$COPPELIA_DIR/coppeliaSim.sh" "${hl[@]}" "-GzmqRemoteApi.rpcPort=$port" "$scene")
    if command -v systemd-run >/dev/null 2>&1; then
      run=(systemd-run --user --scope -q -p "MemoryMax=$MEM_MAX" "${run[@]}")
    fi
    # CoppeliaSim quits on stdin EOF (its sandbox script reads stdin), so give it a stdin that never closes.
    fifo="$WORK/stdin-$port"; rm -f "$fifo"; mkfifo "$fifo"
    ( exec 3<>"$fifo"; exec sleep infinity ) >/dev/null 2>&1 &
    echo $! >"$holdf"
    nohup "${run[@]}" <"$fifo" >"$logf" 2>&1 &
    echo $! >"$pidf"
    sleep 4
    if alive; then echo "started on port $port (pid $(cat "$pidf")), log: $logf"
    else kill "$(cat "$holdf")" 2>/dev/null || true; rm -f "$holdf" "$fifo"; echo "failed to start; see $logf" >&2; exit 1; fi
    ;;
  stop)
    if [[ -f "$holdf" ]]; then kill "$(cat "$holdf")" 2>/dev/null || true; rm -f "$holdf" "$WORK/stdin-$port"; fi
    alive || { echo "nothing to stop on port $port"; rm -f "$pidf"; exit 0; }
    pid="$(cat "$pidf")"
    # coppeliaSim.sh is a wrapper: also stop its children (the real binary), found by parent PID.
    kids="$(pgrep -P "$pid" || true)"
    kill $pid $kids 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 $pid $kids 2>/dev/null || break; sleep 1; done
    kill -9 $pid $kids 2>/dev/null || true
    rm -f "$pidf"; echo "stopped pid $pid"
    ;;
  status)
    if alive; then echo "running on port $port (pid $(cat "$pidf"))"; else echo "not running on port $port"; fi
    ;;
  *) echo "usage: $0 start|stop|status [port]" >&2; exit 2;;
esac
