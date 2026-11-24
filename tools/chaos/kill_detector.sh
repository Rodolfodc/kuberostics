#!/usr/bin/env bash
# (009a-R15, R16) Fault injection: delete the detector Pod N times while the robot cleans, record OffloadStatus (and the
# detections, notifications and orchestrator mode) to a rosbag, then compute time to OPEN, time to CLOSED and the drop counters.
# Usage: tools/chaos/kill_detector.sh [--kills 5] [--distro jazzy] [--settle 40] [--out DIR] [--yes]
# Preconditions: the robot stack runs with offload:=true and its graph is reachable from this host (same RMW and discovery as the
# robot: ROBOT_RMW, default rmw_cyclonedds_cpp, localhost discovery); the cluster is up and the detector Deployment is Ready;
# the image vacuum-detector:<distro> is built (docker/detector). Cleaning must be started by the operator beforehand.
# The Pod deletion goes through tools/k3s.sh and needs the user's confirmation (the `delete` verb is permission-gated); this
# script also asks unless --yes. Output (bag, kills.csv, report.md/json) goes to tools/chaos/local-logs/<timestamp>/, gitignored.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
K="$REPO/tools/k3s.sh"
KILLS=5 DISTRO=jazzy SETTLE=40 YES=0 NS=vacuum-cloud
OUT="$REPO/tools/chaos/local-logs/$(date +%Y%m%d-%H%M%S)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --kills) KILLS="$2"; shift 2;;
    --distro) DISTRO="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;   # seconds to wait after the Pod is Ready, so the breaker can close (stable_window_s 3 s plus margin)
    --out) OUT="$2"; shift 2;;
    --yes) YES=1; shift;;
    *) echo "unknown arg $1" >&2; exit 2;;
  esac
done
IMG="vacuum-detector:$DISTRO"
RMW="${ROBOT_RMW:-rmw_cyclonedds_cpp}"
REC="chaos-bag-$$"
mkdir -p "$OUT"
now() { date +%s.%N; }

docker image inspect "$IMG" >/dev/null 2>&1 || { echo "image $IMG missing: docker build -f docker/detector/Dockerfile --build-arg ROS_DISTRO=$DISTRO ." >&2; exit 1; }
"$K" -n "$NS" get deploy detector >/dev/null

echo "Will delete the Pod of deploy/detector in namespace $NS $KILLS times (settle ${SETTLE}s). Output: $OUT"
if [[ $YES -eq 0 ]]; then read -r -p "Proceed? [y/N] " a; [[ "$a" == y ]] || exit 1; fi

cleanup() { docker rm -f "$REC" >/dev/null 2>&1 || true; }
trap cleanup EXIT

# The recorder runs as the host user so that the bag is owned by it. Host network: same graph as the robot.
docker run -d --name "$REC" --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e RMW_IMPLEMENTATION="$RMW" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v "$OUT":/out \
  --entrypoint /opt/detector/entrypoint.sh "$IMG" \
  ros2 bag record -o /out/bag /myRobot/perception/offload_status /myRobot/perception/detections \
    /myRobot/perception/detections/cloud /myRobot/perception/detections/local /myRobot/notifications /myRobot/behaviour/mode >/dev/null
sleep 5
echo "index,kill_epoch,ready_epoch" >"$OUT/kills.csv"

for i in $(seq 1 "$KILLS"); do
  echo ">> kill $i/$KILLS"
  t_kill="$(now)"
  "$K" -n "$NS" delete pod -l app=detector --wait=false
  "$K" -n "$NS" wait --for=delete pod -l app=detector --timeout=120s >/dev/null 2>&1 || true
  "$K" -n "$NS" rollout status deploy/detector --timeout=300s
  t_ready="$(now)"
  echo "$i,$t_kill,$t_ready" >>"$OUT/kills.csv"
  sleep "$SETTLE"
done

docker stop -t 10 "$REC" >/dev/null
docker logs "$REC" >"$OUT/recorder.log" 2>&1 || true
echo ">> analysis"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$OUT":/out -v "$REPO/tools/chaos":/chaos:ro \
  --entrypoint /opt/detector/entrypoint.sh "$IMG" python3 -I /chaos/analyze_offload_bag.py /out/bag /out/kills.csv
