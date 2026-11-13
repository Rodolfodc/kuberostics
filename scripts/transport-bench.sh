#!/usr/bin/env bash
# T902 (009a-R03, Q-06): robot <-> K3s transport measurement, rmw_zenoh vs zenoh-bridge-ros2dds.
# The robot side is a Docker container on the k3d network; the cloud side is a Pod in namespace t902 of the local
# cluster "kuberostics" (tools/k3s.sh, own kubeconfig). Round trip = timestamp echo of a ~60 KB JPEG at 2 Hz.
# Usage: scripts/transport-bench.sh [--duration S] [--distro lyrical] [--scenarios "lan wan"] [--transports "rmw bridge"]
# Needs: docker, k3d, kubectl. `tools/k3s.sh up` is run if the cluster is absent. Results: deploy/measurements/t902/.
# Cleanup: scripts/transport-bench.sh --cleanup   (removes only namespace t902 and the t902-* containers)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K="$REPO/tools/k3s.sh"
OUT="$REPO/deploy/measurements/t902"
DISTRO=lyrical DURATION=300 SCENARIOS="lan wan" TRANSPORTS="rmw bridge"
BRIDGE_IMG="eclipse/zenoh-bridge-ros2dds:1.10.2"
declare -A NETEM=( [lan]="" [wan]="delay 20ms 5ms loss 1%" )   # robot -> cloud direction only (egress of the robot netns)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2;;
    --distro) DISTRO="$2"; shift 2;;
    --scenarios) SCENARIOS="$2"; shift 2;;
    --transports) TRANSPORTS="$2"; shift 2;;
    --cleanup) "$K" delete namespace t902 --ignore-not-found
               docker ps -a --filter name='^t902-' -q | xargs -r docker rm -f; exit 0;;
    *) echo "unknown arg $1" >&2; exit 2;;
  esac
done
IMG="t902-bench:$DISTRO"
mkdir -p "$OUT"

echo ">> image $IMG"
docker build -q --build-arg ROS_DISTRO="$DISTRO" -t "$IMG" "$REPO/deploy/transport-bench" >/dev/null
docker pull -q "$BRIDGE_IMG" >/dev/null

echo ">> cluster"
k3d cluster list kuberostics >/dev/null 2>&1 || "$K" up
[[ -f "$REPO/.k3s/kubeconfig" ]] || "$K" up
# Only the bench image is imported: importing the multi-arch bridge image fails in ctr; the Pod pulls it instead.
k3d image import -c kuberostics "$IMG" >/dev/null
sed "s#t902-bench:lyrical#$IMG#" "$REPO/deploy/transport-bench/k8s/transport-bench.yaml" | "$K" apply -f - >/dev/null
NODE_IP="$(docker inspect -f '{{(index .NetworkSettings.Networks "k3d-kuberostics").IPAddress}}' k3d-kuberostics-server-0)"
echo "   node IP $NODE_IP"

cloud_up()   { "$K" -n t902 scale deploy "$1" --replicas=1 >/dev/null; "$K" -n t902 rollout status deploy "$1" --timeout=180s >/dev/null; }
cloud_down() { "$K" -n t902 scale deploy "$1" --replicas=0 >/dev/null; "$K" -n t902 wait --for=delete pod -l app="$1" --timeout=120s >/dev/null 2>&1 || true; }

run_one() {   # $1 transport (rmw|bridge) $2 scenario
  local t="$1" s="$2" name="t902-robot-$1-$2" dep tmode port
  local label="$DISTRO-$t-$s"
  if [[ $t == rmw ]]; then dep=cloud-rmw; tmode=rmw; port=30447; else dep=cloud-bridge; tmode=dds; port=30448; fi
  echo ">> $label (netem: '${NETEM[$s]:-none}')"
  cloud_up "$dep"
  docker rm -f "$name" "$name-bridge" >/dev/null 2>&1 || true
  docker create --name "$name" --network k3d-kuberostics --cap-add NET_ADMIN \
    -e NETEM="${NETEM[$s]}" -e ZENOH_UPSTREAM="tcp/$NODE_IP:$port" "$IMG" \
    "$tmode" pub --duration "$DURATION" --csv /tmp/rtt.csv --label "$label" >/dev/null
  docker start "$name" >/dev/null
  if [[ $t == bridge ]]; then
    docker run -d --name "$name-bridge" --network "container:$name" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DISTRO="$DISTRO" \
      -v "$REPO/deploy/transport-bench/bridge-robot.json5":/cfg/bridge.json5:ro "$BRIDGE_IMG" \
      -e "tcp/$NODE_IP:$port" --no-multicast-scouting -c /cfg/bridge.json5 >/dev/null
  fi
  docker wait "$name" >/dev/null
  docker cp "$name":/tmp/rtt.csv "$OUT/$label.csv" 2>/dev/null || echo "   (no csv produced)"
  docker logs "$name" >"$OUT/$label.log" 2>&1 || true
  [[ $t == bridge ]] && { docker logs "$name-bridge" >"$OUT/$label.bridge.log" 2>&1 || true; }
  grep '^SUMMARY' "$OUT/$label.log" | tee -a "$OUT/summary.txt" || echo "   no SUMMARY line, see $OUT/$label.log"
  docker rm -f "$name" "$name-bridge" >/dev/null 2>&1 || true
  cloud_down "$dep"
}

for s in $SCENARIOS; do for t in $TRANSPORTS; do run_one "$t" "$s"; done; done
echo ">> done; summary in $OUT/summary.txt"
