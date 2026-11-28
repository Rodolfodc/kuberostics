#!/usr/bin/env bash
# Robot side of the LOCAL offload run (009a): a synthetic camera, the real image_throttle, a local detector (hot standby), the
# detection_selector and the projector, plus the zenoh-bridge-ros2dds sidecar that connects to the cloud detector Pod in the local
# k3d cluster. Everything runs in Docker (no simulator, no host ROS) in ONE container network namespace, so it cannot join any other
# ROS graph on the machine. Never touches a remote cluster.
# Usage: tools/chaos/local_robot.sh start|stop|status|logs [--distro jazzy] [--local-model yolo11n.pt] [--with-projector]
# Needs: image vacuum-detector:<distro> (docker/detector), the k3d cluster kuberostics up, and in
# tools/chaos/local-logs/weights/ (gitignored): <local-model> and sample.jpg (scripts/offload-local-run.sh fetches them).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CMD="${1:-}"; shift || true
DISTRO=jazzy LOCAL_MODEL=yolo11n.pt
PROJECTOR=0   # --with-projector: off by default, the projector node hangs on its activate transition (see the PR notes)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --distro) DISTRO="$2"; shift 2;;
    --local-model) LOCAL_MODEL="$2"; shift 2;;
    --with-projector) PROJECTOR=1; shift;;
    *) echo "unknown arg $1" >&2; exit 2;;
  esac
done
IMG="vacuum-detector:$DISTRO" NAME=chaos-robot BR=chaos-robot-bridge
DOMAIN="${CHAOS_DOMAIN_ID:-50}"
W="$REPO/tools/chaos/local-logs/weights"
BRIDGE_IMG="eclipse/zenoh-bridge-ros2dds:1.10.2@sha256:0183b7e83790a8c57a069743776c2d913041760eb19e64d8cc4fbf7374269034"

case "$CMD" in
  start)
    [[ -s "$W/$LOCAL_MODEL" && -s "$W/sample.jpg" ]] || { echo "missing $W/$LOCAL_MODEL or $W/sample.jpg" >&2; exit 1; }
    NODE_IP="$(docker inspect -f '{{(index .NetworkSettings.Networks "k3d-kuberostics").IPAddress}}' k3d-kuberostics-server-0)"
    docker rm -f "$NAME" "$BR" >/dev/null 2>&1 || true
    # Resource caps: the host is shared. Unprivileged, read-only root filesystem, /tmp is a tmpfs.
    docker run -d --name "$NAME" --network k3d-kuberostics --cpus 2 --memory 4g --read-only --tmpfs /tmp:exec \
      --cap-drop ALL --security-opt no-new-privileges:true -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros-log -e YOLO_CONFIG_DIR=/tmp/ultralytics \
      -e ROS_DOMAIN_ID="$DOMAIN" -v "$W":/w:ro -v "$REPO/tools/chaos":/chaos:ro \
      --entrypoint /opt/detector/entrypoint.sh "$IMG" bash -c '
        python3 /chaos/synthetic_camera.py /w/sample.jpg --rate 2 >/tmp/camera.log 2>&1 &
        ros2 launch vacuum_detection detection.launch.py offload:=true with_manager:=false local_weights:=/w/'"$LOCAL_MODEL"' \
          >/tmp/detection.log 2>&1 &
        if [ '"$PROJECTOR"' = 1 ]; then
          ros2 run vacuum_detection projector --ros-args -p check_mapped_area:=false >/tmp/projector.log 2>&1 &
        fi
        wait' >/dev/null
    # Nobody manages these nodes here (with_manager:=false), so configure, then activate them. Retried until each node exists.
    docker exec "$NAME" /opt/detector/entrypoint.sh bash -c '
      lc() { for _ in $(seq 1 90); do ros2 lifecycle set "$1" "$2" --no-daemon >/dev/null 2>&1 && return 0; sleep 1; done
             echo "lifecycle $2 failed for $1" >&2; return 1; }
      nodes="detector_local detection_selector image_throttle"; [ '"$PROJECTOR"' = 1 ] && nodes="$nodes projector"
      for n in $nodes; do lc /$n configure & done; wait
      for n in $nodes; do lc /$n activate; done'
    # The bridge starts only after every node exists: started earlier, it did not discover them and created no routes.
    docker run -d --name "$BR" --network "container:$NAME" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DISTRO="$DISTRO" \
      -e ROS_DOMAIN_ID="$DOMAIN" -e CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery><SPDPInterval>1s</SPDPInterval></Discovery></Domain></CycloneDDS>" -v "$REPO/deploy/bridge/robot.json5":/cfg/bridge.json5:ro --read-only --cap-drop ALL \
      --security-opt no-new-privileges:true "$BRIDGE_IMG" -e "tcp/$NODE_IP:30448" --no-multicast-scouting -c /cfg/bridge.json5 >/dev/null
    echo "robot side up: container $NAME (domain $DOMAIN), bridge $BR -> tcp/$NODE_IP:30448";;
  stop)   docker rm -f "$BR" "$NAME" >/dev/null 2>&1 || true; echo "robot side stopped";;
  status) docker ps --filter name='^chaos-robot' --format '{{.Names}} {{.Status}}'
          docker exec "$NAME" /opt/detector/entrypoint.sh bash -c 'ros2 topic echo --once --no-daemon /myRobot/perception/offload_status' 2>&1 | head -20;;
  logs)   for f in camera detection projector; do echo "== $f"; docker exec "$NAME" tail -n 25 "/tmp/$f.log" 2>&1 || true; done
          echo "== bridge"; docker logs --tail 25 "$BR" 2>&1;;
  *) echo "usage: $0 start|stop|status|logs" >&2; exit 2;;
esac
