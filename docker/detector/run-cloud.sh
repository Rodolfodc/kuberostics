#!/usr/bin/env bash
# Starts the cloud detector node and takes it through its lifecycle (configure, activate), because the node is a LifecycleNode
# without autostart and nothing else manages it in the Pod. Exits when the node exits, so Kubernetes restarts the Pod.
# Usage: run-cloud.sh <params.yaml>   (node name detector_cloud is fixed by the contract, 009a-R17)
set -eo pipefail
source /opt/ros/"$ROS_DISTRO"/setup.bash
source /opt/vacuum/setup.bash
export PATH=/opt/venv/bin:$PATH
# The console scripts use /usr/bin/python3; the venv (ultralytics, torch) is added through PYTHONPATH, not only through PATH.
for d in /opt/venv/lib/python3*/site-packages; do export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$d"; done
NODE=/detector_cloud
ros2 run vacuum_detection detector --ros-args -r __node:=detector_cloud --params-file "${1:?params file}" &
PID=$!
trap 'kill "$PID" 2>/dev/null || true' TERM INT
for _ in $(seq 1 60); do
  ros2 lifecycle get "$NODE" --no-daemon >/dev/null 2>&1 && break
  kill -0 "$PID" 2>/dev/null || { echo "detector exited before it appeared in the graph" >&2; exit 1; }
  sleep 1
done
# configure loads the model (yolo11x takes seconds on CPU); a failure here ends the Pod so it restarts and the cause is in the log
ros2 lifecycle set "$NODE" configure --no-daemon || { echo "configure failed" >&2; kill "$PID"; exit 1; }
ros2 lifecycle set "$NODE" activate --no-daemon || { echo "activate failed" >&2; kill "$PID"; exit 1; }
echo "detector_cloud active"
wait "$PID"
