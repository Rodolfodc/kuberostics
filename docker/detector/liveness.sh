#!/usr/bin/env bash
# Process-level liveness: the detector node is in the ROS graph. A heartbeat from the node (last inference time) would be a better
# probe but needs support in the detector itself. Exit 0 = alive. Used by the Kubernetes livenessProbe (exec).
set -eo pipefail   # no -u: the ROS setup scripts reference unset variables
source /opt/ros/"$ROS_DISTRO"/setup.bash
timeout 8 ros2 node list --no-daemon 2>/dev/null | grep -qx '/detector_cloud'
