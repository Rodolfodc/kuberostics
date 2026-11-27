#!/usr/bin/env bash
# Sources ROS, our overlay and the venv, then runs the command.
set -eo pipefail
source /opt/ros/"$ROS_DISTRO"/setup.bash
source /opt/vacuum/setup.bash
export PATH=/opt/venv/bin:$PATH
exec "$@"
