#!/usr/bin/env bash
# Sources ROS, our overlay and the venv, then runs the command.
set -eo pipefail
source /opt/ros/"$ROS_DISTRO"/setup.bash
source /opt/vacuum/setup.bash
export PATH=/opt/venv/bin:$PATH
# The console scripts use /usr/bin/python3; the venv (ultralytics, torch) is added through PYTHONPATH, not only through PATH.
for d in /opt/venv/lib/python3*/site-packages; do export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$d"; done
exec "$@"
