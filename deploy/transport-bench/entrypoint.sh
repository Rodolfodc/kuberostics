#!/usr/bin/env bash
# Usage: entrypoint.sh <rmw|dds> <echo|pub> [bench.py args...]
#   rmw: RMW_IMPLEMENTATION=rmw_zenoh_cpp with a local rmw_zenohd router. If ZENOH_UPSTREAM is set
#        (tcp/host:port) the router connects to it (robot side); otherwise it only listens (cloud side).
#   dds: CycloneDDS, localhost discovery only; the zenoh-bridge-ros2dds runs in another container
#        sharing this network namespace.
set -eo pipefail
source /opt/ros/"$ROS_DISTRO"/setup.bash
transport="$1"; role="$2"; shift 2
export ROS_DOMAIN_ID=0
# Optional link emulation on the egress of this network namespace (needs --cap-add NET_ADMIN), e.g. NETEM="delay 20ms 5ms loss 1%"
if [[ -n "${NETEM:-}" ]]; then tc qdisc add dev eth0 root netem $NETEM; fi
if [[ "$transport" == rmw ]]; then
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  if [[ -n "${ZENOH_UPSTREAM:-}" ]]; then
    export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"$ZENOH_UPSTREAM\"]"
  fi
  ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenohd.log 2>&1 &
  unset ZENOH_CONFIG_OVERRIDE   # only the router connects upstream; the node connects to the local router
  sleep 3
else
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
fi
exec python3 /opt/bench/bench.py "$role" "$@"
