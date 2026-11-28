"""rclpy glue shared by the live contract tests: graph endpoints as `contract.Endpoint`, and the isolation of the
test graph from every other ROS graph (Art. X). The decisions stay in `contract.py`; this file only converts."""
import os
import random

from vacuum_contract_tests.contract import Endpoint, QosSpec

# Domain ID range reserved for this package (plan 001 "Domain ID allocation" lists the per-package ranges).
DOMAIN_RANGE = (190, 220)
DISCOVERY_RANGE = 'LOCALHOST'


def isolate_environment() -> int:
    """Restricts discovery to localhost and picks a random ROS_DOMAIN_ID inside DOMAIN_RANGE, once per process
    (an ID already inside the range is kept, so conftest and test module agree). Returns the domain ID."""
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = DISCOVERY_RANGE
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    current = os.environ.get('ROS_DOMAIN_ID', '')
    if not (current.isdigit() and DOMAIN_RANGE[0] <= int(current) < DOMAIN_RANGE[1]):
        os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(*DOMAIN_RANGE))
    return int(os.environ['ROS_DOMAIN_ID'])


def qos_of(info, expected):
    """Graph QoS of one endpoint. Jazzy's rmw reports depth 0 for endpoints it does not own (the queue depth is
    local, not part of DDS compatibility); depth 0 therefore means "not reported" and takes the contract value,
    while reliability and durability (what decides compatibility) are always compared. Lyrical reports the depth."""
    q = info.qos_profile
    depth = int(q.depth)
    if depth == 0 and expected is not None:
        depth = expected.depth
    return QosSpec(q.reliability.name.lower(), q.durability.name.lower(), depth)


def endpoints(infos, exp):
    return [Endpoint(i.node_name, i.topic_type, qos_of(i, exp.qos)) for i in infos]
