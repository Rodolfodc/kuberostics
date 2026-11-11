"""Contract tests against the live ROS graph (thin rclpy glue over vacuum_contract_tests.contract).

Not run by default: without a graph they are SKIPPED, with a reason. To run them:

    ros2 launch vacuum_bringup skeleton.launch.py with_teleop:=false &     # T013
    VACUUM_CONTRACT_LIVE=1 python3 -m pytest src/vacuum_contract_tests/test/test_live_graph.py

With VACUUM_CONTRACT_LIVE=1 and no graph the tests FAIL (T005: "must fail while the launch does not exist").
Knobs: VACUUM_CONTRACT_WITH_SIM=1 also expects the CoppeliaSim sensor topics (/scan, /camera, ...);
VACUUM_CONTRACT_SETTLE_S (default 4) is the DDS discovery wait; VACUUM_TOPICS_YAML overrides the contract.
Needs the IA368II nodes (launch default with_ia368ii:=true); topics of absent owners are reported as missing.
"""
import os
import time

import pytest

from vacuum_contract_tests.contract import (
    build_expectations, check_named_type, check_topic, Endpoint, find_contract, load_contract, QosSpec,
    SIM_OWNERS)

# A module-level pytest.skip would abort collection of sibling modules under the launch_testing pytest plugin
# shipped with ROS (seen on Lyrical), so the skip is a per-test marker.
pytestmark = [
    pytest.mark.live,
    pytest.mark.skipif(os.environ.get('VACUUM_CONTRACT_LIVE') != '1',
                       reason='live graph tests: set VACUUM_CONTRACT_LIVE=1 with skeleton.launch.py running'),
]


def _qos(info):
    q = info.qos_profile
    return QosSpec(q.reliability.name.lower(), q.durability.name.lower(), int(q.depth))


def _endpoints(infos):
    return [Endpoint(i.node_name, i.topic_type, _qos(i)) for i in infos]


@pytest.fixture(scope='module')
def node():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    n = rclpy.create_node('contract_test_probe')
    time.sleep(float(os.environ.get('VACUUM_CONTRACT_SETTLE_S', '4')))   # DDS discovery
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _expectations(kind):
    if os.environ.get('VACUUM_CONTRACT_LIVE') != '1':
        return []          # skipped anyway: do not look for the contract file at collection time
    exclude = () if os.environ.get('VACUUM_CONTRACT_WITH_SIM') == '1' else SIM_OWNERS
    contract = load_contract(find_contract())
    return [e for e in build_expectations(contract, exclude_owners=exclude) if e.kind == kind]


def _ids(exps):
    return [e.name for e in exps]


@pytest.mark.parametrize('exp', _expectations('topic'), ids=_ids(_expectations('topic')))
def test_001_R04_topic_exists_with_declared_type_and_qos(node, exp):
    """001-R04 (+ 001-R03 max_publishers/owner of /myRobot/cmd_vel, 001-R12 sink and docking remaps)."""
    problems = check_topic(exp, _endpoints(node.get_publishers_info_by_topic(exp.name)),
                           _endpoints(node.get_subscriptions_info_by_topic(exp.name)))
    assert not problems, '; '.join(problems)


def test_001_R03_cmd_vel_has_exactly_one_publisher_twist_mux(node):
    """001-R03, with literals on purpose: independent of what topics.yaml says."""
    pubs = node.get_publishers_info_by_topic('/myRobot/cmd_vel')
    assert [p.node_name for p in pubs] == ['twist_mux']
    assert pubs[0].topic_type == 'geometry_msgs/msg/Twist'


@pytest.mark.parametrize('exp', _expectations('service'), ids=_ids(_expectations('service')))
def test_001_R04_service_exists_with_declared_type(node, exp):
    problems = check_named_type(exp, node.get_service_names_and_types())
    assert not problems, '; '.join(problems)


@pytest.mark.parametrize('exp', _expectations('action'), ids=_ids(_expectations('action')))
def test_001_R04_action_exists_with_declared_type(node, exp):
    from rclpy.action import get_action_names_and_types
    problems = check_named_type(exp, get_action_names_and_types(node))
    assert not problems, '; '.join(problems)
