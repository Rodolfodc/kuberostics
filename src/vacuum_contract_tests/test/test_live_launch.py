"""T022, spec 001: the live contract checks against `skeleton.launch.py`, with no environment variable.

The test launches the real skeleton with `with_ia368ii:=false with_wheel_bridge:=false with_teleop:=false`, waits
(conditions only) until the nodes are discoverable, then runs the same checks as test_live_graph.py through
`contract.py`. Interfaces whose owner is not part of this graph are excluded and named below; they need the
IA368II checkout or CoppeliaSim and stay open (T023): 001-R12 (`legacy/cleaning_cmd_vel` sink, `mux/docking`
from `autodocking_server`), `behaviour_server`/`myrobot` topics and services, and the sensor topics.

It fails while the launch does not exist, while a node of the skeleton is missing, and while
/myRobot/cmd_vel has a publisher other than `twist_mux`.
"""
import os
import time
import unittest

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.action import get_action_names_and_types
from std_srvs.srv import Trigger

from vacuum_contract_tests.contract import (
    build_expectations, check_named_type, check_topic, find_contract, load_contract, SIM_OWNERS)
from vacuum_contract_tests.graph import DISCOVERY_RANGE, endpoints, isolate_environment

DOMAIN_ID = isolate_environment()
CMD_VEL = '/myRobot/cmd_vel'
SKELETON_NODES = ['twist_mux', 'supervisor', 'stop_guard', 'coverage_server', 'escape_server', 'stuck_detector',
                  'lifecycle_manager_core']
# Owners that are not in this graph (with_ia368ii:=false, no simulator): their interfaces are covered by T023.
ABSENT_OWNERS = SIM_OWNERS | {'behaviour_server', 'myrobot', 'cleaning_server', 'autodocking_server'}
STARTUP_TIMEOUT_S = 60.0
CHECK_TIMEOUT_S = 20.0       # discovery of one more endpoint after the nodes are up: condition, not a sleep


@pytest.mark.launch_test
def generate_test_description():
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'launch', 'skeleton.launch.py')
    return LaunchDescription([
        # actions, not os.environ: launch_testing restores the environment after generate_test_description
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(DOMAIN_ID)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', DISCOVERY_RANGE),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source), launch_arguments={
            'with_ia368ii': 'false', 'with_wheel_bridge': 'false', 'with_teleop': 'false'}.items()),
        ReadyToTest(),
    ])


def wait_for(predicate, timeout_s, period_s=0.1):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


class TestSkeletonContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=DOMAIN_ID)
        cls.node = rclpy.create_node('contract_launch_probe', context=cls.context)
        # staged start: the graph is checked only after the nodes are discoverable and the manager reports active
        cls.nodes_up = wait_for(
            lambda: set(SKELETON_NODES) <= {n for n, ns in cls.node.get_node_names_and_namespaces() if ns == '/'},
            STARTUP_TIMEOUT_S)
        cls.active = cls.nodes_up and wait_for(cls._manager_active, STARTUP_TIMEOUT_S, 0.5)
        contract = load_contract(find_contract())
        cls.expectations = build_expectations(contract, exclude_owners=ABSENT_OWNERS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.context.try_shutdown()

    @classmethod
    def _manager_active(cls):
        client = cls.node.create_client(Trigger, '/lifecycle_manager_core/is_active')
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(cls.node, future, timeout_sec=5.0)
            return future.done() and bool(future.result().success)
        finally:
            cls.node.destroy_client(client)

    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=DOMAIN_ID)
        cls.node = rclpy.create_node('contract_launch_probe', context=cls.context)
        # staged start: the graph is checked only after the nodes are discoverable and the manager reports active
        cls.nodes_up = wait_for(
            lambda: set(SKELETON_NODES) <= {n for n, ns in cls.node.get_node_names_and_namespaces() if ns == '/'},
            STARTUP_TIMEOUT_S)
        cls.active = cls.nodes_up and wait_for(cls._manager_active, STARTUP_TIMEOUT_S, 0.5)
        contract = load_contract(find_contract())
        cls.expectations = build_expectations(contract, exclude_owners=ABSENT_OWNERS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.context.try_shutdown()

    @classmethod
    def _manager_active(cls):
        client = cls.node.create_client(Trigger, '/lifecycle_manager_core/is_active')
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False
            future = client.call_async(Trigger.Request())
            if not wait_for(future.done, 5.0, 0.02) and not cls._spin_until(future):
                return False
            return bool(future.result().success)
        finally:
            cls.node.destroy_client(client)

    @classmethod
    def _spin_until(cls, future):
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=5.0)
        return future.done()

    def setUp(self):
        self.assertTrue(self.nodes_up, f'nodes never discovered: {SKELETON_NODES}')
        self.assertTrue(self.active, 'lifecycle_manager_core never reported is_active == true')

    def _check_all(self, kind, check):
        found = [e for e in self.expectations if e.kind == kind]
        self.assertTrue(found, f'no {kind} in scope: contract or exclusion list is wrong')
        failures = []
        for exp in found:
            end = time.monotonic() + CHECK_TIMEOUT_S
            problems = check(exp)
            while problems and time.monotonic() < end:     # condition wait: discovery of the endpoint
                time.sleep(0.1)
                problems = check(exp)
            if problems:
                failures.append('; '.join(problems))
        self.assertFalse(failures, '\n'.join(failures))

    def test_001_R04_topics_exist_with_declared_type_and_qos(self):
        """001-R04 (+ 001-R03 max_publishers/owner for /myRobot/cmd_vel), for the topics of this graph."""
        self._check_all('topic', lambda exp: check_topic(
            exp, endpoints(self.node.get_publishers_info_by_topic(exp.name), exp),
            endpoints(self.node.get_subscriptions_info_by_topic(exp.name), exp)))

    def test_001_R04_services_exist_with_declared_type(self):
        self._check_all('service', lambda exp: check_named_type(exp, self.node.get_service_names_and_types()))

    def test_001_R04_actions_exist_with_declared_type(self):
        self._check_all('action', lambda exp: check_named_type(exp, get_action_names_and_types(self.node)))

    def test_001_R03_cmd_vel_has_exactly_one_publisher_twist_mux(self):
        """001-R03, with literals on purpose: independent of what topics.yaml says."""
        self.assertTrue(wait_for(lambda: self.node.get_publishers_info_by_topic(CMD_VEL), CHECK_TIMEOUT_S),
                        f'{CMD_VEL} has no publisher')
        pubs = self.node.get_publishers_info_by_topic(CMD_VEL)
        self.assertEqual([p.node_name for p in pubs], ['twist_mux'])
        self.assertEqual(pubs[0].topic_type, 'geometry_msgs/msg/Twist')

    def test_001_R04_mode_topic_is_consumed_by_the_supervisor(self):
        """001-R04: behaviour_server is not here, but the topic it owns must have the supervisor as subscriber."""
        mode = '/myRobot/behaviour/mode'
        self.assertTrue(wait_for(
            lambda: 'supervisor' in [s.node_name for s in self.node.get_subscriptions_info_by_topic(mode)],
            CHECK_TIMEOUT_S), f'supervisor does not subscribe to {mode}')
        for s in self.node.get_subscriptions_info_by_topic(mode):
            self.assertEqual(s.topic_type, 'std_msgs/msg/String')
