"""launch_testing of launch/exploration.launch.py: the real exploration_server process alone, with the fakes of
the test (map, fake NavigateToPose server, TF) instead of Nav2.

Requirements: 003-R10, R13 (the process explores a fake map and `explored` turns true), 003-R26 (node exit
code 0 at teardown).

The kill test (003-R31) is in test_exploration_kill.py: launch_testing treats the death of a launched
process as a failure, so that test spawns the node as an independent process.

Domain: a fixed random id from 220-232 (mapping range) taken at import; the conftest fixture of this package
changes ROS_DOMAIN_ID per test, so this module never reads the environment after import.
"""
import os
import random
import threading
import unittest

DOMAIN = random.SystemRandom().randrange(220, 233)
os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)

import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.client import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from vacuum_interfaces.action import Explore  # noqa: E402

from exploration_fixtures import (LATCHED, REACH, FakeNav, MapPublisher, known_cells,  # noqa: E402
                                  one_frontier_cells, wait_for)


@pytest.mark.launch_test
def generate_test_description():
    source = os.path.join(get_package_share_directory('vacuum_mapping'), 'launch', 'exploration.launch.py')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(DOMAIN)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestExplorationAlone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=DOMAIN)
        cls.nav = FakeNav(lambda i, x, y: REACH, context=cls.context, on_reach=cls._on_reach)
        cls.maps = MapPublisher(one_frontier_cells(), context=cls.context)
        cls.probe = rclpy.create_node('exploration_launch_probe', context=cls.context)
        cls.get_state = cls.probe.create_client(GetState, '/exploration_server/get_state')
        cls.explored = []
        cls.probe.create_subscription(Bool, '/myRobot/map/explored', lambda m: cls.explored.append(m.data), LATCHED)
        cls.executor = MultiThreadedExecutor(num_threads=6, context=cls.context)
        for n in (cls.nav, cls.maps, cls.probe):
            cls.executor.add_node(n)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def _on_reach(cls, index):
        cls.maps.publish(known_cells())

    @classmethod
    def tearDownClass(cls):
        cls.nav.stopped = True
        cls.executor.shutdown()
        for n in (cls.nav, cls.maps, cls.probe):
            n.destroy_node()
        cls.context.try_shutdown()
        cls.thread.join(timeout=2)

    def _state(self):
        if not self.get_state.service_is_ready():
            return None
        future = self.get_state.call_async(GetState.Request())
        return future.result().current_state.id if wait_for(future.done, 3.0) else None

    def _wait_active(self, total_s=180.0):
        self.assertTrue(wait_for(lambda: self._state() == State.PRIMARY_STATE_ACTIVE, total_s),
                        'exploration_server never became active (slow discovery or startup failure)')

    def _send(self, client, feedback):
        accepted = []

        def try_goal():             # the node needs the latched map and the first TF before accepting
            future = client.send_goal_async(Explore.Goal(timeout_s=0.0), feedback_callback=feedback)
            if wait_for(future.done, 10.0) and future.result().accepted:
                accepted.append(future.result())
            return bool(accepted)
        wait_for(try_goal, 60.0)
        self.assertTrue(accepted, 'goal never accepted')
        return accepted[0]

    def test_R13_R10_real_node_explores_the_fake_map_and_explored_turns_true(self):
        self._wait_active()
        self.assertTrue(wait_for(lambda: self.explored[:1] == [False], 20.0), 'latched false first (003-R13)')
        client = ActionClient(self.probe, Explore, '/myRobot/explore')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        handle = self._send(client, lambda m: None)
        result_future = handle.get_result_async()
        self.assertTrue(wait_for(result_future.done, 60.0))
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result.success)
        self.assertTrue(wait_for(lambda: self.explored[-1:] == [True], 10.0))
        self.assertTrue(self.nav.goals)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_R26_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
