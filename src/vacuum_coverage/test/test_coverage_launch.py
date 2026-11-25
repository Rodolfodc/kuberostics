"""launch_testing of launch/coverage.launch.py: the real coverage_server process alone, with the fakes of the
test (static map, fake NavigateToPose server, TF) instead of Nav2 and the simulator (005-R28, 005-R26).

The domain ID comes from conftest.py (range 1-5; sim bridge 6-9, stubs 10-39, safety 40-69, supervisor 70-99, bringup 100-129, sequencer graph 130-159, detection 160-189, contract tests 190-219, mapping 220-232, domain 0 free) and discovery is restricted to localhost; the launched
node gets the same values through actions, because launch_testing restores the environment after
generate_test_description. Requirements: 005-R16, R19, R21, R27, R28; node exit code 0 at teardown (Art. III).
"""
import os
import threading
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.executors import MultiThreadedExecutor
from vacuum_interfaces.action import CoverRoom

from coverage_fixtures import rect_room
from node_fixtures import REACH, FakeNav, MapPublisher, wait_for


@pytest.mark.launch_test
def generate_test_description():
    source = os.path.join(get_package_share_directory('vacuum_coverage'), 'launch', 'coverage.launch.py')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ['ROS_DOMAIN_ID']),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCoverageAlone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # an own context, as the graph tests of vacuum_bringup do: the default one is also used by launch_testing
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=int(os.environ['ROS_DOMAIN_ID']))
        cls.nav = FakeNav(lambda i, x, y: REACH, context=cls.context)
        cls.maps = MapPublisher(rect_room(2.0, 2.0), context=cls.context)
        cls.probe = rclpy.create_node('coverage_launch_probe', context=cls.context)
        # clients are created once: a client created per call can lose the first response while the server
        # has not matched its reader yet (Fast DDS), which made the first polls time out
        cls.get_state = cls.probe.create_client(GetState, '/coverage_server/get_state')
        cls.executor = MultiThreadedExecutor(num_threads=6, context=cls.context)
        for n in (cls.nav, cls.maps, cls.probe):
            cls.executor.add_node(n)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
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

    def _active_within(self, total_s):
        # Jazzy (Fast DDS) sometimes needs about a minute before the launched process and the test process see each
        # other's services; the window is long on purpose, the test is quick when discovery is.
        # No manual transitions here: a second `activate` on an already active node makes rclpy raise and the
        # process exit on Jazzy, so the launch autostart chain is the only thing that drives the lifecycle.
        return wait_for(lambda: self._state() == State.PRIMARY_STATE_ACTIVE, total_s)

    def test_R28_R21_real_node_reaches_active_and_covers_the_fake_room(self):
        self.assertTrue(self._active_within(180.0),
                        'coverage_server did not become active (autostart)')
        client = ActionClient(self.probe, CoverRoom, '/myRobot/cover_room')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        accepted = []

        def try_goal():             # the node needs the latched map and the first TF before accepting
            future = client.send_goal_async(CoverRoom.Goal(room_id='', passes=1))
            if wait_for(future.done, 10.0) and future.result().accepted:
                accepted.append(future.result())
            return bool(accepted)

        wait_for(try_goal, 30.0)
        self.assertTrue(accepted, 'goal never accepted')
        result_future = accepted[0].get_result_async()
        self.assertTrue(wait_for(result_future.done, 60.0))
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreaterEqual(result.result.covered_ratio, 0.95)
        self.assertTrue(self.nav.goals)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_R28_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
