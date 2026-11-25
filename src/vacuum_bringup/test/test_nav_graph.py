"""Live-graph test of Nav2 (spec 005, T510) without CoppeliaSim: real nav.launch.py, real nav2_map_server, a fake
robot (fake_robot.py: odometry, TF, empty /scan, integrates /myRobot/mux/nav).

* 005-R19: a NavigateToPose goal on the saved map is accepted and the robot reaches it, driven only by
  controller_server on /myRobot/mux/nav.
* 005-R34 (Nav2 part): cancelling the goal stops controller_server publishing on /myRobot/mux/nav.
* 005-R23: Twist on mux/nav is unstamped geometry_msgs/Twist (the subscription would not receive it otherwise).
* 005-R29: the whole thing runs on Jazzy and Lyrical.

NOT covered here (needs CoppeliaSim, left to the sim-operator, see tools/sim_bench/nav_spike.md): the real wheel
bridge, the real laser, and the House map.
"""
import math
import os
import sys
import tempfile
import threading
import time
import unittest

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

import math
import os
import sys
import time
import unittest

import pytest
from action_msgs.msg import GoalStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_support as ns  # noqa: E402

MODULE = 20          # domain from the vacuum_bringup range 100-129 (graph_support); modules run one at a time


@pytest.mark.launch_test
def generate_test_description():
    return ns.launch_nav(MODULE)


class TestNav(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = ns.NavProbe(MODULE)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_goal_is_accepted_and_reached_driving_only_mux_nav(self):
        """005-R19, R23: accepted, SUCCEEDED, with non-zero velocity seen on /myRobot/mux/nav."""
        handle = self.probe.send(4.5, 4.5)
        self.assertTrue(handle.accepted, 'NavigateToPose goal rejected')
        result_future = handle.get_result_async()
        self.assertTrue(ns.wait_for(result_future.done, 120.0), 'goal did not finish in 120 s')
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_SUCCEEDED)
        with self.probe.lock:
            cmds = list(self.probe.cmds)
        self.assertTrue(any(v > 0.0 for _, v, _ in cmds), 'controller_server never moved the robot')
        self.assertTrue(all(abs(v) <= 0.5 and abs(w) <= math.radians(90.0) for _, v, w in cmds),
                        'velocity above the robot limits (0.5 m/s, 90 deg/s)')

    def test_02_cancel_stops_controller_server_publishing_on_mux_nav(self):
        """005-R34 (Nav2 side): after the cancel completes, mux/nav goes silent (so the mux input times out)."""
        handle = self.probe.send(1.0, 1.0)
        self.assertTrue(handle.accepted)
        before = self.probe.n_cmds()
        self.assertTrue(ns.wait_for(lambda: self.probe.n_cmds() >= before + 10, 30.0), 'no velocity before cancel')
        cancel_future = handle.cancel_goal_async()
        self.assertTrue(ns.wait_for(cancel_future.done, 10.0), 'cancel not answered')
        time.sleep(0.5)                              # grace for the controller's final zero velocity
        after = self.probe.n_cmds()
        time.sleep(2.0)                              # quiet window: 20 controller periods
        self.assertEqual(self.probe.n_cmds(), after,
                         'controller_server kept publishing on /myRobot/mux/nav after the cancel')
