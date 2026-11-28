"""Live-graph test of Nav2 (spec 005, T510) without CoppeliaSim: real nav.launch.py, real nav2_map_server, a fake
robot (fake_robot.py: odometry, TF, empty /scan, integrates /myRobot/mux/nav).

* 005-R19: a NavigateToPose goal on the saved map is accepted and the robot reaches it, driven only by
  controller_server on /myRobot/mux/nav.
* 005-R34 (Nav2 part): cancelling the goal stops controller_server publishing on /myRobot/mux/nav.
* 005-R23: Twist on mux/nav is unstamped geometry_msgs/Twist (the subscription would not receive it otherwise).
* 005-R29: the test runs on whichever distribution is in the container (CI runs it once per distro); it has no
  distro-specific branches.

NOT covered here (needs CoppeliaSim, left to the sim-operator: test_nav_sim_live.py, VACUUM_SIM_LIVE=1): the real wheel
bridge, the real laser, and the House map.
"""
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
        self.assertTrue(self.probe.wait_silent(1.0, 10.0),
                        'controller_server kept publishing on /myRobot/mux/nav after the cancel')
