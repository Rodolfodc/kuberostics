"""Live-graph test (spec 005-R34): the BT `Timeout` decorator bounds a goal. Same graph as test_nav_graph.py but with
a copy of nav_bt.xml whose Timeout is 2 s: a goal that needs more than that must end ABORTED (not run on) and
controller_server must go silent on /myRobot/mux/nav. This proves the decorator exists on this distribution and
that the production tree (same file with msec 180000) is bounded the same way.
"""
import os
import re
import sys
import tempfile
import time
import unittest

import pytest
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_support as ns  # noqa: E402

MODULE = 21


def short_tree():
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'config', 'nav_bt.xml')
    text = open(source).read()
    assert 'msec="180000"' in text
    path = os.path.join(tempfile.mkdtemp(prefix='nav_bt_'), 'nav_bt_2s.xml')
    open(path, 'w').write(re.sub(r'msec="180000"', 'msec="2000"', text))
    return path


@pytest.mark.launch_test
def generate_test_description():
    return ns.launch_nav(MODULE, bt_xml=short_tree())


class TestNavTimeout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = ns.NavProbe(MODULE)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_005_R34_goal_longer_than_the_timeout_is_aborted_and_mux_nav_goes_silent(self):
        handle = self.probe.send(5.0, 5.0)           # about 5.6 m at 0.25 m/s: far above 2 s
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        self.assertTrue(ns.wait_for(result_future.done, 30.0), 'the Timeout decorator did not end the goal')
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_ABORTED)
        self.assertGreater(self.probe.n_cmds(), 0, 'the robot never started: the test proved nothing')
        time.sleep(0.5)
        after = self.probe.n_cmds()
        time.sleep(2.0)
        self.assertEqual(self.probe.n_cmds(), after, 'controller_server kept publishing after the timeout')
