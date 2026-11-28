"""Live-graph test: `sim_slowdown` reaches the running Nav2 nodes (005-R34, 005-R19). Same graph as test_nav_graph.py
launched with sim_slowdown:=4; the parameters read back from the nodes must be 4x the shipped values, and the BT file
bt_navigator was given must carry the 4x Timeout. Factor 1.0 (the default of every other graph test) is covered by
test_slowdown.py.
"""
import os
import sys
import unittest

import pytest
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_support as ns  # noqa: E402

MODULE = 22
FACTOR = 4


@pytest.mark.launch_test
def generate_test_description():
    return ns.launch_nav(MODULE, sim_slowdown=FACTOR)


class TestNavSlowdown(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = ns.NavProbe(MODULE)
        # clients are created once: a client created per call can lose its first response (Fast DDS)
        cls.clients = {n: cls.probe.node.create_client(GetParameters, f'/{n}/get_parameters')
                       for n in ('controller_server', 'planner_server', 'bt_navigator')}

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def get(self, node, name):
        client = self.clients[node]
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f'{node} has no parameter service')
        request = GetParameters.Request()
        request.names = [name]
        future = client.call_async(request)
        self.assertTrue(ns.wait_for(future.done, 10.0))
        value = future.result().values[0]
        self.assertEqual(value.type, ParameterType.PARAMETER_DOUBLE, f'{node} {name} not declared as double')
        return value.double_value

    def test_01_005_R34_timeouts_of_the_running_nodes_are_scaled(self):
        self.assertTrue(self.probe.ready, 'nav nodes never became active with the overlay parameters')
        self.assertEqual(self.get('controller_server', 'progress_checker.movement_time_allowance'), 15.0 * FACTOR)
        self.assertEqual(self.get('controller_server', 'costmap_update_timeout'), 1.0 * FACTOR)
        self.assertEqual(self.get('planner_server', 'costmap_update_timeout'), 1.0 * FACTOR)

    def test_02_005_R34_bt_navigator_loaded_the_scaled_tree(self):
        client = self.clients['bt_navigator']
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        request = GetParameters.Request()
        request.names = ['default_nav_to_pose_bt_xml']
        future = client.call_async(request)
        self.assertTrue(ns.wait_for(future.done, 10.0))
        path = future.result().values[0].string_value
        self.assertIn(f'msec="{180000 * FACTOR}"', open(path).read())
