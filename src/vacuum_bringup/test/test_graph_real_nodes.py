"""Live-graph test: skeleton.launch.py with_sequencer:=true real_nodes:=true (001-R01, 005-R28, 005-R33, 005-R38).

The real escape_server and stuck_detector of vacuum_coverage (and, implied, the real coverage_server) replace the
stubs under the same node names; the sequencer must reach ALL_ACTIVE and /myRobot/mux/escape must have exactly one
publisher: the escape_server (Art. VI). Domain: bringup range 100-129 through graph_support.domain_for. Condition
waits only, generous timeouts (the host may be loaded).
"""
import os
import sys
import unittest

import pytest
from diagnostic_msgs.msg import DiagnosticArray
from launch_testing.actions import ReadyToTest
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 25
TIMEOUT_S = 240.0


@pytest.mark.launch_test
def generate_test_description():
    description = gs.launch_skeleton(MODULE, extra_args={'with_sequencer': 'true', 'real_nodes': 'true'})
    return description


class TestRealNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.states = []
        cls.probe.node.create_subscription(
            DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer',
            lambda m: cls.states.append(m.status[0].message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_001_R01_sequencer_reaches_all_active_with_the_real_nodes(self):
        self.assertTrue(self.probe.wait_for(lambda: self.states[-1:] == ['ALL_ACTIVE'], TIMEOUT_S),
                        f'sequencer states seen: {self.states[-5:]}')

    def test_02_005_R33_real_escape_and_stuck_are_active(self):
        for name in ('escape_server', 'stuck_detector', 'coverage_server'):
            self.assertTrue(self.probe.wait_for(lambda n=name: self.probe.lifecycle_label(n) == 'active', 60.0),
                            f'{name}: {self.probe.lifecycle_label(name)}')

    def test_03_005_R33_the_real_escape_server_runs_not_the_stub(self):
        # the real node runs with `outcome: auto` (stuck_escape.launch.py); the stub uses `succeed` (005-R33)
        client = self.probe.node.create_client(GetParameters, '/escape_server/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))
        request = GetParameters.Request()
        request.names = ['outcome']
        response = self.probe.call(client, request, 30.0)
        self.assertEqual(response.values[0].string_value, 'auto')
        names = [n for n, ns in self.probe.node_names() if ns == '/']
        for name in ('escape_server', 'stuck_detector'):
            self.assertEqual(names.count(name), 1, names)

    def test_04_006_one_publisher_on_mux_escape_and_it_is_escape_server(self):
        self.assertTrue(self.probe.wait_for_publishers('/myRobot/mux/escape', 1, 60.0))
        publishers = self.probe.publishers_of('/myRobot/mux/escape')
        self.assertEqual([p.node_name for p in publishers], ['escape_server'])
