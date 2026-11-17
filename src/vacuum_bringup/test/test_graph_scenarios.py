"""T014, spec 001: live graph, scenarios 1, 2 and 4 with the escape stub on `outcome: succeed` (the default).

Launches the real skeleton.launch.py (with_ia368ii:=false, with_wheel_bridge:=false, with_teleop:=false) and
plays the orchestrator from a probe node. Also checks the graph itself: the core manager reports its nodes
active (001-R02), each node's real name matches config/lifecycle_core.yaml, and twist_mux is the only publisher
of /myRobot/cmd_vel (001-R03 and Art. VI, checked again on the live graph here).

Tests run in name order and share one graph; each step depends on the previous one.
"""
import os
import sys
import time
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 1


@pytest.mark.launch_test
def generate_test_description():
    return gs.launch_skeleton(MODULE)


class TestSkeletonScenarios(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.core_active = cls.probe.wait_core_active()

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_core_manager_reports_active(self):
        """001-R02: lifecycle_manager_core answers true on is_active and every managed node is `active`."""
        self.assertTrue(self.core_active, 'lifecycle_manager_core never reported is_active == true')
        for name in gs.core_config_nodes():
            self.assertEqual(self.probe.lifecycle_label(name), 'active', f'{name} is not active')

    def test_02_core_node_names_match_lifecycle_core_yaml(self):
        """001-R02: the names the manager is configured with are the names the nodes really have."""
        names = gs.core_config_nodes()
        self.assertEqual(sorted(names), sorted(['stop_guard', 'coverage_server', 'escape_server',
                                                'stuck_detector', 'supervisor']))
        live = {name for name, namespace in self.probe.node_names() if namespace == '/'}
        for name in names:
            self.assertIn(name, live, f'lifecycle_core.yaml lists {name} but no such node is running')
        self.assertIn('lifecycle_manager_core', live)
        self.assertIn('twist_mux', live)

    def test_03_twist_mux_is_the_only_publisher_of_cmd_vel(self):
        """001-R03, 001-R10, Art. VI: exactly one publisher on /myRobot/cmd_vel, twist_mux, with type Twist."""
        publishers = self.probe.publishers_of(gs.CMD_VEL)
        self.assertEqual([p.node_name for p in publishers], ['twist_mux'])
        self.assertEqual(publishers[0].topic_type, 'geometry_msgs/msg/Twist')

    def test_04_scenario_1_happy_path(self):
        """Scenario 1, 001-R06: CLEANING -> COVERING within 0.5 s and the robot moves through coverage."""
        gs.start_cleaning(self, self.probe)

    def test_05_scenario_2_stuck_then_escape_succeeds(self):
        """Scenario 2, 001-R07, 001-R08: StuckEvent -> ESCAPING (escape drives) -> back to COVERING."""
        probe = self.probe
        t0 = gs.provoke_stuck(self, probe)
        gs.assert_escaping_within(self, probe, t0)
        # escape has priority over coverage: the reverse step of the stub reaches /myRobot/cmd_vel
        self.assertTrue(probe.wait_for(lambda: any(v[0] == -0.1 for v in probe.cmd_vel(t0)), 3.0),
                        'no escape velocity on /myRobot/cmd_vel (goal not sent to /myRobot/escape?)')
        self.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'COVERING', 10.0),
                        f'did not return to COVERING (submodes: {probe.submodes(t0)})')
        seen = probe.submodes(t0)
        self.assertEqual(seen[-2:], ['ESCAPING', 'COVERING'], f'unexpected submode sequence {seen}')
        self.assertNotIn('ESCAPE_FAILED', seen)
        self.assertIsNot(probe.lock_value(), True, 'the stop lock must stay off after a successful escape')
        t1 = time.monotonic()
        self.assertTrue(probe.wait_for(lambda: any(v[0] == 0.15 for v in probe.cmd_vel(t1)), 5.0),
                        'coverage did not resume after the escape')

    def test_06_scenario_4_teleop_has_precedence(self):
        """Scenario 4, 001-R11: teleop on the mux input overrides coverage; coverage resumes after it stops."""
        probe = self.probe
        self.assertEqual(probe.last_submode(), 'COVERING')
        t0 = time.monotonic()
        probe.teleop_for(2.0, 0.5)
        during = probe.cmd_vel(t0 + 0.8)       # skip the first moments: input timeout of the lower priority
        end = probe.cmd_vel(time.monotonic() - 0.6)
        self.assertTrue(during, 'no /myRobot/cmd_vel while teleoperating')
        self.assertTrue(end and all(v[0] == 0.5 for v in end),
                        f'teleop did not win over coverage: last commands {end[-5:]}')
        t1 = time.monotonic()
        self.assertTrue(probe.wait_for(lambda: any(v[0] == 0.15 for v in probe.cmd_vel(t1)), 5.0),
                        'coverage did not resume after the teleoperation stopped')
