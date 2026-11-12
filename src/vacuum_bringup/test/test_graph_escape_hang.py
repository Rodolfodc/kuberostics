"""T014, spec 001: live graph, scenario 3 with the escape stub on `outcome: hang` (the escape never ends, so
the supervisor's `escape_timeout_s` must fire).

`outcome` and `escape_timeout_s` are set on the running nodes through their parameter services before the
first goal; both are read per goal. The timeout is shortened from 20 s to 3 s to keep the test fast; the 20 s default of 001-R09 is therefore not
tested here (it is a parameter default, checked by the supervisor unit/node tests).
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 3
TIMEOUT_S = 3.0


@pytest.mark.launch_test
def generate_test_description():
    return gs.launch_skeleton(MODULE)


class TestEscapeHangs(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.core_active = cls.probe.wait_core_active()
        if cls.core_active:
            cls.probe.set_parameter('escape_server', 'outcome', 'hang')
            cls.probe.set_parameter('supervisor', 'escape_timeout_s', TIMEOUT_S)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_scenario_3_escape_times_out(self):
        """Scenario 3, 001-R09, 001-R16: a hanging escape ends at escape_timeout_s with ESCAPE_FAILED, the
        stop lock and an ERROR notification."""
        self.assertTrue(self.core_active, 'lifecycle_manager_core never reported is_active == true')
        gs.start_cleaning(self, self.probe)
        t0 = gs.provoke_stuck(self, self.probe)
        gs.assert_escaping_within(self, self.probe, t0)
        # still escaping well before the timeout: the failure is due to the timeout, not to a quick abort
        self.assertEqual(self.probe.last_submode(), 'ESCAPING')
        gs.assert_escape_failed(self, self.probe, t0, within_s=TIMEOUT_S + 5.0)
        failed_at = self.probe.first_submode_time('ESCAPE_FAILED', t0)
        self.assertGreaterEqual(failed_at - t0, TIMEOUT_S - 0.5, 'ESCAPE_FAILED before escape_timeout_s')

    def test_02_robot_stays_still_after_the_timeout(self):
        """001-R10: after the timeout the velocity is exactly zero."""
        gs.assert_robot_still(self, self.probe)
        self.assertEqual([p.node_name for p in self.probe.publishers_of(gs.CMD_VEL)], ['twist_mux'])
