"""T014, spec 001: live graph, scenario 3 with the escape stub on `outcome: fail`, plus the stop lock against
teleoperation (scenario 4, "the stop lock still takes precedence over teleoperation").

`outcome` is set on the running escape_server through its parameter service before the first goal (the
launch file does not expose it, and the stub reads it per goal).
"""
import os
import sys
import time
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 2


@pytest.mark.launch_test
def generate_test_description():
    return gs.launch_skeleton(MODULE)


class TestEscapeFails(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.core_active = cls.probe.wait_core_active()
        if cls.core_active:
            cls.probe.set_parameter('escape_server', 'outcome', 'fail')

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_scenario_3_escape_fails_and_robot_stops(self):
        """Scenario 3, 001-R09, 001-R16: failed escape -> ESCAPE_FAILED, lock_stop true, ERROR notification."""
        self.assertTrue(self.core_active, 'lifecycle_manager_core never reported is_active == true')
        gs.start_cleaning(self, self.probe)
        t0 = gs.provoke_stuck(self, self.probe)
        gs.assert_escaping_within(self, self.probe, t0)
        gs.assert_escape_failed(self, self.probe, t0, within_s=10.0)
        self.assertEqual(self.probe.submodes(t0)[-2:], ['ESCAPING', 'ESCAPE_FAILED'])

    def test_02_lock_forces_zero_velocity_and_twist_mux_stays_sole_publisher(self):
        """001-R10: with the lock on, /myRobot/cmd_vel is exactly zero and twist_mux is still its only publisher."""
        gs.assert_robot_still(self, self.probe)
        self.assertEqual([p.node_name for p in self.probe.publishers_of(gs.CMD_VEL)], ['twist_mux'])

    def test_03_lock_beats_teleoperation(self):
        """Scenario 4 / 001-R10: teleop (priority 100) cannot move the robot while the lock (200) is on."""
        self.probe.teleop_for(2.0, 0.5)
        t0 = time.monotonic() - 1.0              # the second half of the teleoperation
        seen = self.probe.cmd_vel(t0)
        self.assertTrue(seen, 'twist_mux stopped publishing')
        self.assertTrue(all(gs.zero(v) for v in seen), f'teleop moved the locked robot: {seen[:3]}')

    def test_04_reset_releases_the_lock_and_teleop_works_again(self):
        """001-R11: after the lock is released teleop is forwarded again. The release goes through the reset service
        of 001-R22, which is only exercised here as a means (001-R22 itself is verified by unit and node tests)."""
        self.probe.set_mode('IDLE')              # do not restart coverage behind the test's back
        response = self.probe.trigger('/myRobot/supervisor/reset')
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.probe.wait_for(lambda: self.probe.lock_value() is False, 5.0),
                        'lock_stop did not go back to false')
        time.sleep(1.0)
        t0 = time.monotonic()
        self.probe.teleop_for(1.5, 0.5)
        end = self.probe.cmd_vel(time.monotonic() - 0.6)
        self.assertTrue(end and all(v[0] == 0.5 for v in end), f'teleop not forwarded after reset: {end[-5:]}')
        self.assertTrue(any(v[0] == 0.5 for v in self.probe.cmd_vel(t0)))
