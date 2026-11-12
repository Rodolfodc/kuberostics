"""T014, spec 001: live graph with the IA368II nodes (001-R01, 001-R02, 001-R03, 001-R12).

IA368II comes from `vcs import` and is not in the Docker test containers (Art. II, rosdep cannot install it), so
this module launches with_ia368ii:=true only when `myrobot_bringup` is installed; otherwise it launches nothing and
skips. It needs no simulator: the wheel bridge is off.
"""
import os
import sys
import unittest

import pytest

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 4


def ia368ii_manager_config():
    """(manager node name, managed node names) read from myrobot_bringup/config/lifecycle_manager.yaml, the file
    the IA368II lifecycle manager is launched with (skeleton.launch.py); None when the package is absent."""
    import yaml
    try:
        share = get_package_share_directory('myrobot_bringup')
    except PackageNotFoundError:
        return None
    with open(os.path.join(share, 'config', 'lifecycle_manager.yaml')) as f:
        document = yaml.safe_load(f)
    ((manager, body),) = document.items()
    return manager, list(body['ros__parameters']['node_names'])


def ia368ii_installed() -> bool:  # noqa: D103
    try:
        get_package_share_directory('myrobot_bringup')
        return True
    except PackageNotFoundError:
        return False


HAVE_IA368II = ia368ii_installed()


@pytest.mark.launch_test
def generate_test_description():
    if not HAVE_IA368II:        # nothing to test: do not launch the whole core graph just to skip
        from launch import LaunchDescription
        from launch.actions import ExecuteProcess
        from launch_testing.actions import ReadyToTest
        # an idle process keeps the launch alive until the (skipped) tests are reported
        return LaunchDescription([ExecuteProcess(cmd=['sleep', '30']), ReadyToTest()])
    return gs.launch_skeleton(MODULE, with_ia368ii=True)


@unittest.skipUnless(HAVE_IA368II, 'IA368II (myrobot_bringup) not installed: run `vcs import src/deps < deps.repos`')
class TestWithIa368ii(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.core_active = cls.probe.wait_core_active()
        cls.manager, cls.managed = ia368ii_manager_config()
        cls.ia368ii_active = cls.probe.wait_for(
            lambda: cls.probe.manager_is_active(cls.manager), gs.STARTUP_TIMEOUT_S, 0.5)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_both_managers_report_active(self):
        """001-R02: the IA368II manager and lifecycle_manager_core both answer true on is_active."""
        self.assertTrue(self.core_active, 'lifecycle_manager_core never reported is_active == true')
        self.assertTrue(self.ia368ii_active, 'IA368II lifecycle_manager never reported is_active == true')

    def test_02_ia368ii_node_names(self):
        """001-R02: every node in the IA368II lifecycle_manager.yaml node_names is running with that name."""
        live = {name for name, namespace in self.probe.node_names()}
        self.assertTrue(self.managed)
        for name in self.managed:
            self.assertIn(name, live)
        self.assertIn(self.manager, live)

    def test_03_twist_mux_still_the_only_publisher_of_cmd_vel(self):
        """001-R03, 001-R12: cleaning_server and autodocking_server are remapped away from /myRobot/cmd_vel."""
        self.assertEqual([p.node_name for p in self.probe.publishers_of(gs.CMD_VEL)], ['twist_mux'])
        sink = self.probe.publishers_of('/myRobot/legacy/cleaning_cmd_vel')
        self.assertEqual([p.node_name for p in sink], ['cleaning_server'])
        self.assertEqual(self.probe.node.count_subscribers('/myRobot/legacy/cleaning_cmd_vel'), 0,
                         'D-04: nobody may subscribe to the cleaning sink')
        docking = self.probe.publishers_of('/myRobot/mux/docking')
        self.assertEqual([p.node_name for p in docking], ['autodocking_server'])
