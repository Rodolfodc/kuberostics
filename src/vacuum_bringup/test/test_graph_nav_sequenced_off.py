"""Live-graph test: skeleton.launch.py with_sequencer:=true nav:=false (005-R35), with a map_server and the fake robot
standing in for SLAM and the sim bridges. No nav manager is launched or listed (and optional has no nodes), so the
sequencer reaches ALL_ACTIVE with only lifecycle_manager_core in its telemetry. Companion of
test_graph_nav_sequenced.py. Domain: bringup range 100-129 through graph_support.domain_for. Condition waits only.
"""
import atexit
import os
import shutil
import sys
import unittest

import pytest
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402
import nav_support as ns  # noqa: E402

MODULE = 24
FACTOR = 3


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('vacuum_bringup')
    directory = tempfile.mkdtemp(prefix='nav_seq_')
    atexit.register(shutil.rmtree, directory, ignore_errors=True)
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(gs.domain_for(MODULE))),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', gs.DISCOVERY_RANGE),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': ns.write_map(directory)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server'], 'bond_timeout': 0.0}]),
        ExecuteProcess(cmd=[sys.executable, os.path.join(ns.HERE, 'fake_robot.py'), '1.0', '1.0', '0.0'],
                       output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'skeleton.launch.py')),
            launch_arguments={'with_ia368ii': 'false', 'with_wheel_bridge': 'false', 'with_teleop': 'false',
                              'with_sequencer': 'true', 'nav': 'false', 'sim_slowdown': str(FACTOR)}.items()),
        ReadyToTest(),
    ])


class TestNoNavSequenced(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        node = cls.probe.node
        cls.states = []
        cls.values = {}
        node.create_subscription(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer',
                                 lambda m: (cls.states.append(m.status[0].message),
                                            cls.values.update({kv.key: kv.value for kv in m.status[0].values})), 10)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_005_R35_sequencer_reaches_all_active_without_nav(self):
        # only managers with nodes are launched and listed: no 5 s startup timeout of an empty manager
        self.assertTrue(ns.wait_for(lambda: self.states[-1:] == ['ALL_ACTIVE'], 150.0),
                        f'sequencer states seen: {self.states[-5:]}')

    def test_02_005_R35_telemetry_lists_only_the_managers_that_have_nodes(self):
        # a manager without nodes cannot be launched (the Nav2 manager aborts on node_names: []), so it is absent
        # from the sequencer's managers and from its per-manager telemetry; the ones listed are active, none failed
        self.assertTrue(ns.wait_for(lambda: 'startup_lifecycle_manager_core_outcome' in self.values, 30.0))
        self.assertEqual(self.values['startup_lifecycle_manager_core_outcome'], 'OK')
        self.assertEqual(self.values.get('startup_lifecycle_manager_nav_outcome'), None)
        self.assertNotIn('startup_lifecycle_manager_optional_outcome', self.values)
