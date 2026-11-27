"""Live-graph test: Nav2 under the lifecycle sequencer in the real skeleton.launch.py (005-R35, 005-R19, 005-R34).

skeleton.launch.py with_sequencer:=true nav:=true sim_slowdown:=3, with a map_server and the fake robot standing in
for SLAM and the sim bridges. The sequencer must bring lifecycle_manager_nav (listing the three Nav2 nodes) active
after the core manager, and the timeouts of the running nodes must be scaled. Domain: bringup range 100-129 through
graph_support.domain_for. Condition waits only.
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
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402
import nav_support as ns  # noqa: E402

MODULE = 23
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
                              'with_sequencer': 'true', 'nav': 'true', 'sim_slowdown': str(FACTOR)}.items()),
        ReadyToTest(),
    ])


class TestNavSequenced(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        node = cls.probe.node
        cls.states = []
        cls.values = {}
        node.create_subscription(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer',
                                 lambda m: (cls.states.append(m.status[0].message),
                                            cls.values.update({kv.key: kv.value for kv in m.status[0].values})), 10)
        cls.is_active = node.create_client(Trigger, '/lifecycle_manager_nav/is_active')
        cls.params = node.create_client(GetParameters, '/controller_server/get_parameters')

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_005_R35_sequencer_reaches_all_active_with_nav_inside(self):
        # only managers with nodes are launched and listed: no 5 s startup timeout of an empty manager
        self.assertTrue(ns.wait_for(lambda: self.states[-1:] == ['ALL_ACTIVE'], 150.0),
                        f'sequencer states seen: {self.states[-5:]}')

    def test_02_005_R35_nav_manager_manages_the_three_nodes_and_is_active(self):
        self.assertTrue(self.is_active.wait_for_service(timeout_sec=30.0))
        future = self.is_active.call_async(Trigger.Request())
        self.assertTrue(ns.wait_for(future.done, 10.0))
        self.assertTrue(future.result().success)
        for name in ('controller_server', 'planner_server', 'bt_navigator'):
            self.assertEqual(self.probe.lifecycle_label(name), 'active', name)

    def test_03_005_R34_sim_slowdown_reached_the_running_controller(self):
        self.assertTrue(self.params.wait_for_service(timeout_sec=30.0))
        request = GetParameters.Request()
        request.names = ['progress_checker.movement_time_allowance']
        future = self.params.call_async(request)
        self.assertTrue(ns.wait_for(future.done, 10.0))
        value = future.result().values[0]
        self.assertEqual(value.type, ParameterType.PARAMETER_DOUBLE)
        self.assertEqual(value.double_value, 15.0 * FACTOR)

    def test_04_005_R35_telemetry_lists_only_the_managers_that_have_nodes(self):
        # a manager without nodes cannot be launched (the Nav2 manager aborts on node_names: []), so it is absent
        # from the sequencer's managers and from its per-manager telemetry; the ones listed are active, none failed
        self.assertTrue(ns.wait_for(lambda: 'startup_lifecycle_manager_core_outcome' in self.values, 30.0))
        self.assertEqual(self.values['startup_lifecycle_manager_core_outcome'], 'OK')
        self.assertEqual(self.values.get('startup_lifecycle_manager_nav_outcome'), 'OK')
        self.assertNotIn('startup_lifecycle_manager_optional_outcome', self.values)
