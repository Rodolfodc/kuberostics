"""Support code of the Nav2 live-graph tests (not a test module): the map, the launch description (real nav.launch.py,
real nav2_map_server, fake_robot.py) and `NavProbe`, an rclpy node in its own executor thread that records
/myRobot/mux/nav and talks to /navigate_to_pose."""
import os
import sys
import tempfile
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STARTUP_TIMEOUT_S = 150.0


def write_map(directory):
    """6 x 6 m room, 0.05 m cells, 0.15 m wall all around, origin (0, 0)."""
    n, wall = 120, 3
    rows = [bytes(0 if (r < wall or r >= n - wall or c < wall or c >= n - wall) else 254 for c in range(n))
            for r in range(n)]
    with open(os.path.join(directory, 'room.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (n, n) + b''.join(rows))
    with open(os.path.join(directory, 'room.yaml'), 'w') as f:
        f.write('image: room.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return os.path.join(directory, 'room.yaml')


def launch_nav(module, bt_xml=None, **launch_arguments):
    share = get_package_share_directory('vacuum_bringup')
    directory = tempfile.mkdtemp(prefix='nav_graph_')
    args = {'bt_xml': bt_xml} if bt_xml else {}
    args.update({k: str(v) for k, v in launch_arguments.items()})
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(gs.domain_for(module))),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', gs.DISCOVERY_RANGE),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': write_map(directory)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server'], 'bond_timeout': 0.0}]),
        ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, 'fake_robot.py'), '1.0', '1.0', '0.0'],
                       output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'nav.launch.py')),
                                 launch_arguments=args.items()),
        ReadyToTest(),
    ])


def goal_to(x, y):
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.w = 1.0
    return goal


def wait_for(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


class NavProbe:
    def __init__(self, module=None):
        """`module` picks the isolated test domain; None keeps the caller's ROS_DOMAIN_ID (the live sim test)."""
        self.context = rclpy.Context()
        if module is None:
            rclpy.init(context=self.context)
        else:
            gs.isolate_environment(module)
            rclpy.init(context=self.context, domain_id=gs.domain_for(module))
        self.node = rclpy.create_node('nav_graph_probe', context=self.context)
        self.lock = threading.Lock()
        self.cmds = []                                  # (monotonic time, linear x, angular z)
        self.node.create_subscription(Twist, '/myRobot/mux/nav', self._on_cmd, 50)
        self.odom = []                                  # (monotonic time, x, y, linear x, angular z)
        self.node.create_subscription(Odometry, '/myrobot/odom', self._on_odom, 20)
        self.client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.ready = self._wait_nav_active()

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        self.context.try_shutdown()

    def _on_cmd(self, msg):
        with self.lock:
            self.cmds.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_odom(self, msg):
        with self.lock:
            self.odom.append((time.monotonic(), msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            del self.odom[:-200]

    def _wait_nav_active(self):
        """The action server exists once bt_navigator is configured, but it rejects goals until active:
        ask lifecycle_manager_nav (is_active) instead of only waiting for the server."""
        is_active = self.node.create_client(Trigger, '/lifecycle_manager_nav/is_active')
        end = time.monotonic() + STARTUP_TIMEOUT_S
        while time.monotonic() < end:
            if is_active.wait_for_service(timeout_sec=1.0):
                future = is_active.call_async(Trigger.Request())
                wait_for(future.done, 5.0)
                if future.done() and future.result().success:
                    return self.client.wait_for_server(timeout_sec=5.0)
            time.sleep(0.5)
        return False

    def send(self, x, y):
        assert self.ready, '/navigate_to_pose never became available (nav nodes not active)'
        future = self.client.send_goal_async(goal_to(x, y))
        assert wait_for(future.done, 30.0), 'no answer to the goal'
        return future.result()

    def wait_silent(self, quiet_s=1.0, timeout=10.0):
        """True once /myRobot/mux/nav has been quiet for `quiet_s` (10 controller periods at 10 Hz, twice the 0.5 s
        twist_mux input timeout) within `timeout`; a controller that keeps publishing never gets there."""
        def quiet():
            with self.lock:
                return bool(self.cmds) and time.monotonic() - self.cmds[-1][0] >= quiet_s
        return wait_for(quiet, timeout)

    def n_cmds(self):
        with self.lock:
            return len(self.cmds)
