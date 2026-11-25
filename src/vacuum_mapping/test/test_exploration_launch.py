"""launch_testing of launch/exploration.launch.py: the real exploration_server process alone, with the fakes of
the test (map, fake NavigateToPose server, TF, fake velocity publisher on mux/nav) instead of Nav2.

Requirements: 003-R10, R13 (the process explores a fake map and `explored` turns true), 003-R26 (node exit
code 0 at teardown), 003-R31 (kill test, see below).

KILL TEST (partial). 003-R31 / 005-R34 want: kill `exploration_server` mid-goal and no non-zero command on
mux/nav within feedback_stale_s + nav_cancel_s + 0.5 s = 4.0 s. The real supervisor watchdog (T512, and the
supervisor's `explore` client, T315) is not in this package or in main_local yet, so a test-local STAND-IN
watchdog implements the 005-R34 rule (no `explore` feedback for 2.5 s -> cancel all `/navigate_to_pose`
goals, zero goal id and stamp, within 1.0 s). The test therefore proves what is checkable here: the real node
process publishes feedback often enough for that rule, the orphan Nav2 goal is a real hazard after SIGKILL
(the fake controller keeps publishing), and the cancel-all path silences it inside 4.0 s. It does NOT prove
the supervisor's own cancel path; swap the stand-in for the real supervisor when T315/T512 land.

Domain: a fixed random id from 220-232 (mapping range) taken at import; the conftest fixture of this package
changes ROS_DOMAIN_ID per test, so this module never reads the environment after import.
"""
import os
import random
import signal
import threading
import time
import unittest

DOMAIN = random.SystemRandom().randrange(220, 233)
os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)

import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalInfo  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable  # noqa: E402
from launch.events.process import ProcessExited  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.client import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from vacuum_interfaces.action import Explore  # noqa: E402

from exploration_fixtures import (HANG, LATCHED, REACH, FakeNav, MapPublisher, known_cells,  # noqa: E402
                                  one_frontier_cells, wait_for)

FEEDBACK_STALE_S = 2.5          # 005-R34 defaults
NAV_CANCEL_S = 1.0
BUDGET_S = FEEDBACK_STALE_S + NAV_CANCEL_S + 0.5
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


@pytest.mark.launch_test
def generate_test_description():
    source = os.path.join(get_package_share_directory('vacuum_mapping'), 'launch', 'exploration.launch.py')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(DOMAIN)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source)),
        launch_testing.actions.ReadyToTest(),
    ])


def find_pids(needle: str):
    """PIDs of the processes whose command line holds `needle` (the node itself, not `ros2 launch`)."""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if needle in cmd and '--ros-args' in cmd:
            pids.append(int(entry))
    return pids


class TestExplorationAlone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=DOMAIN)
        cls.nav = FakeNav(lambda i, x, y: REACH, context=cls.context, on_reach=cls._on_reach)
        cls.maps = MapPublisher(one_frontier_cells(), context=cls.context)
        cls.probe = rclpy.create_node('exploration_launch_probe', context=cls.context)
        cls.get_state = cls.probe.create_client(GetState, '/exploration_server/get_state')
        cls.cancel_all = cls.probe.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        cls.explored = []
        cls.probe.create_subscription(Bool, '/myRobot/map/explored', lambda m: cls.explored.append(m.data), LATCHED)
        cls.twists = []               # (monotonic, non-zero?) on mux/nav
        cls.probe.create_subscription(
            Twist, '/myRobot/mux/nav', lambda m: cls.twists.append((time.monotonic(), m.linear.x != 0.0)), QOS)
        cls.velocity_pub = cls.probe.create_publisher(Twist, '/myRobot/mux/nav', QOS)
        cls.velocity_timer = cls.probe.create_timer(0.1, cls._publish_velocity)
        cls.feedback = []
        cls.executor = MultiThreadedExecutor(num_threads=6, context=cls.context)
        for n in (cls.nav, cls.maps, cls.probe):
            cls.executor.add_node(n)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def _on_reach(cls, index):
        cls.maps.publish(known_cells())

    @classmethod
    def _publish_velocity(cls):
        """Fake controller_server: fresh non-zero velocity while a Nav2 goal is active, zero otherwise."""
        msg = Twist()
        msg.linear.x = 0.2 if cls.nav.active else 0.0
        cls.velocity_pub.publish(msg)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        for n in (cls.nav, cls.maps, cls.probe):
            n.destroy_node()
        cls.context.try_shutdown()
        cls.thread.join(timeout=2)

    def _state(self):
        if not self.get_state.service_is_ready():
            return None
        future = self.get_state.call_async(GetState.Request())
        return future.result().current_state.id if wait_for(future.done, 3.0) else None

    def _wait_active(self, total_s=180.0):
        self.assertTrue(wait_for(lambda: self._state() == State.PRIMARY_STATE_ACTIVE, total_s),
                        'exploration_server never became active (slow discovery or startup failure)')

    def _send(self, client, feedback):
        accepted = []

        def try_goal():             # the node needs the latched map and the first TF before accepting
            future = client.send_goal_async(Explore.Goal(timeout_s=0.0), feedback_callback=feedback)
            if wait_for(future.done, 10.0) and future.result().accepted:
                accepted.append(future.result())
            return bool(accepted)
        wait_for(try_goal, 60.0)
        self.assertTrue(accepted, 'goal never accepted')
        return accepted[0]

    def test_R13_R10_real_node_explores_the_fake_map_and_explored_turns_true(self):
        self._wait_active()
        self.assertTrue(wait_for(lambda: self.explored[:1] == [False], 20.0), 'latched false first (003-R13)')
        client = ActionClient(self.probe, Explore, '/myRobot/explore')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        handle = self._send(client, lambda m: None)
        result_future = handle.get_result_async()
        self.assertTrue(wait_for(result_future.done, 60.0))
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result.success)
        self.assertTrue(wait_for(lambda: self.explored[-1:] == [True], 10.0))
        self.assertTrue(self.nav.goals)

    def test_R31_kill_mid_goal_stand_in_watchdog_silences_mux_nav_within_budget(self):
        self._wait_active()
        # hang the next goal: the fake controller keeps publishing until a cancel arrives
        hang_from = len(self.nav.goals)
        self.nav.script = lambda i, x, y: HANG if i >= hang_from else REACH
        self.maps.publish(one_frontier_cells())
        client = ActionClient(self.probe, Explore, '/myRobot/explore')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        self._send(client, lambda m: self.feedback.append(time.monotonic()))
        self.assertTrue(wait_for(lambda: self.nav.active, 30.0), 'the Nav2 goal never started')
        self.assertTrue(wait_for(lambda: len(self.feedback) >= 4, 10.0))
        gaps = [b - a for a, b in zip(self.feedback, self.feedback[1:])]
        self.assertLessEqual(max(gaps), 1.0, 'feedback must be 1 Hz or faster (003-R31): ' + str(gaps))

        pids = find_pids('exploration_server')
        self.assertTrue(pids, 'node process not found')
        t_kill = time.monotonic()
        for pid in pids:
            os.kill(pid, signal.SIGKILL)
        self.assertTrue(self.nav.active, 'the Nav2 goal is orphaned by the kill: the hazard is real')

        # stand-in for the supervisor watchdog (005-R34): no feedback for feedback_stale_s -> cancel all goals
        last_feedback = self.feedback[-1]
        self.assertTrue(wait_for(lambda: time.monotonic() - max(last_feedback, self.feedback[-1]) > FEEDBACK_STALE_S,
                                 FEEDBACK_STALE_S + 2.0))
        self.assertTrue(self.cancel_all.wait_for_service(timeout_sec=5.0))
        request = CancelGoal.Request(goal_info=GoalInfo())       # zero goal id and zero stamp: cancel all
        future = self.cancel_all.call_async(request)
        self.assertTrue(wait_for(future.done, NAV_CANCEL_S))
        self.assertTrue(wait_for(lambda: not self.nav.active, NAV_CANCEL_S))

        # last non-zero command must be older than the budget; wait for the budget to pass and check
        deadline = t_kill + BUDGET_S
        wait_for(lambda: time.monotonic() > deadline, BUDGET_S)
        late = [t for t, nonzero in self.twists if nonzero and t > deadline]
        self.assertFalse(late, f'{len(late)} non-zero commands on mux/nav after kill + {BUDGET_S} s')
        self.assertTrue([t for t, nonzero in self.twists if nonzero and t > t_kill],
                        'the fake controller published nothing after the kill: the test proves nothing')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_R26_node_exits_cleanly_or_was_killed(self, proc_info):
        # -9: the kill test ended the process on purpose
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -9, -15])
