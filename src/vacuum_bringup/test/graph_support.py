"""Support code of the live-graph launch_testing tests (T014, spec 001). Not a test module.

`Probe` is a rclpy node plus recorders running in its own executor thread. It plays the role of the
orchestrator (publishes /myRobot/behaviour/mode like `behaviour_server`, since the tests run without IA368II)
and of the operator (teleop, reset), and records what the skeleton publishes.

Every test module launches its own graph on its own ROS_DOMAIN_ID, so modules (and other packages' tests run
by colcon in parallel) never see each other's nodes.
"""
import os
import threading
import time
from typing import Callable, List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from vacuum_interfaces.action import CoverRoom, Escape
from vacuum_interfaces.msg import Notification, Submode

CMD_VEL = '/myRobot/cmd_vel'
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
QOS_LOG = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
QOS_BEST_EFFORT = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
STARTUP_TIMEOUT_S = 60.0


def domain_for(module_index: int) -> int:
    """A ROS_DOMAIN_ID distinct per test module and per process (0..101 is safe on every platform)."""
    return 1 + (os.getpid() * 7 + module_index * 13) % 100


def launch_skeleton(module_index: int, with_ia368ii: bool = False) -> LaunchDescription:
    """The real skeleton.launch.py, minus teleop (needs a terminal) and the wheel bridge (needs CoppeliaSim)."""
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'launch', 'skeleton.launch.py')
    return LaunchDescription([
        # an action, not os.environ: launch_testing restores the environment after generate_test_description
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_for(module_index))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source), launch_arguments={
            'with_ia368ii': 'true' if with_ia368ii else 'false',
            'with_wheel_bridge': 'false',
            'with_teleop': 'false',
        }.items()),
        ReadyToTest(),
    ])


def core_config_nodes() -> List[str]:
    """Node names listed in config/lifecycle_core.yaml (the file lifecycle_manager_core really reads)."""
    import yaml
    path = os.path.join(get_package_share_directory('vacuum_bringup'), 'config', 'lifecycle_core.yaml')
    with open(path) as f:
        return list(yaml.safe_load(f)['lifecycle_manager_core']['ros__parameters']['node_names'])


def zero(twist: Tuple[float, ...]) -> bool:
    return all(v == 0.0 for v in twist)


class Probe:
    def __init__(self, module_index: int):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain_for(module_index))
        self.node = rclpy.create_node('vacuum_launch_probe', context=self.context)
        self._lock = threading.Lock()
        self._submodes: List[Tuple[float, str]] = []
        self._notifications: List[Tuple[float, Notification]] = []
        self._locks: List[Tuple[float, bool]] = []
        self._cmd_vel: List[Tuple[float, Tuple[float, ...]]] = []
        self._mode = 'IDLE'
        n = self.node
        n.create_subscription(Submode, '/myRobot/behaviour/submode', self._on_submode, QOS_LOG)
        n.create_subscription(Notification, '/myRobot/notifications', self._on_notification, QOS_LOG)
        n.create_subscription(Bool, '/myRobot/mux/lock_stop', self._on_lock, QOS_LOG)
        n.create_subscription(Twist, CMD_VEL, self._on_cmd_vel, QOS_BEST_EFFORT)
        self._pub_mode = n.create_publisher(String, '/myRobot/behaviour/mode', QOS_IA368II)
        self._pub_teleop = n.create_publisher(Twist, '/myRobot/mux/teleop', QOS_IA368II)
        self._timer = n.create_timer(0.2, self._republish_mode)       # behaviour_server republishes every 0.2 s
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(n)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        self.context.try_shutdown()

    # ------------------------------------------------------------ recorders
    def _on_submode(self, msg):
        with self._lock:
            self._submodes.append((time.monotonic(), msg.submode))

    def _on_notification(self, msg):
        with self._lock:
            self._notifications.append((time.monotonic(), msg))

    def _on_lock(self, msg):
        with self._lock:
            self._locks.append((time.monotonic(), bool(msg.data)))

    def _on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        with self._lock:
            self._cmd_vel.append((time.monotonic(), values))

    def submodes(self, since: float = 0.0) -> List[str]:
        """Distinct consecutive submodes seen after `since` (monotonic seconds)."""
        with self._lock:
            seen = [s for t, s in self._submodes if t >= since]
        return [s for i, s in enumerate(seen) if i == 0 or s != seen[i - 1]]

    def last_submode(self) -> str:
        with self._lock:
            return self._submodes[-1][1] if self._submodes else ''

    def first_submode_time(self, submode: str, since: float):
        with self._lock:
            return next((t for t, s in self._submodes if t >= since and s == submode), None)

    def lock_value(self):
        with self._lock:
            return self._locks[-1][1] if self._locks else None

    def notifications(self, since: float = 0.0) -> List[Notification]:
        with self._lock:
            return [m for t, m in self._notifications if t >= since]

    def cmd_vel(self, since: float = 0.0) -> List[Tuple[float, ...]]:
        with self._lock:
            return [v for t, v in self._cmd_vel if t >= since]

    # ------------------------------------------------------------ actions
    def set_mode(self, mode: str) -> float:
        """Changes the orchestrator mode; returns the monotonic time of the change."""
        t0 = time.monotonic()
        self._mode = mode
        self._pub_mode.publish(String(data=mode))
        return t0

    def _republish_mode(self):
        self._pub_mode.publish(String(data=self._mode))

    def teleop_for(self, seconds: float, linear_x: float, rate_hz: float = 10.0) -> None:
        end = time.monotonic() + seconds
        msg = Twist()
        msg.linear.x = linear_x
        while time.monotonic() < end:
            self._pub_teleop.publish(msg)
            time.sleep(1.0 / rate_hz)

    def wait_for(self, predicate: Callable[[], bool], timeout_s: float, period_s: float = 0.02) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(period_s)
        return predicate()

    def wait_for_discovery(self, timeout_s: float = STARTUP_TIMEOUT_S) -> bool:
        return self.wait_for(lambda: self._pub_mode.get_subscription_count() > 0, timeout_s)

    def call(self, client, request, timeout_s: float = 15.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise AssertionError(f'service {client.srv_name} not available')
        future = client.call_async(request)
        if not self.wait_for(future.done, timeout_s):
            raise AssertionError(f'service {client.srv_name} did not answer in {timeout_s} s')
        return future.result()

    def trigger(self, service: str) -> Trigger.Response:
        return self.call(self.node.create_client(Trigger, service), Trigger.Request())

    def set_parameter(self, node_name: str, name: str, value) -> None:
        if isinstance(value, str):
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        client = self.node.create_client(SetParameters, f'/{node_name}/set_parameters')
        result = self.call(client, SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))
        assert result.results[0].successful, f'{node_name}.{name} = {value!r}: {result.results[0].reason}'

    # ------------------------------------------------------------ graph
    def manager_is_active(self, manager: str) -> bool:
        client = self.node.create_client(Trigger, f'/{manager}/is_active')
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        future = client.call_async(Trigger.Request())
        if not self.wait_for(future.done, 5.0):
            return False
        return bool(future.result().success)

    def lifecycle_label(self, node_name: str) -> str:
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=1.0):
            return 'unreachable'
        future = client.call_async(GetState.Request())
        if not self.wait_for(future.done, 5.0):
            return 'timeout'
        return future.result().current_state.label

    def node_names(self) -> List[Tuple[str, str]]:
        return self.node.get_node_names_and_namespaces()

    def publishers_of(self, topic: str):
        return self.node.get_publishers_info_by_topic(topic)

    def wait_core_active(self, timeout_s: float = STARTUP_TIMEOUT_S) -> bool:
        ok = self.wait_for(lambda: self.manager_is_active('lifecycle_manager_core'), timeout_s, 0.5)
        if not ok:
            print(f'DIAG rmw={os.environ.get("RMW_IMPLEMENTATION")} env={ {k: v for k, v in os.environ.items() if k.startswith(("ROS", "RMW", "FASTRTPS", "CYCLONE"))} } probe domain={self.context.get_domain_id()} '
                  f'nodes={self.node_names()} services={self.node.get_service_names_and_types()[:40]}', flush=True)
        return ok


# ---------------------------------------------------------------------------- shared scenario steps
def start_cleaning(tc, probe: Probe) -> None:
    """Scenario 1 (001-R06): mode CLEANING -> COVERING within 0.5 s, and the coverage stub moves the robot."""
    tc.assertTrue(probe.wait_for_discovery(), 'supervisor never subscribed to /myRobot/behaviour/mode')
    # the supervisor's action clients must have discovered both servers: it treats an escape server it does
    # not see yet as a failed escape (open issue reported with T014)
    for action, action_type in (('/myRobot/cover_room', CoverRoom), ('/myRobot/escape', Escape)):
        tc.assertTrue(ActionClient(probe.node, action_type, action).wait_for_server(timeout_sec=15.0),
                      f'{action} server not available')
    time.sleep(1.0)
    tc.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'NONE', 10.0),
                  f'supervisor did not publish NONE while IDLE (last: {probe.last_submode()!r})')
    t0 = probe.set_mode('CLEANING')
    tc.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'COVERING', 5.0),
                  f'COVERING never published (submodes: {probe.submodes(t0)})')
    delay = probe.first_submode_time('COVERING', t0) - t0
    tc.assertLessEqual(delay, 0.5, f'COVERING after {delay:.2f} s (001-R06 asks for 0.5 s)')
    # the goal reached coverage_server: it drives forward through the mux (linear_x 0.15, T010)
    tc.assertTrue(probe.wait_for(lambda: any(v[0] == 0.15 for v in probe.cmd_vel(t0)), 5.0),
                  'no coverage velocity on /myRobot/cmd_vel: cover_room goal not served')


def provoke_stuck(tc, probe: Probe) -> float:
    """Calls the stuck_detector stub; returns the time just before the call."""
    t0 = time.monotonic()
    response = probe.trigger('/myRobot/stuck_detector/trigger')
    tc.assertTrue(response.success, response.message)
    return t0


def assert_escaping_within(tc, probe: Probe, t0: float) -> None:
    """001-R07: ESCAPING within 0.5 s of the stuck event."""
    tc.assertTrue(probe.wait_for(lambda: probe.first_submode_time('ESCAPING', t0) is not None, 5.0),
                  f'ESCAPING never published (submodes: {probe.submodes(t0)})')
    delay = probe.first_submode_time('ESCAPING', t0) - t0
    tc.assertLessEqual(delay, 0.5, f'ESCAPING after {delay:.2f} s (001-R07 asks for 0.5 s)')


def assert_escape_failed(tc, probe: Probe, t0: float, within_s: float) -> None:
    """Scenario 3 (001-R09): ESCAPE_FAILED, lock_stop true and an ERROR Notification with code ESCAPE_FAILED."""
    tc.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'ESCAPE_FAILED', within_s),
                  f'ESCAPE_FAILED never published (submodes: {probe.submodes(t0)})')
    tc.assertTrue(probe.wait_for(lambda: probe.lock_value() is True, 5.0), 'lock_stop never became true')
    tc.assertTrue(probe.wait_for(lambda: bool(probe.notifications(t0)), 5.0), 'no Notification emitted')
    notes = probe.notifications(t0)
    errors = [m for m in notes if m.severity == Notification.ERROR and m.code == 'ESCAPE_FAILED']
    tc.assertTrue(errors, f'no ERROR/ESCAPE_FAILED notification among {[(m.severity, m.code) for m in notes]}')


def assert_robot_still(tc, probe: Probe, window_s: float = 1.5) -> None:
    """001-R10: while the lock is on, /myRobot/cmd_vel carries only exact zeros (and twist_mux is still
    publishing: the zero comes from the mux `stop` input fed by stop_guard, D-22)."""
    time.sleep(1.0)                      # let the last pre-lock command and the mux input timeouts expire
    t0 = time.monotonic()
    time.sleep(window_s)
    seen = probe.cmd_vel(t0)
    tc.assertGreater(len(seen), 5, 'twist_mux stopped publishing during the lock (stop_guard input missing?)')
    tc.assertTrue(all(zero(v) for v in seen), f'non-zero velocity while locked: {[v for v in seen if not zero(v)][:3]}')
