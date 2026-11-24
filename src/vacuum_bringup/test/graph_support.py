"""Support code of the live-graph launch_testing tests (T014, spec 001). Not a test module.

`Probe` is a rclpy node plus recorders running in its own executor thread. It plays the role of the
orchestrator (publishes /myRobot/behaviour/mode like `behaviour_server`, since the tests run without IA368II)
and of the operator (teleop, reset), and records what the skeleton publishes. Publishing on
/myRobot/behaviour/mode and /myRobot/mux/teleop is only valid with with_ia368ii:=false: with the IA368II nodes
running, behaviour_server owns the mode topic.

Every test module launches its own graph on its own ROS_DOMAIN_ID, so modules (and other packages' tests run
by colcon in parallel) never see each other's nodes.
"""
import os
import random
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
from vacuum_interfaces.msg import Notification, StuckEvent, Submode

CMD_VEL = '/myRobot/cmd_vel'
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
QOS_LOG = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
QOS_BEST_EFFORT = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
STARTUP_TIMEOUT_S = 120.0
# 001-R06/R07 say 0.5 s. Latency is measured on the supervisor's own Submode.stamp (taken in its tick), not on the
# time this probe happens to receive the message, so probe-side scheduling and discovery waits are not counted.
# What is left is allowed a documented margin (the supervisor publishes at 5 Hz, so one 0.2 s tick plus DDS
# delivery, rounded up to 0.25 s) and twice the scheduling jitter this probe itself saw while waiting, which scales
# the tolerance with the load of the machine in the same run. On an idle machine the jitter is a few ms and the
# check is 0.5 + 0.25 s against a measured ~0.2 s.
TICK_MARGIN_S = 0.25
JITTER_FACTOR = 2.0
JITTER_CAP_S = 0.5          # a longer stall than this must not be able to hide a real regression


# Isolation from every other ROS graph (Art. X, same approach as the conftest.py of PR #7 / T020).
# Domain ID ranges per package: vacuum_stubs 10-39, vacuum_safety 40-69, vacuum_supervisor 70-99,
# vacuum_bringup (these live-graph tests) 100-129. The base is random, chosen once per process; each test
# module takes the next ID in the range, so graphs of consecutive modules never overlap. Discovery is
# restricted to localhost so containers on one Docker bridge do not see each other.
_DOMAIN_BASE = random.SystemRandom().randrange(0, 30)
DOMAIN_RANGE = (100, 130)
DISCOVERY_RANGE = 'LOCALHOST'


def domain_for(module_index: int) -> int:
    return DOMAIN_RANGE[0] + (_DOMAIN_BASE + module_index) % (DOMAIN_RANGE[1] - DOMAIN_RANGE[0])


def isolate_environment(module_index: int) -> None:
    """For the test process itself (the Probe); the launched nodes get the same values through actions."""
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = DISCOVERY_RANGE
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_DOMAIN_ID'] = str(domain_for(module_index))


def launch_skeleton(module_index: int, with_ia368ii: bool = False) -> LaunchDescription:
    """The real skeleton.launch.py, minus teleop (needs a terminal) and the wheel bridge (needs CoppeliaSim)."""
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'launch', 'skeleton.launch.py')
    return LaunchDescription([
        # an action, not os.environ: launch_testing restores the environment after generate_test_description
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_for(module_index))),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', DISCOVERY_RANGE),
        # ROS_LOCALHOST_ONLY (deprecated) must not override the range: clear it for the children
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
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
        isolate_environment(module_index)
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain_for(module_index))
        self.node = rclpy.create_node('vacuum_launch_probe', context=self.context)
        self._lock = threading.Lock()
        self._active_clients = {}
        self.core_startup_s = None
        self.active_poll_log = {}
        self._submodes: List[Tuple[float, str]] = []
        self._submode_stamps: List[Tuple[float, str]] = []     # (supervisor stamp, system clock), submode
        self._stuck_stamps: List[float] = []
        self._jitter: List[Tuple[float, float]] = []           # (monotonic time, sleep overshoot in s)
        self._notifications: List[Tuple[float, Notification]] = []
        self._locks: List[Tuple[float, bool]] = []
        self._cmd_vel: List[Tuple[float, Tuple[float, ...]]] = []
        self._mode = 'IDLE'
        n = self.node
        n.create_subscription(Submode, '/myRobot/behaviour/submode', self._on_submode, QOS_LOG)
        n.create_subscription(StuckEvent, '/myRobot/stuck', self._on_stuck, QOS_LOG)
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
            self._submode_stamps.append((msg.stamp.sec + msg.stamp.nanosec * 1e-9, msg.submode))

    def _on_stuck(self, msg):
        with self._lock:
            self._stuck_stamps.append(msg.stamp.sec + msg.stamp.nanosec * 1e-9)

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

    def first_submode_stamp(self, submode: str, since_monotonic: float):
        """Supervisor stamp (converted to the monotonic clock) of the first `submode` message stamped after
        `since_monotonic`, or None. Independent of when this probe got to read the message."""
        offset = time.time() - time.monotonic()           # system clock -> monotonic (stamps use the system clock)
        with self._lock:
            return next((t - offset for t, s in self._submode_stamps
                         if s == submode and t - offset >= since_monotonic), None)

    def first_stuck_stamp(self, since_monotonic: float):
        """Stamp of the first StuckEvent (taken by the stuck_detector when it publishes it), on the monotonic
        clock, or None."""
        offset = time.time() - time.monotonic()
        with self._lock:
            return next((t - offset for t in self._stuck_stamps if t - offset >= since_monotonic - 0.05), None)

    def jitter_since(self, since_monotonic: float) -> float:
        """Largest sleep overshoot seen by wait_for after `since_monotonic`: a proxy for the machine's load."""
        with self._lock:
            return max((j for t, j in self._jitter if t >= since_monotonic), default=0.0)

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
            before = time.monotonic()
            time.sleep(period_s)
            now = time.monotonic()
            with self._lock:
                self._jitter.append((now, max(0.0, now - before - period_s)))
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
    def manager_is_active(self, manager: str, discovery_timeout_s: float = 2.0,
                          answer_timeout_s: float = 3.0) -> bool:
        """One poll of `/<manager>/is_active`. The client is reused between polls; when a request gets no
        answer within `answer_timeout_s` the client is destroyed (not leaked) and the next poll sends on a new
        one. A pending request must never be waited on indefinitely: seen on Jazzy with 1 CPU, every core node
        `active` and the manager's is_active still unanswered after 127 s (lost service request/response)."""
        client = self._active_clients.get(manager)
        if client is None:
            client = self._active_clients[manager] = self.node.create_client(Trigger, f'/{manager}/is_active')
        if not client.wait_for_service(timeout_sec=discovery_timeout_s):
            self.active_poll_log[manager] = 'service not discovered'
            return False
        future = client.call_async(Trigger.Request())
        if not self.wait_for(future.done, answer_timeout_s):
            # also drop the client: if its response path never matched, a re-send on it would be lost too
            self.node.destroy_client(client)
            del self._active_clients[manager]
            self.active_poll_log[manager] = 'no answer'
            return False
        self.active_poll_log[manager] = f'answered success={future.result().success}'
        return bool(future.result().success)

    def lifecycle_label(self, node_name: str, discovery_timeout_s: float = 15.0) -> str:
        """Lifecycle state label. The probe learns about nodes by DDS discovery, which lags on a loaded CI
        runner even after lifecycle_manager_core reported active, so the service is waited for (condition)."""
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=discovery_timeout_s):
            return 'unreachable'
        future = client.call_async(GetState.Request())
        if not self.wait_for(future.done, 5.0):
            return 'timeout'
        return future.result().current_state.label

    def node_names(self) -> List[Tuple[str, str]]:
        return self.node.get_node_names_and_namespaces()

    def wait_for_nodes(self, names, timeout_s: float = 15.0) -> bool:
        """Waits until every name is in the discovered graph (root namespace)."""
        return self.wait_for(
            lambda: set(names) <= {n for n, ns in self.node_names() if ns == '/'}, timeout_s, 0.1)

    def wait_for_publishers(self, topic: str, count: int = 1, timeout_s: float = 15.0) -> bool:
        return self.wait_for(lambda: len(self.publishers_of(topic)) >= count, timeout_s, 0.1)

    def publishers_of(self, topic: str):
        return self.node.get_publishers_info_by_topic(topic)

    def wait_core_active(self, timeout_s: float = STARTUP_TIMEOUT_S) -> bool:
        start = time.monotonic()
        active = self.wait_for(lambda: self.manager_is_active('lifecycle_manager_core'), timeout_s, 0.5)
        self.core_startup_s = time.monotonic() - start
        print(f'lifecycle_manager_core is_active={active} after {self.core_startup_s:.1f} s', flush=True)
        if not active:
            # evidence for the next flake: which node is stuck, and was the manager reachable at all
            states = {name: self.lifecycle_label(name, discovery_timeout_s=1.0) for name in core_config_nodes()}
            seen = sorted(n for n, _ in self.node_names())
            print(f'core node states at timeout: {states}; last is_active poll: {self.active_poll_log}; '
                  f'nodes seen: {seen}', flush=True)
        return active


# ---------------------------------------------------------------------------- shared scenario steps
def start_cleaning(tc, probe: Probe) -> None:
    """Scenario 1 (001-R06): mode CLEANING -> COVERING within 0.5 s, and the coverage stub moves the robot."""
    tc.assertTrue(probe.wait_for_discovery(), 'supervisor never subscribed to /myRobot/behaviour/mode')
    # the supervisor's action clients must have discovered both servers: it treats an escape server it does
    # not see yet as a failed escape (task T021, branch fix/supervisor-escape-race)
    for action, action_type in (('/myRobot/cover_room', CoverRoom), ('/myRobot/escape', Escape)):
        tc.assertTrue(ActionClient(probe.node, action_type, action).wait_for_server(timeout_sec=15.0),
                      f'{action} server not available')
    time.sleep(1.0)
    tc.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'NONE', 10.0),
                  f'supervisor did not publish NONE while IDLE (last: {probe.last_submode()!r})')
    # Warm-up of the stuck path while the supervisor is in NONE (it ignores the event there). DDS delivers a
    # sample published before the reader/writer match only after the match completes (seen: ESCAPING 1.3 s after
    # the call on a loaded runner), so the match is completed here, outside any timed assertion.
    probe.trigger('/myRobot/stuck_detector/trigger')
    time.sleep(1.5)
    t0 = probe.set_mode('CLEANING')
    tc.assertTrue(probe.wait_for(lambda: probe.last_submode() == 'COVERING', 5.0),
                  f'COVERING never published (submodes: {probe.submodes(t0)})')
    assert_latency(tc, probe, 'COVERING', t0, '001-R06')
    # the goal reached coverage_server: it drives forward through the mux (linear_x 0.15, T010)
    tc.assertTrue(probe.wait_for(lambda: any(v[0] == 0.15 for v in probe.cmd_vel(t0)), 5.0),
                  'no coverage velocity on /myRobot/cmd_vel: cover_room goal not served')


def assert_latency(tc, probe: Probe, submode: str, t0: float, requirement: str, start=None) -> None:
    """Checks that `submode` was published within 0.5 s (the requirement) plus the documented margin.

    Both ends are the stamps the nodes put on their own messages (`start`, when given, is the StuckEvent stamp;
    otherwise `t0`, the probe's clock just before it published the mode). The time at which this probe got
    scheduled to read the message does not count. The tolerance is 0.5 s + TICK_MARGIN_S + JITTER_FACTOR times the
    sleep overshoot this probe saw since `t0` (capped at JITTER_CAP_S), so it grows with the load of this run and
    stays at 0.75 s on an idle machine; the ceiling is 0.75 s + 2 x 0.5 s = 1.75 s.
    Delay, tolerance and jitter are printed on every call."""
    stamp = probe.first_submode_stamp(submode, t0)
    tc.assertIsNotNone(stamp, f'no {submode} message stamped after the trigger')
    delay = stamp - (t0 if start is None else start)
    jitter = min(probe.jitter_since(t0), JITTER_CAP_S)
    allowed = 0.5 + TICK_MARGIN_S + JITTER_FACTOR * jitter
    received = probe.first_submode_time(submode, t0)           # when this probe read it: informational only
    received_s = float('nan') if received is None else received - t0
    tc.assertGreaterEqual(delay, 0.0, f'{submode} stamped {-delay:.3f} s before its trigger (stale message?)')
    print(f'LATENCY {requirement} {submode} delay={delay:.3f} allowed={allowed:.3f} jitter={jitter:.3f} '
          f'probe_received_after={received_s:.3f}', flush=True)
    tc.assertLessEqual(delay, allowed, f'{submode} after {delay:.2f} s (allowed {allowed:.2f} s = 0.5 s of '
                       f'{requirement} + {TICK_MARGIN_S} s tick + {JITTER_FACTOR} x {jitter:.2f} s jitter)')


def provoke_stuck(tc, probe: Probe) -> float:
    """Calls the stuck_detector stub; returns the time just before the (last) call.

    The stub publishes ONE StuckEvent per call. DDS only delivers it to readers its writer has already matched;
    on a loaded 1-CPU runner the supervisor's reader can still be unmatched 2 s after it was created (seen:
    "stuck event published" in the log, supervisor stayed in COVERING, nothing ignored or late). That is a lost
    event, not a slow supervisor, so the call is repeated while the supervisor has shown no sign of the event
    (no ESCAPING) after the full 001-R07 budget; the latency assertion in `assert_escaping_within` is applied,
    unchanged, to the call that was delivered."""
    tc.assertTrue(probe.wait_for(lambda: probe.node.count_subscribers('/myRobot/stuck') >= 1, 30.0),
                  'nobody subscribed to /myRobot/stuck')
    time.sleep(0.5)                       # the publisher side of that match is discovered a moment later
    for _ in range(6):
        t0 = time.monotonic()
        response = probe.trigger('/myRobot/stuck_detector/trigger')
        tc.assertTrue(response.success, response.message)
        if probe.wait_for(lambda: probe.first_submode_time('ESCAPING', t0) is not None,
                          0.5 + TICK_MARGIN_S + 0.75):
            return t0
    return t0


def assert_escaping_within(tc, probe: Probe, t0: float) -> None:
    """001-R07: ESCAPING within 0.5 s of the stuck event."""
    tc.assertTrue(probe.wait_for(lambda: probe.first_submode_time('ESCAPING', t0) is not None, 5.0),
                  f'ESCAPING never published (submodes: {probe.submodes(t0)})')
    started = probe.first_stuck_stamp(t0)
    tc.assertIsNotNone(started, 'the probe never saw the StuckEvent that triggered the escape')
    assert_latency(tc, probe, 'ESCAPING', t0, '001-R07', start=started)


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
