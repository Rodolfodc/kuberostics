"""T521: 005-R38 (and 005-R35 to R37) on a live graph: the three REAL Nav2 lifecycle managers, the REAL
`lifecycle_sequencer` and dummy lifecycle nodes (`dummy_lifecycle_node`, test parameter `fail_on_activate`).

Why processes started by the test and not one launch_testing description: the scenarios need several graphs
(5 clean starts, one failure per manager) and the shutdown must be triggered on the sequencer ALONE. `ros2
launch` sends SIGINT to every process at once, which would kill the managers together with the sequencer and
hide the order of 005-R37. Every wait is a condition wait with a timeout, no fixed sleeps. Each scenario has its
own ROS_DOMAIN_ID (range 130-159, reserved for this module) and LOCALHOST discovery (Art. X).

Graph: core = [stop_guard, core_a], nav = [nav_a, nav_b], optional = [opt_a]. `stop_guard` is a dummy here.
"""
import os
import random
import signal
import subprocess
import tempfile
import threading
import time

import pytest
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from vacuum_interfaces.msg import Notification

DOMAIN_RANGE = (130, 160)
_BASE = random.SystemRandom().randrange(0, 30)
GRAPH = {
    'lifecycle_manager_core': ['stop_guard', 'core_a'],
    'lifecycle_manager_nav': ['nav_a', 'nav_b'],
    'lifecycle_manager_optional': ['opt_a'],
}
ALL_NODES = [n for names in GRAPH.values() for n in names]
TIMEOUT_S = 60.0
_counter = [0]


def next_domain():
    _counter[0] += 1
    return DOMAIN_RANGE[0] + (_BASE + _counter[0]) % (DOMAIN_RANGE[1] - DOMAIN_RANGE[0])


def wait_for(predicate, timeout_s=TIMEOUT_S, period_s=0.05):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


class Graph:
    """The processes, a recording probe, and an orderly teardown."""

    def __init__(self, failing=None, startup_timeout_s=20.0):
        self.domain = next_domain()
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                        ROS_LOCALHOST_ONLY='0')
        self.tmp = tempfile.TemporaryDirectory()
        self.procs = {}
        self.commands = {}
        self.respawns = []          # helper processes that died before their service appeared
        self.failing = failing
        self.startup_timeout_s = startup_timeout_s
        os.environ.update(ROS_DOMAIN_ID=str(self.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=self.domain)
        self.node = rclpy.create_node('sequencer_test_probe', context=self.context)
        self.lock = threading.Lock()
        self.notifications = []
        self.state = ''
        self.diag_values = {}
        self.node.create_subscription(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer', self._on_diag, 10)
        self.node.create_subscription(Notification, '/myRobot/notifications', self._on_note, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    # ---- recorders
    def _on_diag(self, msg):
        with self.lock:
            self.state = msg.status[0].message
            self.diag_values = {kv.key: kv.value for kv in msg.status[0].values}

    def _on_note(self, msg):
        with self.lock:
            self.notifications.append((msg.severity, msg.code))

    def diagnostics(self):
        """What every process printed, for the message of a failed wait."""
        return '\n'.join(f'--- {name}\n{self.log(name, tail=1500)}' for name in
                         ['lifecycle_sequencer', *GRAPH, *ALL_NODES] if name in self.procs)

    def wait_state(self, expected, timeout_s=TIMEOUT_S):
        assert wait_for(lambda: self.current_state() == expected, timeout_s), (
            f'state {self.current_state()!r}, expected {expected!r}\n{self.diagnostics()}')

    def current_state(self):
        with self.lock:
            return self.state

    def codes(self):
        """(severity, code) of the Notifications heard on the topic."""
        with self.lock:
            return list(self.notifications)

    def telemetry_codes(self):
        """Notification codes the sequencer reports in its periodic telemetry. The Notification topic is
        volatile, so a one-shot message published before the probe finished discovery can be missed; the
        telemetry repeats it every second and is what the assertions use."""
        with self.lock:
            return [c for c in self.diag_values.get('codes', '').split(',') if c]

    def shutdown_order(self):
        """Nodes in the order they ran on_shutdown, from the monotonic times the dummy nodes print (the host
        shares CLOCK_MONOTONIC across processes). Nodes that have not shut down yet are not listed."""
        stamps = []
        for name in ALL_NODES:
            for line in self.log(name, tail=None).splitlines():
                if line.startswith('TRANSITION shutdown '):
                    stamps.append((int(line.split()[2]), name))
        return [name for _, name in sorted(stamps)]

    # ---- processes
    def _spawn(self, name, command):
        self.commands[name] = command
        log = open(os.path.join(self.tmp.name, f'{name}.log'), 'a')
        self.procs[name] = subprocess.Popen(command, env=self.env, start_new_session=True,
                                            stdout=log, stderr=subprocess.STDOUT)

    def log(self, name, tail=3000):
        """Output of one process, for assertion messages."""
        with open(os.path.join(self.tmp.name, f'{name}.log')) as f:
            text = f.read()
        return text if tail is None else text[-tail:]

    def start(self):
        ros = ['--ros-args']
        for names in GRAPH.values():
            for n in names:
                params = ['-p', f'fail_on_activate:={"true" if n == self.failing else "false"}']
                self._spawn(n, ['ros2', 'run', 'vacuum_bringup', 'dummy_lifecycle_node', *ros,
                                '-r', f'__node:={n}', *params])
        # Staged start. A Nav2 manager that sends its first request before its reply path to a node is matched
        # blocks forever when the reply is dropped (seen on a 1-CPU run: the node logged "failed to send
        # response", the manager never returned from "Configuring nav_a"). Starting the managers only after
        # the nodes are discoverable, and the sequencer only after the managers' services are, narrows the
        # window; every step is a condition wait.
        self._wait_services([(GetState, f'/{n}/get_state', n) for n in ALL_NODES])
        for manager, names in GRAPH.items():
            path = os.path.join(self.tmp.name, f'{manager}.yaml')
            with open(path, 'w') as f:
                yaml.safe_dump({manager: {'ros__parameters': {
                    'autostart': False, 'bond_timeout': 0.0, 'node_names': names}}}, f)
            self._spawn(manager, ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', *ros,
                                  '-r', f'__node:={manager}', '--params-file', path])
        self._wait_services([(Trigger, f'/{m}/is_active', m) for m in GRAPH]
                            + [(ManageLifecycleNodes, f'/{m}/manage_nodes', m) for m in GRAPH])
        self._spawn('lifecycle_sequencer', [
            'ros2', 'run', 'vacuum_bringup', 'lifecycle_sequencer', *ros,
            '-p', f'manager_startup_timeout_s:={self.startup_timeout_s}', '-p', 'manager_shutdown_timeout_s:=15.0'])

    def _wait_services(self, services, timeout_s=TIMEOUT_S, max_respawns=3):
        """Waits for each service. `owner` is the helper process that provides it: if that process dies before
        the service appears (seen once in ~80 starts: `ros2 run` of a dummy node segfaulted in rclpy start-up
        on Python 3.14), it is started again, at most `max_respawns` times per process. The unit under test, the
        sequencer, is never restarted by the test."""
        for srv_type, name, owner in services:
            client = self.node.create_client(srv_type, name)
            try:
                end = time.monotonic() + timeout_s
                while not client.wait_for_service(timeout_sec=1.0):
                    proc = self.procs[owner]
                    if proc.poll() is not None and self.respawns.count(owner) < max_respawns:
                        self.respawns.append(owner)
                        self._spawn(owner, self.commands[owner])
                    assert time.monotonic() < end, f'{name} never appeared\n{self.diagnostics()}'
            finally:
                self.node.destroy_client(client)

    def label(self, name):
        client = self.node.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                return 'unreachable'
            future = client.call_async(GetState.Request())
            if not wait_for(future.done, 5.0):
                return 'timeout'
            return future.result().current_state.label
        finally:
            self.node.destroy_client(client)

    def labels(self):
        return {n: self.label(n) for n in ALL_NODES}

    def sigint_sequencer(self):
        os.killpg(self.procs['lifecycle_sequencer'].pid, signal.SIGINT)

    def close(self):
        for proc in self.procs.values():
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in self.procs.values():
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        self.context.try_shutdown()
        self.tmp.cleanup()


@pytest.fixture
def graph_factory():
    graphs = []

    def make(**kwargs):
        g = Graph(**kwargs)
        graphs.append(g)
        g.start()
        return g

    yield make
    for g in graphs:
        g.close()


@pytest.mark.parametrize('attempt', range(5))
def test_005_R38_all_nodes_of_all_managers_become_active_over_5_starts(graph_factory, attempt):
    g = graph_factory()
    g.wait_state('ALL_ACTIVE')
    assert set(g.labels().values()) == {'active'}, g.labels()
    assert g.telemetry_codes() == []


def test_005_R38_core_node_failing_gives_FAILED_CORE_and_nothing_else_is_started(graph_factory):
    g = graph_factory(failing='core_a')
    g.wait_state('FAILED_CORE')
    assert wait_for(lambda: 'CORE_START_FAILED' in g.telemetry_codes())
    labels = g.labels()
    assert labels['nav_a'] != 'active' and labels['nav_b'] != 'active' and labels['opt_a'] != 'active', labels


def test_005_R38_nav_node_failing_gives_DEGRADED_NAV_core_stays_active_nav_left_inactive(graph_factory):
    g = graph_factory(failing='nav_b')
    g.wait_state('DEGRADED_NAV')
    assert wait_for(lambda: 'NAV_UNAVAILABLE' in g.telemetry_codes())
    # the reset of the nav manager is part of the transition to DEGRADED_NAV: wait for it by condition
    assert wait_for(lambda: all(g.label(n) != 'active' for n in ('nav_a', 'nav_b'))), g.labels()
    labels = g.labels()
    assert labels['stop_guard'] == labels['core_a'] == 'active', labels
    assert labels['opt_a'] == 'active', labels          # optional is still tried after a nav failure


def test_005_R38_optional_node_failing_gives_DEGRADED_OPTIONAL_core_and_nav_unaffected(graph_factory):
    g = graph_factory(failing='opt_a')
    g.wait_state('DEGRADED_OPTIONAL')
    labels = g.labels()
    assert all(labels[n] == 'active' for n in ('stop_guard', 'core_a', 'nav_a', 'nav_b')), labels
    assert g.telemetry_codes() == ['OPTIONAL_UNAVAILABLE']


def test_005_R38_shutdown_order_is_optional_then_nav_then_core_with_stop_guard_last(graph_factory):
    g = graph_factory()
    g.wait_state('ALL_ACTIVE')
    g.sigint_sequencer()
    assert wait_for(lambda: len(g.shutdown_order()) == len(ALL_NODES)), f'shut down: {g.shutdown_order()}'
    order = g.shutdown_order()
    assert order[0] == 'opt_a', order
    assert set(order[1:3]) == {'nav_a', 'nav_b'}, order     # inside a manager the Nav2 order is not our concern
    assert order[-1] == 'stop_guard', order                 # 005-R37: stop_guard is the last to go down
    assert g.procs['lifecycle_sequencer'].wait(timeout=30) == 0


def test_005_R36_stop_guard_leaving_active_deactivates_nav_and_raises_ERROR(graph_factory):
    g = graph_factory()
    g.wait_state('ALL_ACTIVE')
    client = g.node.create_client(ChangeState, '/stop_guard/change_state')
    assert client.wait_for_service(timeout_sec=10.0)
    request = ChangeState.Request()
    request.transition.id = Transition.TRANSITION_DEACTIVATE
    future = client.call_async(request)
    assert wait_for(future.done, 10.0) and future.result().success
    assert wait_for(lambda: 'STOP_GUARD_LOST' in g.telemetry_codes()), g.log('lifecycle_sequencer')
    g.wait_state('DEGRADED_NAV')
    assert wait_for(lambda: all(g.label(n) != 'active' for n in ('nav_a', 'nav_b'))), g.labels()
    assert g.label('core_a') == 'active'
