"""`lifecycle_sequencer` (spec 005, D-26): a PLAIN node on purpose (it only orchestrates managers and has
nothing for a manager to configure or clean up). Thin glue between ROS and `ManagerSequencer` (D-18).

It wires the sequencer's `call` and `wait_active` callables to the Nav2 lifecycle manager services
(`/<manager>/manage_nodes`, nav2_msgs/srv/ManageLifecycleNodes, and `/<manager>/is_active`,
std_srvs/srv/Trigger; same on Jazzy and Lyrical), publishes Notifications and a DiagnosticArray with the
sequencer metrics, and watches `/stop_guard/transition_event` (005-R36).

Threads: the main thread spins the executor; ONE worker thread runs the sequence and makes the blocking
service calls (a blocking call inside an executor callback would deadlock). SIGINT/SIGTERM only set a flag;
the worker then runs the ordered shutdown while the context is still alive, and rclpy is shut down after.
"""
import queue
import signal
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.msg import Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from vacuum_interfaces.msg import Notification

from vacuum_bringup.manager_sequencer import (
    Action, GuardWatch, ManagerSequencer, Outcome, Severity, State, default_specs)

COMMANDS = {
    Action.STARTUP: ManageLifecycleNodes.Request.STARTUP,
    Action.PAUSE: ManageLifecycleNodes.Request.PAUSE,
    Action.RESET: ManageLifecycleNodes.Request.RESET,
    Action.SHUTDOWN: ManageLifecycleNodes.Request.SHUTDOWN,
}
POLL_PERIOD_S = 0.5
SERVICE_WAIT_S = 1.0


class SequencerNode(Node):
    def __init__(self):
        super().__init__('lifecycle_sequencer')
        self.declare_parameter('managers', [
            'lifecycle_manager_core', 'lifecycle_manager_nav', 'lifecycle_manager_optional'])
        self.declare_parameter('manager_startup_timeout_s', 30.0)
        self.declare_parameter('manager_shutdown_timeout_s', 15.0)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self.declare_parameter('stop_guard_node', 'stop_guard')
        self.declare_parameter('stop_guard_poll_period_s', 1.0)
        names = list(self.get_parameter('managers').value)
        self._pub_notify = self.create_publisher(Notification, '/myRobot/notifications', 10)
        self._pub_telemetry = self.create_publisher(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer', 10)
        self._manage = {n: self.create_client(ManageLifecycleNodes, f'/{n}/manage_nodes') for n in names}
        self._active = {n: self.create_client(Trigger, f'/{n}/is_active') for n in names}
        self._events: 'queue.Queue[str]' = queue.Queue()
        guard = self.get_parameter('stop_guard_node').value
        self.create_subscription(TransitionEvent, f'/{guard}/transition_event', self._on_guard_event, 10)
        self._guard = guard
        self._poll_period_s = max(0.1, float(self.get_parameter('stop_guard_poll_period_s').value))
        # unreachable samples count only when almost a poll period apart (events can come in bursts)
        self._watch = GuardWatch(min_spacing_s=0.9 * self._poll_period_s)
        self.sequencer = ManagerSequencer(
            default_specs(names), self._call, self._wait_active, time.monotonic, self._log, self._notify,
            startup_timeout_s=float(self.get_parameter('manager_startup_timeout_s').value),
            shutdown_timeout_s=float(self.get_parameter('manager_shutdown_timeout_s').value))
        period = 1.0 / max(0.1, float(self.get_parameter('telemetry_rate_hz').value))
        self.create_timer(period, self._publish_telemetry)

    # ------------------------------------------------------------ callables injected into the sequencer
    def _log(self, level: str, message: str) -> None:
        # one call site per severity: rclpy forbids changing the severity of a call site between calls
        logger = self.get_logger()
        if level == 'error':
            logger.error(message)
        elif level == 'warn':
            logger.warning(message)
        else:
            logger.info(message)

    def _notify(self, severity: Severity, code: str, text: str) -> None:
        msg = Notification(severity=int(severity), source=self.get_name(), code=code, text=text)
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_notify.publish(msg)

    def _call_service(self, client, request, timeout_s: float):
        """Blocking call from the worker thread. Returns (Outcome, response or None)."""
        deadline = time.monotonic() + timeout_s
        if not client.wait_for_service(timeout_sec=min(timeout_s, SERVICE_WAIT_S * 5)):
            return Outcome.TIMEOUT, None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(max(0.0, deadline - time.monotonic())):
            return Outcome.TIMEOUT, None
        return Outcome.OK, future.result()

    def _call_once(self, srv_type, name: str, request, timeout_s: float):
        client = self.create_client(srv_type, name)
        try:
            return self._call_service(client, request, timeout_s)
        finally:
            self.destroy_client(client)

    def _call(self, manager: str, action: Action, timeout_s: float) -> Outcome:
        if action is Action.DEACTIVATE_NODES:
            return self._deactivate_nodes(manager, timeout_s)
        request = ManageLifecycleNodes.Request()
        request.command = COMMANDS[action]
        outcome, response = self._call_service(self._manage[manager], request, timeout_s)
        if outcome is not Outcome.OK:
            return outcome
        return Outcome.OK if response is not None and response.success else Outcome.FAILED

    def _deactivate_nodes(self, manager: str, timeout_s: float) -> Outcome:
        """Deactivates every node of `manager` that is still active (names from its `node_names` parameter)."""
        deadline = time.monotonic() + timeout_s

        def left():
            return max(0.1, deadline - time.monotonic())

        request = GetParameters.Request(names=['node_names'])
        outcome, response = self._call_once(GetParameters, f'/{manager}/get_parameters', request, left())
        if outcome is not Outcome.OK or not response.values:
            return Outcome.FAILED if outcome is Outcome.OK else outcome
        result = Outcome.OK
        for name in reversed(list(response.values[0].string_array_value)):
            outcome, state = self._call_once(GetState, f'/{name}/get_state', GetState.Request(),
                                             min(left(), 2.0))
            if outcome is not Outcome.OK or state.current_state.id != LifecycleState.PRIMARY_STATE_ACTIVE:
                continue                    # unreachable or not active: nothing to deactivate
            change = ChangeState.Request()
            change.transition.id = Transition.TRANSITION_DEACTIVATE
            outcome, done = self._call_once(ChangeState, f'/{name}/change_state', change, left())
            self._log('info', f'{manager}: deactivated {name}: {outcome.value}')
            if outcome is not Outcome.OK or not done.success:
                result = Outcome.FAILED
        return result

    def _wait_active(self, manager: str, timeout_s: float) -> Outcome:
        deadline = time.monotonic() + timeout_s
        while True:
            left = deadline - time.monotonic()
            if self.sequencer.aborted:
                return Outcome.FAILED
            if left <= 0.0:
                return Outcome.TIMEOUT
            outcome, response = self._call_service(self._active[manager], Trigger.Request(), min(left, 2.0))
            if outcome is Outcome.OK and response is not None and response.success:
                return Outcome.OK
            time.sleep(min(POLL_PERIOD_S, max(0.0, deadline - time.monotonic())))

    # ------------------------------------------------------------ events and telemetry
    def _on_guard_event(self, msg: TransitionEvent) -> None:
        """Only a wake-up: a transition event is volatile and can be missed, so the worker always confirms
        with get_state (and polls every stop_guard_poll_period_s) instead of trusting the event."""
        self._events.put('stop_guard_event')

    def _guard_sample(self) -> str:
        outcome, state = self._call_once(GetState, f'/{self._guard}/get_state', GetState.Request(), 2.0)
        if outcome is not Outcome.OK:
            return 'unreachable'
        return 'active' if state.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE else 'not_active'

    def _check_stop_guard(self) -> None:
        if self.sequencer.state not in (State.ALL_ACTIVE, State.DEGRADED_NAV, State.DEGRADED_OPTIONAL):
            return
        sample = self._guard_sample()
        lost = self._watch.feed(sample, time.monotonic())
        if sample == 'unreachable':
            self.get_logger().warning(
                f'stop_guard sample: unreachable ({self._watch.unreachable_count} counted in a row)')
        elif sample == 'not_active':
            self.get_logger().warning('stop_guard sample: not active')
        if lost:
            self.sequencer.on_stop_guard_lost()

    def _publish_telemetry(self) -> None:
        metrics = self.sequencer.metrics()
        failed = self.sequencer.state in (State.FAILED_CORE,)
        degraded = self.sequencer.state in (State.DEGRADED_NAV, State.DEGRADED_OPTIONAL)
        status = DiagnosticStatus(
            name=self.get_name(), hardware_id='lifecycle_sequencer', message=str(metrics['state']),
            level=DiagnosticStatus.ERROR if failed else DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK,
            values=[KeyValue(key=str(k), value=str(v)) for k, v in metrics.items()])
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_telemetry.publish(msg)

    # ------------------------------------------------------------ worker
    def request_shutdown(self) -> None:
        self.sequencer.request_abort()
        self._events.put('shutdown')

    def run(self) -> None:
        """The worker: startup, then events until shutdown, then the ordered shutdown."""
        try:
            self.sequencer.run_startup()
        except Exception as error:      # keep the process alive: the ordered shutdown must still run
            self.get_logger().error(f'startup sequence crashed: {error!r}')
        self._check_stop_guard()          # a loss during startup is seen at once
        while True:
            try:
                event = self._events.get(timeout=self._poll_period_s)
            except queue.Empty:
                event = 'poll'
            if event == 'shutdown':
                break
            self._check_stop_guard()
        self.sequencer.run_shutdown()
        self._publish_telemetry()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SequencerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    finished = threading.Event()

    def worker():
        try:
            node.run()
        finally:
            finished.set()

    thread = threading.Thread(target=worker, name='sequence')
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.request_shutdown())
    thread.start()
    try:
        while not finished.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
