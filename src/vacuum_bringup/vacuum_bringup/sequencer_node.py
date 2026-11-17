"""`lifecycle_sequencer` (T521, spec 005, D-26): a PLAIN node on purpose (it only orchestrates managers and has
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
from lifecycle_msgs.msg import TransitionEvent
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from vacuum_interfaces.msg import Notification

from vacuum_bringup.manager_sequencer import (
    Action, ManagerSequencer, Outcome, Severity, State, default_specs)

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
        names = list(self.get_parameter('managers').value)
        self._pub_notify = self.create_publisher(Notification, '/myRobot/notifications', 10)
        self._pub_telemetry = self.create_publisher(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer', 10)
        self._manage = {n: self.create_client(ManageLifecycleNodes, f'/{n}/manage_nodes') for n in names}
        self._active = {n: self.create_client(Trigger, f'/{n}/is_active') for n in names}
        self._events: 'queue.Queue[str]' = queue.Queue()
        self._stop_guard_seen_active = False
        guard = self.get_parameter('stop_guard_node').value
        self.create_subscription(TransitionEvent, f'/{guard}/transition_event', self._on_guard_event, 10)
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

    def _call(self, manager: str, action: Action, timeout_s: float) -> Outcome:
        request = ManageLifecycleNodes.Request()
        request.command = COMMANDS[action]
        outcome, response = self._call_service(self._manage[manager], request, timeout_s)
        if outcome is not Outcome.OK:
            return outcome
        return Outcome.OK if response is not None and response.success else Outcome.FAILED

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
        if msg.goal_state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self._stop_guard_seen_active = True
        elif self._stop_guard_seen_active:
            self._stop_guard_seen_active = False
            self._events.put('stop_guard_lost')      # handled by the worker: it makes blocking calls

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
        while True:
            event = self._events.get()
            if event == 'shutdown':
                break
            if event == 'stop_guard_lost':
                self.sequencer.on_stop_guard_lost()
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
