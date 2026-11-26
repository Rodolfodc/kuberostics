"""my_robot_cmd_vel_node (plain Node on purpose, see README): /cmd_vel (unstamped Twist; the launch remaps it to /myRobot/cmd_vel, D-06) -> wheel
speed signals of the model. Rewrites the signals every cycle (the model clears them) and zeroes them when the
command stream stops. Plain Node on purpose, see README."""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from vacuum_sim_bridge.node_support import declare_connection, declare_profile, make_client
from vacuum_sim_bridge.wheel_core import WheelCommandCore


class CmdVelNode(Node):
    def __init__(self, client=None, clock=time.monotonic, **kwargs):
        super().__init__('my_robot_cmd_vel_node', **kwargs)
        declare_connection(self)
        self.declare_parameter('write_rate_hz', 20.0)
        self._p = declare_profile(self)
        self._client = client if client is not None else make_client(self)
        self._now = clock
        self._core = WheelCommandCore(self._p)
        self._core_lock = threading.Lock()
        self._handle = None
        self._was_idle = True
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        # Simulator IO runs in its own thread: a Remote API call takes about one simulation step (0.26 s at
        # 4 steps/s), and blocking the executor would delay the command subscription. Commands are stamped on
        # arrival in the executor, so a slow call cannot expire cmd_timeout.
        self._period = 1.0 / self.get_parameter('write_rate_hz').value
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._io_loop, name='cmd_vel_io', daemon=True)
        self._thread.start()

    def _io_loop(self):
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception as exc:   # an unexpected error must not kill the IO thread silently
                try:
                    self.get_logger().error(f'cmd_vel IO loop error: {exc!r}', throttle_duration_sec=5.0)
                except Exception:
                    pass               # node already destroyed
            self._stop.wait(self._period)

    def destroy_node(self):
        self._stop.set()
        # longer than the socket timeout (3 s default) plus margin, so a call in flight can finish
        self._thread.join(timeout=5.0)
        if not self._thread.is_alive() and self._handle is not None:
            try:
                self._client.set_wheel_speeds(self._handle, 0.0, 0.0)    # final stop
            except Exception:
                pass
        super().destroy_node()

    def _on_cmd(self, msg: Twist):
        with self._core_lock:
            self._core.command(msg.linear.x, msg.angular.z, self._now())

    def tick(self):
        with self._core_lock:
            left, right = self._core.output(self._now())
        idle = left == 0.0 and right == 0.0
        try:
            if self._handle is None:
                self._handle = self._client.get_object(self._p.robot_path)
            if idle and self._was_idle:
                return          # signals are consumed and cleared by the model; nothing to hold at zero
            self._client.set_wheel_speeds(self._handle, left, right)
        except Exception as exc:
            self._handle = None
            self.get_logger().warning(f'simulator not ready: {exc}', throttle_duration_sec=5.0)
            return
        if idle != self._was_idle:
            self.get_logger().info('wheels stopped' if idle else 'wheels driven')
        self._was_idle = idle


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
