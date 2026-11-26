"""my_robot_odometry_node (plain Node on purpose, see README): /myrobot/odom (nav_msgs/Odometry) and TF odom -> footprint -> base_link.

odom_source: 'ground_truth' (default, model pose relative to the first sample, as the 2025S2 project used the
simulator ground truth, Q-03) or 'wheels' (integrated joint angles; only valid while a wheel turns less than half a revolution
between two polls, so not when the simulation runs much faster than the poll rate)."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from vacuum_sim_bridge.geometry import quaternion_from_yaw, wrap_angle
from vacuum_sim_bridge.node_support import (declare_connection, declare_profile, make_client,
                                            static_transform)
from vacuum_sim_bridge.odometry_core import GroundTruthOdometry, WheelOdometry


class OdometryNode(Node):
    def __init__(self, client=None, **kwargs):
        super().__init__('my_robot_odometry_node', **kwargs)
        declare_connection(self)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('odom_source', 'ground_truth')
        self._p = declare_profile(self)
        source = self.get_parameter('odom_source').value
        if source not in ('ground_truth', 'wheels'):
            raise ValueError(f"odom_source must be 'ground_truth' or 'wheels', got {source!r}")
        self._source = source
        self._new_core = (lambda: GroundTruthOdometry(self._p)) if source == 'ground_truth' else (
            lambda: WheelOdometry(self._p))
        self._core = self._new_core()
        self._client = client if client is not None else make_client(self)
        self._handles = None
        self._last_joints = None
        self._pub = self.create_publisher(Odometry, '/myrobot/odom', 10)
        self._tf = TransformBroadcaster(self)
        self._static = static_transform(self, self._p.footprint_frame, self._p.base_frame,
                                        (0.0, 0.0, self._p.footprint_to_base_z), (0.0, 0.0, 0.0, 1.0))
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def _read(self):
        if self._handles is None:
            if self._source == 'ground_truth':
                self._handles = (self._client.get_object(self._p.robot_path),)
            else:
                self._handles = (self._client.get_object(self._p.left_motor_path),
                                 self._client.get_object(self._p.right_motor_path))
        t = self._client.get_sim_time()
        if self._source == 'ground_truth':
            return self._core.update(self._client.get_object_pose(self._handles[0]), t)
        joints = (self._client.get_joint_position(self._handles[0]),
                  self._client.get_joint_position(self._handles[1]))
        if self._last_joints is not None:
            jump = max(abs(wrap_angle(a - b)) for a, b in zip(joints, self._last_joints))
            raw = max(abs(a - b) for a, b in zip(joints, self._last_joints))
            if jump > math.pi / 2 or raw > math.pi / 2:
                self.get_logger().warning(
                    f'wheel joint moved {math.degrees(max(jump, raw)):.0f} deg in one poll (> quarter turn): '
                    'wheel odometry may alias, poll faster or use odom_source=ground_truth')
        self._last_joints = joints
        return self._core.update(*joints, t)

    def tick(self):
        try:
            state = self._read()
        except Exception as exc:
            self._handles = None
            self._last_joints = None
            self._core = self._new_core()      # a restarted simulator restarts time and pose: begin again at 0
            self.get_logger().warning(f'simulator not ready: {exc}', throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()
        q = quaternion_from_yaw(state.yaw)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._p.odom_frame
        msg.child_frame_id = self._p.footprint_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = state.x, state.y
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = state.vx, state.vy
        msg.twist.twist.angular.z = state.wz
        self._pub.publish(msg)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._p.odom_frame
        t.child_frame_id = self._p.footprint_frame
        t.transform.translation.x, t.transform.translation.y = state.x, state.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
