"""Glue shared by the bridge nodes: profile <-> ROS parameters, static TF, client creation."""
from dataclasses import replace

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from vacuum_sim_bridge.profile import RobotProfile, profile_fields
from vacuum_sim_bridge.sim_client import ZmqSimClient


def declare_profile(node) -> RobotProfile:
    """Declares every RobotProfile field as a parameter (default = documented default) and reads it back."""
    values = {}
    for name, default in profile_fields():
        if isinstance(default, tuple):
            default = list(default)
        node.declare_parameter(name, default)
        value = node.get_parameter(name).value
        values[name] = tuple(value) if isinstance(default, list) else value
    return replace(RobotProfile(), **values)


def declare_connection(node):
    node.declare_parameter('zmq_host', 'localhost')
    node.declare_parameter('zmq_port', 23000)
    node.declare_parameter('atomic_wheel_write', False)   # opt-in; executeScriptString is absent on 4.10


def make_client(node):
    return ZmqSimClient(node.get_parameter('zmq_host').value, int(node.get_parameter('zmq_port').value),
                        node.get_logger(),
                        atomic_wheel_write=bool(node.get_parameter('atomic_wheel_write').value))


def static_transform(node, parent, child, xyz, quat) -> StaticTransformBroadcaster:
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = map(float, quat)
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(t)
    return broadcaster
