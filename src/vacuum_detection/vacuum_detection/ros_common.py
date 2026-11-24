"""Small ROS helpers shared by the detection nodes (the nodes stay glue; no decisions here, D-18)."""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Profiles of specs/000-system/contracts/topics.yaml.
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
QOS_SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time to float seconds (the unit the pure cores use)."""
    return stamp.sec + stamp.nanosec * 1e-9


def node_now_sec(node) -> float:
    """Robot clock of `node` in seconds: the one every age in spec 009a is measured against."""
    return node.get_clock().now().nanoseconds * 1e-9


def build_diagnostics(node, message: str, values: dict) -> DiagnosticArray:
    """One DiagnosticStatus with the metrics of a core (D-20)."""
    status = DiagnosticStatus(
        name=node.get_name(), hardware_id='vacuum', level=DiagnosticStatus.OK, message=message,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
    array = DiagnosticArray(status=[status])
    array.header.stamp = node.get_clock().now().to_msg()
    return array
