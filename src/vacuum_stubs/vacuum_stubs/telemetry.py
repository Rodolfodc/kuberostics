"""Shared D-20 telemetry helper: one DiagnosticStatus per node, 1 Hz."""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# QoS of the IA368II contract (profile ia368ii in specs/000-system/contracts/topics.yaml).
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


def build_diagnostics(node, message: str, values: dict, level=DiagnosticStatus.OK) -> DiagnosticArray:
    status = DiagnosticStatus(
        name=node.get_name(), hardware_id='vacuum', level=level, message=message,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
    array = DiagnosticArray(status=[status])
    array.header.stamp = node.get_clock().now().to_msg()
    return array
