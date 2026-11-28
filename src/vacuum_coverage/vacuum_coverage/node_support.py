"""ROS glue shared by the real stuck_detector and escape_server nodes (spec 005-R26, R27): QoS profiles of the
contract, telemetry message, the redundant-transition guard and a resilient spin. No decisions here."""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# profiles of specs/000-system/contracts/topics.yaml
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
QOS_DEFAULT = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
# subscriber side of /scan: contract profile `default` (reliable); a best-effort subscriber would also match a
# best-effort publisher, so this is the stricter one and matches the 2025S2 bridge on both distros.
QOS_SCAN = QOS_DEFAULT


def build_diagnostics(node, message: str, values: dict, level=DiagnosticStatus.OK) -> DiagnosticArray:
    status = DiagnosticStatus(
        name=node.get_name(), hardware_id='vacuum', level=level, message=message,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
    array = DiagnosticArray(status=[status])
    array.header.stamp = node.get_clock().now().to_msg()
    return array


def guarded_change_state(node, parent_call, request, response):
    """A redundant or impossible lifecycle transition answers success=false instead of raising out of the service
    callback (Jazzy's rclpy raises, which used to end the process; same guard as coverage_server)."""
    try:
        return parent_call(request, response)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f'transition {request.transition.id} refused: {type(exc).__name__}')
        response.success = False
        return response
