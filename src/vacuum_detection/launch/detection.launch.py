"""detection.launch.py (spec 009a-R01): the robot-side perception chain, local only or with the cloud.

    ros2 launch vacuum_detection detection.launch.py [offload:=true|false] [local_weights:=yolo11n.pt]
                                                     [backend:=ultralytics|null] [device:=cpu] [with_manager:=true]
                                                     [with_projector:=false] [autostart:=false]

Only `detection_selector` publishes /myRobot/perception/detections (the contract allows one publisher), so the
selector runs in both modes and the local detector always publishes to .../detections/local.

  offload:=false (default)  detector_local + detection_selector. No cloud detector exists, no image leaves the
                            robot (no image_throttle), and the selector passes the local stream through (it
                            opens its breaker after open_threshold_s without cloud responses, then forwards
                            local). Same behaviour as spec 004 apart from that start-up delay (R01).
  offload:=true             the same, plus image_throttle sending the compressed RGB to the cloud (R03, R11).
                            A cloud detector (K3s; not started here) answers on .../detections/cloud and the
                            selector forwards it while healthy and local otherwise (R05 to R10). The local
                            detector is hot standby in both modes (R04).

`autostart:=true` configures and then activates every node of the chain FROM THIS LAUNCH (launch lifecycle events:
a node that reaches `inactive` is activated, a node that starts is configured), for use without a manager. The
nodes go one after the other (detector_local, detection_selector, image_throttle, projector): a node is
configured when the previous one is active. It needs
`with_manager:=false`: with the manager (default) the nodes are already started by it and `autostart` is ignored
with a warning, so the manager and the launch never both drive a transition. Default false: the behaviour
described below is unchanged (manager autostart when `with_manager:=true`, nothing otherwise).
`with_projector:=true` adds the `projector` (R12, R13; default false: it needs TF and the map, which the
caller provides).

The nodes are lifecycle nodes (D-26) under `lifecycle_manager_optional` (D-09): a perception failure is reported by
the manager but never stops the core (Art. VII).
`with_manager:=false` leaves the manager out, for a launch that already has one (skeleton.launch.py with the
sequencer: then these node names go into its lifecycle_optional.yaml). """
import launch
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

LOCAL_TOPIC = '/myRobot/perception/detections/local'


def autostart_actions(node: LifecycleNode, name: str, previous: LifecycleNode = None):
    """Configure `node` (when its process starts, or when `previous` is active) and activate it once inactive."""
    configure = [
        LogInfo(msg=f'autostart: configuring {name}'),
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))]
    if previous is None:
        first = RegisterEventHandler(OnProcessStart(target_action=node, on_start=configure))
    else:
        first = RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=previous, goal_state='active', entities=configure))
    return [
        first,
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive', entities=[
                LogInfo(msg=f'autostart: activating {name}'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))])),
    ]


def detection_actions(offload: bool, local_weights: str, backend: str, device: str, with_manager: bool,
                      local_image_topic: str = '/camera', autostart: bool = False, with_projector: bool = False):
    """The actions of the chain for one value of `offload` (pure function of its arguments, testable)."""
    launch_drives = autostart and not with_manager       # the manager and the launch never both drive transitions
    actions = []
    if autostart and with_manager:
        actions.append(LogInfo(msg='autostart ignored: lifecycle_manager_optional starts the nodes (with_manager)'))
    names = []
    previous = []

    def add(executable, name, parameters=None):
        kwargs = dict(package='vacuum_detection', executable=executable, name=name, output='screen',
                      parameters=parameters or [])
        if launch_drives:
            node = LifecycleNode(namespace='', **kwargs)
            actions.append(node)
            actions.extend(autostart_actions(node, name, previous[-1] if previous else None))
            previous.append(node)
        else:
            actions.append(Node(**kwargs))
        names.append(name)

    add('detector', 'detector_local',
        [{'backend': backend, 'weights': local_weights, 'device': device, 'input_type': 'image',
          'input_topic': local_image_topic, 'output_topic': LOCAL_TOPIC}])
    add('detection_selector', 'detection_selector')
    if offload:
        add('image_throttle', 'image_throttle', [{'input_topic': local_image_topic}])
    if with_projector:
        add('projector', 'projector')
    if with_manager:
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_optional',
            output='screen',
            # autostart true: stand-alone use. bond_timeout 0: our nodes create no bonds (as lifecycle_core.yaml).
            parameters=[{'autostart': True, 'bond_timeout': 0.0, 'service_timeout': 60.0, 'node_names': names}]))
    return actions


def _build(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    offload = arg('offload').lower()
    if offload not in ('true', 'false'):
        raise RuntimeError(f"offload must be 'true' or 'false', got {offload!r}")
    return detection_actions(offload == 'true', arg('local_weights'), arg('backend'), arg('device'),
                             arg('with_manager').lower() == 'true',
                             autostart=arg('autostart').lower() == 'true',
                             with_projector=arg('with_projector').lower() == 'true')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offload', default_value='false',
                              description='true: also send images to the cloud detector (selector falls back to local); '
                                          'false: local detector only, no image leaves the robot (009a-R01)'),
        DeclareLaunchArgument('local_weights', default_value='yolo11n.pt',
                              description='weights of the local detector (hot standby)'),
        DeclareLaunchArgument('backend', default_value='ultralytics',
                              description='ultralytics, or null (detects nothing; bring-up without weights)'),
        DeclareLaunchArgument('device', default_value='cpu', description='inference device of the local detector'),
        DeclareLaunchArgument('with_manager', default_value='true',
                              description='start lifecycle_manager_optional for these nodes'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true (with with_manager:=false): this launch configures and activates '
                                          'each node itself; ignored with the manager, which already starts them'),
        DeclareLaunchArgument('with_projector', default_value='false',
                              description='also start the projector (needs TF and /topologic_map from the caller)'),
        OpaqueFunction(function=_build),
    ])
