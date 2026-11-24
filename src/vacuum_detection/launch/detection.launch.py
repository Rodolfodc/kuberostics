"""detection.launch.py (T912, spec 009a-R01): the robot-side perception chain, local only or with the cloud.

    ros2 launch vacuum_detection detection.launch.py [offload:=true|false] [local_weights:=yolo11n.pt]
                                                     [backend:=ultralytics|null] [device:=cpu] [with_manager:=true]

  offload:=false (default)  only the local detector runs, publishing straight to /myRobot/perception/detections.
                            No image leaves the robot and there is no selector: same behaviour as spec 004 (R01).
  offload:=true             the local detector stays up as hot standby (R04) and publishes to .../detections/local;
                            image_throttle sends the compressed RGB to the cloud (R03, R11); a cloud detector
                            (K3s, T910/T911; not started here) is expected to answer on .../detections/cloud; and
                            detection_selector forwards cloud while healthy and local otherwise (R05 to R10).

The nodes are lifecycle nodes (D-26) under `lifecycle_manager_optional` (D-09): a perception failure is reported by
the manager but never stops the core (Art. VII), and with offload:=false or a dead cloud the robot keeps cleaning.
`with_manager:=false` leaves the manager out, for a launch that already has one (skeleton.launch.py with the
sequencer: then these node names go into its lifecycle_optional.yaml). The `projector` is launched by its own
description when its node lands.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCAL_TOPIC = '/myRobot/perception/detections/local'
OUTPUT_TOPIC = '/myRobot/perception/detections'


def detection_actions(offload: bool, local_weights: str, backend: str, device: str, with_manager: bool,
                      local_image_topic: str = '/camera'):
    """The actions of the chain for one value of `offload` (pure function of its arguments, testable)."""
    detector = Node(
        package='vacuum_detection', executable='detector', name='detector_local', output='screen',
        parameters=[{'backend': backend, 'weights': local_weights, 'device': device, 'input_type': 'image',
                     'input_topic': local_image_topic,
                     'output_topic': LOCAL_TOPIC if offload else OUTPUT_TOPIC}])
    names = ['detector_local']
    actions = [detector]
    if offload:
        actions.append(Node(package='vacuum_detection', executable='image_throttle', name='image_throttle',
                            output='screen', parameters=[{'input_topic': local_image_topic}]))
        actions.append(Node(package='vacuum_detection', executable='detection_selector',
                            name='detection_selector', output='screen'))
        names += ['image_throttle', 'detection_selector']
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
                             arg('with_manager').lower() == 'true')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offload', default_value='false',
                              description='true: cloud detector expected, local fallback via detection_selector; '
                                          'false: local detector only (009a-R01)'),
        DeclareLaunchArgument('local_weights', default_value='yolo11n.pt',
                              description='weights of the local detector (hot standby)'),
        DeclareLaunchArgument('backend', default_value='ultralytics',
                              description='ultralytics, or null (detects nothing; bring-up without weights)'),
        DeclareLaunchArgument('device', default_value='cpu', description='inference device of the local detector'),
        DeclareLaunchArgument('with_manager', default_value='true',
                              description='start lifecycle_manager_optional for these nodes'),
        OpaqueFunction(function=_build),
    ])
