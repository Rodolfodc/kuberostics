"""Runs the optional exploration_server alone (spec 003, Art. III.1). NOT part of any default launch: autonomous
exploration is optional and late (teleoperated mapping with `map_file` is the default).

Its inputs come from elsewhere: /map (slam_toolbox or map_server), TF map -> base_link and the Nav2 action
/navigate_to_pose. Lifecycle transitions are done by `lifecycle_manager_core` in a full bring-up;
`autostart:=true` (default here) does configure + activate so the module runs alone.
`map_loaded:=true` is what the mapping launch passes with `map_file` (003-R06): `explored` is then true.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    node = LifecycleNode(
        package='vacuum_mapping', executable='exploration_server', name='exploration_server', namespace='',
        output='screen',
        parameters=[{'map_loaded': ParameterValue(LaunchConfiguration('map_loaded'), value_type=bool)}])
    autostart = LaunchConfiguration('autostart')
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE),
        condition=IfCondition(autostart))
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map_loaded', default_value='false'),
        node,
        RegisterEventHandler(OnProcessStart(target_action=node, on_start=[configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
            entities=[activate])),
    ])
