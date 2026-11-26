"""Runs the real coverage_server alone (Art. III.1, T513): the node and nothing else.

Its inputs come from elsewhere: /map (map_server or slam_toolbox), TF map -> base_link and the Nav2 action
/navigate_to_pose, or the fakes of the launch test. The lifecycle transitions are done by
`lifecycle_manager_core` in the full bring-up; `autostart:=true` (default here) does configure + activate so
the module runs alone.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    node = LifecycleNode(package='vacuum_coverage', executable='coverage_server', name='coverage_server',
                         namespace='', output='screen')
    autostart = LaunchConfiguration('autostart')
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE),
        condition=IfCondition(autostart))
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        node,
        RegisterEventHandler(OnProcessStart(target_action=node, on_start=[configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
            entities=[activate])),
    ])
