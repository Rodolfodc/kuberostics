"""Runs the real coverage_server alone (Art. III.1, T513): the node and nothing else.

Its inputs come from elsewhere: /map (map_server or slam_toolbox), TF map -> base_link and the Nav2 action
/navigate_to_pose, or the fakes of the launch test. The lifecycle transitions are done by
`lifecycle_manager_core` in the full bring-up; `autostart:=true` (default here) does configure + activate so
the module runs alone.

`sim_slowdown:=<factor>` (default 1.0, real time: no parameter is overridden) multiplies the wall-clock timeouts
`waypoint_timeout_s`, `coverage_timeout_s` and `resume_window_s` for a slow simulator (about 0.09x real time with
the laser model, use_sim_time stays false). Example: `sim_slowdown:=11` makes the 120 s waypoint timeout 1320 s.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

from vacuum_coverage.core.slowdown import scaled_timeout_params


def _actions(context, *args, **kwargs):
    overrides = scaled_timeout_params(LaunchConfiguration('sim_slowdown').perform(context))
    node = LifecycleNode(package='vacuum_coverage', executable='coverage_server', name='coverage_server',
                         namespace='', output='screen', parameters=[overrides] if overrides else [])
    autostart = LaunchConfiguration('autostart')
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE),
        condition=IfCondition(autostart))
    return [
        node,
        RegisterEventHandler(OnProcessStart(target_action=node, on_start=[configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
            entities=[activate])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('sim_slowdown', default_value='1.0',
                              description='factor > 0 that scales the wall-clock timeouts (1.0 = real time)'),
        OpaqueFunction(function=_actions),
    ])
