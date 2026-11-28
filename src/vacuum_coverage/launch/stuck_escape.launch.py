"""Runs the real stuck_detector and escape_server alone (Art. III.1, 005-R28, R33): the two nodes and nothing else.

Their inputs come from elsewhere: /myRobot/cmd_vel (twist_mux), /myrobot/odom and /scan (the sim bridge) and TF
map -> base_link, or the fakes of the launch test. `escape_server` runs with `outcome: auto` (the script; the stub
uses `succeed`, 005-R33). `autostart:=true` (default here) does configure + activate so the pair runs alone; with
`autostart:=false` the nodes stay unconfigured for `lifecycle_manager_core` (skeleton-compatible: same node names,
same actions, topics and services as the stubs of vacuum_stubs).
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
from lifecycle_msgs.msg import Transition


def _autostart(node, autostart):
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE),
        condition=IfCondition(autostart))
    return [
        RegisterEventHandler(OnProcessStart(target_action=node, on_start=[configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state='configuring', goal_state='inactive', entities=[activate])),
    ]


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    stuck = LifecycleNode(package='vacuum_coverage', executable='stuck_detector', name='stuck_detector',
                          namespace='', output='screen')
    escape = LifecycleNode(package='vacuum_coverage', executable='escape_server', name='escape_server',
                           namespace='', output='screen', parameters=[{'outcome': 'auto'}])
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        stuck, escape,
        *_autostart(stuck, autostart), *_autostart(escape, autostart),
    ])
