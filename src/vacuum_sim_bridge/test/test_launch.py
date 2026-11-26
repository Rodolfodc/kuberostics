"""The launch file replaces the stock bridges by name (Art. II, D-06): same executables, cmd_vel remapped.
Requirement: 001-R23."""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def load():
    spec = importlib.util.spec_from_file_location('sim_bridges', os.path.join(HERE, '..', 'launch',
                                                                              'sim_bridges.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


def test_launch_starts_the_four_same_name_bridges():
    from launch_ros.actions import Node
    nodes = [e for e in load().entities if isinstance(e, Node)]
    assert sorted(n.node_executable for n in nodes) == [
        'my_robot_cmd_vel_node', 'my_robot_odometry_node', 'my_robot_scan_node', 'my_robot_vision_node']
