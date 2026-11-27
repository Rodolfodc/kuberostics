"""009a-R01 / R03: structure of detection.launch.py for both values of `offload`, through public attributes only
(the launch is never started here; the live graph is test_launch_offload_false.py / test_launch_offload_true.py)."""
import importlib.util
import pathlib

import pytest

pytest.importorskip('launch_ros')
from launch import LaunchContext                                   # noqa: E402
from launch.actions import SetLaunchConfiguration                  # noqa: E402
from launch_ros.actions import Node                                # noqa: E402

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'detection.launch.py'


def load():
    spec = importlib.util.spec_from_file_location('detection_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def executables(offload, **kw):
    args = dict(local_weights='yolo11n.pt', backend='null', device='cpu', with_manager=True)
    args.update(kw)
    actions = load().detection_actions(offload, **args)
    return sorted(a.node_executable for a in actions if isinstance(a, Node))


def test_009a_R01_offload_false_runs_local_detector_and_selector_without_throttle():
    # only detection_selector may publish the final topic (contract max_publishers 1)
    assert executables(False) == ['detection_selector', 'detector', 'lifecycle_manager']


def test_009a_R03_offload_false_sends_no_image_anywhere():
    assert 'image_throttle' not in executables(False)


def test_009a_R01_R04_offload_true_adds_the_throttle_and_keeps_the_local_detector():
    assert executables(True) == ['detection_selector', 'detector', 'image_throttle', 'lifecycle_manager']


def test_009a_R01_without_manager_no_manager_is_started():
    assert 'lifecycle_manager' not in executables(True, with_manager=False)


def build_with(offload):
    ctx = LaunchContext()
    for name, value in (('offload', offload), ('local_weights', 'yolo11n.pt'), ('backend', 'null'),
                        ('device', 'cpu'), ('with_manager', 'true'), ('autostart', 'false'),
                        ('with_projector', 'false')):
        SetLaunchConfiguration(name, value).visit(ctx)
    return load()._build(ctx)


@pytest.mark.parametrize('value,throttle', [('true', True), ('false', False)])
def test_009a_R01_the_offload_argument_selects_the_chain(value, throttle):
    found = {a.node_executable for a in build_with(value) if isinstance(a, Node)}
    assert ('image_throttle' in found) is throttle and 'detection_selector' in found


def test_009a_R01_a_bad_offload_value_is_rejected():
    with pytest.raises(RuntimeError, match='offload'):
        build_with('maybe')


# --- autostart and with_projector (this launch drives configure + activate when there is no manager) ---------
def actions_of(**kw):
    args = dict(local_weights='yolo11n.pt', backend='null', device='cpu', with_manager=False, offload=True)
    args.update(kw)
    return load().detection_actions(args.pop('offload', True), **args)


def test_009a_R01_autostart_is_off_by_default_so_the_nodes_are_plain_nodes():
    from launch_ros.actions import LifecycleNode
    actions = actions_of(offload=True)
    assert not any(isinstance(a, LifecycleNode) for a in actions)
    assert [type(a).__name__ for a in actions if 'RegisterEventHandler' in type(a).__name__] == []


def test_009a_R01_autostart_without_manager_registers_configure_and_activate_for_every_node():
    from launch_ros.actions import LifecycleNode
    actions = actions_of(offload=True, autostart=True, with_projector=True)
    nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert sorted(n.node_executable for n in nodes) == ['detection_selector', 'detector', 'image_throttle', 'projector']
    handlers = [a for a in actions if type(a).__name__ == 'RegisterEventHandler']
    assert len(handlers) == 2 * len(nodes)                  # one configure (process start), one activate (inactive)
    assert 'lifecycle_manager' not in executables_of(actions)


def test_009a_R01_autostart_with_the_manager_is_ignored_and_the_manager_stays_the_only_driver():
    from launch.actions import LogInfo
    from launch_ros.actions import LifecycleNode
    actions = actions_of(offload=False, autostart=True, with_manager=True)
    assert not any(isinstance(a, LifecycleNode) for a in actions)
    assert any(isinstance(a, LogInfo) for a in actions)
    assert 'lifecycle_manager' in executables_of(actions)


def test_009a_R13_with_projector_adds_the_projector_only_when_asked():
    assert 'projector' not in executables(False)
    assert 'projector' in executables_of(actions_of(offload=False, with_projector=True))


def executables_of(actions):
    return sorted(a.node_executable for a in actions if isinstance(a, Node))


def test_001_R01_projector_chain_does_not_depend_on_detector_local():
    """Art. VII / D-09: a failing detector_local must not stop the projector, and the others keep their order."""
    plan = load().autostart_plan(['detector_local', 'detection_selector', 'image_throttle', 'projector'])
    assert plan == {'detector_local': None, 'detection_selector': 'detector_local',
                    'image_throttle': 'detection_selector', 'projector': None}


def test_001_R01_projector_handlers_are_wired_to_its_own_process_start_in_the_action_list():
    from launch.event_handlers import OnProcessStart
    actions = actions_of(offload=True, autostart=True, with_projector=True)
    starts = [a for a in actions if type(a).__name__ == 'RegisterEventHandler'
              and isinstance(getattr(a, 'event_handler', None), OnProcessStart)]
    assert len(starts) == 2                    # detector_local and projector are roots; the others wait
