"""T912: 009a-R01 / R04 / R03. The launch description is built for both values of `offload` (never launched here);
node names, parameters and the manager list are checked. Also builds the real description through the
`offload` argument, so the argument plumbing is covered."""
import importlib.util
import pathlib

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext                                   # noqa: E402
from launch.actions import SetLaunchConfiguration                  # noqa: E402
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions   # noqa: E402
from launch_ros.actions import Node                                # noqa: E402

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'detection.launch.py'


def load():
    spec = importlib.util.spec_from_file_location('detection_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve(value):
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(value))


def by_name(actions):
    return {resolve(a._Node__node_name): a for a in actions if isinstance(a, Node)}


def plain(value):
    """Resolves the substitutions launch_ros leaves in a parameter value (tuples of substitutions, nested)."""
    if isinstance(value, tuple) and value and not isinstance(value[0], (str, int, float, bool, list, tuple)):
        return yaml.safe_load(resolve(value))   # launch_ros encodes each value as a YAML document
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def params(node):
    merged = {}
    for d in node._Node__parameters:
        for k, v in d.items():
            merged[plain(k) if not isinstance(k, str) else k] = plain(v)
    return {(k[0] if isinstance(k, list) else k): v for k, v in merged.items()}


def build(offload, **kw):
    args = dict(local_weights='yolo11n.pt', backend='null', device='cpu', with_manager=True)
    args.update(kw)
    return by_name(load().detection_actions(offload, **args))


def test_009a_R01_offload_false_runs_only_the_local_detector_straight_to_the_output():
    nodes = build(False)
    assert set(nodes) == {'detector_local', 'lifecycle_manager_optional'}
    assert params(nodes['detector_local'])['output_topic'] == '/myRobot/perception/detections'
    assert params(nodes['lifecycle_manager_optional'])['node_names'] == ['detector_local']


def test_009a_R03_offload_false_sends_no_image_anywhere():
    assert 'image_throttle' not in build(False)


def test_009a_R01_R04_offload_true_keeps_the_local_detector_and_adds_throttle_and_selector():
    nodes = build(True)
    assert set(nodes) == {'detector_local', 'image_throttle', 'detection_selector', 'lifecycle_manager_optional'}
    assert params(nodes['detector_local'])['output_topic'] == '/myRobot/perception/detections/local'
    assert params(nodes['lifecycle_manager_optional'])['node_names'] == [
        'detector_local', 'image_throttle', 'detection_selector']


def test_009a_R01_local_detector_parameters_come_from_the_launch_arguments():
    p = params(build(True, local_weights='/w/custom.pt', backend='ultralytics', device='cuda:0')['detector_local'])
    assert (p['weights'], p['backend'], p['device']) == ('/w/custom.pt', 'ultralytics', 'cuda:0')


def test_009a_R01_without_manager_no_manager_is_started():
    assert 'lifecycle_manager_optional' not in build(True, with_manager=False)


@pytest.mark.parametrize('value,expected', [('true', {'image_throttle', 'detection_selector'}), ('false', set())])
def test_009a_R01_the_offload_argument_selects_the_chain(value, expected):
    module = load()
    ctx = LaunchContext()
    for name, default in (('offload', value), ('local_weights', 'yolo11n.pt'), ('backend', 'null'),
                          ('device', 'cpu'), ('with_manager', 'true')):
        SetLaunchConfiguration(name, default).visit(ctx)
    names = set(by_name(module._build(ctx)))
    assert expected <= names and (names & {'image_throttle', 'detection_selector'}) == expected


def test_009a_R01_a_bad_offload_value_is_rejected():
    module = load()
    ctx = LaunchContext()
    for name, default in (('offload', 'maybe'), ('local_weights', 'x'), ('backend', 'null'), ('device', 'cpu'),
                          ('with_manager', 'true')):
        SetLaunchConfiguration(name, default).visit(ctx)
    with pytest.raises(RuntimeError, match='offload'):
        module._build(ctx)
