"""Node classification test: 005-R39 (and the static part of 005-R35): every node of skeleton.launch.py that is not on the plain list
(config/node_classification.yaml, the copy of docs/lifecycle-managers.md) must be a lifecycle node named in the
node_names of a lifecycle manager config. The launch description is built, never launched.
"""
import importlib.util
import pathlib

import pytest
import yaml
from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

PKG = pathlib.Path(__file__).resolve().parent.parent
CONFIG = PKG / 'config'
MANAGER_CONFIGS = {
    'lifecycle_manager_core': 'lifecycle_core.yaml',
    'lifecycle_manager_nav': 'lifecycle_nav.yaml',
    'lifecycle_manager_optional': 'lifecycle_optional.yaml',
}


def load(name):
    return yaml.safe_load((CONFIG / name).read_text())


def launched_node_names():
    spec = importlib.util.spec_from_file_location('skeleton_launch', PKG / 'launch' / 'skeleton.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, Node):
            raw = getattr(entity, '_Node__node_name')
            names.append(perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(raw)))
    return names


def managed_names():
    """Lifecycle nodes under our managers (names in node_names) plus the IA368II ones (third party)."""
    names = set(load('node_classification.yaml')['third_party_managed'])
    for manager, filename in MANAGER_CONFIGS.items():
        names |= set(load(filename)[manager]['ros__parameters']['node_names'])
    return names


def unclassified(names, plain, managed):
    return [n for n in names if n not in plain and n not in managed]


def test_005_R39_every_launched_node_is_on_the_plain_list_or_is_managed():
    plain = load('node_classification.yaml')['plain']
    offenders = unclassified(launched_node_names(), plain, managed_names())
    assert offenders == [], f'nodes neither plain-listed (with a reason) nor managed by a lifecycle manager: {offenders}'


def test_005_R39_the_check_catches_an_unmanaged_node():
    assert unclassified(['stop_guard', 'rogue_node'], {}, {'stop_guard'}) == ['rogue_node']


def test_005_R39_every_plain_entry_has_a_reason_and_a_known_kind():
    for name, entry in load('node_classification.yaml')['plain'].items():
        assert entry['kind'] in ('plain', 'manager'), name
        assert entry['reason'].strip(), f'{name} has no reason (D-26)'


def test_005_R39_a_lifecycle_node_is_never_on_the_plain_list():
    plain = set(load('node_classification.yaml')['plain'])
    assert plain.isdisjoint(managed_names())


@pytest.mark.parametrize('manager,filename', sorted(MANAGER_CONFIGS.items()))
def test_005_R35_manager_configs_exist_and_only_the_core_autostarts_alone(manager, filename):
    params = load(filename)[manager]['ros__parameters']
    assert params['bond_timeout'] == 0.0
    if manager != 'lifecycle_manager_core':
        assert params['autostart'] is False      # the sequencer starts them, after the previous one is active


def test_005_R35_the_sequencer_parameters_match_the_manager_roles():
    from vacuum_bringup.manager_sequencer import MANAGER_ROLES
    params = load('lifecycle_sequencer.yaml')['lifecycle_sequencer']['ros__parameters']
    assert params['managers'] == ['lifecycle_manager_core', 'lifecycle_manager_nav', 'lifecycle_manager_optional']
    assert set(params['managers']) <= set(MANAGER_ROLES)
    assert params['manager_startup_timeout_s'] == 30.0
    assert params['manager_shutdown_timeout_s'] == 15.0
