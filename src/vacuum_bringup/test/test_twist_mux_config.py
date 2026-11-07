"""T007: twist_mux configuration tests, checked against the contract (topics.yaml).

No ROS and no twist_mux needed: they validate the YAML file. The behaviour of the running mux
(001-R10, 001-R11) is verified by launch_testing in T014.
"""
import pathlib

import pytest
import yaml

HERE = pathlib.Path(__file__).resolve().parent
CONFIG = HERE.parent / 'config' / 'twist_mux.yaml'
CONTRACT = next(p for p in HERE.parents if (p / 'specs').is_dir()) / 'specs' / '000-system' / 'contracts' / 'topics.yaml'


@pytest.fixture(scope='module')
def params():
    return yaml.safe_load(CONFIG.read_text())['twist_mux']['ros__parameters']


@pytest.fixture(scope='module')
def contract():
    return yaml.safe_load(CONTRACT.read_text())


def mux_inputs(contract):
    return {t['name']: t for t in contract['topics']
            if t['name'].startswith('/myRobot/mux/') and 'priority' in t and t.get('since') == '001'}


def test_001_R18_use_stamped_is_explicitly_false(params):
    assert params['use_stamped'] is False


def test_001_R11_inputs_match_the_contract_names_and_priorities(params, contract):
    configured = {v['topic']: v['priority'] for v in params['topics'].values()}
    expected = {name: t['priority'] for name, t in mux_inputs(contract).items()}
    assert configured == expected


def test_001_R11_teleop_has_the_highest_input_priority(params):
    prio = {k: v['priority'] for k, v in params['topics'].items()}
    assert prio['teleop'] == max(prio.values())


def test_001_R11_every_input_has_a_positive_timeout(params):
    assert all(v['timeout'] > 0 for v in params['topics'].values())


def test_001_R10_stop_lock_uses_the_contract_topic_and_beats_every_input(params, contract):
    lock = params['locks']['stop']
    assert lock['topic'] == '/myRobot/mux/lock_stop'
    assert lock['priority'] > max(v['priority'] for v in params['topics'].values())
    assert lock['topic'] in {t['name'] for t in contract['topics']}


def test_001_R10_stop_lock_has_no_timeout(params):
    assert params['locks']['stop']['timeout'] == 0.0
