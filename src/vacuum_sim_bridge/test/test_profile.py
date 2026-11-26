"""The shipped YAML profile must equal the documented defaults (no hidden second source of truth).
Requirement: 001-R23."""
import os

import yaml

from vacuum_sim_bridge.profile import profile_fields

HERE = os.path.dirname(os.path.abspath(__file__))


def test_yaml_matches_defaults():
    path = os.path.join(HERE, '..', 'config', 'sim_bridge.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    defaults = {k: (list(v) if isinstance(v, tuple) else v) for k, v in profile_fields()}
    assert params.keys() == defaults.keys()
    for k, v in defaults.items():
        assert params[k] == v, k
