"""Tests of the learned-escape toolbox (spec 008, D-31): observation builder, action table, inference, load gate.

No ROS. Each test cites the requirement it verifies."""
import hashlib
import json
import math
import os

import numpy as np
import pytest

from vacuum_coverage.core import escape_policy as ep
from vacuum_coverage.core.escape_policy import (ACTIONS, LoadGate, ObservationBuilder, PolicyError,
                                                PolicyInference)

DATA = os.path.join(os.path.dirname(__file__), 'data')


def weights_dict(seed=0, hidden=6, n_obs=43, n_actions=8, **over):
    rng = np.random.default_rng(seed)
    d = {
        'format': 'vacuum-escape-policy/1', 'obs_version': 1, 'n_obs': n_obs, 'n_actions': n_actions,
        'action_table_version': 1, 'range_cap_m': 2.0, 'n_sectors': 30, 'run_id': 'synthetic',
        'git_sha': '0' * 40, 'config_hash': 'abc',
        'layers': [
            {'W': rng.normal(size=(hidden, n_obs)).tolist(), 'b': rng.normal(size=hidden).tolist(),
             'activation': 'tanh'},
            {'W': rng.normal(size=(n_actions, hidden)).tolist(), 'b': rng.normal(size=n_actions).tolist(),
             'activation': 'linear'},
        ],
    }
    d.update(over)
    return d


def write(tmp_path, d, sidecar=True, name='w.json'):
    p = tmp_path / name
    p.write_text(json.dumps(d))
    if sidecar:
        (tmp_path / (name + '.sha256')).write_text(hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + name + '\n')
    return str(p)


def wall(n=452, r=0.3):
    return [r] * n


# ---- action table (008-R09) ----

def test_008_R09_action_table_v1():
    v, w = ep.REVERSE_SPEED_M_S, ep.TURN_SPEED_RAD_S
    assert (v, w) == (0.15, 1.0)
    assert ACTIONS == ((v, 0.0), (-v, 0.0), (0.0, w), (0.0, -w), (v, w / 2), (v, -w / 2), (-v, w / 2), (-v, -w / 2))
    assert ep.ACTION_TABLE_VERSION == 1 and len(ACTIONS) == ep.N_ACTIONS == 8
    assert ep.PUBLISH_RATE_HZ == 10.0


def test_008_R09_command_for_action():
    assert ep.command_for_action(1) == (-0.15, 0.0)
    for bad in (-1, 8, 1.5, True, None):
        with pytest.raises(PolicyError):
            ep.command_for_action(bad)


# ---- observation (008-R08) ----

def test_008_R08_layout_and_size():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0.0, 0.0, 0.0))
    obs = b.build(wall(), (0.0, 0.0, 0.0), 0.0, None)
    assert len(obs) == 43 == ep.N_OBS
    assert all(isinstance(x, float) and 0.0 <= x <= 1.0 for x in obs)
    # no motion: dx=dy=0 -> 0.5, dyaw=0 -> sin 0.5, cos 1.0
    assert obs[38:] == pytest.approx([0.0, 0.5, 0.5, 0.5, 1.0])
    assert obs[30:38] == [0.0] * 8


def test_008_R08_sector_minimum_and_cap():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    r = [float('inf')] * 452
    r[0] = 0.5                       # sector 0 (beams 0..14)
    r[15] = 1.0                      # sector 1 (beams 15..30)
    obs = b.build(r, (0, 0, 0), 0.0, None)
    assert obs[0] == pytest.approx(0.25) and obs[1] == pytest.approx(0.5)
    assert obs[2] == 1.0             # inf -> cap -> 1.0
    r2 = [5.0] * 452                 # beyond the cap -> clipped
    assert b.build(r2, (0, 0, 0), 0.0, None)[:30] == [1.0] * 30


def test_008_R08_sector_boundaries_match_spec_formula():
    n = 452
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    for i in range(30):
        lo, hi = (i * n) // 30, ((i + 1) * n) // 30 - 1
        for beam, expect_sector in ((lo, i), (hi, i)):
            r = [2.0] * n
            r[beam] = 0.2
            obs = b.build(r, (0, 0, 0), 0.0, None)
            assert [k for k in range(30) if obs[k] < 1.0] == [expect_sector]


def test_008_R08_nan_and_nonpositive_dropped_inf_kept():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    r = [0.8] * 452
    r[0], r[1], r[2] = float('nan'), 0.0, -1.0       # dropped: min of the rest of sector 0 is 0.8
    assert b.build(r, (0, 0, 0), 0.0, None)[0] == pytest.approx(0.4)
    assert b.build([float('nan')] * 452, (0, 0, 0), 0.0, None)[:30] == [1.0] * 30   # all dropped: empty
    assert b.build([0.0] * 452, (0, 0, 0), 0.0, None)[:30] == [1.0] * 30
    assert b.build([float('inf')] * 452, (0, 0, 0), 0.0, None)[:30] == [1.0] * 30


def test_008_R08_scan_with_fewer_beams_than_sectors_gives_empty_sectors():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    obs = b.build([0.4] * 10, (0, 0, 0), 0.0, None)
    assert len(obs) == 43
    assert obs[:30].count(1.0) == 20 and obs[:30].count(pytest.approx(0.2)) == 10
    assert b.build([], (0, 0, 0), 0.0, None)[:30] == [1.0] * 30


def test_008_R08_previous_action_one_hot():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    obs = b.build(wall(), (0, 0, 0), 0.0, 3)
    assert obs[30:38] == [0, 0, 0, 1.0, 0, 0, 0, 0]
    for bad in (8, -1, 2.0):
        with pytest.raises(PolicyError):
            b.build(wall(), (0, 0, 0), 0.0, bad)


def test_008_R08_elapsed_time_clipped():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((0, 0, 0))
    g = lambda t: b.build(wall(), (0, 0, 0), t, None)[38]
    assert g(0.0) == 0.0 and g(2.5) == 0.5 and g(9.0) == 1.0 and g(-1.0) == 0.0


def test_008_R08_odometry_in_start_frame():
    b = ObservationBuilder(budget_s=5.0)
    b.reset((1.0, 2.0, math.pi / 2))              # start heading +y
    # moved 0.25 m along the start heading (world +y): start-frame dx = 0.25, dy = 0
    obs = b.build(wall(), (1.0, 2.25, math.pi / 2), 0.0, None)
    assert obs[39] == pytest.approx((0.25 / 0.5 + 1) / 2) and obs[40] == pytest.approx(0.5)
    # moved 0.1 m to the start's left (world -x): start-frame dy = +0.1
    obs = b.build(wall(), (0.9, 2.0, math.pi / 2), 0.0, None)
    assert obs[39] == pytest.approx(0.5) and obs[40] == pytest.approx((0.1 / 0.5 + 1) / 2)
    # clipped at +-0.5 m
    obs = b.build(wall(), (1.0, 5.0, math.pi / 2), 0.0, None)
    assert obs[39] == 1.0
    obs = b.build(wall(), (1.0, -5.0, math.pi / 2), 0.0, None)
    assert obs[39] == 0.0
    # yaw change of +90 deg: sin = 1 -> 1.0, cos = 0 -> 0.5; wraps across +-pi
    obs = b.build(wall(), (1.0, 2.0, math.pi), 0.0, None)
    assert obs[41] == pytest.approx(1.0) and obs[42] == pytest.approx(0.5)
    b.reset((0, 0, math.pi - 0.1))
    obs = b.build(wall(), (0, 0, -math.pi + 0.1), 0.0, None)
    assert obs[41] == pytest.approx((math.sin(0.2) + 1) / 2) and obs[42] == pytest.approx((math.cos(0.2) + 1) / 2)


def test_008_R08_requires_reset_and_valid_arguments():
    b = ObservationBuilder(budget_s=5.0)
    with pytest.raises(PolicyError):
        b.build(wall(), (0, 0, 0), 0.0, None)
    for bad in ((0, 0), (0, 0, float('nan')), (float('inf'), 0, 0)):
        with pytest.raises(PolicyError):
            b.reset(bad)
    b.reset((0, 0, 0))
    with pytest.raises(PolicyError):
        b.build(wall(), (0, 0), 0.0, None)
    with pytest.raises(PolicyError):
        b.build(wall(), (0, 0, 0), float('nan'), None)
    for kw in ({'budget_s': 0.0}, {'budget_s': 5.0, 'n_sectors': 0}, {'budget_s': 5.0, 'range_cap_m': -1.0},
               {'budget_s': 5.0, 'n_actions': 0}):
        with pytest.raises(PolicyError):
            ObservationBuilder(**kw)


def test_008_R08_other_sector_counts_change_size():
    b = ObservationBuilder(n_sectors=10, budget_s=5.0)
    b.reset((0, 0, 0))
    assert len(b.build(wall(), (0, 0, 0), 0.0, None)) == 10 + 8 + 5 == b.n_obs


# ---- inference (008-R13) ----

def reference_logits(d, obs):
    """Independent pure-Python forward pass (no numpy) in float64."""
    x = list(obs)
    for layer in d['layers']:
        x = [sum(w * v for w, v in zip(row, x)) + b for row, b in zip(layer['W'], layer['b'])]
        if layer['activation'] == 'tanh':
            x = [math.tanh(v) for v in x]
    return x


def test_008_R13_load_and_logits_match_reference(tmp_path):
    d = weights_dict(seed=3)
    p = PolicyInference()
    assert p.load(write(tmp_path, d)) == (True, '')
    assert p.loaded and p.run_id == 'synthetic' and len(p.sha256) == 64
    assert p.range_cap_m == 2.0 and p.n_sectors == 30
    rng = np.random.default_rng(1)
    for _ in range(20):
        obs = rng.random(43).tolist()
        got = p.logits(obs)
        assert len(got) == 8 and got == pytest.approx(reference_logits(d, obs), abs=1e-4)
        assert p.act(obs) == int(np.argmax(reference_logits(d, obs)))


def test_008_R13_float32_computation(tmp_path):
    p = PolicyInference()
    p.load(write(tmp_path, weights_dict()))
    assert p.layers[0][0].dtype == np.float32


def test_008_R13_ties_go_to_lowest_index():
    p = PolicyInference.stub(action=5)
    assert p.act([0.0] * 43) == 5
    d = weights_dict()
    d['layers'] = [{'W': [[0.0] * 43] * 8, 'b': [1.0] * 8, 'activation': 'linear'}]
    p2 = PolicyInference()
    assert p2.load_dict(d) == (True, '')
    assert p2.act([0.3] * 43) == 0


def test_008_R13_deterministic(tmp_path):
    p = PolicyInference()
    p.load(write(tmp_path, weights_dict(seed=9)))
    obs = np.random.default_rng(2).random(43).tolist()
    assert [p.act(obs) for _ in range(5)] == [p.act(obs)] * 5
    assert p.logits(obs) == p.logits(obs)


def test_008_R13_rejects_bad_fields(tmp_path):
    cases = {
        'format': {'format': 'other/1'}, 'obs_version': {'obs_version': 2}, 'n_obs': {'n_obs': 44},
        'n_actions': {'n_actions': 7}, 'action_table_version': {'action_table_version': 2},
        'range_cap': {'range_cap_m': 3.0}, 'n_sectors': {'n_sectors': 20},
    }
    for name, over in cases.items():
        p = PolicyInference()
        ok, reason = p.load(write(tmp_path, weights_dict(**over), name=name + '.json'))
        assert not ok and not p.loaded and reason, name
    assert 'format' in PolicyInference().load(write(tmp_path, weights_dict(format='x'), name='f.json'))[1]


def test_008_R13_rejects_bad_shapes(tmp_path):
    d = weights_dict()
    bad = []
    d1 = json.loads(json.dumps(d)); d1['layers'][0]['W'] = d1['layers'][0]['W'][:-1]; bad.append(d1)       # b/W mismatch
    d2 = json.loads(json.dumps(d)); d2['layers'][1]['W'] = [r[:-1] for r in d2['layers'][1]['W']]; bad.append(d2)
    d3 = json.loads(json.dumps(d)); d3['layers'][0]['W'][0] = d3['layers'][0]['W'][0][:-1]; bad.append(d3)  # ragged
    d4 = json.loads(json.dumps(d)); d4['layers'] = []; bad.append(d4)
    d5 = json.loads(json.dumps(d)); d5['layers'][1]['activation'] = 'relu'; bad.append(d5)
    d6 = json.loads(json.dumps(d)); d6['layers'][1]['W'] = d6['layers'][1]['W'][:-1]
    d6['layers'][1]['b'] = d6['layers'][1]['b'][:-1]; bad.append(d6)                                         # 7 outputs
    d7 = json.loads(json.dumps(d)); d7['layers'][0]['W'] = [r[:-1] for r in d7['layers'][0]['W']]; bad.append(d7)
    d8 = json.loads(json.dumps(d)); d8['layers'] = 'nope'; bad.append(d8)
    d9 = json.loads(json.dumps(d)); d9['layers'][0] = 3; bad.append(d9)
    d10 = json.loads(json.dumps(d)); d10['layers'][0]['W'] = 'x'; bad.append(d10)
    d11 = json.loads(json.dumps(d)); d11['layers'][0]['b'] = [[1.0]] * 6; bad.append(d11)
    d12 = json.loads(json.dumps(d)); del d12['layers'][0]['b']; bad.append(d12)
    for i, dd in enumerate(bad):
        ok, reason = PolicyInference().load(write(tmp_path, dd, name='s%d.json' % i))
        assert not ok and reason, i


def test_008_R13_rejects_non_finite(tmp_path):
    for val in (float('nan'), float('inf')):
        d = weights_dict()
        d['layers'][0]['W'][0][0] = val
        p = PolicyInference()
        ok, reason = p.load(write(tmp_path, d, name='nf.json'))
        assert not ok and 'finite' in reason
    d = weights_dict()
    d['layers'][1]['b'][0] = None
    assert not PolicyInference().load(write(tmp_path, d, name='none.json'))[0]


def test_008_R13_sha256_sidecar(tmp_path):
    path = write(tmp_path, weights_dict())
    # mismatch
    (tmp_path / 'w.json.sha256').write_text('0' * 64 + '\n')
    p = PolicyInference()
    ok, reason = p.load(path)
    assert not ok and 'sha256' in reason
    # malformed sidecar
    (tmp_path / 'w.json.sha256').write_text('not-a-hash\n')
    assert not p.load(path)[0]
    # missing sidecar is rejected (assumption A2) ...
    os.remove(tmp_path / 'w.json.sha256')
    ok, reason = p.load(path)
    assert not ok and 'sidecar' in reason
    # ... unless explicitly allowed
    assert p.load(path, require_sidecar=False) == (True, '')
    # modification after the sidecar was written
    path2 = write(tmp_path, weights_dict(), name='m.json')
    with open(path2, 'a') as f:
        f.write(' ')
    assert not p.load(path2)[0]


def test_008_R13_rejects_unreadable_and_not_json(tmp_path):
    p = PolicyInference()
    assert not p.load(str(tmp_path / 'missing.json'))[0]
    assert not p.load('')[0]
    q = tmp_path / 'bad.json'
    q.write_text('{not json')
    assert not p.load(str(q), require_sidecar=False)[0]
    q.write_text('[1, 2]')
    assert not p.load(str(q), require_sidecar=False)[0]
    q.write_bytes(b'\xff\xfe')
    assert not p.load(str(q), require_sidecar=False)[0]


def test_008_R13_expected_range_cap(tmp_path):
    path = write(tmp_path, weights_dict())
    p = PolicyInference()
    assert p.load(path, expected_range_cap_m=2.0)[0]
    d3 = write(tmp_path, weights_dict(range_cap_m=3.0), name='c3.json')
    assert not p.load(d3)[0] and p.load(d3, expected_range_cap_m=None)[0]
    assert p.range_cap_m == 3.0
    for bad in ('x', -1.0, True, float('inf')):
        assert not p.load_dict(weights_dict(range_cap_m=bad), expected_range_cap_m=None)[0]
    assert not p.load_dict(weights_dict(n_sectors=True))[0] and not p.load_dict([1])[0]
    ok, reason = p.load(path, expected_range_cap_m=1.5)
    assert not ok and 'range_cap_m' in reason


def test_008_R15_rejected_load_keeps_previous_weights(tmp_path):
    p = PolicyInference()
    assert p.load(write(tmp_path, weights_dict(seed=1)))[0]
    sha = p.sha256
    assert not p.load(write(tmp_path, weights_dict(format='x'), name='bad.json'))[0]
    assert p.loaded and p.sha256 == sha


def test_008_R17_logits_errors(tmp_path):
    with pytest.raises(PolicyError):
        PolicyInference().act([0.0] * 43)                        # not loaded
    p = PolicyInference()
    p.load(write(tmp_path, weights_dict()))
    with pytest.raises(PolicyError):
        p.act([0.0] * 42)                                         # wrong size
    with pytest.raises(PolicyError):
        p.act([float('nan')] + [0.0] * 42)                        # non-finite input -> non-finite logit
    big = PolicyInference()
    d = weights_dict()
    d['layers'] = [{'W': [[1e38] * 43] * 8, 'b': [0.0] * 8, 'activation': 'linear'}]
    assert big.load_dict(d)[0]
    with pytest.raises(PolicyError):
        big.act([1.0] * 43)                                       # overflows float32


def test_008_R13_stub_is_weights_free_and_valid():
    p = PolicyInference.stub()
    assert p.loaded and p.is_stub and p.run_id == 'stub' and p.sha256 == 'stub'
    assert p.act([0.5] * 43) == 1
    assert PolicyInference.stub(action=2).act([0.0] * 43) == 2
    with pytest.raises(PolicyError):
        PolicyInference.stub(action=9)
    assert not PolicyInference().is_stub


def test_008_R13_committed_synthetic_weights_and_golden():
    """The committed pair is tiny and synthetic (not a trained policy): logits within 1e-4 of the golden file and
    the argmax equal on every row with a top-2 margin of at least 1e-4 (the near-ties are counted)."""
    p = PolicyInference()
    assert p.load(os.path.join(DATA, 'synthetic_policy.json')) == (True, '')
    rows = json.load(open(os.path.join(DATA, 'golden.json')))['rows']
    assert len(rows) >= 200
    near_ties = 0
    for row in rows:
        got = p.logits(row['obs'])
        assert got == pytest.approx(row['logits'], abs=1e-4)
        s = sorted(row['logits'], reverse=True)
        if s[0] - s[1] < 1e-4:
            near_ties += 1
        else:
            assert p.act(row['obs']) == row['action']
    assert near_ties < len(rows) // 10


# ---- load gate (008-R16) ----

@pytest.mark.parametrize('mode,running,cur,file,expect', [
    ('CHARGED', False, None, 'a', 'load'),
    ('CHARGED', False, 'a', 'b', 'load'),
    ('CHARGED', False, 'a', 'a', 'skip'),
    ('CHARGED', True, None, 'a', 'skip'),
    ('CLEANING', False, None, 'a', 'skip'),
    ('IDLE', False, 'a', 'b', 'skip'),
    (None, False, None, 'a', 'skip'),
    ('CHARGED', False, 'a', None, 'reject'),
])
def test_008_R16_load_gate_truth_table(mode, running, cur, file, expect):
    action, reason = LoadGate.decide(mode, running, cur, file)
    assert action == expect and (action != 'reject' or reason)


def test_008_R13_unreadable_sidecar_rejected(tmp_path):
    path = write(tmp_path, weights_dict())
    os.remove(path + '.sha256')
    os.mkdir(path + '.sha256')                       # exists but cannot be read as a file
    ok, reason = PolicyInference().load(path)
    assert not ok and 'sidecar' in reason
    os.rmdir(path + '.sha256')
    (tmp_path / 'w.json.sha256').write_bytes(b'\xff\xfe')
    ok, reason = PolicyInference().load(path)
    assert not ok and 'sidecar' in reason
    (tmp_path / 'w.json.sha256').write_text('')
    assert not PolicyInference().load(path)[0]


def test_008_R08_pose_rejects_strings_and_consumes_iterables_once():
    b = ObservationBuilder(budget_s=5.0)
    for bad in (('1', '2', '3'), 5, None, (True, 0, 0), (0, 0, 0, 0)):
        with pytest.raises(PolicyError):
            b.reset(bad)
    b.reset(x for x in (1.0, 2.0, 0.5))                      # a generator is accepted, read once
    assert b._start == (1.0, 2.0, 0.5)
    b.reset(np.array([0.0, 0.0, 0.0]))
