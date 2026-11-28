"""Regenerates synthetic_policy.json(+.sha256) and golden.json in this folder (seeded, deterministic).
Run from src/vacuum_coverage:  python3 test/data/generate_synthetic_policy.py
The golden logits are computed in float64 pure Python, independent of numpy. Not a trained policy."""
import json, math, hashlib, random, sys
sys.path.insert(0, '.')
import os
OUT = os.path.join('test', 'data')
from vacuum_coverage.core.escape_policy import ObservationBuilder
rnd = random.Random(8)
def mat(o, i): return [[round(rnd.gauss(0, 0.6), 4) for _ in range(i)] for _ in range(o)]
def vec(n): return [round(rnd.gauss(0, 0.3), 4) for _ in range(n)]
d = {'format': 'vacuum-escape-policy/1', 'obs_version': 1, 'n_obs': 43, 'n_actions': 8, 'action_table_version': 1,
     'range_cap_m': 2.0, 'n_sectors': 30, 'run_id': 'synthetic-test-pair', 'git_sha': '0'*40, 'config_hash': 'synthetic',
     'layers': [{'W': mat(8, 43), 'b': vec(8), 'activation': 'tanh'}, {'W': mat(8, 8), 'b': vec(8), 'activation': 'tanh'},
                {'W': mat(8, 8), 'b': vec(8), 'activation': 'linear'}]}
raw = json.dumps(d, separators=(',', ':'))
open(os.path.join(OUT, 'synthetic_policy.json'), 'w').write(raw)
open(OUT + '/synthetic_policy.json.sha256', 'w').write(hashlib.sha256(raw.encode()).hexdigest() + '  synthetic_policy.json\n')
def fwd(x):
    for l in d['layers']:
        x = [sum(w*v for w, v in zip(r, x)) + b for r, b in zip(l['W'], l['b'])]
        if l['activation'] == 'tanh': x = [math.tanh(v) for v in x]
    return x
rows = []
b = ObservationBuilder(budget_s=5.0)
for k in range(240):
    b.reset((0, 0, 0))
    r = [rnd.choice([float('inf'), rnd.uniform(0.06, 3.0)]) for _ in range(452)]
    obs = b.build(r, (rnd.uniform(-.5, .5), rnd.uniform(-.5, .5), rnd.uniform(-3, 3)), rnd.uniform(0, 5), rnd.choice([None] + list(range(8))))
    lg = fwd(obs)
    rows.append({'obs': [round(v, 7) for v in obs], 'logits': lg, 'action': lg.index(max(lg))})
# recompute with rounded obs so the file is self-consistent
for row in rows:
    row['logits'] = fwd(row['obs']); row['action'] = row['logits'].index(max(row['logits']))
json.dump({'note': 'synthetic, computed in float64 pure Python; not a trained policy', 'rows': rows}, open(OUT + '/golden.json', 'w'), separators=(',', ':'))
