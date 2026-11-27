"""Learned-escape toolbox (spec 008, decision D-31): observation builder, action table, weights inference, load gate.

One module shared by the robot node and the 2D training environment (`rl/`), so that the observation seen in
training and at run time is produced by the same code. Pure: numpy and the standard library only, no ROS, no torch,
no publishing, no topic reads, no policy about what to do with a result (D-18, D-19). The caller owns timing,
limits, logging and the decision to fall back. Errors are raised as `PolicyError`; `load` reports them as
`(ok, reason)` instead, so a bad file never raises into a node.

Every constant of observation version 1 and action table version 1 is defined once, below.
Covers 008-R08 (observation), 008-R09 (actions), 008-R13 (inference), 008-R16 (load gate), 008-R17 (errors).
"""
import hashlib
import json
import math
import os
from typing import List, Optional, Sequence, Tuple

import numpy as np

# ---- constants of version 1 (single place; spec 008 is not reviewed yet, change here only) ----
OBS_VERSION = 1
ACTION_TABLE_VERSION = 1
WEIGHTS_FORMAT = 'vacuum-escape-policy/1'
N_SECTORS = 30
RANGE_CAP_M = 2.0
N_ACTIONS = 8
N_OBS = N_SECTORS + N_ACTIONS + 1 + 4          # 43
ODOM_SCALE_M = 0.5                             # displacement mapped to [0, 1] over +-0.5 m
REVERSE_SPEED_M_S = 0.15                       # same speeds as EscapeScript, so the comparison is fair
TURN_SPEED_RAD_S = 1.0
PUBLISH_RATE_HZ = 10.0                         # one action per 0.1 s (escape_server publish_rate_hz)

_V, _W = REVERSE_SPEED_M_S, TURN_SPEED_RAD_S
# (linear m/s, angular rad/s): forward, reverse, left, right, fwd-left, fwd-right, rev-left, rev-right
ACTIONS = ((_V, 0.0), (-_V, 0.0), (0.0, _W), (0.0, -_W), (_V, _W / 2), (_V, -_W / 2), (-_V, _W / 2), (-_V, -_W / 2))

_ACTIVATIONS = ('tanh', 'linear')
_REQUIRED_INT_FIELDS = (('obs_version', OBS_VERSION), ('n_obs', N_OBS), ('n_actions', N_ACTIONS),
                        ('action_table_version', ACTION_TABLE_VERSION))


class PolicyError(ValueError):
    """Invalid argument, invalid weights, or a failed inference (non-finite logits)."""


def _is_index(a, n: int) -> bool:
    return isinstance(a, int) and not isinstance(a, bool) and 0 <= a < n


def command_for_action(action: int) -> Tuple[float, float]:
    """`(v, w)` of an action index (008-R09)."""
    if not _is_index(action, len(ACTIONS)):
        raise PolicyError('action %r is not an index in 0..%d' % (action, len(ACTIONS) - 1))
    return ACTIONS[action]


def _finite_pose(pose) -> Tuple[float, float, float]:
    try:
        values = tuple(pose)
    except TypeError:
        raise PolicyError('pose must be (x, y, yaw), got %r' % (pose,))
    ok = len(values) == 3 and all(isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, bool)
                                  and math.isfinite(v) for v in values)
    if not ok:
        raise PolicyError('pose must be three finite numbers, got %r' % (pose,))
    return float(values[0]), float(values[1]), float(values[2])


class ObservationBuilder:
    """Builds the observation vector (version 1, 008-R08) from plain lists and floats.

    Layout: `n_sectors` laser sectors (minimum valid range, capped at `range_cap_m`, divided by it), the previous
    action one-hot (all zero at the start), elapsed policy time over `budget_s` clipped to [0, 1], then the
    odometry change since `reset` in the start frame: dx and dy (over `ODOM_SCALE_M`, clipped to [-1, 1], mapped
    to [0, 1]), (sin dyaw + 1) / 2 and (cos dyaw + 1) / 2."""

    def __init__(self, budget_s: float, n_sectors: int = N_SECTORS, range_cap_m: float = RANGE_CAP_M,
                 n_actions: int = N_ACTIONS):
        if not (isinstance(budget_s, (int, float)) and budget_s > 0 and math.isfinite(budget_s)):
            raise PolicyError('budget_s must be positive and finite')
        if not (isinstance(n_sectors, int) and n_sectors > 0):
            raise PolicyError('n_sectors must be a positive integer')
        if not (isinstance(range_cap_m, (int, float)) and range_cap_m > 0 and math.isfinite(range_cap_m)):
            raise PolicyError('range_cap_m must be positive and finite')
        if not (isinstance(n_actions, int) and n_actions > 0):
            raise PolicyError('n_actions must be a positive integer')
        self.budget_s = float(budget_s)
        self.n_sectors = n_sectors
        self.range_cap_m = float(range_cap_m)
        self.n_actions = n_actions
        self.n_obs = n_sectors + n_actions + 1 + 4
        self._start: Optional[Tuple[float, float, float]] = None

    def reset(self, start_pose: Sequence[float]) -> None:
        """Remember the pose at the goal start; the displacement is measured from it."""
        self._start = _finite_pose(start_pose)

    def _sectors(self, ranges: Sequence[float]) -> List[float]:
        n = len(ranges)
        out = []
        for i in range(self.n_sectors):
            best = self.range_cap_m
            for r in ranges[(i * n) // self.n_sectors:((i + 1) * n) // self.n_sectors]:
                r = float(r)
                if math.isnan(r) or r <= 0.0:
                    continue
                best = min(best, r)           # inf stays "free" and is capped
            out.append(best / self.range_cap_m)
        return out

    def build(self, ranges: Sequence[float], pose: Sequence[float], elapsed_s: float,
              prev_action: Optional[int]) -> List[float]:
        if self._start is None:
            raise PolicyError('reset(start_pose) must be called before build')
        x, y, yaw = _finite_pose(pose)
        if not (isinstance(elapsed_s, (int, float)) and math.isfinite(elapsed_s)):
            raise PolicyError('elapsed_s must be finite')
        if prev_action is not None and not _is_index(prev_action, self.n_actions):
            raise PolicyError('prev_action %r is not None or an index in 0..%d' % (prev_action, self.n_actions - 1))
        sx, sy, syaw = self._start
        dx, dy = x - sx, y - sy
        c, s = math.cos(syaw), math.sin(syaw)
        fx, fy = c * dx + s * dy, -s * dx + c * dy          # world displacement in the start frame
        dyaw = yaw - syaw

        def unit(v):
            return (max(-1.0, min(1.0, v / ODOM_SCALE_M)) + 1.0) / 2.0

        obs = self._sectors(ranges)
        obs += [1.0 if i == prev_action else 0.0 for i in range(self.n_actions)]
        obs.append(max(0.0, min(1.0, elapsed_s / self.budget_s)))
        obs += [unit(fx), unit(fy), (math.sin(dyaw) + 1.0) / 2.0, (math.cos(dyaw) + 1.0) / 2.0]
        return [float(v) for v in obs]


def _check_matrix(x, name: str) -> np.ndarray:
    try:
        a = np.asarray(x, dtype=np.float64)
    except (TypeError, ValueError):
        raise PolicyError('%s is not a numeric array' % name)
    if not np.all(np.isfinite(a)):
        raise PolicyError('%s has a non-finite value' % name)
    return a


class PolicyInference:
    """Pure-numpy MLP forward pass from a weights file (format `vacuum-escape-policy/1`, 008-R12/R13).

    `W` is stored as rows of the PyTorch `Linear.weight` layout, `(out, in)`, so `y = x W^T + b`. The pass runs in
    float32 like SB3, and `act` is the argmax of the logits with ties going to the lowest index. A failed `load`
    leaves the weights already loaded untouched."""

    def __init__(self):
        self.layers: List[Tuple[np.ndarray, np.ndarray, str]] = []
        self.loaded = False
        self.is_stub = False
        self.sha256 = ''
        self.run_id = ''
        self.range_cap_m = RANGE_CAP_M
        self.n_sectors = N_SECTORS

    # -- construction --
    @classmethod
    def stub(cls, action: int = 1) -> 'PolicyInference':
        """Weights-free default: a valid zero network whose bias always selects `action` (default 1, reverse).
        It goes through the same forward pass, so callers can test their wiring without a weights file."""
        if not _is_index(action, N_ACTIONS):
            raise PolicyError('stub action %r is not an index in 0..%d' % (action, N_ACTIONS - 1))
        bias = [0.0] * N_ACTIONS
        bias[action] = 1.0
        p = cls()
        p._install([(np.zeros((N_ACTIONS, N_OBS), np.float32), np.asarray(bias, np.float32), 'linear')],
                   sha='stub', run_id='stub', cap=RANGE_CAP_M, n_sectors=N_SECTORS)
        p.is_stub = True
        return p

    def _install(self, layers, sha, run_id, cap, n_sectors):
        self.layers, self.sha256, self.run_id = layers, sha, run_id
        self.range_cap_m, self.n_sectors = cap, n_sectors
        self.loaded, self.is_stub = True, False

    # -- loading --
    def load(self, path: str, require_sidecar: bool = True,
             expected_range_cap_m: Optional[float] = RANGE_CAP_M) -> Tuple[bool, str]:
        """Read and validate `path`; returns `(ok, reason)`. The sidecar `<path>.sha256` must hold the file hash
        (a missing sidecar is a rejection unless `require_sidecar` is False). The file's `range_cap_m` must equal
        `expected_range_cap_m` (default `RANGE_CAP_M`; None skips the check)."""
        try:
            with open(path, 'rb') as f:
                raw = f.read()
        except (OSError, TypeError, ValueError) as e:
            return False, 'cannot read %r: %s' % (path, e)
        digest = hashlib.sha256(raw).hexdigest()
        side = path + '.sha256'
        if os.path.exists(side):
            try:
                with open(side, 'r') as f:
                    token = f.read().split()
                expected = token[0].lower() if token else ''
            except (OSError, UnicodeDecodeError) as e:
                return False, 'cannot read sidecar %r: %s' % (side, e)
            if len(expected) != 64 or any(c not in '0123456789abcdef' for c in expected):
                return False, 'sidecar %r does not hold a sha256' % side
            if expected != digest:
                return False, 'sha256 mismatch: file %s, sidecar %s' % (digest[:12], expected[:12])
        elif require_sidecar:
            return False, 'sha256 sidecar %r is missing' % side
        try:
            data = json.loads(raw.decode('utf-8'))
        except (ValueError, UnicodeDecodeError) as e:
            return False, 'not valid JSON: %s' % e
        return self.load_dict(data, sha=digest, expected_range_cap_m=expected_range_cap_m)

    def load_dict(self, data, sha: str = '', expected_range_cap_m: Optional[float] = RANGE_CAP_M) -> Tuple[bool, str]:
        """Validate an already parsed weights document; returns `(ok, reason)`."""
        try:
            layers, run_id, cap, n_sectors = self._parse(data, expected_range_cap_m)
        except PolicyError as e:
            return False, str(e)
        self._install(layers, sha or hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest(),
                      run_id, cap, n_sectors)
        return True, ''

    @staticmethod
    def _parse(data, expected_cap):
        if not isinstance(data, dict):
            raise PolicyError('weights document is not an object')
        if data.get('format') != WEIGHTS_FORMAT:
            raise PolicyError('format %r, expected %r' % (data.get('format'), WEIGHTS_FORMAT))
        for key, want in _REQUIRED_INT_FIELDS:
            if data.get(key) != want or isinstance(data.get(key), bool):
                raise PolicyError('%s is %r, expected %r' % (key, data.get(key), want))
        cap = data.get('range_cap_m')
        if not (isinstance(cap, (int, float)) and not isinstance(cap, bool) and cap > 0 and math.isfinite(cap)):
            raise PolicyError('range_cap_m is %r, expected a positive number' % (cap,))
        if expected_cap is not None and abs(cap - expected_cap) > 1e-9:
            raise PolicyError('range_cap_m is %r, the caller expects %r' % (cap, expected_cap))
        n_sectors = data.get('n_sectors')
        if n_sectors != N_SECTORS or isinstance(n_sectors, bool):
            raise PolicyError('n_sectors is %r, expected %d' % (n_sectors, N_SECTORS))
        raw_layers = data.get('layers')
        if not isinstance(raw_layers, list) or not raw_layers:
            raise PolicyError('layers must be a non-empty list')
        layers, width = [], N_OBS
        for i, layer in enumerate(raw_layers):
            if not isinstance(layer, dict) or not all(k in layer for k in ('W', 'b', 'activation')):
                raise PolicyError('layer %d needs W, b and activation' % i)
            if layer['activation'] not in _ACTIVATIONS:
                raise PolicyError('layer %d activation %r not in %s' % (i, layer['activation'], _ACTIVATIONS))
            w = _check_matrix(layer['W'], 'layer %d W' % i)
            b = _check_matrix(layer['b'], 'layer %d b' % i)
            if w.ndim != 2 or b.ndim != 1:
                raise PolicyError('layer %d: W must be 2-D and b 1-D, got %s and %s' % (i, w.shape, b.shape))
            if w.shape[1] != width or w.shape[0] != b.shape[0]:
                raise PolicyError('layer %d: W is %s and b is %s, expected (n, %d) and (n,)'
                                  % (i, w.shape, b.shape, width))
            layers.append((w.astype(np.float32), b.astype(np.float32), layer['activation']))
            width = w.shape[0]
        if width != N_ACTIONS:
            raise PolicyError('last layer has %d outputs, expected %d' % (width, N_ACTIONS))
        return layers, str(data.get('run_id', '')), float(cap), n_sectors

    # -- inference --
    def logits(self, obs: Sequence[float]) -> List[float]:
        """Raw network output. Raises `PolicyError` if not loaded, on a wrong size, or on a non-finite logit."""
        if not self.loaded:
            raise PolicyError('no weights loaded')
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if x.shape[0] != N_OBS:
            raise PolicyError('observation has %d values, expected %d' % (x.shape[0], N_OBS))
        with np.errstate(all='ignore'):
            for w, b, act in self.layers:
                x = w @ x + b
                if act == 'tanh':
                    x = np.tanh(x)
        if not np.all(np.isfinite(x)):
            raise PolicyError('non-finite logit')
        return [float(v) for v in x]

    def act(self, obs: Sequence[float]) -> int:
        """Argmax of the logits; ties go to the lowest index (`np.argmax` returns the first maximum)."""
        return int(np.argmax(self.logits(obs)))


class LoadGate:
    """Decides whether a weights file may be loaded now (008-R16, Art. VII.3): only in mode CHARGED, with no goal
    running, and only when the file hash changed. Returns `(action, reason)`, action in `load`, `skip`, `reject`."""

    @staticmethod
    def decide(mode: Optional[str], goal_running: bool, current_sha: Optional[str],
               file_sha: Optional[str]) -> Tuple[str, str]:
        if mode != 'CHARGED':
            return 'skip', 'mode is %s, weights load only in CHARGED' % mode
        if goal_running:
            return 'skip', 'a goal is running'
        if file_sha is None:
            return 'reject', 'weights file missing or unreadable'
        if file_sha == current_sha:
            return 'skip', 'weights unchanged'
        return 'load', 'weights changed'
