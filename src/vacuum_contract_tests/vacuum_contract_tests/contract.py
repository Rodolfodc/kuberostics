"""Pure-Python core of the contract tests (D-18): no ROS imports.

Reads `topics.yaml`, builds expectations and compares them with endpoint descriptions that the caller took
from the live graph (the rclpy glue is in test/test_live_graph.py). Toolbox only: it computes violations,
the caller decides what to do with them.
"""
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import yaml

CONTRACT_RELATIVE = Path('specs') / '000-system' / 'contracts' / 'topics.yaml'

# Owners that only exist with CoppeliaSim running (2025S2 sensor bridges); skipped unless asked for (Art. V).
SIM_OWNERS = frozenset({'my_robot_scan_node', 'my_robot_vision_node'})


@dataclass(frozen=True)
class QosSpec:
    reliability: str
    durability: str
    depth: int


@dataclass(frozen=True)
class Endpoint:
    """One publisher or subscriber as seen in the graph."""
    node: str
    type: str
    qos: Optional[QosSpec] = None


@dataclass(frozen=True)
class Expectation:
    kind: str                      # 'topic' | 'service' | 'action'
    name: str
    type: str
    qos: Optional[QosSpec]
    owner: str
    frozen: bool
    since: str
    max_publishers: Optional[int]
    sink: bool


def find_contract(start: Optional[Path] = None) -> Path:
    """VACUUM_TOPICS_YAML wins; otherwise walk up from `start` to specs/000-system/contracts/topics.yaml."""
    env = os.environ.get('VACUUM_TOPICS_YAML')
    if env:
        return Path(env)
    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        candidate = parent / CONTRACT_RELATIVE
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f'{CONTRACT_RELATIVE} not found above {here}; set VACUUM_TOPICS_YAML')


def load_contract(path: Path) -> dict:
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def normalize_type(type_name: str) -> str:
    """'pkg/Name' is accepted as 'pkg/msg/Name'; the contract itself always uses the three-part form."""
    parts = type_name.split('/')
    if len(parts) == 2:
        return f'{parts[0]}/msg/{parts[1]}'
    return type_name


def applies(entry: dict) -> bool:
    """001-R04 scope (spec 001, PR #9 wording): `since == "001"` or no `since`, regardless of `frozen`.

    A missing `since` means inherited. The rule is by `since` alone. A later `since` (e.g. /detections_markers,
    since 009a) is out of scope even when frozen: it is not in the skeleton yet.
    """
    return str(entry.get('since', '')) in ('', '001')


def _qos(contract: dict, entry: dict) -> Optional[QosSpec]:
    if 'qos' not in entry:
        return None
    profile = contract['qos_profiles'][entry['qos']]      # KeyError = unknown profile name = contract bug
    return QosSpec(profile['reliability'], profile['durability'], int(profile['depth']))


def build_expectations(contract: dict, exclude_owners: Iterable[str] = ()) -> List[Expectation]:
    excluded = set(exclude_owners)
    out: List[Expectation] = []
    for section, kind in (('topics', 'topic'), ('services', 'service'), ('actions', 'action')):
        for entry in contract.get(section) or []:
            if not applies(entry) or entry.get('owner') in excluded:
                continue
            out.append(Expectation(
                kind=kind, name=entry['name'], type=entry['type'], qos=_qos(contract, entry),
                owner=entry.get('owner', ''), frozen=bool(entry.get('frozen')),
                since=str(entry.get('since', '')), max_publishers=entry.get('max_publishers'),
                sink=bool(entry.get('sink'))))
    return out


def contract_problems(contract: dict) -> List[str]:
    """Static sanity of the contract itself (no graph needed)."""
    problems: List[str] = []
    profiles = contract.get('qos_profiles', {})
    seen = set()
    for section in ('topics', 'services', 'actions'):
        for entry in contract.get(section) or []:
            name = entry.get('name', '?')
            if (section, name) in seen:
                problems.append(f'duplicate {section[:-1]} {name}')
            seen.add((section, name))
            if not name.startswith('/'):
                problems.append(f'{name}: name is not absolute')
            if len(entry.get('type', '').split('/')) != 3:
                problems.append(f'{name}: malformed type {entry.get("type")!r}')
            if section == 'topics' and entry.get('qos') not in profiles:
                problems.append(f'{name}: unknown qos profile {entry.get("qos")!r}')
    return problems


def check_topic(exp: Expectation, publishers: Sequence[Endpoint],
                subscribers: Sequence[Endpoint]) -> List[str]:
    """Violations of one topic expectation. Empty list = conforms.

    - existence: at least one publisher or subscriber (mux inputs may have only the mux listening);
    - type: every endpoint has the declared type (001-R04);
    - QoS: every publisher offers exactly the declared profile (001-R04);
    - max_publishers (001-R03); for `max_publishers: 1` topics and sinks only the owner node may publish;
    - sink: nobody subscribes (D-04) and the owner publishes (001-R12).
    """
    endpoints = [*publishers, *subscribers]
    if not endpoints:
        return [f'{exp.name}: not in the graph']
    problems: List[str] = []
    want_type = normalize_type(exp.type)
    for ep in endpoints:
        if normalize_type(ep.type) != want_type:
            problems.append(f'{exp.name}: {ep.node} uses type {ep.type}, contract says {exp.type}')
    if exp.qos is not None:
        for ep in publishers:
            if ep.qos is not None and ep.qos != exp.qos:
                problems.append(f'{exp.name}: publisher {ep.node} QoS {ep.qos}, contract says {exp.qos}')
    if exp.max_publishers is not None and len(publishers) > exp.max_publishers:
        problems.append(f'{exp.name}: {len(publishers)} publishers, max_publishers is {exp.max_publishers}')
    if exp.max_publishers == 1 or exp.sink:
        for ep in publishers:
            if exp.owner not in ('', '*') and ep.node != exp.owner:
                problems.append(f'{exp.name}: published by {ep.node}, only {exp.owner} may publish')
    if exp.sink:
        if subscribers:
            problems.append(f'{exp.name}: sink topic has subscribers '
                            f'{sorted(ep.node for ep in subscribers)}')
        if not publishers:
            problems.append(f'{exp.name}: sink topic has no publisher (is the remapping in place?)')
    return problems


def check_named_type(exp: Expectation, found: Sequence[Tuple[str, Sequence[str]]]) -> List[str]:
    """Services and actions: `found` is [(name, [types])] from the graph; checks existence and type."""
    for name, types in found:
        if name == exp.name:
            if exp.type not in types:
                return [f'{exp.name}: types {list(types)}, contract says {exp.type}']
            return []
    return [f'{exp.name}: not in the graph']
