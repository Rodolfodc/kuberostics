"""Reading and writing `maps/<map_name>_rooms.yaml` and `config/room_rules.yaml` (003-R21, R23, R30).

WHY: user labels must survive a restart and a new segmentation (003-R23), and the robot start pose
belongs to the map (003-R30). File layout, kept human-editable:

    map_name: house
    start_pose_xyyaw: [0.0, 0.0, 0.0]      # optional
    next_index: 4                          # first index never used, so indexes are never reused
    rooms:
      - {index: 1, id: room_01, label: kitchen, label_source: user, centroid: [1.5, 2.0], area_m2: 9.5}

Rule file: `rules: {refrigerator: {label: kitchen, weight: 2.0}}`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import yaml

from .core.room_labeler import Rule
from .core.types import LabelSource, Room


@dataclass(frozen=True)
class RoomsFile:
    """Content of a rooms file."""
    map_name: str
    start_pose: Optional[Tuple[float, float, float]]
    next_index: int
    rooms: Tuple[Room, ...]


def write_rooms(path: str, content: RoomsFile) -> None:
    data: Dict = {'map_name': content.map_name}
    if content.start_pose is not None:
        data['start_pose_xyyaw'] = [float(v) for v in content.start_pose]
    data['next_index'] = int(content.next_index)
    data['rooms'] = [{
        'index': r.index, 'id': r.id, 'label': r.label,
        'label_source': r.label_source.name.lower(),
        'centroid': [float(r.centroid[0]), float(r.centroid[1])],
        'area_m2': float(r.area_m2),
    } for r in content.rooms]
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


def read_rooms(path: str) -> RoomsFile:
    """Parse a rooms file; raises ValueError on an unknown `label_source`."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    rooms = []
    for item in data.get('rooms') or []:
        try:
            source = LabelSource[str(item['label_source']).upper()]
        except KeyError:
            raise ValueError('%s: unknown label_source %r' % (path, item['label_source'])) from None
        rooms.append(Room(int(item['index']), str(item['id']), str(item['label']), source,
                          (float(item['centroid'][0]), float(item['centroid'][1])), float(item['area_m2'])))
    pose = data.get('start_pose_xyyaw')
    next_index = int(data.get('next_index', max((r.index for r in rooms), default=0) + 1))
    return RoomsFile(str(data['map_name']), tuple(float(v) for v in pose) if pose else None,
                     next_index, tuple(rooms))


def read_rules(path: str) -> Dict[str, Rule]:
    """Parse the class -> (label, weight) table of `RoomLabeler`; weight defaults to 1.0."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return {str(cls): Rule(str(spec['label']), float(spec.get('weight', 1.0)))
            for cls, spec in (data.get('rules') or {}).items()}
