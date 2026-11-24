"""Rooms file and rule file I/O (003-R23, R21). Files live in pytest's tmp_path."""
import pytest

from vacuum_mapping.core.room_labeler import Rule
from vacuum_mapping.core.types import LabelSource, Room
from vacuum_mapping.rooms_io import RoomsFile, read_rooms, read_rules, write_rooms

ROOMS = (
    Room(1, 'room_01', 'kitchen', LabelSource.USER, (1.5, 2.25), 9.5),
    Room(3, 'room_03', 'room_03', LabelSource.DEFAULT, (-4.0, 0.5), 3.0),
)


def test_003_R23_rooms_file_roundtrips_labels_sources_and_next_index(tmp_path):
    path = str(tmp_path / 'house_rooms.yaml')
    original = RoomsFile('house', (0.5, -1.0, 1.57), 4, ROOMS)
    write_rooms(path, original)
    assert read_rooms(path) == original


def test_003_R30_start_pose_is_optional(tmp_path):
    path = str(tmp_path / 'r.yaml')
    write_rooms(path, RoomsFile('house', None, 2, ROOMS[:1]))
    assert read_rooms(path).start_pose is None


def test_003_R23_label_source_is_stored_by_name(tmp_path):
    path = tmp_path / 'r.yaml'
    write_rooms(str(path), RoomsFile('house', None, 4, ROOMS))
    text = path.read_text()
    assert 'label_source: user' in text and 'label_source: default' in text


def test_003_R23_unknown_label_source_is_rejected(tmp_path):
    path = tmp_path / 'r.yaml'
    path.write_text('map_name: h\nnext_index: 2\nrooms:\n- {index: 1, id: room_01, label: a, '
                    'label_source: robot, centroid: [0, 0], area_m2: 1}\n')
    with pytest.raises(ValueError):
        read_rooms(str(path))


def test_003_R23_next_index_defaults_to_one_past_the_largest_index(tmp_path):
    path = tmp_path / 'r.yaml'
    path.write_text('map_name: h\nrooms:\n- {index: 4, id: room_04, label: room_04, label_source: default, '
                    'centroid: [0, 0], area_m2: 1}\n')
    assert read_rooms(str(path)).next_index == 5


def test_003_R21_rules_file_is_read_as_class_to_label_and_weight(tmp_path):
    path = tmp_path / 'room_rules.yaml'
    path.write_text('rules:\n  refrigerator: {label: kitchen, weight: 2.0}\n  bed: {label: bedroom}\n')
    rules = read_rules(str(path))
    assert rules == {'refrigerator': Rule('kitchen', 2.0), 'bed': Rule('bedroom', 1.0)}
