"""RoomSession tests (003-R16 to R20, R22, R23, R26): segmentation + tracking + labels + persistence, no ROS."""
import numpy as np

from grid_fixtures import two_rooms
from vacuum_mapping.core.room_labeler import Observation, Rule
from vacuum_mapping.core.room_session import RoomSession
from vacuum_mapping.core.types import LabelSource
from vacuum_mapping.rooms_io import read_rooms

RULES = {'oven': Rule('kitchen', 1.0), 'sink': Rule('kitchen', 1.0)}


def session(tmp_path, **kw):
    return RoomSession(rooms_path=str(tmp_path / 'house_rooms.yaml'), map_name='house', rules=RULES,
                       door_width_m=0.9, min_room_area_m2=2.0, min_votes=2, **kw)


def test_003_R18_R20_segment_gives_indexed_rooms_with_default_labels(tmp_path):
    s = session(tmp_path)
    result = s.segment(two_rooms(0.7))
    assert [r.index for r in result.rooms] == [1, 2]
    assert [r.id for r in result.rooms] == ['room_01', 'room_02']
    assert all(r.label == r.id and r.label_source == LabelSource.DEFAULT for r in result.rooms)
    assert set(np.unique(result.labels)) == {0, 1, 2}


def test_003_R23_set_label_marks_user_persists_and_survives_resegmentation(tmp_path):
    s = session(tmp_path)
    raster = two_rooms(0.7)
    s.segment(raster)
    ok, _ = s.set_label('room_02', 'bedroom')
    assert ok
    saved = read_rooms(str(tmp_path / 'house_rooms.yaml'))
    assert [(r.id, r.label, r.label_source) for r in saved.rooms][1] == ('room_02', 'bedroom', LabelSource.USER)
    s.segment(raster)
    assert {r.id: r.label for r in s.rooms}['room_02'] == 'bedroom'


def test_003_R23_unknown_id_and_empty_label_are_rejected(tmp_path):
    s = session(tmp_path)
    s.segment(two_rooms(0.7))
    assert s.set_label('room_09', 'x')[0] is False
    assert s.set_label('room_01', '')[0] is False
    assert not (tmp_path / 'house_rooms.yaml').exists() or \
        read_rooms(str(tmp_path / 'house_rooms.yaml')).rooms[0].label_source == LabelSource.DEFAULT


def test_003_R23_labels_survive_a_restart(tmp_path):
    first = session(tmp_path)
    raster = two_rooms(0.7)
    first.segment(raster)
    first.set_label('room_01', 'kitchen')
    second = session(tmp_path)           # new process: nothing in memory, the file is read
    result = second.segment(raster)
    assert {r.id: (r.label, r.label_source) for r in result.rooms}['room_01'] == ('kitchen', LabelSource.USER)
    assert result.next_index == 3        # indexes are never reused after a restart


def test_003_R19_map_that_grew_keeps_room_identity_by_centroid(tmp_path):
    s = session(tmp_path)
    raster = two_rooms(0.7)
    s.segment(raster)
    s.set_label('room_01', 'kitchen')
    bigger = np.full((raster.cells.shape[0] + 6, raster.cells.shape[1] + 6), -1, dtype=np.int8)
    bigger[:raster.cells.shape[0], :raster.cells.shape[1]] = raster.cells
    from vacuum_mapping.core.types import Raster
    result = s.segment(Raster(bigger, raster.resolution, raster.origin_x, raster.origin_y))
    assert {r.id: r.label for r in result.rooms}['room_01'] == 'kitchen'
    assert [r.index for r in result.rooms] == [1, 2]


def test_003_R21_R22_observations_label_rooms_and_republish_does_not_inflate(tmp_path):
    s = session(tmp_path)
    raster = two_rooms(0.7)
    result = s.segment(raster)
    x, y = result.rooms[0].centroid
    one = [Observation('oven', x, y)]
    for _ in range(5):
        s.set_observations(one)
    assert s.rooms[0].label_source == LabelSource.DEFAULT        # one oven is one vote, min_votes is 2
    s.set_observations(one + [Observation('sink', x + 0.2, y)])
    assert (s.rooms[0].label, s.rooms[0].label_source) == ('kitchen', LabelSource.RULE)
    s.set_label('room_01', 'mine')
    s.set_observations(one + [Observation('sink', x + 0.2, y)])
    assert s.rooms[0].label == 'mine'                            # never overrides a user label


def test_003_R30_start_pose_is_kept_in_the_rooms_file(tmp_path):
    s = session(tmp_path, start_pose=(1.0, 2.0, 0.5))
    s.segment(two_rooms(0.7))
    assert read_rooms(str(tmp_path / 'house_rooms.yaml')).start_pose == (1.0, 2.0, 0.5)
