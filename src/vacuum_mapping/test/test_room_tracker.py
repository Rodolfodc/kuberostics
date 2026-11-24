"""RoomTracker tests (003-R19, R20). Label rasters are hand-made, resolution 1 m per cell
so that areas equal cell counts."""
import dataclasses

import numpy as np
import pytest

from vacuum_mapping.core.room_tracker import RoomTracker, TrackedMap
from vacuum_mapping.core.types import LabelSource

RES = 1.0


def grid(rows):
    """Label raster from strings: '.' no room, digits are raw labels."""
    return np.array([[0 if c == '.' else int(c) for c in row] for row in rows], dtype=np.int32)


def track(labels, previous=None, **kw):
    return RoomTracker(**kw).track(previous, labels, RES, 0.0, 0.0)


def by_index(result):
    return {r.index: r for r in result.rooms}


def test_003_R20_first_tracking_assigns_indexes_from_one_with_default_labels():
    result = track(grid(['1122', '1122']))
    rooms = by_index(result)
    assert sorted(rooms) == [1, 2]
    r1 = rooms[1]
    assert (r1.id, r1.label, r1.label_source) == ('room_01', 'room_01', LabelSource.DEFAULT)
    assert r1.area_m2 == 4.0
    assert r1.centroid == pytest.approx((1.0, 1.0))  # cells x 0..1, y 0..1, centres at 0.5 and 1.5 -> (1, 1)
    assert result.next_index == 3


def test_003_R20_output_raster_holds_room_indexes():
    result = track(grid(['1.2']))
    assert result.labels.tolist() == [[1, 0, 2]]


def test_003_R19_same_segmentation_keeps_index_id_label_and_source():
    first = track(grid(['1122', '1122']))
    named = dataclasses.replace(first, rooms=tuple(
        dataclasses.replace(r, label='kitchen', label_source=LabelSource.USER) if r.index == 2 else r
        for r in first.rooms))
    # raw labels now come in the opposite order, as after a different scan
    result = track(grid(['2211', '2211']), named)
    rooms = by_index(result)
    assert rooms[2].label == 'kitchen' and rooms[2].label_source == LabelSource.USER
    assert rooms[2].centroid == pytest.approx((3.0, 1.0))  # index 2 is still the right-hand room
    assert result.labels.tolist() == [[1, 1, 2, 2], [1, 1, 2, 2]]
    assert result.next_index == 3


def test_003_R19_split_keeps_index_for_the_larger_part_and_gives_a_new_one_to_the_other():
    previous = track(grid(['1111111111']))                  # one room, 10 cells
    result = track(grid(['1111111222']), previous)          # 7 + 3
    assert result.labels.tolist() == [[1] * 7 + [2] * 3]
    assert result.next_index == 3


def test_003_R19_the_smaller_part_cannot_take_the_same_previous_room():
    previous = track(grid(['1111111111']))
    result = track(grid(['2222222111']), previous)          # larger part has raw label 2
    assert result.labels.tolist() == [[1] * 7 + [2] * 3]  # index 1 followed the larger part


def test_003_R19_overlap_below_half_of_the_new_room_creates_a_new_index():
    previous = track(grid(['11......']))                    # 2 cells
    result = track(grid(['.1111111']), previous)            # 7 cells, overlap 1 -> 14 % of new area
    assert result.labels.tolist() == [[0] + [2] * 7]
    assert result.next_index == 3


def test_003_R19_overlap_of_exactly_half_keeps_the_index():
    previous = track(grid(['1111....']))
    result = track(grid(['..111111']), previous)                     # 6 cells, overlap 2 of 6 -> no
    assert result.labels.tolist() == [[0, 0, 2, 2, 2, 2, 2, 2]]
    result = track(grid(['..11....']), previous)                     # 2 cells, overlap 2 of 2 -> yes
    assert result.labels.tolist() == [[0, 0, 1, 1, 0, 0, 0, 0]]
    result = track(grid(['..1111..']), previous)                     # 4 cells, overlap 2 of 4 = 50 % -> yes
    assert result.labels.tolist() == [[0, 0, 1, 1, 1, 1, 0, 0]]


def test_003_R19_indexes_are_never_reused():
    first = track(grid(['1.2']))                            # indexes 1 and 2
    second = track(grid(['1..']), first)                    # room 2 vanishes
    assert by_index(second).keys() == {1}
    third = track(grid(['1.2']), second)
    assert by_index(third).keys() == {1, 3}                 # new room gets 3, not the retired 2
    assert third.next_index == 4


def test_003_R19_merge_keeps_the_index_of_the_larger_previous_room():
    previous = track(grid(['11122']))                       # 3 + 2
    result = track(grid(['11111']), previous)
    assert result.labels.tolist() == [[1] * 5]
    assert result.next_index == 3


def test_003_R19_tracking_is_deterministic_on_ties():
    previous = track(grid(['1122']))
    a = track(grid(['1111']), previous)                     # equal overlap with both previous rooms
    b = track(grid(['1111']), previous)
    assert np.array_equal(a.labels, b.labels) and a.rooms == b.rooms
    assert a.labels.tolist() == [[1, 1, 1, 1]]              # lowest previous index wins the tie


def test_003_R19_logger_reports_kept_and_new_rooms():
    lines = []
    previous = track(grid(['1.2']))
    track(grid(['1.1']), previous, log=lines.append)
    assert lines and all(isinstance(line, str) for line in lines)


def test_003_R19_empty_segmentation_keeps_next_index():
    previous = track(grid(['1.2']))
    result = track(grid(['...']), previous)
    assert result.rooms == () and result.next_index == 3


def test_003_R19_tracked_map_first_state_helper():
    assert TrackedMap.empty().next_index == 1


def test_003_R19_shape_change_logs_a_warning_and_assigns_new_indexes():
    lines = []
    previous = track(grid(['1.2']))
    result = track(grid(['1.2.']), previous, log=lines.append)
    assert any('WARNING' in line and 'cannot be carried over' in line for line in lines)
    assert sorted(by_index(result)) == [3, 4]
