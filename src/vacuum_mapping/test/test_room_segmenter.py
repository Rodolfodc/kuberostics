"""RoomSegmenter tests (003-R16, R17), synthetic grids, door_width_m = 0.9."""
import time

import numpy as np
import pytest

from grid_fixtures import RES, boxed_room, two_rooms
from vacuum_mapping.core.room_segmenter import RoomSegmenter
from vacuum_mapping.core.types import FREE, OCCUPIED, UNKNOWN, Raster


def segmenter(**kw):
    return RoomSegmenter(door_width_m=kw.pop('door_width_m', 0.9),
                         min_room_area_m2=kw.pop('min_room_area_m2', 2.0), **kw)


def count(labels):
    return len(set(np.unique(labels)) - {0})


def test_003_R17_opening_of_080_m_gives_two_rooms():
    assert count(segmenter().segment(two_rooms(0.80))) == 2


def test_003_R17_opening_of_100_m_stays_one_room():
    assert count(segmenter().segment(two_rooms(1.00))) == 1


def test_003_R17_open_rectangle_is_one_room():
    assert count(segmenter().segment(boxed_room(rows=100, cols=160))) == 1


def test_003_R17_no_label_on_occupied_or_unknown_cells():
    raster = two_rooms(0.80)
    raster.cells[3:6, 3:6] = UNKNOWN
    labels = segmenter().segment(raster)
    assert np.all(labels[raster.cells == OCCUPIED] == 0)
    assert np.all(labels[raster.cells == UNKNOWN] == 0)


def test_003_R16_every_free_cell_connected_to_a_seed_gets_a_label():
    raster = two_rooms(0.80)
    labels = segmenter().segment(raster)
    assert np.all(labels[raster.cells == FREE] > 0)  # includes the cells inside the narrow opening


def test_003_R16_opening_cells_go_to_the_nearest_seed():
    raster = two_rooms(0.80)
    labels = segmenter().segment(raster)
    mid_row = raster.height // 2
    left, right = labels[mid_row, 5], labels[mid_row, -6]
    assert left != right
    assert labels[mid_row, 1 + 60] == left  # first wall column of the opening: closest to the left room
    assert labels[mid_row, 1 + 61] == right


def test_003_R16_labels_are_consecutive_from_one_in_scan_order():
    labels = segmenter().segment(two_rooms(0.80))
    assert sorted(np.unique(labels[labels > 0])) == [1, 2]
    assert labels[labels > 0][0] == 1  # the first free cell in scan order belongs to the first room


def test_003_R16_free_pocket_smaller_than_min_room_area_is_not_a_room():
    raster = boxed_room(rows=100, cols=160)
    raster.cells[40:60, 60:62] = OCCUPIED  # wall, leaves the room alone
    pocket = np.full((12, 12), OCCUPIED, dtype=np.int8)
    pocket[3:-3, 3:-3] = FREE  # 6 x 6 cells = 0.09 m2 of free space, disconnected
    cells = np.full((100, 200), UNKNOWN, dtype=np.int8)
    cells[:, :160] = raster.cells
    cells[10:22, 170:182] = pocket
    labels = segmenter().segment(Raster(cells, RES))
    assert count(labels) == 1
    assert np.all(labels[10:22, 170:182] == 0)


def test_003_R16_noise_speckles_do_not_split_a_room():
    rng = np.random.default_rng(0)
    raster = boxed_room(rows=100, cols=160)
    rows = rng.integers(15, 85, size=8)
    cols = rng.integers(15, 145, size=8)
    raster.cells[rows[:4], cols[:4]] = OCCUPIED  # isolated occupied cells (laser noise)
    raster.cells[rows[4:], cols[4:]] = UNKNOWN   # isolated unknown cells
    labels = segmenter().segment(raster)
    assert count(labels) == 1
    assert np.all(labels[raster.cells == FREE] > 0)


def test_003_R16_segmentation_is_deterministic():
    raster = two_rooms(0.80)
    assert np.array_equal(segmenter().segment(raster), segmenter().segment(raster))


def test_003_R16_door_width_parameter_changes_the_result():
    raster = two_rooms(1.00)
    assert count(segmenter(door_width_m=1.2).segment(raster)) == 2


def test_003_R16_logger_reports_room_count():
    lines = []
    segmenter(log=lines.append).segment(two_rooms(0.80))
    assert any('2 rooms' in line for line in lines)


def test_003_R16_300_by_300_grid_is_segmented_quickly():
    cells = np.full((300, 300), OCCUPIED, dtype=np.int8)
    cells[1:-1, 1:-1] = FREE
    cells[:, 150] = OCCUPIED
    cells[140:156, 150] = FREE  # 0.80 m opening
    start = time.perf_counter()
    labels = segmenter().segment(Raster(cells, RES))
    elapsed = time.perf_counter() - start
    assert count(labels) == 2
    assert elapsed < 3.0  # generous bound for slow CI; typical well under 1 s


def test_003_R16_rejects_invalid_parameters():
    with pytest.raises(ValueError):
        RoomSegmenter(door_width_m=0.0, min_room_area_m2=2.0)
    with pytest.raises(ValueError):
        RoomSegmenter(door_width_m=0.9, min_room_area_m2=-1.0)
