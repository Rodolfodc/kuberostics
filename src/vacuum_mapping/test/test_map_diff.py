"""MapDiff tests (003-R34), on small synthetic grids. No ROS, no files."""
import dataclasses

import numpy as np
import pytest

from grid_fixtures import boxed_room
from vacuum_mapping.core.map_diff import MapDiff, format_report
from vacuum_mapping.core.types import OCCUPIED, UNKNOWN, Raster


def test_003_R34_identical_maps_give_zero_percent():
    a = boxed_room()
    report = MapDiff().compare(a, boxed_room())
    assert report.pixel_diff_percent == 0.0
    assert report.differing_cells == 0
    assert not report.resolution_changed and not report.origin_changed and not report.size_changed
    assert report.median_occupied_distance_m == 0.0


def test_003_R34_extra_wall_counts_only_the_changed_cells():
    a = boxed_room()
    cells = a.cells.copy()
    cells[10:20, 30] = OCCUPIED  # 10 cells of an extra wall inside the room
    report = MapDiff().compare(a, dataclasses.replace(a, cells=cells))
    assert report.differing_cells == 10
    assert report.pixel_diff_percent == pytest.approx(100.0 * 10 / a.cells.size)
    assert report.median_occupied_distance_m == 0.0  # most occupied cells are unchanged walls


def test_003_R34_median_distance_of_a_displaced_wall_is_the_displacement():
    def wall_at(col):
        cells = np.full((20, 40), UNKNOWN, dtype=np.int8)
        cells[:, col] = OCCUPIED
        return Raster(cells, 0.05)
    report = MapDiff().compare(wall_at(10), wall_at(13))
    assert report.median_occupied_distance_m == pytest.approx(0.15)


def test_003_R34_shifted_origin_is_reported_and_compared_in_world_frame():
    a = boxed_room()
    b = boxed_room(origin=(0.5, 0.0))  # same picture, 10 cells to the right
    report = MapDiff().compare(a, b)
    assert report.origin_changed
    assert report.origin_shift_m == pytest.approx((0.5, 0.0))
    assert report.pixel_diff_percent > 0.0
    assert not report.size_changed


def test_003_R34_shift_by_whole_cells_matches_when_content_is_shifted_too():
    a = boxed_room(rows=20, cols=20)
    wide = np.full((20, 30), UNKNOWN, dtype=np.int8)
    wide[:, 10:] = a.cells  # a's picture placed 10 cells to the right in a wider raster
    b = Raster(wide, a.resolution, -10 * a.resolution, 0.0)
    report = MapDiff().compare(a, b)
    assert report.pixel_diff_percent == 0.0  # the extra columns are unknown on both sides
    assert report.size_changed


def test_003_R34_different_resolution_is_reported():
    a = boxed_room(rows=40, cols=60, res=0.05)
    b = boxed_room(rows=20, cols=30, res=0.10)
    report = MapDiff().compare(a, b)
    assert report.resolution_changed
    assert (report.resolution_committed, report.resolution_new) == (0.05, 0.10)
    assert report.size_changed
    # both cover the same 3.0 x 2.0 m, so the finer grid is the comparison grid
    assert report.comparison_resolution_m == 0.05
    assert report.pixel_diff_percent < 20.0  # walls are 1 cell vs 2 cells thick


def test_003_R34_cells_outside_the_other_extent_count_as_unknown():
    a = boxed_room(rows=10, cols=10)
    bigger = np.full((10, 20), OCCUPIED, dtype=np.int8)  # same origin, twice as wide, all occupied
    b = Raster(bigger, a.resolution)
    report = MapDiff().compare(a, b)
    # union = 10 x 20 cells. Right half: a is unknown (outside), b is occupied -> 100 differing cells.
    # Left half: a has a free interior of 8 x 8 = 64 cells that b has occupied.
    assert report.compared_cells == 200
    assert report.differing_cells == 100 + 64


def test_003_R34_no_occupied_cells_means_distance_not_computable():
    empty = Raster(np.full((5, 5), UNKNOWN, dtype=np.int8), 0.05)
    report = MapDiff().compare(empty, boxed_room())
    assert report.median_occupied_distance_m is None


def test_003_R34_report_text_lists_resolution_origin_and_size_changes():
    a = boxed_room(rows=40, cols=60, res=0.05)
    b = boxed_room(rows=20, cols=30, res=0.10, origin=(1.0, 2.0))
    text = format_report(MapDiff().compare(a, b))
    assert 'resolution' in text.lower() and '0.05' in text and '0.1' in text
    assert 'origin' in text.lower() and '1.0' in text and '2.0' in text
    assert 'size' in text.lower() and '60 x 40' in text and '30 x 20' in text
    assert 'pixel' in text.lower()


def test_003_R34_report_text_says_unchanged_when_equal():
    text = format_report(MapDiff().compare(boxed_room(), boxed_room()))
    assert text.count('unchanged') == 3
    assert '0.00 %' in text


def test_003_R34_logger_receives_a_summary_line():
    lines = []
    MapDiff(log=lines.append).compare(boxed_room(), boxed_room())
    assert len(lines) == 1 and 'pixel' in lines[0]
