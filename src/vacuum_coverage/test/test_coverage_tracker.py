"""T506: CoverageTracker and the coverable mask (005-R21)."""
import numpy as np
import pytest

from coverage_fixtures import RES, rect_room
from vacuum_coverage.core.coverage_tracker import CoverageTracker, coverable_mask
from vacuum_mapping.core.types import FREE

CLEARANCE = 0.21
TOOL = 0.15


def make(raster=None):
    raster = raster or rect_room(3.0, 2.0)
    region = raster.cells == FREE
    coverable = coverable_mask(raster, region, CLEARANCE, TOOL)
    return raster, region, coverable, CoverageTracker(raster, coverable, TOOL, region=region)


def test_005_R21_coverable_leaves_out_the_band_the_robot_cannot_reach_along_the_walls():
    raster, region, coverable, _ = make()
    # the nearest lane position is CLEARANCE from the wall; the tool reaches TOOL further in from there
    assert not coverable[1, 30]                      # first free row, against the wall
    assert coverable[raster.world_to_cell(0.5, 0.5)]  # well inside
    assert coverable.sum() < region.sum()
    assert not (coverable & ~region).any()


def test_005_R21_ratio_starts_at_zero_and_grows_with_the_poses():
    raster, _, coverable, tracker = make()
    assert tracker.covered_ratio == 0.0
    assert tracker.mark_pose(1.0, 1.0) > 0
    first = tracker.covered_ratio
    assert 0.0 < first < 1.0
    tracker.mark_pose(2.0, 1.0)
    assert tracker.covered_ratio > first


def test_005_R21_marking_the_same_pose_twice_adds_nothing():
    _, _, _, tracker = make()
    tracker.mark_pose(1.0, 1.0)
    before = tracker.covered_ratio
    assert tracker.mark_pose(1.0, 1.0) == 0
    assert tracker.covered_ratio == before


def test_005_R21_cells_within_tool_radius_of_the_pose_are_marked_and_no_others():
    raster, _, coverable, tracker = make()
    tracker.mark_pose(1.5, 1.0)
    marked = tracker.covered_cells
    rows, cols = np.nonzero(marked)
    for r, c in zip(rows, cols):
        x, y = raster.cell_to_world(r, c)
        assert np.hypot(x - 1.5, y - 1.0) <= TOOL + 1e-9
    r, c = raster.world_to_cell(1.5, 1.0)
    assert marked[r, c]
    assert not marked[raster.world_to_cell(1.5 + TOOL + 0.1, 1.0)]


def test_005_R21_pose_outside_the_raster_is_ignored():
    _, _, _, tracker = make()
    assert tracker.mark_pose(-50.0, 40.0) == 0


def test_005_R21_mark_segment_covers_a_band_between_two_poses():
    _, _, _, tracker = make()
    tracker.mark_segment((0.6, 1.0), (2.4, 1.0))
    assert tracker.covered_ratio > 0.4
    assert tracker.covered_cells[tracker_cell(tracker, 1.5, 1.0)]


def tracker_cell(tracker, x, y):
    return tracker.raster.world_to_cell(x, y)


def test_005_R21_ratio_is_one_when_there_is_nothing_to_cover():
    raster = rect_room(0.2, 0.2)
    region = raster.cells == FREE
    coverable = coverable_mask(raster, region, CLEARANCE, TOOL)
    assert not coverable.any()
    assert CoverageTracker(raster, coverable, TOOL, region=region).covered_ratio == 1.0


def test_005_R21_covered_grid_encodes_100_0_and_minus_one_outside_the_region():
    raster, region, _, tracker = make()
    tracker.mark_pose(1.0, 1.0)
    grid = tracker.covered_grid()
    assert grid.dtype == np.int8 and grid.shape == raster.cells.shape
    assert set(np.unique(grid)) <= {-1, 0, 100}
    assert (grid[~region] == -1).all()
    assert (grid == 100).sum() == tracker.covered_cells.sum()


def test_005_R21_coverable_count_and_covered_count_are_reported_for_telemetry():
    _, _, coverable, tracker = make()
    tracker.mark_pose(1.0, 1.0)
    assert tracker.coverable_count == coverable.sum()
    assert 0 < tracker.covered_count <= tracker.coverable_count
    assert tracker.covered_ratio == pytest.approx(tracker.covered_count / tracker.coverable_count)


def test_005_R21_shape_mismatch_is_rejected():
    raster = rect_room(1.0, 1.0)
    with pytest.raises(ValueError):
        CoverageTracker(raster, np.zeros((3, 3), dtype=bool), TOOL)
