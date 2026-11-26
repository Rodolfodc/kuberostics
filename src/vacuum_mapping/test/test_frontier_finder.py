"""FrontierFinder tests (003-R09). Synthetic grids on a 0.05 m raster, no ROS."""
import numpy as np
import pytest

from vacuum_grid.grid_view import FREE, OCCUPIED, UNKNOWN, GridView
from vacuum_mapping.core.frontier_finder import Frontier, FrontierFinder

RES = 0.05


def room_with_doors(gaps, rows=40, cols=60, pad=3):
    """Walled free room; the right wall has openings (row ranges) leading to `pad` unknown columns."""
    cells = np.full((rows, cols + pad), UNKNOWN, dtype=np.int8)
    cells[:, :cols] = OCCUPIED
    cells[1:-1, 1:cols - 1] = FREE
    for lo, hi in gaps:
        cells[lo:hi, cols - 1] = FREE
    return GridView(cells, RES)


def closed_room(rows=40, cols=60):
    cells = np.full((rows, cols), OCCUPIED, dtype=np.int8)
    cells[1:-1, 1:-1] = FREE
    return GridView(cells, RES)


def finder(**kw):
    kw.setdefault('robot_radius_m', 0.0)
    return FrontierFinder(**kw)


def test_003_R09_empty_known_room_has_no_frontier():
    assert finder().find(closed_room(), (1.0, 1.0)) == []


def test_003_R09_door_to_unknown_is_one_frontier_with_its_size():
    grid = room_with_doors([(12, 29)])
    result = finder().find(grid, (1.0, 1.0))
    assert len(result) == 1
    assert result[0].size == 17
    x, y = result[0].centroid
    assert x == pytest.approx(grid.cell_to_world(0, 59)[0])
    assert y == pytest.approx(grid.cell_to_world(20, 0)[1])   # odd size: centroid is a cell centre, no snap


def test_003_R09_pocket_below_min_frontier_cells_is_dropped():
    grid = closed_room()
    grid.cells[1:3, 1:3] = UNKNOWN                 # corner pocket: ring of 5 free cells
    assert finder(min_frontier_cells=8).find(grid, (1.0, 1.0)) == []
    kept = finder(min_frontier_cells=5).find(grid, (1.0, 1.0))
    assert len(kept) == 1 and kept[0].size == 5


def test_003_R09_cluster_of_exactly_min_frontier_cells_is_kept():
    grid = closed_room()
    grid.cells[20, 30] = UNKNOWN                   # isolated unknown cell: ring of exactly 8 free cells
    kept = finder(min_frontier_cells=8).find(grid, (1.0, 1.0))
    assert len(kept) == 1 and kept[0].size == 8
    assert finder(min_frontier_cells=9).find(grid, (1.0, 1.0)) == []
    # the centroid is the unknown cell itself, so it snaps to the first of the equally near free cells
    assert grid.world_to_cell(*kept[0].centroid) == (19, 30)


def test_003_R09_frontier_is_free_cell_next_to_unknown_in_8_neighbourhood():
    grid = closed_room()
    grid.cells[20, 30] = UNKNOWN
    finder(min_frontier_cells=1)  # constructor accepts 1
    result = finder(min_frontier_cells=1).find(grid, (1.0, 1.0))
    assert len(result) == 1 and result[0].size == 8   # diagonal neighbours count


def test_003_R09_separate_clusters_are_separate_frontiers():
    grid = room_with_doors([(5, 15), (25, 35)])
    result = finder().find(grid, (1.0, 1.0))
    assert [f.size for f in sorted(result, key=lambda f: f.centroid[1])] == [10, 10]


def test_003_R09_centroid_snaps_to_free_cell_with_clearance():
    grid = room_with_doors([(18, 22)])               # 4-cell door: every door cell is within 3 cells of a wall
    radius = 0.15
    result = finder(min_frontier_cells=3, robot_radius_m=radius).find(grid, (1.0, 1.0))
    assert len(result) == 1
    row, col = grid.world_to_cell(*result[0].centroid)
    assert grid.cells[row, col] == FREE
    assert not grid.inflated_obstacles(radius)[row, col]
    assert col <= 56                                   # moved into the room, away from the door cells


def test_003_R09_frontier_without_a_reachable_snap_cell_is_dropped_and_logged():
    logs = []
    grid = room_with_doors([(18, 22)])
    f = finder(min_frontier_cells=3, robot_radius_m=5.0, max_snap_m=1.0, log=logs.append)
    assert f.find(grid, (1.0, 1.0)) == []
    assert any('snap' in m for m in logs)


def test_003_R09_blacklisted_frontier_is_skipped():
    grid = room_with_doors([(5, 15), (25, 35)])
    f = finder()
    first, second = sorted(f.find(grid, (1.0, 1.0)), key=lambda fr: fr.centroid[1])
    assert f.find(grid, (1.0, 1.0), blacklist=[first.centroid]) == [second]
    assert f.find(grid, (1.0, 1.0), blacklist=[first.centroid, second.centroid]) == []


def test_003_R09_blacklist_matches_within_tolerance_only():
    grid = room_with_doors([(5, 15)])
    f = finder(blacklist_tolerance_m=0.3)
    target = f.find(grid, (1.0, 1.0))[0]
    assert f.find(grid, (1.0, 1.0), blacklist=[(target.centroid[0], target.centroid[1] + 0.29)]) == []
    assert len(f.find(grid, (1.0, 1.0), blacklist=[(target.centroid[0], target.centroid[1] + 0.5)])) == 1


def test_003_R09_order_is_distance_minus_gain_times_size():
    grid = room_with_doors([(2, 8), (20, 36)])        # small frontier near the robot, big one far
    near_small = grid.cell_to_world(5, 59)
    robot = (near_small[0] - 0.5, near_small[1])
    by_distance = finder(gain_weight=0.0, min_frontier_cells=3).find(grid, robot)
    assert [f.size for f in by_distance] == [6, 16]
    by_gain = finder(gain_weight=0.5, min_frontier_cells=3).find(grid, robot)  # 0.5 * (16 - 6) = 5 m outweighs the extra distance
    assert [f.size for f in by_gain] == [16, 6]


def test_003_R09_deterministic_and_ties_break_on_centroid():
    grid = room_with_doors([(5, 16), (24, 35)])
    mid_y = grid.cell_to_world(19.5, 0)[1]   # midway between the two door centroids
    robot = (3.0, mid_y)                               # equidistant from both identical doors
    a = finder().find(grid, robot)
    b = finder().find(grid, robot)
    assert a == b and len(a) == 2
    assert a[0].centroid[1] < a[1].centroid[1]         # tie: lower y first
    assert all(isinstance(f, Frontier) for f in a)


def test_003_R09_does_not_modify_the_grid_and_reports_metrics():
    grid = room_with_doors([(5, 15), (25, 35)])
    before = grid.cells.copy()
    seen = []
    finder(metrics=seen.append).find(grid, (1.0, 1.0))
    assert np.array_equal(grid.cells, before)
    assert seen and seen[-1]['frontiers'] == 2 and seen[-1]['clusters'] == 2
