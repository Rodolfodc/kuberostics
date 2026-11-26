"""GridView tests (003-R09 grid arithmetic, spec 005 reuse). Pure numpy, no ROS."""
import numpy as np
import pytest

from vacuum_grid.grid_view import FREE, OCCUPIED, UNKNOWN, GridView


def make(rows=10, cols=20, res=0.5, origin=(-1.0, 2.0)):
    return GridView(np.zeros((rows, cols), dtype=np.int8), res, *origin)


def test_003_R09_cell_to_world_is_cell_centre():
    g = make()
    assert g.cell_to_world(0, 0) == pytest.approx((-0.75, 2.25))
    assert g.cell_to_world(3, 4) == pytest.approx((-1.0 + 4.5 * 0.5, 2.0 + 3.5 * 0.5))


def test_003_R09_world_to_cell_uses_floor_and_row_is_y():
    g = make()
    assert g.world_to_cell(-0.75, 2.25) == (0, 0)
    assert g.world_to_cell(-1.0, 2.0) == (0, 0)          # lower-left corner belongs to cell (0, 0)
    assert g.world_to_cell(-0.51, 2.49) == (0, 0)
    assert g.world_to_cell(-0.5, 2.5) == (1, 1)
    assert g.world_to_cell(0.0, 3.0) == (2, 2)


def test_003_R09_world_to_cell_outside_grid_gives_out_of_range_indexes():
    g = make()
    assert g.world_to_cell(-1.2, 1.7) == (-1, -1)
    assert not g.in_bounds(-1, -1)
    assert not g.in_bounds(10, 0)
    assert not g.in_bounds(0, 20)
    assert g.in_bounds(9, 19)


def test_003_R09_round_trip_of_every_cell():
    g = make(7, 9, 0.05, (0.3, -0.2))
    for r in range(7):
        for c in range(9):
            assert g.world_to_cell(*g.cell_to_world(r, c)) == (r, c)


def test_003_R09_rejects_bad_input():
    with pytest.raises(ValueError):
        GridView(np.zeros(5, dtype=np.int8), 0.1)
    with pytest.raises(ValueError):
        GridView(np.zeros((2, 2), dtype=np.int8), 0.0)


def single_wall(rows=11, cols=11):
    cells = np.zeros((rows, cols), dtype=np.int8)
    cells[5, 5] = OCCUPIED
    return GridView(cells, 0.1)


def test_003_R09_inflation_radius_zero_is_the_obstacles():
    g = single_wall()
    mask = g.inflated_obstacles(0.0)
    assert mask.sum() == 1 and mask[5, 5]


def test_003_R09_inflation_is_a_euclidean_disk():
    g = single_wall()
    mask = g.inflated_obstacles(0.2)                      # 2 cells
    assert mask[5, 7] and mask[7, 5] and mask[3, 5] and mask[5, 3]
    assert mask[6, 6] and mask[4, 4]                      # diagonal 1.41 cells
    assert not mask[7, 7]                                 # diagonal 2.83 cells
    assert not mask[5, 8]
    assert mask.sum() == 13                               # cells with dr^2 + dc^2 <= 4


def test_003_R09_inflation_clips_at_the_border_and_border_is_not_an_obstacle():
    cells = np.zeros((5, 5), dtype=np.int8)
    cells[0, 0] = OCCUPIED
    mask = GridView(cells, 0.1).inflated_obstacles(0.2)
    assert mask[0, 2] and mask[2, 0] and mask[1, 1]
    assert not mask[4, 4]
    free = GridView(np.zeros((5, 5), dtype=np.int8), 0.1).inflated_obstacles(0.3)
    assert not free.any()


def test_003_R09_unknown_does_not_inflate_by_default():
    cells = np.zeros((7, 7), dtype=np.int8)
    cells[3, 3] = UNKNOWN
    g = GridView(cells, 0.1)
    assert not g.inflated_obstacles(0.2).any()
    with_unknown = g.inflated_obstacles(0.1, include_unknown=True)
    assert with_unknown[3, 3] and with_unknown[3, 4] and not with_unknown[4, 4]


def test_003_R09_inflation_does_not_modify_the_grid():
    g = single_wall()
    before = g.cells.copy()
    g.inflated_obstacles(0.3)
    assert np.array_equal(g.cells, before)
    assert (g.cells == FREE).sum() == before.size - 1


def test_003_R09_inflation_radius_larger_than_the_grid_does_not_fail():
    cells = np.zeros((3, 4), dtype=np.int8)
    cells[1, 1] = OCCUPIED
    assert GridView(cells, 0.1).inflated_obstacles(5.0).all()
