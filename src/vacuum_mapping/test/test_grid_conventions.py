"""Pins that vacuum_grid.GridView and vacuum_mapping Raster agree on grid arithmetic (003-R09)."""
import numpy as np

from vacuum_grid.grid_view import FREE, OCCUPIED, UNKNOWN, GridView
from vacuum_mapping.core import types


def test_003_R09_grid_view_and_raster_agree_on_conversions_and_codes():
    cells = np.zeros((12, 17), dtype=np.int8)
    raster = types.Raster(cells, 0.05, -0.7, 1.3)
    view = GridView(cells, 0.05, -0.7, 1.3)
    assert (types.UNKNOWN, types.FREE, types.OCCUPIED) == (UNKNOWN, FREE, OCCUPIED)
    for r in range(12):
        for c in range(17):
            assert raster.cell_to_world(r, c) == view.cell_to_world(r, c)
    for x, y in [(-0.7, 1.3), (-0.69, 1.31), (0.1, 1.6), (-1.0, 0.9), (5.0, 5.0)]:
        assert raster.world_to_cell(x, y) == view.world_to_cell(x, y)
