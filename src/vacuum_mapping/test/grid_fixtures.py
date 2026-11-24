"""Small synthetic rasters shared by the tests (no files, no ROS)."""
import numpy as np

from vacuum_mapping.core.types import FREE, OCCUPIED, UNKNOWN, Raster

RES = 0.05


def boxed_room(rows: int = 40, cols: int = 60, res: float = RES, origin=(0.0, 0.0)) -> Raster:
    """A free rectangle enclosed by a one-cell wall; the outside is unknown."""
    cells = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    cells[:] = OCCUPIED
    cells[1:-1, 1:-1] = FREE
    return Raster(cells, res, origin[0], origin[1])


def two_rooms(opening_m: float, res: float = RES, room_w_m: float = 3.0, room_h_m: float = 3.0,
              wall_cells: int = 2) -> Raster:
    """Two square rooms side by side, joined by one opening of `opening_m` in the shared wall."""
    rw, rh = round(room_w_m / res), round(room_h_m / res)
    rows, cols = rh + 2, 2 * rw + 2 + wall_cells
    cells = np.full((rows, cols), OCCUPIED, dtype=np.int8)
    cells[1:-1, 1:1 + rw] = FREE
    cells[1:-1, 1 + rw + wall_cells:-1] = FREE
    gap = round(opening_m / res)
    start = 1 + (rh - gap) // 2
    cells[start:start + gap, 1 + rw:1 + rw + wall_cells] = FREE
    return Raster(cells, res)
