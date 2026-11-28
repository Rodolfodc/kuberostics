"""Synthetic rasters and a conservative path simulator shared by the coverage tests (no files, no ROS).

The grid helpers of vacuum_mapping/test/grid_fixtures.py are not installed, so a small local copy lives here
(a shared `vacuum_grid` package is planned by T309 of spec 003).
"""
import numpy as np

from vacuum_mapping.core.types import FREE, OCCUPIED, UNKNOWN, Raster

RES = 0.05


def rect_room(width_m: float, height_m: float, res: float = RES) -> Raster:
    """A free rectangle enclosed by a one-cell wall."""
    rows, cols = round(height_m / res) + 2, round(width_m / res) + 2
    cells = np.full((rows, cols), OCCUPIED, dtype=np.int8)
    cells[1:-1, 1:-1] = FREE
    return Raster(cells, res)


def with_box(raster: Raster, x0: float, y0: float, x1: float, y1: float) -> Raster:
    """Copy of `raster` with an occupied box (world metres, inclusive of the cells it touches)."""
    cells = raster.cells.copy()
    r0, c0 = raster.world_to_cell(x0, y0)
    r1, c1 = raster.world_to_cell(x1, y1)
    cells[r0:r1 + 1, c0:c1 + 1] = OCCUPIED
    return Raster(cells, raster.resolution, raster.origin_x, raster.origin_y)


def two_rooms_with_door(room_w_m: float = 3.0, room_h_m: float = 3.0, door_m: float = 0.9,
                        res: float = RES, wall_cells: int = 2) -> Raster:
    """Two square rooms side by side, joined by one door in the shared wall."""
    rw, rh = round(room_w_m / res), round(room_h_m / res)
    cells = np.full((rh + 2, 2 * rw + 2 + wall_cells), OCCUPIED, dtype=np.int8)
    cells[1:-1, 1:1 + rw] = FREE
    cells[1:-1, 1 + rw + wall_cells:-1] = FREE
    gap = round(door_m / res)
    start = 1 + (rh - gap) // 2
    cells[start:start + gap, 1 + rw:1 + rw + wall_cells] = FREE
    return Raster(cells, res)


def two_room_labels(raster: Raster, rw_cells: int = 60, wall_cells: int = 2) -> np.ndarray:
    """Hand-made label raster for `two_rooms_with_door` (room 1 left, room 2 right, 0 elsewhere)."""
    labels = np.zeros(raster.cells.shape, dtype=np.int32)
    free = raster.cells == FREE
    labels[:, :1 + rw_cells + wall_cells // 2] = 1
    labels[:, 1 + rw_cells + wall_cells // 2:] = 2
    labels[~free] = 0
    return labels


def unknown_border(raster: Raster) -> Raster:
    cells = raster.cells.copy()
    cells[0, :] = UNKNOWN
    return Raster(cells, raster.resolution, raster.origin_x, raster.origin_y)


def simulate_pass(tracker, reachable: np.ndarray, raster: Raster, waypoints, step_m: float = 0.02) -> None:
    """Drive the tool through the waypoints and mark the cells it covers.

    Conservative on purpose: only straight hops whose every sample lies on a reachable cell are credited.
    Nav2 plans the real detour between two waypoints; counting a straight line through an obstacle would
    credit coverage the robot never gives.
    """
    for a, b in zip(waypoints, waypoints[1:]):
        length = float(np.hypot(b.x - a.x, b.y - a.y))
        n = max(1, int(np.ceil(length / step_m)))
        pts = [(a.x + (b.x - a.x) * i / n, a.y + (b.y - a.y) * i / n) for i in range(n + 1)]
        ok = True
        for x, y in pts:
            r, c = raster.world_to_cell(x, y)
            if not (0 <= r < raster.height and 0 <= c < raster.width and reachable[r, c]):
                ok = False
                break
        if ok:
            for x, y in pts:
                tracker.mark_pose(x, y)
    if waypoints:
        tracker.mark_pose(waypoints[0].x, waypoints[0].y)
