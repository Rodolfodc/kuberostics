"""T506: RegionBuilder on synthetic grids (005-R16)."""
import numpy as np
import pytest

from coverage_fixtures import RES, rect_room, two_room_labels, two_rooms_with_door
from vacuum_coverage.core.region_builder import NoRegion, RegionBuilder, UnknownRoom
from vacuum_mapping.core.types import FREE, OCCUPIED, LabelSource, Raster, Room


def room(index, room_id=None):
    return Room(index, room_id or 'room_%02d' % index, 'room', LabelSource.DEFAULT, (0.0, 0.0), 1.0)


def test_005_R16_room_id_selects_the_cells_of_that_room_index():
    raster = two_rooms_with_door()
    labels = two_room_labels(raster)
    region = RegionBuilder().build(raster, (1.0, 1.0), room_id='room_02', rooms=[room(1), room(2)],
                                   labels=labels)
    assert region.source == 'room'
    assert region.mask.sum() == ((labels == 2) & (raster.cells == FREE)).sum() > 0
    assert not (region.mask & (labels == 1)).any()


def test_005_R16_unknown_non_empty_room_id_is_rejected():
    raster = two_rooms_with_door()
    with pytest.raises(UnknownRoom):
        RegionBuilder().build(raster, (1.0, 1.0), room_id='room_09', rooms=[room(1), room(2)],
                              labels=two_room_labels(raster))


def test_005_R16_non_empty_room_id_without_room_topics_is_rejected():
    with pytest.raises(UnknownRoom):
        RegionBuilder().build(rect_room(2.0, 2.0), (1.0, 1.0), room_id='room_01')


def test_005_R16_empty_room_id_takes_the_room_under_the_robot():
    raster = two_rooms_with_door()
    labels = two_room_labels(raster)
    x, y = raster.cell_to_world(30, 100)          # inside the right room
    region = RegionBuilder().build(raster, (x, y), rooms=[room(1), room(2)], labels=labels)
    assert region.source == 'robot_room'
    assert region.room_index == 2
    assert not (region.mask & (labels == 1)).any()


def test_005_R16_robot_on_a_cell_without_room_takes_the_nearest_room():
    raster = two_rooms_with_door()
    labels = two_room_labels(raster)
    wall_x, wall_y = raster.cell_to_world(5, 62)   # shared wall, label 0, a bit closer to the left room
    assert labels[5, 62] == 0
    region = RegionBuilder().build(raster, (wall_x, wall_y), rooms=[room(1), room(2)], labels=labels)
    assert region.source == 'robot_room'
    assert region.room_index in (1, 2)
    # deterministic: the same call gives the same room
    again = RegionBuilder().build(raster, (wall_x, wall_y), rooms=[room(1), room(2)], labels=labels)
    assert again.room_index == region.room_index


def test_005_R16_no_room_topics_gives_the_connected_free_cells_and_warns():
    raster = two_rooms_with_door()                  # both rooms are connected through the door
    logs = []
    region = RegionBuilder(log=logs.append).build(raster, (1.0, 1.0))
    assert region.source == 'connected'
    assert region.mask.sum() == (raster.cells == FREE).sum()
    assert any('WARN' in m for m in logs)


def test_005_R16_connected_region_excludes_a_free_island_the_robot_cannot_reach():
    raster = rect_room(3.0, 2.0)
    cells = raster.cells.copy()
    cells[10:14, 20:24] = OCCUPIED                  # a closed ring around an island
    cells[10:14, 20:24] = OCCUPIED
    cells[11:13, 21:23] = FREE
    island = Raster(cells, raster.resolution)
    region = RegionBuilder().build(island, (0.5, 0.5))
    assert not region.mask[11:13, 21:23].any()
    assert region.mask[5, 5]


def test_005_R16_region_is_capped_by_max_region_area_in_bfs_order_from_the_robot():
    raster = rect_room(6.0, 6.0)                    # 36 m2 of free cells
    builder = RegionBuilder(max_region_area_m2=4.0)
    region = builder.build(raster, (3.0, 3.0))
    assert region.source == 'connected' and region.truncated
    assert region.mask.sum() * RES * RES == pytest.approx(4.0, abs=RES * RES)
    r, c = raster.world_to_cell(3.0, 3.0)
    assert region.mask[r, c]
    # deterministic, and farther cells are the ones left out
    again = builder.build(raster, (3.0, 3.0))
    assert np.array_equal(region.mask, again.mask)
    assert not region.mask[2, 2]


def test_005_R16_robot_slightly_inside_a_wall_snaps_to_the_nearest_free_cell():
    raster = rect_room(2.0, 2.0)
    region = RegionBuilder().build(raster, (0.01, 0.01))     # the wall cell (0, 0)
    assert region.mask.sum() == (raster.cells == FREE).sum()


def test_005_R16_robot_with_no_free_cell_nearby_has_no_region():
    cells = np.full((20, 20), OCCUPIED, dtype=np.int8)
    with pytest.raises(NoRegion):
        RegionBuilder().build(Raster(cells, RES), (0.5, 0.5))
