"""T506: CoveragePlanner on synthetic grids (005-R17, R18)."""
import math

import numpy as np
import pytest

from coverage_fixtures import (RES, rect_room, simulate_pass, two_room_labels, two_rooms_with_door,
                               unknown_border, with_box)
from vacuum_coverage.core.coverage_planner import CoveragePlanner
from vacuum_coverage.core.coverage_tracker import CoverageTracker, coverable_mask
from vacuum_coverage.core.grid_ops import reachable_mask
from vacuum_mapping.core.types import FREE

CLEARANCE = 0.16 + 0.05


def free_region(raster):
    return raster.cells == FREE


def lanes_of(waypoints):
    """Waypoints grouped by lane (same y for horizontal lanes), in order."""
    out = []
    for w in waypoints:
        key = round(w.y, 4)
        if not out or out[-1][0] != key:
            out.append([key, []])
        out[-1][1].append(w)
    return out


def test_005_R17_empty_rectangle_has_parallel_lanes_with_the_configured_spacing():
    raster = rect_room(4.0, 2.0)
    wps = CoveragePlanner().plan(raster, free_region(raster))
    lanes = lanes_of(wps)
    ys = [k for k, _ in lanes]
    assert len(lanes) >= 6
    gaps = np.diff(ys)
    assert gaps[:-1] == pytest.approx([0.25] * (len(gaps) - 1), abs=RES / 2)
    assert (gaps <= 0.25 + RES / 2).all()


def test_005_R17_lane_direction_alternates_boustrophedon():
    raster = rect_room(4.0, 2.0)
    lanes = lanes_of(CoveragePlanner().plan(raster, free_region(raster)))
    for i, (_, wps) in enumerate(lanes):
        assert len(wps) == 2
        forward = wps[1].x > wps[0].x
        assert forward == (i % 2 == 0)
        assert wps[0].yaw == pytest.approx(0.0 if forward else math.pi)
    # the next lane starts on the side where the previous one ended
    for (_, a), (_, b) in zip(lanes, lanes[1:]):
        assert abs(a[1].x - b[0].x) < 1e-6


def test_005_R17_first_pass_runs_along_the_longer_axis():
    wide = rect_room(4.0, 2.0)
    tall = rect_room(2.0, 4.0)
    wp_wide = CoveragePlanner().plan(wide, free_region(wide))
    wp_tall = CoveragePlanner().plan(tall, free_region(tall))
    assert wp_wide[0].y == wp_wide[1].y and wp_wide[0].x != wp_wide[1].x
    assert wp_tall[0].x == wp_tall[1].x and wp_tall[0].y != wp_tall[1].y
    assert abs(wp_tall[0].yaw) == pytest.approx(math.pi / 2)


def test_005_R17_waypoints_keep_robot_radius_plus_margin_from_obstacles():
    raster = with_box(rect_room(4.0, 3.0), 1.8, 1.3, 2.2, 1.7)
    wps = CoveragePlanner().plan(raster, free_region(raster))
    assert wps
    reach = reachable_mask(raster, CLEARANCE)
    for w in wps:
        r, c = raster.world_to_cell(w.x, w.y)
        assert reach[r, c]


def test_005_R17_unknown_cells_count_as_obstacles():
    raster = unknown_border(rect_room(3.0, 2.0))
    wps = CoveragePlanner().plan(raster, np.ones(raster.cells.shape, dtype=bool))
    reach = reachable_mask(raster, CLEARANCE)
    assert wps and all(reach[raster.world_to_cell(w.x, w.y)] for w in wps)


def test_005_R17_central_obstacle_splits_the_lanes_it_crosses():
    raster = with_box(rect_room(4.0, 3.0), 1.8, 1.3, 2.2, 1.7)
    lanes = lanes_of(CoveragePlanner().plan(raster, free_region(raster)))
    crossed = [wps for _, wps in lanes if len(wps) == 4]
    assert crossed, 'at least one lane must be cut by the obstacle'
    for wps in crossed:
        xs = sorted(w.x for w in wps)
        assert xs[2] - xs[1] > 0.4          # the gap around the box


def test_005_R17_segments_shorter_than_min_segment_are_skipped():
    raster = rect_room(2.0, 1.0)
    # a 0.30 m minimum segment cannot fit between a box and the wall clearance: lane pieces vanish
    boxed = with_box(raster, 0.95, 0.0, 1.05, 1.1)          # splits the room in two ~0.9 m halves
    short = CoveragePlanner(min_segment_m=0.30).plan(boxed, free_region(boxed))
    none = CoveragePlanner(min_segment_m=5.0).plan(boxed, free_region(boxed))
    assert short and none == []
    for _, wps in lanes_of(short):
        for a, b in zip(wps[::2], wps[1::2]):
            assert abs(b.x - a.x) + RES >= 0.30


def test_005_R17_second_pass_rotates_the_lanes_by_ninety_degrees():
    raster = rect_room(4.0, 2.0)
    wps = CoveragePlanner().plan(raster, free_region(raster), passes=2)
    first = [w for w in wps if w.pass_index == 0]
    second = [w for w in wps if w.pass_index == 1]
    assert first and second and len(first) + len(second) == len(wps)
    assert wps.index(second[0]) == len(first)                    # pass 1 completes before pass 2
    assert first[0].y == first[1].y                              # horizontal lanes
    assert second[0].x == second[1].x                            # vertical lanes
    assert abs(second[0].yaw) == pytest.approx(math.pi / 2)


def test_005_R17_plan_is_deterministic():
    raster = with_box(rect_room(4.0, 3.0), 1.8, 1.3, 2.2, 1.7)
    a = CoveragePlanner().plan(raster, free_region(raster), passes=2)
    b = CoveragePlanner().plan(raster, free_region(raster), passes=2)
    assert a == b


def test_005_R17_empty_region_gives_no_waypoints():
    raster = rect_room(2.0, 2.0)
    assert CoveragePlanner().plan(raster, np.zeros(raster.cells.shape, dtype=bool)) == []


def test_005_R17_invalid_parameters_are_rejected():
    for kw in ({'lane_spacing_m': 0.0}, {'robot_radius_m': -0.1}, {'min_segment_m': -1.0}):
        with pytest.raises(ValueError):
            CoveragePlanner(**kw)


def coverage_after(raster, region, passes=1, tool=0.15):
    planner = CoveragePlanner()
    wps = planner.plan(raster, region, passes=passes)
    coverable = coverable_mask(raster, region, CLEARANCE, tool)
    tracker = CoverageTracker(raster, coverable, tool, region=region)
    simulate_pass(tracker, reachable_mask(raster, CLEARANCE) & region, raster, wps)
    return tracker.covered_ratio


@pytest.mark.parametrize('name,raster', [
    ('empty rectangle', rect_room(4.0, 3.0)),
    ('central obstacle', with_box(rect_room(4.0, 3.0), 1.8, 1.3, 2.2, 1.7)),
    ('two rooms', two_rooms_with_door()),
])
def test_005_R18_simulated_pass_covers_at_least_95_percent(name, raster):
    ratio = coverage_after(raster, free_region(raster))
    assert ratio >= 0.95, f'{name}: {ratio:.3f}'


def test_005_R18_each_room_of_the_two_room_map_is_covered_on_its_own():
    raster = two_rooms_with_door()
    labels = two_room_labels(raster)
    for index in (1, 2):
        region = (labels == index) & free_region(raster)
        assert coverage_after(raster, region) >= 0.95


def test_005_R18_two_passes_cover_at_least_as_much_as_one():
    raster = with_box(rect_room(4.0, 3.0), 1.8, 1.3, 2.2, 1.7)
    region = free_region(raster)
    assert coverage_after(raster, region, passes=2) >= coverage_after(raster, region, passes=1)
