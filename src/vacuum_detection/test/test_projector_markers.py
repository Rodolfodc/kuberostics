"""pure tests of the projector pieces (DetectionSet, DetectionFilter, marker_spec, pipeline, caches).

Each test cites the requirement it verifies (Art. VIII). No ROS. The consumer replica follows
specs/009a-offload/markers-format.md (read from the 2025S2 `_detections_cb`, not executed: unverified).
"""
import math

import numpy as np
import pytest

from vacuum_detection.depth_sampling import depth_at
from vacuum_detection.detection_filter import DetectionFilter, GridArea
from vacuum_detection.detection_set import DetectionSet
from vacuum_detection.marker_spec import TEXT_VIEW_FACING, build_markers
from vacuum_detection.projection_core import ProjectorCore, Transform
from vacuum_detection.projector_pipeline import ProjectionPipeline
from vacuum_detection.stamp_cache import StampCache

IDENTITY = (0.0, 0.0, 0.0, 1.0)
K = dict(fx=100.0, fy=100.0, cx=50.0, cy=50.0)


# --- faithful replica of the consumer rules (_detections_cb, my_robot_semantic_segmentation_node.py:125-153) ----
class Marker:
    pass


def consumer_cb(markers, label_to_room, lookup_room_id, has_map=True):
    """Returns {room_id: room_name}. int(nan) raises ValueError exactly like the consumer."""
    if not has_map:
        return {}
    assignments = {}
    for m in markers:
        if m.ns != 'detections_text' or not m.text:
            continue
        label = m.text.split('(')[0].strip().lower()
        if not label:
            continue
        room_name = label_to_room.get(label)
        if room_name is None:
            continue
        room_id = lookup_room_id(m.x, m.y)
        if room_id is None or room_id <= 0:
            continue
        assignments[room_id] = room_name
    return assignments


def lookup_room(x, y):
    col, row = int(x / 1.0), int(y / 1.0)        # resolution 1 m, origin 0
    return 7 if (0 <= col < 10 and 0 <= row < 10) else None


def make_pipeline(tf=None, merge=0.5, **policy):
    tf = tf or (lambda stamp, timeout: Transform((2.0, 3.0, 0.0), IDENTITY))
    return ProjectionPipeline(ProjectorCore(tf, **K), DetectionFilter(**policy), DetectionSet(merge))


# --- 009a-R13: format --------------------------------------------------------------------
def test_009a_R13_publishes_cube_and_text_per_detection_with_original_ids():
    ds = DetectionSet()
    d, _ = ds.add('cup', 1.0, 2.0, 0.5, 10.0, 0.9)
    cube, text = build_markers([d])
    assert (cube.ns, cube.id, cube.frame_id) == ('detections', d.det_id, 'map')
    assert (text.ns, text.id, text.frame_id, text.type) == ('detections_text', d.det_id + 100000, 'map',
                                                             TEXT_VIEW_FACING)
    assert text.text == f'cup ({d.det_id})'
    assert (text.x, text.y, text.z, text.stamp) == (1.0, 2.0, 0.25, 10.0)


def test_009a_R13_never_sends_delete():
    pipe = make_pipeline()
    pipe.process(1.0, 'cup', 0.9, 50, 50, 1.0)
    assert {m.action for m in pipe.markers()} == {0}


def test_009a_R13_consumer_parses_the_produced_markers():
    pipe = make_pipeline()          # camera axis 1 m ahead, translation (2, 3): lands at (2, 3)
    assert pipe.process(1.0, 'Cup', 0.9, 50, 50, 0.0001) == 'new'
    specs = pipe.markers()
    markers = []
    for s in specs:
        m = Marker()
        m.ns, m.text, m.x, m.y = s.ns, s.text, s.x, s.y
        markers.append(m)
    assert consumer_cb(markers, {'cup': 'kitchen'}, lookup_room) == {7: 'kitchen'}


def test_009a_R13_consumer_label_rule_survives_multiword_labels():
    pipe = make_pipeline()
    pipe.process(1.0, ' Dining Table ', 0.9, 50, 50, 1.0)
    ms = []
    for s in pipe.markers():
        m = Marker()
        m.ns, m.text, m.x, m.y = s.ns, s.text, s.x, s.y
        ms.append(m)
    assert consumer_cb(ms, {'dining table': 'kitchen'}, lookup_room) == {7: 'kitchen'}


def test_009a_R13_no_nan_ever_reaches_the_consumer():
    nan_tf = lambda stamp, timeout: Transform((math.nan, 0.0, 0.0), IDENTITY)   # noqa: E731
    pipe = make_pipeline(tf=nan_tf)
    assert pipe.process(1.0, 'cup', 0.9, 50, 50, 1.0) == 'nan'
    assert pipe.markers() == []
    assert pipe.snapshot()['dropped_nan'] == 1
    # and the replica really raises on NaN, so the guard matters
    m = Marker()
    m.ns, m.text, m.x, m.y = 'detections_text', 'cup (0)', math.nan, 0.0
    with pytest.raises(ValueError):
        consumer_cb([m], {'cup': 'k'}, lookup_room)


def test_009a_R13_republishes_the_whole_set_each_time():
    pipe = make_pipeline(merge=0.1)
    pipe.process(1.0, 'cup', 0.9, 50, 50, 1.0)
    pipe.process(2.0, 'bottle', 0.9, 50, 50, 1.0)
    assert len(pipe.markers()) == 4 and len(pipe.markers()) == 4      # unchanged, not consumed by publishing


# --- 009a-R13: policy filters --------------------------------------------------------------
def test_009a_R13_confidence_default_is_0_7():
    pipe = make_pipeline()
    assert pipe.process(1.0, 'cup', 0.69, 50, 50, 1.0) == 'confidence'
    assert pipe.process(1.0, 'cup', 0.7, 50, 50, 1.0) == 'new'
    assert pipe.process(1.0, 'cup', math.nan, 50, 50, 1.0) == 'confidence'


def test_009a_R13_allowed_labels_empty_means_all_otherwise_only_listed():
    assert make_pipeline().process(1.0, 'anything', 0.9, 50, 50, 1.0) == 'new'
    pipe = make_pipeline(allowed_labels=['Cup'])
    assert pipe.process(1.0, 'cup', 0.9, 50, 50, 1.0) == 'new'
    assert pipe.process(1.0, 'bottle', 0.9, 50, 50, 1.0) == 'label'
    assert pipe.snapshot()['dropped_label'] == 1


def test_009a_R13_rejects_labels_that_would_break_the_marker_text():
    pipe = make_pipeline()
    assert pipe.process(1.0, 'cup (2)', 0.9, 50, 50, 1.0) == 'label'
    assert pipe.process(1.0, '  ', 0.9, 50, 50, 1.0) == 'label'


def test_009a_R13_mapped_area_check():
    grid = GridArea(0.0, 0.0, 1.0, 4, 4, [0, 0, 0, 0] + [0, 5, 5, 0] + [0] * 8)
    assert grid(1.5, 1.5) and not grid(0.5, 0.5) and not grid(-1.0, 1.5) and not grid(9.0, 1.5)
    inside = make_pipeline(tf=lambda s, t: Transform((1.5, 1.5, 0.0), IDENTITY), is_mapped=grid)
    assert inside.process(1.0, 'cup', 0.9, 50, 50, 1.0) == 'new'
    outside = make_pipeline(tf=lambda s, t: Transform((9.5, 1.5, 0.0), IDENTITY), is_mapped=grid)
    assert outside.process(1.0, 'cup', 0.9, 50, 50, 1.0) == 'area'
    assert outside.snapshot()['dropped_area'] == 1


# --- 009a-R12 through the pipeline ------------------------------------------------------------
def test_009a_R12_pipeline_counts_missing_tf_and_depth_and_publishes_nothing():
    pipe = make_pipeline(tf=lambda s, t: None)
    assert pipe.process(1.0, 'cup', 0.9, 50, 50, 1.0) == 'tf'
    assert make_pipeline().process(1.0, 'cup', 0.9, 50, 50, math.nan) == 'depth'
    assert pipe.snapshot()['dropped_tf'] == 1 and pipe.markers() == []


def test_009a_R12_filtered_detections_do_not_query_tf():
    queries = []
    pipe = make_pipeline(tf=lambda s, t: queries.append(s))
    pipe.process(1.0, 'cup', 0.1, 50, 50, 1.0)
    assert queries == []


# --- dedup ---------------------------------------------------------------------------------
def test_009a_R13_same_label_near_merges_far_or_other_label_does_not():
    ds = DetectionSet(merge_distance_m=0.5)
    a, new_a = ds.add('cup', 1.0, 1.0, 0.0, 1.0, 0.8)
    b, new_b = ds.add('cup', 1.3, 1.0, 0.0, 2.0, 0.9)
    assert new_a and not new_b and b is a and len(ds) == 1
    assert (a.x, a.confidence, a.count, a.stamp) == (pytest.approx(1.15), 0.9, 2, 1.0)
    assert ds.add('cup', 2.0, 1.0, 0.0, 3.0, 0.9)[1]                  # 0.85 m from the mean
    assert ds.add('bottle', 1.15, 1.0, 0.0, 3.0, 0.9)[1]
    assert len(ds) == 3 and len({d.det_id for d in ds.items()}) == 3


def test_009a_R13_merge_distance_is_a_parameter_and_set_rejects_nan():
    ds = DetectionSet(merge_distance_m=0.0)
    ds.add('cup', 1.0, 1.0, 0.0, 1.0, 0.9)
    assert ds.add('cup', 1.01, 1.0, 0.0, 1.0, 0.9)[1]
    with pytest.raises(ValueError):
        ds.add('cup', math.nan, 0.0, 0.0, 1.0, 0.9)


# --- depth support -------------------------------------------------------------------------
def test_009a_R12_stamp_cache_nearest_within_tolerance_with_size_and_age_limits():
    c = StampCache(max_size=3, max_age_s=1.0)
    for s in (1.0, 1.2, 1.4, 1.6):
        c.put(s, f'd{s}')
    assert len(c) == 3 and c.get(1.0, 0.01) is None                   # size limit evicted the oldest
    assert c.get(1.41, 0.05) == 'd1.4' and c.get(1.5, 0.01) is None
    c.put(3.0, 'late')
    assert len(c) == 1                                                # the rest is older than max_age_s


def test_009a_R12_depth_at_uses_median_of_valid_pixels():
    img = np.full((10, 10), 2.0, dtype=np.float32)
    img[5, 5] = math.nan
    img[5, 6] = 0.0
    assert depth_at(img, 5, 5) == pytest.approx(2.0)
    assert math.isnan(depth_at(np.zeros((4, 4), dtype=np.float32), 1, 1))
    assert math.isnan(depth_at(img, 20, 5)) and math.isnan(depth_at(img, math.nan, 5))
