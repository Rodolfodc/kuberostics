"""Marker adapter tests (003-R22), format of specs/009a-offload/markers-format.md. Duck-typed markers, no ROS."""
import os
from types import SimpleNamespace as NS

import numpy as np

from vacuum_mapping.core.marker_observations import ObservationBuffer, parse_marker
from vacuum_mapping.core.room_labeler import Observation, RoomLabeler
from vacuum_mapping.core.room_tracker import RoomTracker
from vacuum_mapping.core.types import LabelSource
from vacuum_mapping.rooms_io import read_rules

RULES_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'room_rules.yaml')


def marker(text='cup (3)', id=100003, x=2.35, y=-1.10, ns='detections_text'):
    return NS(ns=ns, id=id, text=text, pose=NS(position=NS(x=x, y=y, z=0.25)))


def test_003_R22_section_6_minimal_marker_gives_class_and_position():
    assert parse_marker(marker()) == (100003, Observation('cup', 2.35, -1.10))


def test_003_R22_label_is_lowercased_stripped_and_text_without_suffix_works():
    assert parse_marker(marker('  Dining Table  (7)'))[1].cls == 'dining table'
    assert parse_marker(marker('cell phone'))[1].cls == 'cell phone'


def test_003_R22_other_namespace_empty_text_or_empty_label_are_skipped():
    assert parse_marker(marker(ns='detections')) is None
    assert parse_marker(marker(ns='')) is None
    assert parse_marker(marker(text='')) is None
    assert parse_marker(marker(text=' (3)')) is None


def test_003_R22_non_finite_position_is_skipped():
    assert parse_marker(marker(x=float('nan'))) is None
    assert parse_marker(marker(y=float('inf'))) is None


def test_003_R22_republished_array_does_not_inflate_the_set():
    buf = ObservationBuffer()
    array = NS(markers=[marker('refrigerator (1)', 100001, 1.0, 1.0), marker('oven (2)', 100002, 1.5, 1.0)])
    assert buf.update(array) is True
    first = buf.observations()
    for _ in range(5):                                  # the projector republishes every 2 s
        assert buf.update(array) is False
    assert buf.observations() == first and len(buf) == 2


def test_003_R22_same_id_updates_in_place_and_new_id_is_added():
    buf = ObservationBuffer()
    buf.update([marker('cup (3)', 100003, 1.0, 1.0)])
    assert buf.update([marker('cup (3)', 100003, 1.2, 1.0)]) is True
    assert buf.observations() == [Observation('cup', 1.2, 1.0)]
    buf.update([marker('bed (4)', 100004, 3.0, 1.0)])
    assert [o.cls for o in buf.observations()] == ['cup', 'bed']      # ordered by id
    buf.clear()
    assert len(buf) == 0


def test_003_R22_logs_when_the_set_changes():
    logs = []
    ObservationBuffer(log=logs.append).update([marker()])
    assert logs and '1 observations' in logs[0]


def test_003_R21_shipped_rules_file_loads_and_votes_do_not_grow_with_republish():
    rules = read_rules(RULES_FILE)
    assert rules and all(k == k.lower() and '(' not in k for k in rules)
    raw = np.array([[1, 1, 1, 1, 2, 2, 2, 2]] * 3, dtype=np.int32)
    tracked = RoomTracker().track(None, raw, 1.0, 0.0, 0.0)
    buf = ObservationBuffer()
    array = NS(markers=[marker('oven (1)', 100001, 1.5, 1.5)])
    for _ in range(10):                                 # ten republishes of ONE oven: one vote, below min_votes 2
        buf.update(array)
    out = RoomLabeler(rules, min_votes=2).label_rooms(tracked.rooms, tracked.labels, 1.0, 0.0, 0.0,
                                                      buf.observations())
    assert all(r.label_source == LabelSource.DEFAULT for r in out)
    buf.update([marker('sink (2)', 100002, 2.5, 1.5)])
    out = RoomLabeler(rules, min_votes=2).label_rooms(tracked.rooms, tracked.labels, 1.0, 0.0, 0.0,
                                                      buf.observations())
    assert out[0].label == 'kitchen' and out[0].label_source == LabelSource.RULE
