"""RoomLabeler tests (003-R21). Two rooms on a 1 m grid: room 1 is x in [0, 4), room 2 is x in [4, 8)."""
import dataclasses

import numpy as np

from vacuum_mapping.core.room_labeler import Observation, Rule, RoomLabeler
from vacuum_mapping.core.room_tracker import RoomTracker
from vacuum_mapping.core.types import LabelSource

RULES = {
    'refrigerator': Rule('kitchen', 2.0),
    'oven': Rule('kitchen', 1.0),
    'bed': Rule('bedroom', 2.0),
    'sofa': Rule('living_room', 1.0),
}


def make_rooms():
    raw = np.array([[1, 1, 1, 1, 2, 2, 2, 2]] * 3, dtype=np.int32)
    tracked = RoomTracker().track(None, raw, 1.0, 0.0, 0.0)
    return tracked.rooms, tracked.labels


def label(observations, rules=RULES, min_votes=2, rooms=None, log=None):
    rooms_, labels = make_rooms()
    labeler = RoomLabeler(rules, min_votes=min_votes, **({'log': log} if log else {}))
    out = labeler.label_rooms(rooms or rooms_, labels, 1.0, 0.0, 0.0, observations)
    return {r.index: r for r in out}


def test_003_R21_weighted_vote_picks_the_label_with_most_weight():
    out = label([Observation('refrigerator', 1.5, 1.5), Observation('oven', 2.5, 0.5),
                 Observation('sofa', 1.0, 1.0), Observation('sofa', 3.0, 2.0)])
    # kitchen 2 + 1 = 3 votes weight, living_room 1 + 1 = 2
    assert out[1].label == 'kitchen' and out[1].label_source == LabelSource.RULE
    assert out[2].label == 'room_02' and out[2].label_source == LabelSource.DEFAULT


def test_003_R21_fewer_than_min_votes_keeps_the_default_label():
    out = label([Observation('refrigerator', 1.5, 1.5)])
    assert out[1].label == 'room_01' and out[1].label_source == LabelSource.DEFAULT


def test_003_R21_min_votes_counts_observations_of_the_winning_label_only():
    out = label([Observation('bed', 5.0, 1.0), Observation('bed', 6.0, 1.0), Observation('sofa', 5.5, 1.5)])
    assert out[2].label == 'bedroom'  # 2 bed observations, weight 4 against 1


def test_003_R21_min_votes_of_one_accepts_a_single_observation():
    out = label([Observation('bed', 5.0, 1.0)], min_votes=1)
    assert out[2].label == 'bedroom'


def test_003_R21_tie_in_weight_keeps_the_current_label():
    out = label([Observation('bed', 1.0, 1.0), Observation('bed', 1.5, 1.0),
                 Observation('refrigerator', 2.0, 1.0), Observation('refrigerator', 2.5, 1.0)])
    assert out[1].label == 'room_01' and out[1].label_source == LabelSource.DEFAULT


def test_003_R21_never_overrides_a_user_label():
    rooms, _ = make_rooms()
    user = tuple(dataclasses.replace(r, label='my office', label_source=LabelSource.USER) if r.index == 1 else r
                 for r in rooms)
    out = label([Observation('refrigerator', 1.0, 1.0), Observation('oven', 2.0, 1.0)], rooms=user)
    assert out[1].label == 'my office' and out[1].label_source == LabelSource.USER


def test_003_R21_observations_outside_rooms_or_of_unknown_classes_are_ignored():
    out = label([Observation('refrigerator', 20.0, 1.0), Observation('refrigerator', -3.0, 1.0),
                 Observation('toaster', 1.0, 1.0), Observation('toaster', 1.5, 1.0)])
    assert all(r.label_source == LabelSource.DEFAULT for r in out.values())


def test_003_R21_a_rule_label_reverts_to_default_when_the_evidence_goes_away():
    rooms, _ = make_rooms()
    ruled = tuple(dataclasses.replace(r, label='kitchen', label_source=LabelSource.RULE) if r.index == 1 else r
                  for r in rooms)
    out = label([], rooms=ruled)
    assert out[1].label == 'room_01' and out[1].label_source == LabelSource.DEFAULT


def test_003_R21_result_is_deterministic_in_observation_order():
    obs = [Observation('refrigerator', 1.0, 1.0), Observation('oven', 2.0, 1.0),
           Observation('sofa', 1.0, 2.0), Observation('sofa', 2.0, 2.0)]
    assert label(obs) == label(list(reversed(obs)))


def test_003_R21_logger_reports_label_changes():
    lines = []
    label([Observation('refrigerator', 1.0, 1.0), Observation('oven', 2.0, 1.0)], log=lines.append)
    assert any('room_01' in line and 'kitchen' in line for line in lines)
