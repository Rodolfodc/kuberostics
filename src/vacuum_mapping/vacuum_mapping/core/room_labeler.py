"""RoomLabeler: names rooms from the objects seen in them (003-R21; cuttable, first in the cut order).

WHY: a refrigerator suggests a kitchen. Each observation votes for the label its class maps to; a label
wins only with enough evidence (`min_votes`) so one false detection does not rename a room.
Only computes (D-19): rooms and observations in, rooms out; the node decides when to call it and
keeps user labels safe by construction (`label_source == USER` is never touched).
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

import numpy as np

from .types import LabelSource, Logger, Room, null_logger


@dataclass(frozen=True)
class Rule:
    """Detection class -> room label, with the weight of one observation."""
    label: str
    weight: float = 1.0


@dataclass(frozen=True)
class Observation:
    """One detected object: class name and position in the map frame."""
    cls: str
    x: float
    y: float


class RoomLabeler:
    """Weighted vote of observations per room."""

    def __init__(self, rules: Mapping[str, Rule], min_votes: int = 2, log: Logger = null_logger) -> None:
        self._rules = dict(rules)
        self._min_votes = min_votes
        self._log = log

    def label_rooms(self, rooms: Iterable[Room], labels: np.ndarray, resolution: float,
                    origin_x: float, origin_y: float,
                    observations: Iterable[Observation]) -> Tuple[Room, ...]:
        """Return `rooms` with rule labels applied.

        `labels` holds `Room.index` per cell (output of `RoomTracker`). Per room that is not user-labelled:
        the label with the largest summed weight wins if at least `min_votes` observations voted for it;
        a tie in weight keeps the current label; no winner restores the default label (`id`), so evidence
        that disappears also removes a stale rule label. Observations outside any room or of a class
        without a rule are ignored.
        """
        weights: Dict[int, Dict[str, float]] = {}
        counts: Dict[int, Dict[str, int]] = {}
        for obs in sorted(observations, key=lambda o: (o.cls, o.x, o.y)):  # order-independent sums
            rule = self._rules.get(obs.cls)
            if rule is None:
                continue
            row = int(np.floor((obs.y - origin_y) / resolution))
            col = int(np.floor((obs.x - origin_x) / resolution))
            if not (0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]):
                continue
            index = int(labels[row, col])
            if index <= 0:
                continue
            weights.setdefault(index, {}).setdefault(rule.label, 0.0)
            weights[index][rule.label] += rule.weight
            counts.setdefault(index, {}).setdefault(rule.label, 0)
            counts[index][rule.label] += 1

        out: List[Room] = []
        for room in rooms:
            if room.label_source == LabelSource.USER:
                out.append(room)
                continue
            new = self._decide(room, weights.get(room.index, {}), counts.get(room.index, {}))
            if (new.label, new.label_source) != (room.label, room.label_source):
                self._log('room_labeler: %s %r (%s) -> %r (%s)' % (
                    room.id, room.label, room.label_source.name.lower(), new.label, new.label_source.name.lower()))
            out.append(new)
        return tuple(out)

    def _decide(self, room: Room, weights: Dict[str, float], counts: Dict[str, int]) -> Room:
        default = dataclasses.replace(room, label=room.id, label_source=LabelSource.DEFAULT)
        if not weights:
            return default
        best = max(weights.values())
        leaders = sorted(k for k, w in weights.items() if w == best)
        if len(leaders) > 1:
            return room  # ambiguous evidence: keep what the room has
        winner: Optional[str] = leaders[0]
        if counts[winner] < self._min_votes:
            return default
        return dataclasses.replace(room, label=winner, label_source=LabelSource.RULE)
