"""CoverageRun: one CoverRoom goal from region to result (005-R16, R17, R20, R21, R32). Pure Python, no ROS.

Glue between the toolbox cores: it builds the region, plans the lanes, owns the tracker and the session and
keeps the suspended session for the resume window. It decides nothing about Nav2, TF or topics: the node reads
them, passes the values in (`begin`, `on_pose`, `goal_result`) and acts on what `step` returns (D-18, D-19).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from vacuum_mapping.core.types import Logger, Raster, Room, null_logger

from .coverage_planner import CoveragePlanner
from .coverage_session import CoverageSession, ResumeStore, SessionLimits, Step
from .coverage_tracker import CoverageTracker, coverable_mask
from .region_builder import RegionBuilder


class ConfigError(ValueError):
    """The parameters cannot work together (005-R21): configure fails."""


@dataclass(frozen=True)
class CoverageConfig:
    """Defaults of the contract (`parameters.coverage_server` of topics.yaml); placeholders until the robot
    profile exists. The node passes its parameters, nothing is hard-coded below."""
    lane_spacing_m: float = 0.25
    robot_radius_m: float = 0.18
    margin_m: float = 0.05
    min_segment_m: float = 0.30
    tool_radius_m: float = 0.15
    target_ratio: float = 0.95
    min_acceptable_ratio: float = 0.70
    max_skip_ratio: float = 0.3
    coverage_timeout_s: float = 1800.0
    waypoint_timeout_s: float = 120.0
    resume_window_s: float = 120.0
    max_region_area_m2: float = 150.0

    def validate(self) -> None:
        if self.lane_spacing_m <= 0 or self.tool_radius_m <= 0:
            raise ConfigError('lane_spacing_m and tool_radius_m must be positive')
        if self.lane_spacing_m > 2 * self.tool_radius_m:
            raise ConfigError('lane_spacing_m %.3f > 2 * tool_radius_m %.3f: the lanes would leave gaps'
                              % (self.lane_spacing_m, 2 * self.tool_radius_m))
        if not 0.0 < self.min_acceptable_ratio <= self.target_ratio <= 1.0:
            raise ConfigError('need 0 < min_acceptable_ratio (%.2f) <= target_ratio (%.2f) <= 1'
                              % (self.min_acceptable_ratio, self.target_ratio))
        if self.waypoint_timeout_s <= 0 or self.coverage_timeout_s <= 0 or self.resume_window_s < 0:
            raise ConfigError('timeouts must be positive')
        if self.max_region_area_m2 <= 0 or self.max_skip_ratio < 0:
            raise ConfigError('max_region_area_m2 must be positive and max_skip_ratio not negative')


class CoverageRun:
    def __init__(self, config: CoverageConfig, log: Logger = null_logger, max_pose_gap_m: float = 1.0) -> None:
        config.validate()
        self._cfg = config
        self._log = log
        self._max_gap = max_pose_gap_m
        self._planner = CoveragePlanner(config.lane_spacing_m, config.robot_radius_m, config.margin_m,
                                        config.min_segment_m)
        self._regions = RegionBuilder(config.max_region_area_m2, log)
        self._store = ResumeStore(config.resume_window_s, log)
        self._limits = SessionLimits(
            waypoint_timeout_s=config.waypoint_timeout_s, coverage_timeout_s=config.coverage_timeout_s,
            max_skip_ratio=config.max_skip_ratio, target_ratio=config.target_ratio,
            min_acceptable_ratio=config.min_acceptable_ratio)
        self._room_id = ''
        self._session: Optional[CoverageSession] = None
        self._last_pose: Optional[Tuple[float, float]] = None
        self.region_source = ''
        self.pose_samples = 0
        self.max_hop_m = 0.0          # largest distance between two consecutive poses of the goal
        self.unbridged_hops = 0       # hops longer than max_pose_gap_m: credited at their end only

    # ---- goal lifecycle -------------------------------------------------------------------------------
    def begin(self, t: float, raster: Raster, robot_xy: Tuple[float, float], room_id: str, passes: int,
              rooms: Optional[Sequence[Room]] = None, labels: Optional[np.ndarray] = None) -> bool:
        """Starts a goal; True when it resumed a suspended one (005-R32). Raises `UnknownRoom` or `NoRegion`
        (the node rejects the goal)."""
        self._room_id = room_id
        self._last_pose = None
        self.pose_samples, self.max_hop_m, self.unbridged_hops = 0, 0.0, 0
        if self._store.matches(room_id, t):
            saved = self._store.take(room_id, t)
            self._session = saved
            saved.resume(t)
            self.region_source = 'resumed'
            return True
        # validate first: a rejected goal (unknown room, no region) leaves the saved session untouched
        region = self._regions.build(raster, robot_xy, room_id, rooms, labels)
        self._store.clear()          # an accepted goal for another room (or after the window) drops the old state
        self.region_source = region.source
        waypoints = self._planner.plan(raster, region.mask, passes)
        coverable = coverable_mask(raster, region.mask, self._planner.clearance_m, self._cfg.tool_radius_m,
                                   self._cfg.lane_spacing_m)
        tracker = CoverageTracker(raster, coverable, self._cfg.tool_radius_m, region.mask)
        self._session = CoverageSession(waypoints, tracker, passes, self._limits, self._log)
        self._session.start(t)
        return False

    def on_pose(self, x: float, y: float) -> None:
        """Pose of the robot in the map frame. Hops up to `max_pose_gap_m` are filled in with a straight
        segment (a pose stream sampled too sparsely); a longer jump (relocalisation) credits only its end."""
        tracker = self.session.tracker
        last = self._last_pose
        self.pose_samples += 1
        hop = 0.0 if last is None else math.hypot(x - last[0], y - last[1])
        self.max_hop_m = max(self.max_hop_m, hop)
        if last is not None and hop <= self._max_gap:
            tracker.mark_segment(last, (x, y))
        else:
            if last is not None:
                self.unbridged_hops += 1
            tracker.mark_pose(x, y)
        self._last_pose = (x, y)

    def step(self, t: float) -> Step:
        return self.session.step(t)

    def goal_result(self, t: float, reached: bool) -> None:
        self.session.goal_result(t, reached)

    def suspend(self, t: float) -> None:
        """The goal was cancelled: keep the session for `resume_window_s` (005-R32)."""
        self.session.suspend(t)
        self._store.save(self._room_id, self.session, t)

    def end(self) -> None:
        """The goal ended with success or failure: the saved state is cleared (005-R32)."""
        self._store.clear()

    # ---- state ----------------------------------------------------------------------------------------
    @property
    def session(self) -> CoverageSession:
        if self._session is None:
            raise RuntimeError('no goal started')
        return self._session

    @property
    def covered_ratio(self) -> float:
        return self.session.covered_ratio

    def covered_grid(self) -> np.ndarray:
        return self.session.tracker.covered_grid()
