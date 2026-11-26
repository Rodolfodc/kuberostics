"""Pure analysis of a detector kill run (009a-R15, R16). No ROS imports: tested with pytest, fed by analyze_offload_bag.py.

Inputs are plain tuples with times in seconds on one clock (the bag receive time, same host as the kill log):
  statuses:      (t, state, forwarded, dropped_inactive, dropped_stale, dropped_watermark, transitions)   OffloadStatus, 1 Hz
  notifications: (t, code)                                                                               /myRobot/notifications
  modes:         (t, mode)                                                                               /myRobot/behaviour/mode
  detections:    (t, stamp_s)                                                                            /myRobot/perception/detections
  kills:         (kill_t, ready_t)                                                                       from the chaos script
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

CLOSED, OPEN, HALF_OPEN = 0, 1, 2
DOWN_CODE = "CLOUD_DETECTOR_DOWN"
RECOVERED_CODE = "CLOUD_DETECTOR_RECOVERED"
COUNTERS = ("forwarded", "dropped_inactive", "dropped_stale", "dropped_watermark")


@dataclass
class KillResult:
    index: int
    kill_t: float
    ready_t: Optional[float]
    time_to_open_s: Optional[float]          # from the DOWN notification when present, else first OPEN status (1 s resolution)
    time_to_open_source: str                 # "notification" | "status" | "none"
    time_to_closed_s: Optional[float]        # kill -> first CLOSED status after OPEN (or RECOVERED notification)
    time_closed_after_ready_s: Optional[float]
    open_within_bound: Optional[bool]        # R15: <= open_threshold_s + 0.5
    mode_changed: Optional[bool]             # R15: orchestrator mode must not change
    counter_deltas: dict = field(default_factory=dict)  # counters at the window end minus at the kill


@dataclass
class Report:
    kills: List[KillResult]
    monotonic_violations: int                # R16: detections with stamp <= the running maximum
    detections: int
    bound_s: float


def _first_after(items, t0, pred):
    for it in items:
        if it[0] >= t0 and pred(it):
            return it
    return None


def _status_at(statuses, t):
    """Last status at or before t (counters are cumulative)."""
    last = None
    for s in statuses:
        if s[0] <= t:
            last = s
        else:
            break
    return last


def monotonic_violations(detections: List[Tuple[float, float]]) -> int:
    """R16: a detection whose header stamp is not strictly above everything published before it."""
    top = None
    bad = 0
    for _, stamp in detections:
        if top is not None and stamp <= top:
            bad += 1
        else:
            top = stamp
    return bad


def analyze(statuses, notifications, modes, detections, kills, open_threshold_s: float = 1.5) -> Report:
    statuses = sorted(statuses)
    notifications = sorted(notifications)
    modes = sorted(modes)
    bound = open_threshold_s + 0.5
    results = []
    for i, (kill_t, ready_t) in enumerate(kills, start=1):
        next_kill = kills[i][0] if i < len(kills) else float("inf")
        down = _first_after(notifications, kill_t, lambda n: n[1] == DOWN_CODE and n[0] < next_kill)
        st_open = _first_after(statuses, kill_t, lambda s: s[1] in (OPEN, HALF_OPEN) and s[0] < next_kill)
        if down is not None:
            t_open, src = down[0] - kill_t, "notification"
        elif st_open is not None:
            t_open, src = st_open[0] - kill_t, "status"
        else:
            t_open, src = None, "none"
        open_t = kill_t + t_open if t_open is not None else None
        closed = None
        if open_t is not None:
            closed = _first_after(statuses, open_t, lambda s: s[1] == CLOSED and s[0] < next_kill)
        rec = _first_after(notifications, kill_t, lambda n: n[1] == RECOVERED_CODE and n[0] < next_kill)
        closed_t = None
        if closed is not None:
            closed_t = closed[0]
        if rec is not None and (closed_t is None or rec[0] < closed_t):
            closed_t = rec[0]
        end_t = closed_t if closed_t is not None else min(next_kill, statuses[-1][0] if statuses else kill_t)
        window_modes = {m[1] for m in modes if kill_t <= m[0] <= end_t}
        before = _status_at(statuses, kill_t)
        after = _status_at(statuses, end_t)
        deltas = {}
        if before is not None and after is not None:
            for name, idx in zip(COUNTERS, (2, 3, 4, 5)):
                deltas[name] = after[idx] - before[idx]
        results.append(KillResult(
            index=i, kill_t=kill_t, ready_t=ready_t,
            time_to_open_s=t_open, time_to_open_source=src,
            time_to_closed_s=(closed_t - kill_t) if closed_t is not None else None,
            time_closed_after_ready_s=(closed_t - ready_t) if (closed_t is not None and ready_t is not None) else None,
            open_within_bound=(t_open <= bound) if t_open is not None else None,
            mode_changed=(len(window_modes) > 1) if window_modes else None,
            counter_deltas=deltas))
    return Report(results, monotonic_violations(detections), len(detections), bound)


def _f(x, nd=2):
    return "n/a" if x is None else f"{x:.{nd}f}"


def to_markdown(rep: Report) -> str:
    lines = [
        "| kill | to OPEN (s) | source | within bound | to CLOSED (s) | after Pod ready (s) | mode changed | forwarded | inactive | stale | watermark |",
        "|---|---|---|---|---|---|---|---|---|---|---|"]
    for k in rep.kills:
        d = k.counter_deltas
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            k.index, _f(k.time_to_open_s), k.time_to_open_source, k.open_within_bound, _f(k.time_to_closed_s),
            _f(k.time_closed_after_ready_s), k.mode_changed, d.get("forwarded", "n/a"), d.get("dropped_inactive", "n/a"),
            d.get("dropped_stale", "n/a"), d.get("dropped_watermark", "n/a")))
    lines.append("")
    lines.append(f"Bound for OPEN (R15): {rep.bound_s:.2f} s. Detections: {rep.detections}, "
                 f"stamp violations (R16): {rep.monotonic_violations}.")
    return "\n".join(lines)
