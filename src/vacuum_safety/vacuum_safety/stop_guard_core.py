"""StopGuard core (D-22): turns the stop lock into an explicit zero-velocity command. Pure Python, no ROS.

twist_mux publishes nothing while a lock is active, so the base would keep its last command. The guard
tells the caller what to publish on `/myRobot/mux/stop`; the caller decides when to call `command()` (rate)
and what to do with it (D-19). The lock is latched: if its publisher goes silent the robot stays stopped.
"""
import logging
from typing import Any, Dict, Optional, Tuple


class StopGuard:
    def __init__(self, logger: Optional[Any] = None):
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._locked = False
        self._lock_changes = 0
        self._zero_commands = 0

    def on_lock(self, locked: bool) -> None:
        if locked == self._locked:
            return
        self._locked = locked
        self._lock_changes += 1
        if locked:
            self._log.warning('stop lock engaged: publishing zero velocity')
        else:
            self._log.info('stop lock released: zero velocity stopped')

    def command(self) -> Optional[Tuple[float, float]]:
        """(linear_x, angular_z) to publish, or None when there is nothing to publish."""
        if not self._locked:
            return None
        self._zero_commands += 1
        return (0.0, 0.0)

    def snapshot(self) -> Dict[str, Any]:
        return {'locked': self._locked, 'lock_changes': self._lock_changes,
                'zero_commands': self._zero_commands}
