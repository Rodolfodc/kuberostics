"""Pure logic of the escape_server stub (T011, 001-R07, 001-R16). No ROS imports (D-18)."""
from dataclasses import dataclass
from typing import Optional, Tuple

OUTCOMES = ('succeed', 'fail', 'hang')


@dataclass(frozen=True)
class ScriptStep:
    command: Optional[Tuple[float, float]]    # (linear_x, angular_z); None once the script is finished
    done: bool
    success: Optional[bool] = None            # meaningful only when done


class EscapeScript:
    """Scripted escape: reverse for reverse_s, turn for turn_s, then finish according to `outcome`.

    `hang` never finishes: it keeps returning a zero command (the mux input stays alive, the robot stays still)
    until the caller cancels. The caller owns the clock and the publishing (toolbox style, D-19).
    """

    def __init__(self, reverse_s: float, turn_s: float, reverse_linear_x: float, turn_angular_z: float,
                 outcome: str):
        if outcome not in OUTCOMES:
            raise ValueError(f'outcome must be one of {OUTCOMES}, got {outcome!r}')
        self._reverse_s = reverse_s
        self._turn_s = turn_s
        self._reverse = (float(reverse_linear_x), 0.0)
        self._turn = (0.0, float(turn_angular_z))
        self._outcome = outcome

    def step(self, elapsed_s: float) -> ScriptStep:
        if elapsed_s < self._reverse_s:
            return ScriptStep(self._reverse, False)
        if elapsed_s < self._reverse_s + self._turn_s:
            return ScriptStep(self._turn, False)
        if self._outcome == 'hang':
            return ScriptStep((0.0, 0.0), False)
        return ScriptStep(None, True, self._outcome == 'succeed')
