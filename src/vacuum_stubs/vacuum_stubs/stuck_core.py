"""Pure logic of the stuck_detector stub (T012, 001-R07). No ROS imports (D-18). Real detection is spec 005."""


class StuckStub:
    def __init__(self):
        self._events = 0

    def trigger(self) -> str:
        """Registers one trigger and returns the cause of the event to publish."""
        self._events += 1
        return 'NO_PROGRESS'

    def snapshot(self) -> dict:
        return {'events': self._events}
