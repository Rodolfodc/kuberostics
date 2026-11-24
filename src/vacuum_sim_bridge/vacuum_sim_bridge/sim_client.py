"""The simulator interface the bridge nodes depend on, and its ZeroMQ implementation.

Nodes receive a `SimClient` (injected), so tests use a fake and never import the ZMQ package. Only
`ZmqSimClient` touches `coppeliasim_zmqremoteapi_client`, imported lazily inside it. The ZMQ client is not
thread-safe: one instance per process, every call under a lock.
"""
import threading
from typing import List, Optional, Protocol, Sequence, Tuple


class SimClient(Protocol):
    def get_object(self, path: str) -> int: ...

    def get_float_array(self, handle: int, prop: str) -> List[float]:
        """Float-array property (empty list when it does not exist)."""

    def get_object_pose(self, handle: int) -> Sequence[float]:
        """World pose (x, y, z, qx, qy, qz, qw)."""

    def get_joint_position(self, handle: int) -> float: ...

    def get_sim_time(self) -> float: ...

    def set_wheel_speeds(self, robot: int, left: float, right: float) -> None:
        """Write both wheel speed signals (m/s) in ONE call so the controller never sees only one of them."""

    def read_rgb(self, handle: int) -> Optional[Tuple[bytes, int, int]]: ...

    def read_depth(self, handle: int) -> Optional[Tuple[bytes, int, int]]: ...


class ZmqSimClient:
    """CoppeliaSim 4.10 Remote API (ZeroMQ, port 23000). Calls are the ones exercised on this model in
    .sim/p3/verify*.py; behaviour is verified only against the live simulator (tests marked `sim`). The atomic
    wheel write is UNVERIFIED and falls back to two setFloatSignal calls with a warning."""

    def __init__(self, host: str = 'localhost', port: int = 23000, logger=None):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient   # lazy: not a rosdep key
        self._lock = threading.RLock()
        self._client = RemoteAPIClient(host=host, port=port)
        self._sim = self._client.require('sim')
        self._log = logger
        self._atomic_ok = True
        self._sandbox = None

    def get_object(self, path):
        with self._lock:
            return self._sim.getObject(path)

    def get_float_array(self, handle, prop):
        with self._lock:
            return list(self._sim.getFloatArrayProperty(handle, prop, {'noError': True}) or [])

    def get_object_pose(self, handle):
        with self._lock:
            return list(self._sim.getObjectPose(handle, self._sim.handle_world))

    def get_joint_position(self, handle):
        with self._lock:
            return float(self._sim.getJointPosition(handle))

    def get_sim_time(self):
        with self._lock:
            return float(self._sim.getSimulationTime())

    def set_wheel_speeds(self, robot, left, right):
        names = (f'{robot}leftVel', f'{robot}rightVel')
        with self._lock:
            if self._atomic_ok:
                try:
                    if self._sandbox is None:
                        self._sandbox = self._sim.getScript(self._sim.scripttype_sandbox)
                    code = (f'sim.setFloatSignal("{names[0]}", {left!r}) '
                            f'sim.setFloatSignal("{names[1]}", {right!r})')
                    self._sim.executeScriptString(code, self._sandbox)   # one request: both signals together
                    return
                except Exception as exc:   # API missing or refused: degrade, once
                    self._atomic_ok = False
                    if self._log:
                        self._log.warning(f'atomic wheel write unavailable ({exc}); using two calls')
            self._sim.setFloatSignal(names[1], right)     # the pair used in .sim/p3/verify3.py
            self._sim.setFloatSignal(names[0], left)

    def read_rgb(self, handle):
        # getVisionSensorImg(handle) -> (bytes, [w, h]); the kinect sensors render on their own during the
        # simulation (no handleVisionSensor in the probes that read this model).
        with self._lock:
            return self._unpack(self._sim.getVisionSensorImg(handle))

    def read_depth(self, handle):
        with self._lock:
            return self._unpack(self._sim.getVisionSensorDepth(handle))   # float32 bytes, normalised 0..1

    @staticmethod
    def _unpack(raw):
        if isinstance(raw, (list, tuple)) and len(raw) == 2 and len(raw[1]) == 2:
            return bytes(raw[0]), int(raw[1][0]), int(raw[1][1])
        return None
