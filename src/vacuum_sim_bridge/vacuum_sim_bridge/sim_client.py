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

    def handle_vision_sensor(self, handle: int) -> None:
        """Render one frame of a sensor that is set to explicit handling."""

    def read_rgb(self, handle: int) -> Optional[Tuple[bytes, int, int]]: ...

    def read_depth(self, handle: int) -> Optional[Tuple[bytes, int, int]]: ...


_TRANSIENT_NAMES = ('ZMQError', 'Again', 'ConnectionError', 'TimeoutError')
_API_MARKERS = ('not found', 'nil value', 'no such', 'unknown function', 'not exist', 'attempt to call')


def is_api_error(exc: Exception) -> bool:
    """True only for 'the API does not exist / was refused' errors, never for connection problems."""
    if isinstance(exc, (OSError, TimeoutError)) or type(exc).__name__ in _TRANSIENT_NAMES:
        return False
    return any(m in str(exc).lower() for m in _API_MARKERS)


def _default_factory(host, port, timeout_ms):
    def make():
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient   # lazy: not a rosdep key
        client = RemoteAPIClient(host=host, port=port)
        try:   # bounded waits: a dead simulator must not block a call forever
            import zmq
            client.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
            client.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
            client.socket.setsockopt(zmq.LINGER, 0)
        except Exception:
            pass
        return client.require('sim')
    return make


class ZmqSimClient:
    """CoppeliaSim 4.10 Remote API (ZeroMQ, port 23000). Calls are the ones exercised on this model in
    .sim/p3/verify*.py. Any non-API failure (timeout, connection drop, simulator restart) drops the connection
    and re-raises; the next call reconnects through `sim_factory` (logged), so the nodes recover by themselves
    when the callers retry. Callers must also forget cached object handles on error (the nodes do).
    The atomic wheel write (two signals in one executeScriptString) degrades to two calls only on API errors."""

    def __init__(self, host: str = 'localhost', port: int = 23000, logger=None, sim=None, sim_factory=None,
                 timeout_ms: int = 3000):
        self._lock = threading.RLock()
        if sim_factory is None:
            sim_factory = (lambda: sim) if sim is not None else _default_factory(host, port, timeout_ms)
        self._factory = sim_factory
        self._sim = None
        self._log = logger
        self._atomic_ok = True
        self._sandbox = None
        self._ever_connected = False

    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _call(self, fn):
        with self._lock:
            if self._sim is None:
                self._sim = self._factory()
                self._sandbox = None
                self._info('reconnected to the simulator' if self._ever_connected
                           else 'connected to the simulator')
                self._ever_connected = True
            try:
                return fn(self._sim)
            except Exception as exc:
                if not is_api_error(exc):
                    if self._log:
                        self._log.warning(f'simulator connection dropped ({exc!r}); will reconnect')
                    self._sim = None
                raise

    def get_object(self, path):
        return self._call(lambda sim: sim.getObject(path))

    def get_float_array(self, handle, prop):
        return self._call(lambda sim: list(sim.getFloatArrayProperty(handle, prop, {'noError': True}) or []))

    def get_object_pose(self, handle):
        return self._call(lambda sim: list(sim.getObjectPose(handle, sim.handle_world)))

    def get_joint_position(self, handle):
        return self._call(lambda sim: float(sim.getJointPosition(handle)))

    def get_sim_time(self):
        return self._call(lambda sim: float(sim.getSimulationTime()))

    def _atomic_write(self, sim, names, left, right):
        if self._sandbox is None:
            self._sandbox = sim.getScript(sim.scripttype_sandbox)
        code = (f'sim.setFloatSignal("{names[0]}", {left!r}) '
                f'sim.setFloatSignal("{names[1]}", {right!r})')
        sim.executeScriptString(code, self._sandbox)       # one request: both signals together

    def set_wheel_speeds(self, robot, left, right):
        names = (f'{robot}leftVel', f'{robot}rightVel')

        def write(sim):
            if self._atomic_ok:
                try:
                    self._atomic_write(sim, names, left, right)
                    return
                except Exception as exc:
                    if not is_api_error(exc):
                        raise      # transient: handled by _call (reconnect), nothing latched
                    self._atomic_ok = False   # the API itself is missing or refused: degrade permanently
                    if self._log:
                        self._log.warning(f'atomic wheel write unavailable ({exc}); using two calls')
            sim.setFloatSignal(names[1], right)     # the pair used in .sim/p3/verify3.py
            sim.setFloatSignal(names[0], left)
        self._call(write)

    def handle_vision_sensor(self, handle):
        self._call(lambda sim: sim.handleVisionSensor(handle))

    def read_rgb(self, handle):
        # getVisionSensorImg(handle) -> (bytes, [w, h]), rows bottom to top
        return self._call(lambda sim: self._unpack(sim.getVisionSensorImg(handle)))

    def read_depth(self, handle):
        return self._call(lambda sim: self._unpack(sim.getVisionSensorDepth(handle)))   # float32, 0..1

    @staticmethod
    def _unpack(raw):
        if isinstance(raw, (list, tuple)) and len(raw) == 2 and len(raw[1]) == 2:
            return bytes(raw[0]), int(raw[1][0]), int(raw[1][1])
        return None
