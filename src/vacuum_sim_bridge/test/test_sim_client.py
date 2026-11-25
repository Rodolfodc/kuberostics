"""ZmqSimClient atomic wheel write: degrade only on API errors, retry on transient ones. Basis: D-06, Art. II."""
import pytest

from vacuum_sim_bridge.sim_client import ZmqSimClient, is_api_error


class FakeApi:
    handle_world = 0
    scripttype_sandbox = 1

    def __init__(self, exec_errors):
        self.exec_errors = list(exec_errors)
        self.signals = []
        self.executed = 0

    def getScript(self, kind):
        return 7

    def executeScriptString(self, code, script):
        self.executed += 1
        if self.exec_errors:
            err = self.exec_errors.pop(0)
            if err:
                raise err
        self.signals.append(('atomic', code))

    def setFloatSignal(self, name, value):
        self.signals.append((name, value))


def test_classifier():
    assert is_api_error(RuntimeError('function not found'))
    assert not is_api_error(ConnectionError('not found'))
    assert not is_api_error(RuntimeError('socket closed'))


def test_transient_error_is_raised_and_does_not_latch():
    api = FakeApi([TimeoutError('timeout'), None])
    c = ZmqSimClient(sim=api, atomic_wheel_write=True)
    with pytest.raises(TimeoutError):
        c.set_wheel_speeds(5, 0.1, 0.1)
    c.set_wheel_speeds(5, 0.1, 0.1)             # retried, still atomic
    assert api.signals[-1][0] == 'atomic' and api.executed == 2


def test_api_error_degrades_permanently_to_two_calls():
    api = FakeApi([RuntimeError('attempt to call a nil value (field executeScriptString)')])
    c = ZmqSimClient(sim=api, atomic_wheel_write=True)
    c.set_wheel_speeds(5, 0.1, 0.2)
    c.set_wheel_speeds(5, 0.3, 0.4)
    assert api.executed == 1
    assert api.signals == [('5rightVel', 0.2), ('5leftVel', 0.1), ('5rightVel', 0.4), ('5leftVel', 0.3)]


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warning(self, m):
        self.lines.append(('warning', m))


def test_reconnects_after_a_connection_failure_and_logs():
    made = []

    class Api(FakeApi):
        def getObject(self, path):
            if len(made) == 1:
                raise ConnectionError('socket closed')
            return 3

    def factory():
        made.append(1)
        return Api([])

    log = Logger()
    c = ZmqSimClient(sim_factory=factory, logger=log)
    with pytest.raises(ConnectionError):
        c.get_object('/myRobot')
    assert c.get_object('/myRobot') == 3
    assert len(made) == 2
    assert [k for k, m in log.lines if 'reconnected' in m] == ['info']
    assert any(k == 'warning' for k, _ in log.lines)


def test_factory_failure_propagates_and_is_retried():
    calls = []

    def factory():
        calls.append(1)
        if len(calls) == 1:
            raise TimeoutError('no simulator')
        return FakeApi([])

    c = ZmqSimClient(sim_factory=factory)
    with pytest.raises(TimeoutError):
        c.get_sim_time()
    assert len(calls) == 1


def test_drop_closes_the_old_connection():
    closed = []

    def factory():
        return FakeApi([])
    factory.close_last = lambda: closed.append(1)
    c = ZmqSimClient(sim_factory=factory)
    api_calls = []
    c._call(lambda sim: api_calls.append(1))
    with pytest.raises(ConnectionError):
        c._call(lambda sim: (_ for _ in ()).throw(ConnectionError('x')))
    assert closed == [1]


class NoExecApi:
    """Real attribute absence, like the 4.10 sim object: no executeScriptString at all."""
    handle_world = 0
    scripttype_sandbox = 1

    def __init__(self):
        self.signals = []

    def getScript(self, kind):
        return 7

    def setFloatSignal(self, name, value):
        self.signals.append((name, value))


def test_missing_function_is_an_api_error_and_latches_the_fallback():
    assert is_api_error(AttributeError("'X' object has no attribute 'executeScriptString'"))
    log = Logger()
    api = NoExecApi()
    c = ZmqSimClient(sim=api, atomic_wheel_write=True, logger=log)
    c.set_wheel_speeds(5, 0.1, 0.2)          # falls back in the same call, no exception
    c.set_wheel_speeds(5, 0.3, 0.4)
    assert api.signals == [('5rightVel', 0.2), ('5leftVel', 0.1), ('5rightVel', 0.4), ('5leftVel', 0.3)]
    assert [k for k, _ in log.lines].count('warning') == 1
    assert not any('dropped' in m for _, m in log.lines)    # no reconnect loop


def test_two_calls_are_the_default():
    api = FakeApi([])
    c = ZmqSimClient(sim=api)
    c.set_wheel_speeds(5, 0.1, 0.2)
    assert api.executed == 0 and api.signals == [('5rightVel', 0.2), ('5leftVel', 0.1)]
