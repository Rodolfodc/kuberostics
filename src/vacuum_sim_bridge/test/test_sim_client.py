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
    c = ZmqSimClient(sim=api)
    with pytest.raises(TimeoutError):
        c.set_wheel_speeds(5, 0.1, 0.1)
    c.set_wheel_speeds(5, 0.1, 0.1)             # retried, still atomic
    assert api.signals[-1][0] == 'atomic' and api.executed == 2


def test_api_error_degrades_permanently_to_two_calls():
    api = FakeApi([RuntimeError('attempt to call a nil value (field executeScriptString)')])
    c = ZmqSimClient(sim=api)
    c.set_wheel_speeds(5, 0.1, 0.2)
    c.set_wheel_speeds(5, 0.3, 0.4)
    assert api.executed == 1
    assert api.signals == [('5rightVel', 0.2), ('5leftVel', 0.1), ('5rightVel', 0.4), ('5leftVel', 0.3)]
