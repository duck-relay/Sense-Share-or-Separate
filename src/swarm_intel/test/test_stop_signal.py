"""A stop signal ends the nodes' spin loop between callbacks instead of interrupting rclpy's C++ code."""
import os
import signal

import pytest

from swarm_intel.stop_signal import StopSignal, is_shutdown_error


class _Rclpy:
    def __init__(self, ok):
        self._ok = ok

    def ok(self):
        return self._ok


@pytest.fixture
def restore_handlers():
    old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    yield
    for s, h in old.items():
        signal.signal(s, h)


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_first_signal_only_sets_the_flag(restore_handlers, sig):
    stop = StopSignal()
    assert not stop
    os.kill(os.getpid(), sig)          # handled synchronously in the main thread: no exception here
    assert stop and stop.signum == sig


def test_second_signal_interrupts(restore_handlers):
    stop = StopSignal()
    os.kill(os.getpid(), signal.SIGINT)
    assert stop
    with pytest.raises(KeyboardInterrupt):
        os.kill(os.getpid(), signal.SIGINT)


def test_shutdown_errors(restore_handlers):
    stop = StopSignal()
    err = RuntimeError('Unable to convert call argument to Python object')
    assert not is_shutdown_error(err, stop, _Rclpy(True))     # a real error while running is raised
    assert is_shutdown_error(err, stop, _Rclpy(False))        # context already shut down
    os.kill(os.getpid(), signal.SIGINT)
    assert is_shutdown_error(err, stop, _Rclpy(True))         # stopping: not an error

    class ExternalShutdownException(Exception):
        pass
    assert is_shutdown_error(ExternalShutdownException(), None, _Rclpy(True))
