"""The package's self-test as pytest tests (also: ros2 run swarm_intel selftest)."""
import pytest

from swarm_intel import selftest


@pytest.mark.parametrize('name,check', selftest.CHECKS)
def test_selftest(name, check):
    failed = [f'{what} [{detail}]' for what, ok, detail in check() if not ok]
    assert not failed, f'{name}: ' + '; '.join(failed)
