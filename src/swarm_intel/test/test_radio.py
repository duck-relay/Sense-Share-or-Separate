"""The headless radio link (mission.Radio): ideal by default, lossy / late / range-limited for the controls."""
from swarm_intel import config as C
from swarm_intel.mission import Radio


def _tick(radio, msgs, positions=None):
    """One control step: every drone in msgs publishes to the others, then the link delivers."""
    ids = list(msgs)
    for i, m in msgs.items():
        radio.publish(i, m, receivers=ids, positions=positions)
    radio.deliver()


def test_ideal_link_hands_over_the_latest_message():
    r = Radio(0, 0, C.RadioParams())
    _tick(r, {1: {'k': 1}, 2: {'k': 2}})
    assert r.inbox(1) == {2: {'k': 2}}
    _tick(r, {1: {'k': 3}, 2: {'k': 4}})
    assert r.inbox(2) == {1: {'k': 3}}


def test_delay_and_newest_message_kept():
    r = Radio(0, 0, C.RadioParams(delay_ticks=(2, 2)))
    _tick(r, {1: {'n': 0}, 2: {'n': 0}})
    assert r.inbox(2) == {}                       # sent at tick 0, due at tick 2
    _tick(r, {1: {'n': 1}, 2: {'n': 1}})
    assert r.inbox(2) == {}
    _tick(r, {1: {'n': 2}, 2: {'n': 2}})
    assert r.inbox(2) == {1: {'n': 0}}            # the first message arrives two steps late
    _tick(r, {1: {'n': 3}, 2: {'n': 3}})
    assert r.inbox(2) == {1: {'n': 1}}


def test_out_of_order_arrival_keeps_the_newest():
    r = Radio(0, 0, C.RadioParams(delay_ticks=(0, 5)))
    last = -1
    for n in range(40):
        _tick(r, {1: {'n': n}, 2: {'n': n}})
        got = r.inbox(2).get(1)
        if got is not None:
            assert last <= got['n'] <= n          # never older than what it already had, never from the future
            last = got['n']
    assert last >= 30


def test_total_loss_and_range():
    r = Radio(0, 0, C.RadioParams(loss=1.0))
    for _ in range(5):
        _tick(r, {1: {'a': 1}, 2: {'a': 2}})
    assert r.inbox(1) == {} and r.inbox(2) == {}
    r = Radio(0, 0, C.RadioParams(range_m=5.0))
    _tick(r, {1: {'a': 1}, 2: {'a': 2}, 3: {'a': 3}}, positions={1: (0, 0, 1), 2: (3, 0, 1), 3: (9, 0, 1)})
    assert set(r.inbox(1)) == {2} and set(r.inbox(3)) == set()


def test_rounds_timed_to_the_link_change_nothing_on_an_ideal_link():
    """The delay-aware turn trading (package 1.8.5) flies exactly like the fixed schedule when every message
    arrives in time, so the studies with the ideal radio stay valid."""
    from swarm_intel import layouts
    from swarm_intel.plant import run_headless
    old = C.TTC_ADAPTIVE
    try:
        layouts.apply(3)
        out = []
        for adaptive in (True, False):
            C.TTC_ADAPTIVE = adaptive
            rep, _ = run_headless('cam3_sensor_swarm', seed=1)
            out.append(rep)
    finally:
        C.TTC_ADAPTIVE = old
        layouts.apply(0)
    from swarm_intel import metrics as M
    a, b = (M.extract(r) for r in out)
    same = {k: (a[k], b[k]) for k in a if not k.startswith('fleet_plan_time') and a[k] != b[k]}
    assert not same, same
