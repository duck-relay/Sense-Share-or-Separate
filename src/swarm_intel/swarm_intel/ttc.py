"""Swarm give-way by trading passage turns, for the lowest energy of the whole fleet.

No drone has a fixed priority. Every drone flies to its own goal; where the routes of drones with
the swarm radio would bring them closer than the separation standard, the drones involved decide
together who passes first and what the others do (hold, slow down, step left or right, climb or
descend), so that the fleet as a whole spends the least energy.

Everything runs on the drones (edge computing): every drone with the radio runs its own TurnTrader,
which takes part in a trading round every TTC_PERIOD s over the radio, in three phases:

1. Routes (round start). The drone puts its ROUTE intent in its radio message: where it will be over
   the next 10 s if it simply follows its route at its planned speed.
2. Prices (0.1 s later). From the round's route intents every drone finds the conflicts: two drones
   conflict when their route intents come closer than the downwash-aware separation standard + the
   give-way buffer; conflicting drones form clusters (connected groups). The drone then prices every
   passage turn it could hold in its cluster: for each set A of members that would pass before it, the
   extra energy (J) of its cheapest way to keep the standard + buffer to all of A over the next 10 s -
   hold or slow down (the shortest sufficient delay), step left or right (0.6-2.4 m off its route,
   where its map is free), or climb / descend 0.6 m - as flight time lost x its power, plus the climb
   (DroneAgent.turn_cost, from its own route, map and energy model), and sends its price list.
3. Trading (0.2 s later). Endowment: the members' last orders (as they sent them), newcomers joining
   first-come-first-served (the drone that would reach the others' routes first owns the earlier
   turn; drones that would reach them in the same 0.5 s are ordered by a draw that every drone
   computes alike from the round number, so no drone number is favoured). The drones trade turns
   along cycles - each drone gives up its turn to the next drone in the cycle and takes the turn of
   the one before it - when that lowers the cluster's total energy by more than TTC_SWITCH. With at
   most five drones every order is evaluated, so the cluster flies its lowest-energy order. Any drone
   may end up first; a drone that cannot get out of the way (no safe manoeuvre) is very expensive
   behind others and therefore passes first.

   Relation to top trading cycles (Shapley & Scarf): the trades are trading cycles from an endowment, as
   in TTC, but the objective differs. Classic TTC lets every trader point to the turn it prefers and
   never leaves one worse off than its own turn (individual rationality); here the fleet's total energy
   is minimised, so a drone gives up its turn whenever that saves the fleet energy. ("ttc" in names
   and options stands for this turn trading.)

Every drone runs the same deterministic computation on the same messages of the round, so all reach
the same order without a coordinator. A drone then gives way only to the drones before it in its
cluster's order.

ROUNDS TIMED TO THE LINK (C.TTC_ADAPTIVE, package 1.8.5). Every radio message carries its send time.
Each drone measures the delay of every new message it receives, keeps the largest over the last
DELAY_WINDOW s and sends it along ('dl', in control steps); all drones space the three phases by the
largest delay any drone reports (held for DELAY_HOLD s) plus one step - at least PHASE_PRICES - and a
round lasts at least TTC_PERIOD and two phases plus two steps. With an ideal link (no delay) this is the
fixed schedule above. A drone whose round message from another drone did not arrive in time cannot
know whether their routes conflict: until a round with it succeeds that drone is not 'covered', and the
agent avoids it reactively with the give-way buffer (DroneAgent._covered). With C.TTC_ADAPTIVE = False
(study --ttc-fixed) the trading runs the fixed schedule without this fallback, as in package 1.8.4.
"""
import itertools
import math

import numpy as np

from . import config as C
from . import downwash as DW

GIVE_WAYS = ('ttc', 'priority')
GIVE_WAY_NAMES = {'ttc': 'turn trading (TTC)', 'priority': 'fixed priority'}
TTC_PERIOD = 0.5          # s between trading rounds (at least; longer on a slow link)
PHASE_PRICES = 0.1        # s into a round: routes are in, every drone prices its turns (ideal link)
PHASE_TRADE = 0.2         # s into a round: prices are in, every drone trades (ideal link)
DELAY_WINDOW = 2.0        # s: the link delay a drone reports is the largest it measured over this window
DELAY_HOLD = 5.0          # s: the schedule follows the largest delay reported over this time
TTC_SWITCH = 100.0        # J: a trade must save at least this much fleet energy (no flip-flopping)
CONFLICT_BUFFER = 0.5     # m on top of the separation standard (the give-way buffer)


def _route_conflict(a, b, buf=CONFLICT_BUFFER):
    """Do the route intents a and b ({'route': (K, 2), 'z': height}) come closer than the separation
    standard + buf?"""
    ra, rb = a['route'], b['route']
    k = min(len(ra), len(rb))
    d = np.linalg.norm(np.asarray(ra, float)[:k] - np.asarray(rb, float)[:k], axis=1)
    return bool(d.min() < DW.required_separation(a['z'] - b['z']) + buf)


def _entry_time(a, b, buf=CONFLICT_BUFFER):
    """Index of the first route-intent sample at which drone a comes within the separation standard
    of drone b's route (anywhere along it): when a reaches the airspace b will use."""
    ra, rb = np.asarray(a['route'], float), np.asarray(b['route'], float)
    req = DW.required_separation(a['z'] - b['z']) + buf
    d = np.linalg.norm(ra[:, None, :] - rb[None, :, :], axis=-1).min(axis=1)
    hit = np.nonzero(d < req)[0]
    return int(hit[0]) if len(hit) else len(ra)


def clusters(members, conflicts):
    """Connected groups of drones (union-find over the conflict pairs)."""
    parent = {i: i for i in members}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, j in conflicts:
        ri, rj = root(i), root(j)
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    groups = {}
    for i in members:
        groups.setdefault(root(i), []).append(i)
    return [tuple(sorted(g)) for g in groups.values() if len(g) > 1]


def cycles_between(old, new):
    """The trading cycles that turn the order `old` into the order `new` (each drone takes the
    turn of the next drone in its cycle); fixed points (no trade) are left out."""
    owner = {k: i for k, i in enumerate(old)}           # turn -> drone that owns it
    turn_new = {i: k for k, i in enumerate(new)}        # drone -> turn it gets
    seen, out = set(), []
    for i in old:
        if i in seen or turn_new[i] == old.index(i):
            seen.add(i)
            continue
        cyc, j = [], i
        while j not in seen:
            seen.add(j)
            cyc.append(j)
            j = owner[turn_new[j]]                       # the drone whose turn j takes
        if len(cyc) > 1:
            out.append(cyc)
    return out


def best_order(endowment, cost):
    """Lowest-energy order of a cluster and its energy. cost(i, ahead_tuple) -> J.
    The endowment wins ties, so turns change only when a trade saves energy."""
    def total(order):
        return sum(cost(i, tuple(sorted(order[:k]))) for k, i in enumerate(order))
    e0 = total(endowment)
    best, eb = tuple(endowment), e0
    for perm in itertools.permutations(endowment):
        e = total(perm)
        if e < eb - 1e-9:
            best, eb = perm, e
    return best, eb, e0


def tie_rank(round_no, drone_id):
    """Order of drones that reach a conflict at the same time: a hash of the round number and the
    drone number (32-bit integer mixing, the same on every drone), so over the rounds every drone
    is first equally often - no drone number is favoured."""
    h = (int(round_no) * 0x9E3779B1 + (int(drone_id) + 1) * 0x85EBCA77) & 0xFFFFFFFF
    h ^= h >> 15
    h = (h * 0x2C1B3C6D) & 0xFFFFFFFF
    h ^= h >> 12
    h = (h * 0x297A2D39) & 0xFFFFFFFF
    return h ^ (h >> 15)


def price_key(ahead):
    """Radio key of a price: the drones that would pass before, e.g. '2,4' ('' = nobody)."""
    return ','.join(str(i) for i in sorted(ahead))


class TurnTrader:
    """One drone's part in the turn trading (see the module docstring). Its radio message is
    message(); step() advances the rounds from the messages the drone has received; turn holds the
    drones before and after it in its cluster's current order (None outside a conflict)."""

    def __init__(self, me, log=None):
        self.me = me
        self.log = log or (lambda t, msg: None)
        self.round = None
        self.period = None         # this round's schedule, in control steps: (round length, phase gap)
        self.phase = 0
        self.covered = None        # drones whose message of the last priced round arrived (None: all)
        self.dl = 0                # largest delay this drone measured over DELAY_WINDOW (control steps)
        self._seen = {}            # newest send time seen from every other drone
        self._ages = []            # (receive time, delay in steps) of the new messages of DELAY_WINDOW
        self._team = []            # (time, largest delay reported by any drone) over DELAY_HOLD
        self.snap = None           # this round: {'route', 'z'} of this drone, None if not en route
        self.cluster = None        # this round: its cluster (tuple of drone ids)
        self.prices = None         # this round: {price_key(ahead): J}
        self.order = None          # its cluster's current order (tuple), sent as the next endowment
        self.turn = None
        self.trades = 0.0          # trading cycles it took part in (1/length per cycle: sums to cycles)
        self.saved_j = 0.0         # fleet energy those trades saved, as priced (its share)

    def message(self):
        return {'r': self.round, 'P': self.period, 'dl': self.dl, 'snap': self.snap, 'cluster': self.cluster,
                'prices': self.prices, 'order': self.order}

    def _schedule(self, t, inbox):
        """Round length and phase gap in control steps: (10, 2) - 0.5 s and 0.1 s - on a link without
        delay; on a slow link the gap covers the largest delay any drone measured, plus one step."""
        dt = 1.0 / C.CONTROL_RATE
        if not C.TTC_ADAPTIVE:
            return int(round(TTC_PERIOD / dt)), int(round(PHASE_PRICES / dt))
        for j, msg in inbox.items():                       # delays of the new messages
            ts = (msg or {}).get('ts')
            if ts is not None and ts > self._seen.get(j, -math.inf):
                self._seen[j] = ts
                self._ages.append((t, max(0, int(round((t - ts) / dt)))))
        self._ages = [(tm, a) for tm, a in self._ages if tm > t - DELAY_WINDOW]
        self.dl = max((a for _, a in self._ages), default=0)
        team = self.dl
        for msg in inbox.values():                         # what the others report
            m = (msg or {}).get('ttc')
            if m and m.get('dl') is not None and (msg.get('ts') or -math.inf) > t - DELAY_WINDOW:
                team = max(team, int(m['dl']))
        self._team = [(tm, d) for tm, d in self._team if tm > t - DELAY_HOLD] + [(t, team)]
        d = max(dd for _, dd in self._team)
        gap = max(int(round(PHASE_PRICES / dt)), d + 1)
        return max(int(round(TTC_PERIOD / dt)), 2 * gap + 2), gap

    def _this_round(self, m):
        """Is this 'ttc' part of a radio message from the current round (same schedule)?"""
        return bool(m) and m.get('r') == self.round and tuple(m.get('P') or ()) == tuple(self.period or ())

    def step(self, t, agent, inbox):
        """inbox: {drone id: radio message} of the other drones (their 'ttc' field is used)."""
        dt = 1.0 / C.CONTROL_RATE
        n_round, n_gap = self._schedule(t, inbox)
        period, gap = n_round * dt, n_gap * dt
        r = int(math.floor(t / period + 1e-9))
        off = t - r * period
        if r != self.round or (n_round, n_gap) != tuple(self.period or ()):     # phase 1: routes
            self.round, self.period, self.phase = r, (n_round, n_gap), 1
            self.cluster = self.prices = None
            route = agent.route_intent() if agent.state in ('CRUISE', 'ESCAPE') else None
            self.snap = None if route is None else {'route': route, 'z': float(agent.ekf.pos[2])}
            return
        if self.phase == 1 and off >= gap - 1e-6:           # phase 2: clusters and prices
            self.phase = 2
            snaps = self._round_field(inbox, 'snap')
            if self.snap is None:
                self.covered = None
                return
            if C.TTC_ADAPTIVE:                               # whose round message is in: the others are
                self.covered = {self.me} | {i for i, msg in inbox.items()       # avoided reactively
                                            if self._this_round((msg or {}).get('ttc'))}
            snaps[self.me] = self.snap
            ids = sorted(snaps)
            conf = [(i, j) for x, i in enumerate(ids) for j in ids[x + 1:] if _route_conflict(snaps[i], snaps[j])]
            mine = [g for g in clusters(ids, conf) if self.me in g]
            if not mine:
                return
            self.cluster = mine[0]
            others = [i for i in self.cluster if i != self.me]
            peers = {i: {'route': snaps[i]['route'], 'pos': np.array([0.0, 0.0, snaps[i]['z']])} for i in others}
            self.prices = {}
            for n in range(len(others) + 1):
                for ahead in itertools.combinations(others, n):
                    self.prices[price_key(ahead)] = agent.turn_cost([peers[i] for i in ahead])
            return
        if self.phase == 2 and off >= 2 * gap - 1e-6:       # phase 3: trading
            self.phase = 3
            self._trade(t, inbox)

    def _round_field(self, inbox, field):
        out = {}
        for i, msg in inbox.items():
            m = (msg or {}).get('ttc')
            if self._this_round(m) and m.get(field) is not None:
                out[i] = m[field]
        return out

    def _trade(self, t, inbox):
        g = self.cluster
        if not g:
            self.order, self.turn = None, None
            return
        msgs = {i: (inbox.get(i) or {}).get('ttc') for i in g if i != self.me}
        msgs[self.me] = self.message()
        complete = all(self._this_round(m) and tuple(m.get('cluster') or ()) == g and m.get('prices')
                       is not None for m in msgs.values())
        snaps = {i: m['snap'] for i, m in msgs.items() if m and m.get('snap') is not None}
        endow = self._endowment(g, msgs, snaps)
        if complete:
            prices = {i: msgs[i]['prices'] for i in g}

            def cost(i, ahead):
                return float(prices[i].get(price_key(ahead), 0.0))

            best, e_best, e_endow = best_order(endow, cost)
            if e_endow - e_best > TTC_SWITCH:
                cyc = cycles_between(list(endow), list(best))
                for c in cyc:
                    if self.me in c:
                        self.trades += 1.0 / len(c)
                        self.saved_j += (e_endow - e_best) / len(g)
                if self.me == g[0]:                          # one entry in the mission log per trade
                    txt = '; '.join('(' + ' '.join(f'drone{i}' for i in c) + ')' for c in cyc)
                    self.log(t, f'TTC: turns {"-".join(map(str, endow))} -> {"-".join(map(str, best))}, trading '
                                f'cycles {txt}, fleet saves {e_endow - e_best:.0f} J')
                order = best
            else:
                order = tuple(endow)
        else:                                                # a message is missing: no trade this round
            order = tuple(endow)
        self.order = order
        k = order.index(self.me)
        self.turn = {'cluster': g, 'ahead': set(order[:k]), 'behind': set(order[k + 1:])}

    def _endowment(self, g, msgs, snaps):
        """The members' last orders (in member order, each restricted to the cluster), then the
        newcomers first-come-first-served (ties by tie_rank: a draw from the round number, the same
        on every drone, that favours no drone number)."""
        out = []
        for i in g:
            for j in (msgs.get(i) or {}).get('order') or ():
                if j in g and j not in out:
                    out.append(j)
        if len(snaps) >= 2:
            entry = {i: min((_entry_time(snaps[i], snaps[j]) for j in snaps if j != i), default=10 ** 6)
                     for i in snaps}
        else:
            entry = {}
        def first(i):
            return entry.get(i, 10 ** 6), tie_rank(self.round, i)

        for i in sorted(g, key=first):
            if i not in out:
                k = next((n for n, j in enumerate(out) if first(j) > first(i)), len(out))
                out.insert(k, i)
        return tuple(out)
