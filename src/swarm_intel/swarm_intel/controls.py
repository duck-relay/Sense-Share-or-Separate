#!/usr/bin/env python3
"""Robustness checks of the main findings, computed from finished study folders (no new missions).

    ros2 run swarm_intel controls [--results DIR]     (or: python3 -m swarm_intel.controls)

Missions are paired by forest and seed (common random numbers), and every comparison uses one
reference on exactly the same missions:
  1) radio realism: the radio (C3+V+W) with lost, delayed and range-limited messages vs the ideal radio,
     the size of its messages (flies 1 mission), and how often two drones in a conflict agree on who passes
     first (flies 3 x 10 missions)
  2) layer controls: the layers with drones seen up to 90 deg above/below, and the layers with the
     radio (C3+V+L+W), vs no coordination (C3+V)
  3) reactive buffer: no coordination and the layers with 0.5 m added to the reactive clearance, and the
     layers with both the full view and the buffer
  4) give-way: fixed priorities vs turn trading, per configuration, against C3+V and against the
     configuration's own counterpart without the radio
  5) forest-level test of the main comparison: forest means, exact sign-flip over the 2^10 sign patterns
  6) equivalence (TOST, +-5 %) of the team energy with RRT* and with A*
  7) payload mass: the energy findings if the radio, the odometry or the camera ring add mass
  8) energy facts: power band, energy-flight time correlation, fallback climbs, distinct missions

The studies it needs (the newest of each is used):
  study_headless_L10_*            the main comparison (12 configurations x 10 forests x 10 seeds)
  study_headless_L10_*            a reference study containing cam3_sensor_layers_swarm (4 seeds)
  study_headless_loss*/delay*/view90/rbuf0.5/view90_rbuf0.5_L10_*   the control studies (reproduce --only controls)
  study_headless_priority_L10_*, study_headless_astar_L10_*

Writes <results>/controls_<time>/report.txt and table_controls.tex (the table also into <results>/paper/).
"""
import argparse
import csv
import glob
import itertools
import json
import math
import os
from datetime import datetime

import numpy as np

from . import config as C
from . import stats as S
from .paths import default_results_dir

E, MARGIN, HITS, WAKE, GOALS = ('fleet_energy_kj', 'fleet_min_sep_margin_m', 'fleet_drone_contacts',
                                 'fleet_wake_time_s', 'fleet_goals_reached')
SHORT = {'cam3_sensor': 'C3+V', 'cam2_sensor': 'C2+V', 'cam3': 'C3', 'cam2': 'C2', 'sensor': 'V',
         'swarm': 'W', 'cam3_sensor_swarm': 'C3+V+W', 'cam2_sensor_swarm': 'C2+V+W', 'all': 'C2+V+W',
         'cam3_swarm': 'C3+W', 'cam2_swarm': 'C2+W', 'sensor_swarm': 'V+W',
         'cam3_sensor_layers': 'C3+V+L', 'cam3_sensor_layers_swarm': 'C3+V+L+W'}
NO_RADIO = {'cam3_sensor_swarm': 'cam3_sensor', 'all': 'cam2_sensor', 'cam2_sensor_swarm': 'cam2_sensor',
            'cam3_swarm': 'cam3', 'cam2_swarm': 'cam2', 'sensor_swarm': 'sensor', 'swarm': None}


# ----------------------------------------------------------------------------------------- loading
def _meta(d):
    with open(os.path.join(d, 'meta.json')) as f:
        return json.load(f)


def _rows(d):
    with open(os.path.join(d, 'runs.csv')) as f:
        rows = list(csv.DictReader(f))
    out = {}
    for r in rows:
        out.setdefault(r['config'], {})[(int(r['layout']), int(r['seed']))] = r
    return out


def newest(res, pattern, need=None):
    """Newest study folder matching the pattern (optionally containing configuration `need`)."""
    for d in sorted(glob.glob(os.path.join(res, pattern)), reverse=True):
        if not os.path.isfile(os.path.join(d, 'runs.csv')):
            continue
        if need and need not in _meta(d).get('configurations', []):
            continue
        return d
    return None


def val(r, k):
    v = r.get(k, '')
    return float(v) if v not in ('', None) else float('nan')


def pair(a, b, keys=None):
    """Common (forest, seed) keys of two {key: row} dicts."""
    ks = sorted(set(a) & set(b))
    return [k for k in ks if keys is None or k in keys]


# ------------------------------------------------------------------------------------- statistics
def rel(x, y):
    """Paired change of x vs y in % of y's mean, with the 95 % CI and the sign-flip p."""
    st = S.paired(x, y)
    m = float(np.mean(y))
    return {'pct': 100 * st['diff'] / m, 'ci': 100 * (st['ci'] or 0.0) / m, 'p': st['p'], 'diff': st['diff'],
            'ci_abs': st['ci'] or 0.0}


def forest_signflip(d_by_forest):
    """Exact two-sided sign-flip test on the forest means (one value per forest)."""
    v = np.asarray(d_by_forest, float)
    obs = abs(v.mean())
    n = len(v)
    cnt = 0
    for signs in itertools.product((1.0, -1.0), repeat=n):
        if abs(float(np.dot(signs, v)) / n) >= obs - 1e-12:
            cnt += 1
    return cnt / 2 ** n


def tost_ci90(x, y):
    """90 % CI of the paired mean difference x - y in % of y's mean (TOST at alpha 0.05 each side)."""
    d = np.asarray(x, float) - np.asarray(y, float)
    n = len(d)
    half = t95(n - 1) * d.std(ddof=1) / math.sqrt(n)
    m = float(np.mean(y))
    return 100 * (d.mean() - half) / m, 100 * (d.mean() + half) / m


def t95(df):
    """0.95 quantile of Student's t (Cornish-Fisher expansion, as stats.t975; exact enough for df >= 6)."""
    z = 1.644853627
    g1 = (z ** 3 + z) / 4
    g2 = (5 * z ** 5 + 16 * z ** 3 + 3 * z) / 96
    g3 = (3 * z ** 7 + 19 * z ** 5 + 17 * z ** 3 - 15 * z) / 384
    g4 = (79 * z ** 9 + 776 * z ** 7 + 1482 * z ** 5 - 1920 * z ** 3 - 945 * z) / 92160
    return z + g1 / df + g2 / df ** 2 + g3 / df ** 3 + g4 / df ** 4


# ----------------------------------------------------------------------------------- the sections
def safety_cols(rows):
    """Mean energy, margin, safe missions, contacts, downwash time and goals of a list of rows."""
    g = lambda k: np.array([val(r, k) for r in rows])
    m = g(MARGIN)
    return {'E': g(E).mean(), 'margin': m.mean(), 'safe': int(np.sum(m >= 0)), 'n': len(rows),
            'hits': g(HITS).sum(), 'wake': g(WAKE).mean(), 'goals': g(GOALS).mean(),
            'mission': g('fleet_mission_time_s').mean()}


def compare_rows(label, ref_rows, rows, ref_label):
    ks = pair(ref_rows, rows)
    a = [rows[k] for k in ks]
    b = [ref_rows[k] for k in ks]
    s = safety_cols(a)
    s['label'], s['ref'] = label, ref_label
    s['dE'] = rel([val(r, E) for r in a], [val(r, E) for r in b])
    s['dM'] = S.paired([val(r, MARGIN) for r in a], [val(r, MARGIN) for r in b])
    return s


def fmt_cmp(s):
    dm = s['dM']
    return (f"  {s['label']:<38} n {s['n']:3d}  energy {s['E']:6.2f} kJ ({s['dE']['pct']:+5.1f} % "
            f"+-{s['dE']['ci']:.1f}, p {s['dE']['p']:.3f} vs {s['ref']})  margin {s['margin']:+.2f} m "
            f"({dm['diff']:+.2f} +-{dm['ci'] or 0:.2f}, p {dm['p']:.3f})  safe {s['safe']:2d}/{s['n']}  "
            f"contacts {s['hits']:.0f}  downwash {s['wake']:.2f} s  goals {s['goals']:.2f}  "
            f"mission {s['mission']:.1f} s")


def mass_factor(dm_kg):
    """Rotor power grows with the 1.5 power of the weight in the model (hover-dominated flight)."""
    return (1.0 + dm_kg / C.ENERGY.mass) ** 1.5


def rotor_j(r, cfg):
    nav = C.NAV[cfg]
    ep = C.ENERGY
    p_el = (ep.p_avionics + ep.p_comm_base + (ep.p_cam_nav if nav.cam_points else 0.0)
            + (ep.p_sensor_nav if nav.sensor else 0.0) + (ep.p_comm_swarm if nav.swarm else 0.0))
    return 1000 * val(r, E) - p_el * val(r, 'fleet_flight_time_s'), p_el


def energy_with_mass(rows, cfg, dm):
    out = []
    for r in rows:
        rot, _ = rotor_j(r, cfg)
        out.append((1000 * val(r, E) - rot + rot * mass_factor(dm)) / 1000)
    return np.array(out)


def break_even(rows_x, cfg_x, rows_y, target_mean):
    """Added mass (kg) on configuration x at which its mean energy equals target_mean."""
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if energy_with_mass(rows_x, cfg_x, mid).mean() < target_mean:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------------- agreement check
def agreement(loss=0.0, delay_ticks=(0, 0), layouts_=range(10), seed=0, adaptive=True):
    """Fly C3+V+W with the given radio link (one seed per forest) and count, at every control step, the pairs
    of drones of which at least one has the other in its trading group: whether both agree who passes first,
    whether only one of them knows of the conflict, or whether both claim the same turn or both wait."""
    from . import layouts
    from . import mission as MS
    from .plant import KinematicPlant
    old = (C.RADIO.loss, C.RADIO.delay_ticks, C.TTC_ADAPTIVE)
    C.RADIO.loss, C.RADIO.delay_ticks, C.TTC_ADAPTIVE = loss, tuple(delay_ticks), adaptive
    c = {'pairs': 0, 'agree': 0, 'one_sided': 0, 'both_first': 0, 'both_wait': 0}
    try:
        for lay in layouts_:
            layouts.apply(lay)
            core = MS.MissionCore('cam3_sensor_swarm', seed=seed)
            plant = KinematicPlant(core.world)
            dt, t = 1.0 / C.CONTROL_RATE, 0.0
            while t < C.MISSION_TIMEOUT and core.all_landed_t is None:
                cmds = core.step(t, plant.truth())
                plant.step(cmds, dt)
                t += dt
                tr = {i: a.trader.turn for i, a in core.agents.items() if a.trader is not None and a.trader.turn}
                ids = sorted(tr)
                for x, i in enumerate(ids):
                    for j in ids[x + 1:]:
                        in_i, in_j = j in tr[i]['cluster'], i in tr[j]['cluster']
                        if not (in_i or in_j):
                            continue
                        c['pairs'] += 1
                        if in_i != in_j:
                            c['one_sided'] += 1
                            continue
                        i_first, j_first = j in tr[i]['behind'], i in tr[j]['behind']
                        c['agree' if i_first != j_first else ('both_first' if i_first else 'both_wait')] += 1
    finally:
        C.RADIO.loss, C.RADIO.delay_ticks, C.TTC_ADAPTIVE = old
        layouts.apply(0)
    return c


def radio_load(layout=0, seed=0):
    """Size of the radio messages of C3+V+W in one mission: numbers per message, bytes as the JSON text the ROS 2
    drone node sends (report.dumps), and the data rates per drone and for the team at the control rate."""
    from . import layouts
    from . import mission as MS
    from .plant import KinematicPlant
    from .report import dumps, to_jsonable

    def count(o):
        o = to_jsonable(o)
        if isinstance(o, bool):
            return 0
        if isinstance(o, (int, float)):
            return 1
        if isinstance(o, dict):
            return sum(count(v) for v in o.values())
        if isinstance(o, (list, tuple)):
            return sum(count(v) for v in o)
        return 0
    sizes, nums = [], []
    orig = MS.Radio.publish

    def publish(self, i, msg, receivers=(), positions=None):
        if msg is not None:
            sizes.append(len(dumps(msg)))
            nums.append(count(msg))
        return orig(self, i, msg, receivers=receivers, positions=positions)
    MS.Radio.publish = publish
    try:
        layouts.apply(layout)
        core = MS.MissionCore('cam3_sensor_swarm', seed=seed)
        plant = KinematicPlant(core.world)
        dt, t = 1.0 / C.CONTROL_RATE, 0.0
        while t < C.MISSION_TIMEOUT and core.all_landed_t is None:
            plant.step(core.step(t, plant.truth()), dt)
            t += dt
    finally:
        MS.Radio.publish = orig
        layouts.apply(0)
    n_drones = len(C.DRONES)
    return {'messages': len(sizes), 'json_b': float(np.mean(sizes)), 'json_max_b': int(max(sizes)),
            'numbers': float(np.mean(nums)), 'drone_json_kBps': float(np.mean(sizes)) * C.CONTROL_RATE / 1000,
            'drone_bin_kBps': float(np.mean(nums)) * 4 * C.CONTROL_RATE / 1000,
            'team_json_Mbps': n_drones * float(np.mean(sizes)) * C.CONTROL_RATE * 8 / 1e6,
            'team_bin_Mbps': n_drones * float(np.mean(nums)) * 4 * C.CONTROL_RATE * 8 / 1e6}


# ------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default=None, help='results folder (default: the package results folder)')
    ap.add_argument('--no-agreement', action='store_true',
                    help='skip the agreement check (it flies 3 x 10 missions of C3+V+W, a few minutes)')
    a = ap.parse_args(argv)
    res = a.results or default_results_dir()
    L = []
    say = L.append

    main_d = newest(res, 'study_headless_L10_*', need='swarm')
    ref_d = newest(res, 'study_headless_L10_*', need='cam3_sensor_layers_swarm')
    if not main_d or not ref_d:
        print('Needs the main study and the reference study with cam3_sensor_layers_swarm.')
        return 1
    main, ref = _rows(main_d), _rows(ref_d)
    say('=' * 130)
    say(' ROBUSTNESS CHECKS (paired by forest and seed)')
    say(f' main study      {os.path.basename(main_d)}')
    say(f' reference study {os.path.basename(ref_d)}  ({_meta(ref_d).get("cpu", "?")})')
    say('=' * 130)

    table = []      # (group, label, stats dict) for the LaTeX table

    # 1) radio realism
    say('\n 1) RADIO REALISM: C3+V+W with lost, delayed or range-limited messages vs the ideal radio (same missions)')
    ideal = ref['cam3_sensor_swarm']
    s0 = compare_rows('ideal radio (reference)', ideal, ideal, 'ideal')
    say(fmt_cmp(s0))
    table.append(('radio', 'Ideal radio', s0))
    radio_runs = [('loss0.1', '10\\,\\% loss', '10 % loss'),
                  ('loss0.3', '30\\,\\% loss', '30 % loss'),
                  ('delay2-5', 'delay 100--250\\,ms', 'delay 100-250 ms'),
                  ('loss0.2_delay1-4_range8', '20\\,\\% loss, 50--200\\,ms, 8\\,m', '20 % loss + 50-200 ms + 8 m range')]
    for fixed, group, head in ((True, 'radio_fixed', 'fixed schedule (package 1.8.4: 0.1 s phases, no fallback)'),
                               (False, 'radio', 'rounds paced to the link delay, reactive fallback (package 1.8.5)')):
        say(f'  -- {head}')
        for tag, tex, txt in radio_runs:
            pat = f"study_headless_{tag}{'_ttcfixed' if fixed else ''}_L10_2*"
            d = newest(res, pat)
            if not d:
                say(f'  {txt:<38} (study not found: {pat})')
                continue
            s = compare_rows(txt, ideal, _rows(d)['cam3_sensor_swarm'], 'ideal radio')
            say(fmt_cmp(s))
            table.append((group, tex, s))

    ld = radio_load()
    say(f"\n 1a) RADIO LOAD (C3+V+W, forest 0, seed 0, {ld['messages']} messages): {ld['numbers']:.0f} numbers per message "
        f"= {ld['numbers'] * 4 / 1000:.2f} kB as 32-bit values, {ld['json_b'] / 1000:.2f} kB as JSON text (max "
        f"{ld['json_max_b'] / 1000:.2f} kB); per drone {ld['drone_bin_kBps']:.0f}-{ld['drone_json_kBps']:.0f} kB/s, "
        f"team {ld['team_bin_Mbps']:.2f}-{ld['team_json_Mbps']:.2f} Mbit/s at {C.CONTROL_RATE:g} Hz")
    if not a.no_agreement:
        say('\n 1b) AGREEMENT under the turn trading (C3+V+W, forests 0-9, seed 0): control steps x pairs in which at'
            ' least one drone has the other in its trading group')
        for loss, dl, ad, txt in ((0.0, (0, 0), True, 'ideal radio'), (0.3, (0, 0), False, '30 % loss, fixed'),
                                  (0.0, (2, 5), False, 'delay 100-250 ms, fixed'), (0.3, (0, 0), True, '30 % loss, timed'),
                                  (0.0, (2, 5), True, 'delay 100-250 ms, timed')):
            c = agreement(loss, dl, adaptive=ad)
            n = max(1, c['pairs'])
            say(f"  {txt:<24} pair-steps {c['pairs']:6d}: agree {100 * c['agree'] / n:5.1f} %, only one knows "
                f"{100 * c['one_sided'] / n:5.1f} %, both first {100 * c['both_first'] / n:4.1f} %, both wait "
                f"{100 * c['both_wait'] / n:4.1f} %" + ('  (no trading group formed)' if not c['pairs'] else ''))

    # 2) + 3) separation controls
    say('\n 2) LAYER CONTROLS and 3) REACTIVE BUFFER, vs no coordination (C3+V) on the same missions')
    base = ref['cam3_sensor']
    sb = compare_rows('C3+V, no coordination (reference)', base, base, 'C3+V')
    say(fmt_cmp(sb))
    table.append(('sep', 'C3+V (no coordination)', sb))
    s = compare_rows('C3+V+L, layers', base, ref['cam3_sensor_layers'], 'C3+V')
    say(fmt_cmp(s))
    table.append(('sep', 'C3+V+L (layers)', s))
    dv = newest(res, 'study_headless_view90_L10_*')
    if dv:
        rv = _rows(dv)
        for cfg, tex, txt in (('cam3_sensor', 'C3+V, view $\\pm$90\\textdegree', 'C3+V, view +-90 deg'),
                              ('cam3_sensor_layers', 'C3+V+L, view $\\pm$90\\textdegree', 'C3+V+L, view +-90 deg')):
            if cfg in rv:
                s = compare_rows(txt, base, rv[cfg], 'C3+V')
                say(fmt_cmp(s))
                table.append(('sep', tex, s))
    s = compare_rows('C3+V+L+W, layers and radio', base, ref['cam3_sensor_layers_swarm'], 'C3+V')
    say(fmt_cmp(s))
    table.append(('sep', 'C3+V+L+W (with radio)', s))
    s = compare_rows('C3+V+W, radio', base, ref['cam3_sensor_swarm'], 'C3+V')
    say(fmt_cmp(s))
    table.append(('sep', 'C3+V+W (radio)', s))
    db = newest(res, 'study_headless_rbuf0.5_L10_*')
    if db:
        rb = _rows(db)
        for cfg, tex, txt in (('cam3_sensor', 'C3+V, 0.5\\,m buffer', 'C3+V, 0.5 m reactive buffer'),
                              ('cam3_sensor_layers', 'C3+V+L, 0.5\\,m buffer', 'C3+V+L, 0.5 m reactive buffer')):
            if cfg in rb:
                s = compare_rows(txt, base, rb[cfg], 'C3+V')
                say(fmt_cmp(s))
                table.append(('sep', tex, s))
    dvb = newest(res, 'study_headless_view90_rbuf0.5_L10_*')     # both fixes of the layers together
    if dvb and 'cam3_sensor_layers' in _rows(dvb):
        s = compare_rows('C3+V+L, view +-90 deg and 0.5 m buffer', base, _rows(dvb)['cam3_sensor_layers'], 'C3+V')
        say(fmt_cmp(s))
        table.append(('sep', 'C3+V+L, $\\pm$90\\textdegree{} + buffer', s))
    say('  Layers + radio vs layers alone, and vs the radio alone:')
    say(fmt_cmp(compare_rows('C3+V+L+W vs C3+V+L', ref['cam3_sensor_layers'], ref['cam3_sensor_layers_swarm'], 'C3+V+L')))
    say(fmt_cmp(compare_rows('C3+V+L+W vs C3+V+W', ref['cam3_sensor_swarm'], ref['cam3_sensor_layers_swarm'], 'C3+V+W')))

    # 4) give-way per configuration
    dp = newest(res, 'study_headless_priority_L10_*')
    gw_rows, gw_more = [], []
    if dp:
        pr = _rows(dp)
        say('\n 4) GIVE-WAY: fixed priorities (P) vs turn trading (T) on the same 40 missions; energy in % with 95 % CI')
        say(f"  {'config':<9}{'P vs own':>18}{'T vs own':>18}{'P vs C3+V':>18}{'T vs C3+V':>18}{'T vs P':>24}"
            f"{'safe P/T':>10}{'margin P/T':>16}")
        for cfg in ('cam3_sensor_swarm', 'all', 'cam3_swarm', 'cam2_swarm', 'sensor_swarm', 'swarm'):
            if cfg not in pr:
                continue
            ks = pair(pr[cfg], main[cfg])
            P = np.array([val(pr[cfg][k], E) for k in ks])
            T = np.array([val(main[cfg][k], E) for k in ks])
            B = np.array([val(main['cam3_sensor'][k], E) for k in ks])
            own = NO_RADIO[cfg]
            O = np.array([val(main[own][k], E) for k in ks]) if own else None
            cell = lambda x, y: (f"{rel(x, y)['pct']:+6.1f}+-{rel(x, y)['ci']:4.1f}" if y is not None else '-')
            tp = rel(T, P)
            mP = np.array([val(pr[cfg][k], MARGIN) for k in ks])
            mT = np.array([val(main[cfg][k], MARGIN) for k in ks])
            fp = forest_signflip([np.mean([(T - P)[i] for i, k in enumerate(ks) if k[0] == f])
                                  for f in sorted({k[0] for k in ks})])
            gw_rows.append((SHORT[cfg], rel(P, B), rel(T, B), (rel(P, O), rel(T, O)) if own else None, tp,
                            int(np.sum(mP >= 0)), int(np.sum(mT >= 0)), len(ks)))
            gw_more.append({'mP': float(mP.mean()), 'mT': float(mT.mean()),
                            'hP': int(sum(val(pr[cfg][k], HITS) for k in ks)),
                            'hT': int(sum(val(main[cfg][k], HITS) for k in ks)),
                            'wT': float(np.mean([val(main[cfg][k], WAKE) for k in ks])),
                            'wP': float(np.mean([val(pr[cfg][k], WAKE) for k in ks]))})
            say(f"  {SHORT[cfg]:<9}{cell(P, O):>18}{cell(T, O):>18}{cell(P, B):>18}{cell(T, B):>18}"
                f"{tp['pct']:+8.1f}+-{tp['ci']:4.1f} p{tp['p']:.4f}/{fp:.3f}"
                f"{int(np.sum(mP >= 0)):>5}/{int(np.sum(mT >= 0)):<4}{mP.mean():+7.2f}/{mT.mean():+.2f}")
        say('  p: mission-level sign-flip / forest-level exact sign-flip')

    # 5) forest-level test of the main comparison
    say('\n 5) MAIN COMPARISON, FOREST-LEVEL TEST: exact sign-flip on the 10 forest means (Holm per metric)')
    metrics = [(E, 'energy'), ('fleet_loc_rms_mean_m', 'position error'), ('fleet_track_rms_mean_m', 'tracking'),
               ('fleet_cte_rms_mean_m', 'path dev.'), (GOALS, 'goals'), (MARGIN, 'margin'), (WAKE, 'downwash')]
    cfgs = [c for c in main if c != 'cam3_sensor']
    b = main['cam3_sensor']
    say(f"  {'config':<9}" + ''.join(f'{n:>22}' for _, n in metrics))
    pv = {}
    for k, _ in metrics:
        raw = []
        for cfg in cfgs:
            ks = pair(b, main[cfg])
            d = {f: [] for f in range(10)}
            for kk in ks:
                d[kk[0]].append(val(main[cfg][kk], k) - val(b[kk], k))
            fm = [np.mean(v) for v in d.values() if v]
            raw.append(forest_signflip(fm))
        adj = S.holm(raw)
        for cfg, p_raw, p_adj in zip(cfgs, raw, adj):
            pv[(cfg, k)] = (p_raw, p_adj)
    for cfg in cfgs:
        cells = []
        for k, _ in metrics:
            ks = pair(b, main[cfg])
            x = np.array([val(main[cfg][kk], k) for kk in ks])
            y = np.array([val(b[kk], k) for kk in ks])
            diff = x.mean() - y.mean()
            pct = 100 * diff / abs(y.mean()) if k not in (MARGIN, WAKE) and abs(y.mean()) > 1e-9 else None
            mission_p = S.paired(x, y)['p']
            cells.append((f'{pct:+.0f}%' if pct is not None else f'{diff:+.2f}')
                         + f' {mission_p:.3f}/{pv[(cfg, k)][1]:.3f}')
        say(f'  {SHORT[cfg]:<9}' + ''.join(f'{c:>22}' for c in cells))
    say('  cells: mean change, mission-level p (sign-flip, uncorrected) / forest-level p (exact, Holm-corrected)')

    # 6) equivalence of the planners
    da = newest(res, 'study_headless_astar_L10_*')
    if da:
        ast = _rows(da)
        say('\n 6) PLANNER EQUIVALENCE (TOST, margin +-5 %): RRT* - A* team energy, 90 % CI in % of A*')
        for cfg in ast:
            ks = pair(ast[cfg], main[cfg])
            x = [val(main[cfg][k], E) for k in ks]
            y = [val(ast[cfg][k], E) for k in ks]
            lo, hi = tost_ci90(x, y)
            r = rel(x, y)
            say(f'  {SHORT[cfg]:<9} n {len(ks)}  diff {r["pct"]:+5.1f} %  90 % CI [{lo:+5.1f}, {hi:+5.1f}]  '
                f'p {r["p"]:.3f}  ' + ('EQUIVALENT within 5 %' if lo > -5 and hi < 5 else 'not shown equivalent'))

    # 7) payload mass
    say('\n 7) PAYLOAD MASS (post hoc: the kinematic airframe flies the same paths; rotor energy x (1 + dm/m)^1.5,'
        ' downwash unchanged)')
    for cfg in ('cam3_sensor', 'cam3_sensor_swarm', 'cam3_sensor_layers', 'cam3', 'sensor'):
        rows = list(main[cfg].values())
        rot = np.array([rotor_j(r, cfg)[0] for r in rows])
        tot = np.array([1000 * val(r, E) for r in rows])
        say(f'  {SHORT[cfg]:<8} rotor share of energy {100 * np.mean(rot / tot):.1f} %')
    ks = pair(main['cam3_sensor'], main['cam3_sensor_swarm'])
    Bm = np.array([val(main['cam3_sensor'][k], E) for k in ks])
    Wr = [main['cam3_sensor_swarm'][k] for k in ks]
    Lm = np.array([val(main['cam3_sensor_layers'][k], E) for k in ks])
    for g in (0, 15, 30, 75):
        Wm = energy_with_mass(Wr, 'cam3_sensor_swarm', g / 1000)
        say(f'  radio + {g:3d} g: C3+V+W {100 * (Wm.mean() / Bm.mean() - 1):+5.1f} % vs C3+V '
            f'(layers {100 * (Lm.mean() / Bm.mean() - 1):+5.1f} %)')
    be = break_even(Wr, 'cam3_sensor_swarm', None, Lm.mean())
    say(f'  the radio would have to weigh {1000 * be:.0f} g ({100 * be / C.ENERGY.mass:.1f} % of the drone) '
        f'before C3+V+W used as much energy as the layers')
    c3 = np.array([val(r, E) for r in main['cam3'].values()])
    be = break_even(list(main['cam3_sensor'].values()), 'cam3_sensor', None, c3.mean())
    say(f'  odometry stops saving energy (C3+V = C3) at {1000 * be:.0f} g')
    vv = np.array([val(r, E) for r in main['sensor'].values()])
    be = break_even(list(main['cam3_sensor'].values()), 'cam3_sensor', None, vv.mean())
    say(f'  the camera ring stops saving energy (C3+V = V) at {1000 * be:.0f} g')

    # 8) energy facts
    say('\n 8) ENERGY FACTS (main comparison)')
    allr = [(c, r) for c in main for r in main[c].values()]
    pw = np.array([1000 * val(r, E) / val(r, 'fleet_flight_time_s') for _, r in allr])
    say(f'  mean power per drone, per mission {pw.min():.1f}-{pw.max():.1f} W; per configuration '
        + ', '.join(f"{SHORT[c]} {np.mean([1000 * val(r, E) / val(r, 'fleet_flight_time_s') for r in main[c].values()]):.1f}"
                    for c in main))
    for excl, txt in ((set(), 'all configurations'), ({'swarm'}, 'without W')):
        sel = [r for c, r in allr if c not in excl]
        e = [val(r, E) for r in sel]
        t = [val(r, 'fleet_flight_time_s') for r in sel]
        say(f'  energy vs flight time, {txt}: r = {np.corrcoef(e, t)[0, 1]:.4f} (n {len(sel)})')
    say('  fallback climbs (escapes) per mission: '
        + ', '.join(f"{SHORT[c]} {np.mean([val(r, 'fleet_escapes') for r in main[c].values()]):.2f}" for c in main))
    # distinct missions: studies flown on the main study's computer repeat a mission exactly (same values);
    # control studies flown elsewhere repeat main-study missions in their reference configurations, so only
    # their new conditions count (layers + radio, the radio settings, the wider view, the buffer)
    # A new condition that leaves a mission unchanged (C3+V seen +-90 deg flies at one height; the baseline of the
    # view + buffer study is that of the buffer study) repeats it exactly, so identical results count once.
    cpu = _meta(main_d).get('cpu')
    seen, total, new_runs, repeats = set(), 0, set(), set()
    for d in sorted(glob.glob(os.path.join(res, 'study_headless_*'))):
        same = _meta(d).get('cpu') == cpu
        tag = os.path.basename(d)
        radio_tag = any(x in tag for x in ('_loss', '_delay', '_range'))
        other_tag = any(x in tag for x in ('_view', '_rbuf'))
        for c, rr in _rows(d).items():
            for k, r in rr.items():
                total += 1
                key = (c, k, r[E], r['fleet_cte_rms_mean_m'], r['fleet_flight_time_s'])
                if same:
                    seen.add(key)
                elif c == 'cam3_sensor_layers_swarm' or other_tag or (radio_tag and C.NAV[c].swarm):
                    new_runs.add(key)
                else:
                    repeats.add(key)
    new = len(new_runs - repeats)
    sp = os.path.join(res, 'speed_sweep', 'batch_results.csv')
    n_speed = n_speed_new = 0
    if os.path.isfile(sp):
        with open(sp) as f:
            for r in csv.DictReader(f):
                n_speed += 1
                n_speed_new += abs(float(r.get('speed_scale') or 1.0) - 1.0) > 1e-9
    say(f'  headless missions flown: {total} in the studies + {n_speed} in the speed sweep = {total + n_speed}; '
        f'distinct: {len(seen) + new + n_speed_new} ({len(seen)} on the main computer, {new} new conditions of the '
        f'control studies, {n_speed_new} at other speeds); the rest repeat a mission (e.g. the baseline of each study)')

    # electronics sensitivity: every electronics power doubled (post hoc, the same flights)
    def e_el(r, cfg):
        return rotor_j(r, cfg)[1] * val(r, 'fleet_flight_time_s') / 1000
    eb = np.mean([val(r, E) + e_el(r, 'cam3_sensor') for r in main['cam3_sensor'].values()])
    say('  electronics doubled: energy change vs C3+V (as flown -> with doubled electronics): '
        + ', '.join(f"{SHORT[c]} {100 * (np.mean([val(r, E) for r in main[c].values()]) / np.mean([val(r, E) for r in main['cam3_sensor'].values()]) - 1):+.1f} -> "
                    f"{100 * (np.mean([val(r, E) + e_el(r, c) for r in main[c].values()]) / eb - 1):+.1f} %"
                    for c in ('cam3', 'sensor', 'cam3_sensor_layers', 'cam3_sensor_swarm', 'all')))
    share = [100 * e_el(r, c) / val(r, E) for c in main for r in main[c].values()]
    say(f'  electronics share of the energy: {min(share):.1f}-{max(share):.1f} %')

    out = os.path.join(res, f"controls_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    pdir = os.path.join(res, 'paper')
    os.makedirs(pdir, exist_ok=True)
    for name, tex in (('table_controls.tex', latex_table(table)), ('table_giveway.tex', latex_giveway(gw_rows)),
                      ('table_controls_gw.tex', latex_table(table, list(zip(gw_rows, gw_more))))):
        for p in (os.path.join(out, name), os.path.join(pdir, name)):
            with open(p, 'w') as f:
                f.write(tex)
    print('\n'.join(L))
    print(f'\nSaved to {out}  (report.txt, table_controls.tex, table_giveway.tex, table_controls_gw.tex; tables also in {pdir})')
    return 0


SEP_ORDER = ['C3+V (no coordination)', 'C3+V, 0.5\\,m buffer', 'C3+V+W (radio)', 'C3+V+L (layers)',
             'C3+V+L, view $\\pm$90\\textdegree', 'C3+V+L, 0.5\\,m buffer',
             'C3+V+L, $\\pm$90\\textdegree{} + buffer', 'C3+V+L+W (with radio)']


def latex_table(table, giveway=None):
    """The paper's robustness table: energy change with 95 % CI, margin, safe missions, contacts. The keeping-apart
    rows in SEP_ORDER (C3+V seen +-90 deg is left out: with every drone at one height it equals C3+V). With giveway
    (rows of the give-way comparison) the table also compares turn trading with fixed priorities (the short paper)."""
    ideal = [('ideal',) + x[1:] for x in table if x[0] == 'radio' and x[2]['label'].endswith('(reference)')]
    fixed = [x for x in table if x[0] == 'radio_fixed']
    timed = [x for x in table if x[0] == 'radio' and not x[2]['label'].endswith('(reference)')]
    sep = {x[1]: x for x in table if x[0] == 'sep'}
    table = ideal + fixed + timed + [sep[k] for k in SEP_ORDER if k in sep]
    T = [r'\begin{table}[t]',
         r'\caption{Robustness checks on 40 missions (10 forests $\times$ 4 seeds), flown on one computer: energy '
         r'change with 95\,\% CI against the reference of each group on the same missions, mean separation margin, '
         r'missions with the closest pair outside the safe distance, contacts between drones in total, and time in downwash '
         r'per mission.}',
         r'\label{tab:controls}',
         r'\centering\footnotesize\setlength{\tabcolsep}{2.5pt}',
         r'\begin{tabular}{@{}lrrrrr@{}}',
         r'\toprule',
         r'Variant & Energy & Margin & Safe & Cont. & Downw.\\',
         r' & \% & m & of 40 & & s\\']
    groups = {'ideal': r'\multicolumn{6}{@{}l}{\emph{Radio realism (C3+V+W), against the ideal radio}}\\',
              'radio_fixed': r'\multicolumn{6}{@{}l}{\quad\emph{fixed schedule, no fallback}}\\',
              'radio': r'\multicolumn{6}{@{}l}{\quad\emph{rounds paced to the delay, fallback}}\\',
              'sep': r'\multicolumn{6}{@{}l}{\emph{Keeping apart, against no coordination (C3+V)}}\\'}
    last = None
    for g, label, s in table:
        if g != last:
            if g not in ('radio_fixed', 'radio'):                      # sub-groups of the radio block
                T.append(r'\midrule')
            T.append(groups[g])
            last = g
        ref = s['ref'] in ('ideal', 'C3+V') and s['label'].endswith('(reference)')
        e = '--' if ref else _pm(s['dE'])
        m = f"{s['margin']:+.2f}"
        m = '$-$' + m[1:] if m.startswith('-') else m                       # a real minus sign
        T.append(f"{label} & {e} & {m} & {s['safe']} & {s['hits']:.0f} & {s['wake']:.2f}\\\\")
    if giveway:
        T += [r'\midrule', r'\multicolumn{6}{@{}l}{\emph{Give-way: turn trading against fixed priorities (trading/fixed)}}\\']
        mm = lambda v: ('$-$' + f'{v:+.2f}'[1:]) if v < 0 else f'{v:+.2f}'
        dw = lambda v: '0' if round(v, 2) == 0 else f'{v:.2f}'           # no downwash at all: a plain 0
        for (name, pb, tb, own, tp, sp, st, n), more in giveway:          # every cell: trading/fixed
            T.append(f"{name} & {_pm(tp)} & {mm(more['mT'])}/{mm(more['mP'])} & {st}/{sp} & "
                     f"{more['hT']}/{more['hP']} & {dw(more['wT'])}/{dw(more['wP'])}\\\\")
        T[1] = (r'\caption{Robustness checks and give-way rules, 40 missions each: energy change (95\,\% CI) against '
                r'the group\textquoteright s reference, mean separation margin, missions outside the safe distance, '
                r'contacts, and downwash time per mission. Give-way rows: main computer; others: a second computer '
                r'with its own references.}')
        T[2] = r'\label{tab:controls}'
        T[3] = r'\centering\footnotesize\setlength{\tabcolsep}{1.6pt}'
    T += [r'\bottomrule', r'\end{tabular}', r'\end{table}', '']
    return '\n'.join(T)


def _pm(r):
    t = f"{r['pct']:+.1f}"
    t = '$-$' + t[1:] if t.startswith('-') else t
    return f"{t}\\,$\\pm$\\,{r['ci']:.1f}"


def latex_giveway(rows):
    """Fixed priorities vs turn trading, per radio configuration, on the same missions."""
    if not rows:
        return ''
    n = rows[0][-1]
    T = [r'\begin{table}[t]',
         r'\caption{Give-way rules on the same ' + str(n) + r' missions (10 forests $\times$ 4 seeds): team energy '
         r'against the baseline (C3+V) and of turn trading against fixed priorities, in \% with the 95\,\% CI, and '
         r'the missions in which the closest pair stays outside the safe distance.}',
         r'\label{tab:giveway}',
         r'\centering\footnotesize\setlength{\tabcolsep}{3pt}',
         r'\begin{tabular}{@{}lrrrr@{}}',
         r'\toprule',
         r' & \multicolumn{2}{c}{Energy vs C3+V (\%)} & Trading vs & Safe (of ' + str(n) + r')\\',
         r'\cmidrule(lr){2-3}',
         r'Config. & Fixed & Trading & fixed (\%) & fixed/trading\\',
         r'\midrule']
    for name, pb, tb, own, tp, sp, st, _ in rows:
        T.append(f"{name} & {_pm(pb)} & {_pm(tb)} & {_pm(tp)} & {sp}/{st}\\\\")
    T += [r'\bottomrule', r'\end{tabular}', r'\end{table}', '']
    return '\n'.join(T)


if __name__ == '__main__':
    raise SystemExit(main())
