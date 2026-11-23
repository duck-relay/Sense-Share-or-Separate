#!/usr/bin/env python3
"""Navigation study: every configuration compared with the baseline
(3-point camera + VIO).

  ros2 run swarm_intel study                        # all 12 configs, the Gazebo forest, 10 seeds (no GUI, no Gazebo)
  ros2 run swarm_intel study --layouts 10           # + 9 random forests: results that hold beyond one map
  ros2 run swarm_intel study --gazebo               # NO new simulation: compile your Gazebo runs (all seeds)
  ros2 run swarm_intel study --configs cam3_sensor,cam2,all
  ros2 run swarm_intel study --cam-fov 80           # your single forward 80 deg camera instead of the 360 ring
  ros2 run swarm_intel study --wind 1.0             # mean wind 1 m/s (default 0.2); --no-wind for calm air
  ros2 run swarm_intel study --mixed                # mixed fleets: k of 5 drones carry a camera, with / without swarm
  ros2 run swarm_intel study --planner astar        # planner comparison: A* instead of RRT* (the drones' planner)
  ros2 run swarm_intel study --give-way priority    # give-way comparison: fixed priorities instead of turn trading

In every run all 5 drones use the configuration (or, with --mixed, the fleet's mix), and the
whole fleet is scored: every metric covers all five drones (summed, averaged or the worst drone).

Writes <results>/study_<source>_<time>/ : report.txt, runs.csv, summary.csv and the graphs
study_comparison.png (the detailed final comparison), study_overview.png,
study_changes.png and study_paths.png.

Verdicts: every configuration flies the SAME missions as the baseline (same forest layout and
seed), so the comparison is paired. A configuration is "better" / "worse" than the baseline
on a metric only when its mean paired difference is significant in a sign-flip permutation
test, Holm-corrected across the configurations for that metric (p < 0.05), AND is more than
2 % of the baseline. With too few paired runs for a test to reach significance (e.g. a few
Gazebo runs, under about 9 pairs) the difference must instead exceed 2 % and 2 standard errors.
"""
import argparse
import csv
import math
import os
import time
from datetime import datetime
from multiprocessing import Pool

import numpy as np

from . import config as C
from . import metrics as M
from . import report as R
from .report import col
from . import layouts
from . import stats as ST
from . import blas_kernel_note
from .paths import default_results_dir, runs_of, load_report, study_run
from .runinfo import write_meta
from .grid_planner import PLANNERS, PLANNER_NAMES
from .ttc import GIVE_WAYS, GIVE_WAY_NAMES

TRAJ_KEYS = ('t', 'x', 'y', 'est_x', 'est_y', 'loc_err', 'cte')
LAYER_BASE = 2.3      # lowest red layer (m); the default layers are 2.3, 2.9, 3.5, 4.1 m (gap 0.6 m)


# ---------------------------------------------------------------------------
# running
# ---------------------------------------------------------------------------
# the defaults the overrides change: every study starts from them, so studies run one after
# another in the same process (menu, scripts) never inherit each other's settings
_DEFAULTS = {'cam_fov_deg': C.SENSORS.cam_fov_deg, 'cam_vfov_deg': C.SENSORS.cam_vfov_deg,
             'wind_init': C.SENSORS.wind_init, 'wind_rw': C.SENSORS.wind_rw,
             'mean_speed': C.WIND.mean_speed, 'gust_sigma': C.WIND.gust_sigma,
             'downwash_scale': C.DOWNWASH.scale, 'layers': {d.id: d.layer_alt for d in C.DRONES},
             'planner': C.DEFAULT_PLANNER, 'give_way': C.DEFAULT_GIVE_WAY,
             'radio': (C.RADIO.loss, C.RADIO.delay_ticks, C.RADIO.range_m),
             'proximity_elev_deg': C.SENSORS.proximity_elev_deg, 'reactive_buffer': C.REACTIVE_BUFFER,
             'ttc_adaptive': C.TTC_ADAPTIVE}


def _reset_overrides():
    D = _DEFAULTS
    C.SENSORS.cam_fov_deg, C.SENSORS.cam_vfov_deg = D['cam_fov_deg'], D['cam_vfov_deg']
    C.SENSORS.wind_init, C.SENSORS.wind_rw = D['wind_init'], D['wind_rw']
    C.WIND.mean_speed, C.WIND.gust_sigma = D['mean_speed'], D['gust_sigma']
    C.DOWNWASH.scale = D['downwash_scale']
    C.PLANNER = D['planner']
    C.GIVE_WAY = D['give_way']
    C.RADIO.loss, C.RADIO.delay_ticks, C.RADIO.range_m = D['radio']
    C.SENSORS.proximity_elev_deg = D['proximity_elev_deg']
    C.REACTIVE_BUFFER = D['reactive_buffer']
    C.TTC_ADAPTIVE = D['ttc_adaptive']
    for d in C.DRONES:
        d.layer_alt = D['layers'][d.id]


def _apply_overrides(ov):
    _reset_overrides()
    if ov.get('cam_fov'):
        C.SENSORS.cam_fov_deg = float(ov['cam_fov'])
        if C.SENSORS.cam_fov_deg < 300:                    # single forward camera, 640 px over the FOV
            C.SENSORS.cam_vfov_deg = math.degrees(2 * math.atan(math.tan(math.radians(C.SENSORS.cam_fov_deg) / 2) * 0.75))
    if ov.get('wind') is not None:
        C.WIND.mean_speed = float(ov['wind'])
        C.WIND.gust_sigma = max(0.10, 0.3 * C.WIND.mean_speed)    # turbulence grows with the wind
        # every EKF is tuned for the wind it flies in (as a real system would be): wind prior and
        # wind process noise follow the mean wind and the gusts (unchanged at the default 0.2 m/s)
        C.SENSORS.wind_init = max(0.30, 1.5 * C.WIND.mean_speed)
        C.SENSORS.wind_rw = max(0.10, 1.2 * C.WIND.gust_sigma * math.sqrt(2.0 / C.WIND.gust_tau))
    if ov.get('no_wind'):
        C.WIND.mean_speed = 0.0
        C.WIND.gust_sigma = 0.0
    if ov.get('downwash_scale') is not None:            # sensitivity: weaker / stronger downwash
        C.DOWNWASH.scale = float(ov['downwash_scale'])
    if ov.get('planner'):                               # planner comparison: A* instead of RRT*
        C.PLANNER = ov['planner']
    if ov.get('give_way'):                              # give-way comparison: fixed priorities
        C.GIVE_WAY = ov['give_way']
    # control experiments (headless twin only): a realistic radio link, drones without the radio that see
    # other drones straight above and below, and reactive avoidance with the give-way's buffer
    if ov.get('radio_loss') is not None:
        C.RADIO.loss = float(ov['radio_loss'])
    if ov.get('radio_delay_ticks') is not None:
        C.RADIO.delay_ticks = tuple(int(x) for x in ov['radio_delay_ticks'])
    if ov.get('radio_range') is not None:
        C.RADIO.range_m = float(ov['radio_range'])
    if ov.get('drone_view_deg') is not None:
        C.SENSORS.proximity_elev_deg = float(ov['drone_view_deg'])
    if ov.get('reactive_buffer') is not None:
        C.REACTIVE_BUFFER = float(ov['reactive_buffer'])
    if ov.get('ttc_fixed'):                             # the turn trading of package 1.8.4 (fixed schedule)
        C.TTC_ADAPTIVE = False
    if ov.get('layer_gap') is not None:                 # sensitivity: spacing of the fixed altitude layers
        for d in C.DRONES:
            if d.priority > 0:
                d.layer_alt = round(LAYER_BASE + (d.priority - 1) * float(ov['layer_gap']), 3)


def _job(args):
    key, layout, seed, speed, record, ov = args
    _apply_overrides(ov)
    from .plant import run_headless
    layouts.apply(layout)
    rep, traj = run_headless(key, seed=seed, speed_scale=speed, record=record)
    out = {'config': key, 'layout': layout, 'seed': seed, 'source': 'headless', 'run': f'{key}_L{layout}_s{seed}'}
    out.update(M.extract(rep))
    tr = None
    if record:                                         # every drone's track: {drone id: {...}}
        tr = {}
        for d in C.DRONES:
            rows = [r for r in traj if r[1] == d.id]
            # (t, id, x, y, z, ex, ey, ez, power, energy, state, cte)
            tr[d.id] = {'t': [r[0] for r in rows], 'x': [r[2] for r in rows], 'y': [r[3] for r in rows],
                        'est_x': [r[5] for r in rows], 'est_y': [r[6] for r in rows],
                        'loc_err': [math.hypot(r[2] - r[5], r[3] - r[6]) for r in rows], 'cte': [r[11] for r in rows],
                        'state': [r[10] for r in rows], 'ref': (rep.get('reference_paths') or {}).get(str(d.id))}
    return out, tr


def run_headless_study(keys, layout_ids, seeds, speed, workers, overrides):
    """Every configuration on every (forest layout, seed). The path of layout 0 / seed 0 is recorded."""
    jobs = [(k, L, s, speed, (L == layout_ids[0] and s == 0), overrides)
            for L in layout_ids for s in range(seeds) for k in keys]
    rows, trajs = [], {}
    t0 = time.time()
    workers = workers or max(1, min(os.cpu_count() or 1, 8))
    with Pool(workers) as pool:
        for n, (row, tr) in enumerate(pool.imap_unordered(_job, jobs), 1):
            rows.append(row)
            if tr is not None:
                trajs[row['config']] = tr
            el = time.time() - t0
            eta = el / n * (len(jobs) - n)
            print(f'\r  {n}/{len(jobs)} missions simulated ({el:.0f} s, about {eta / 60:.0f} min left)   ',
                  end='', flush=True)
    print()
    return rows, trajs


def collect_gazebo(keys, results_dir):
    """Gazebo runs of each configuration -> rows, trajectories, missing configs, notes.
    Runs of the Gazebo study (gazebo_study; not the runs watched from the menu) of this package version, planner
    and give-way are used (one per seed: the newest); if a configuration
    only has runs of an older version, its newest run is used and flagged."""
    rows, trajs, missing, notes = [], {}, [], []
    for k in keys:
        reps = []
        for d in runs_of(results_dir, k):
            try:
                reps.append((d, load_report(d)))
            except Exception:
                continue
        if not reps:
            missing.append(k)
            continue
        current = [(d, r) for d, r in reps if r.get('model_version') == C.MODEL_VERSION and study_run(r)
                   and r.get('planner', 'astar') == C.PLANNER and r.get('give_way', 'priority') == C.GIVE_WAY]
        if current:
            by_seed = {}
            for d, r in current:                         # oldest first, so the newest wins
                by_seed[int(r.get('seed', 0))] = (d, r)
            use = [by_seed[sd] for sd in sorted(by_seed)]
        else:
            use = [reps[-1]]
            why = ('is a watched run, not a Gazebo-study run' if use[0][1].get('model_version') == C.MODEL_VERSION
                   else 'was made with an older version of this package')
            notes.append(f'{k}: {os.path.basename(use[0][0])} {why} - run the Gazebo study '
                         f'(ros2 run swarm_intel gazebo_study --configs {k})')
        for d, rep in use:
            run = os.path.basename(d)
            if abs(float(rep.get('speed_scale', 1.0)) - 1.0) > 1e-6:
                notes.append(f"{k}: {run} used cruise speed x{rep.get('speed_scale')}")
            row = {'config': k, 'layout': int(rep.get('layout', 0)), 'seed': int(rep.get('seed', 0)),
                   'source': 'gazebo', 'run': run}
            row.update(M.extract(rep))
            rows.append(row)
        d0 = use[0][0]
        try:
            tr = {d.id: dict({key: [] for key in TRAJ_KEYS}, state=[]) for d in C.DRONES}
            with open(os.path.join(d0, 'telemetry.csv')) as f:
                for r in csv.DictReader(f):
                    t_i = tr.get(int(r['id']))
                    if t_i is None:
                        continue
                    for key in TRAJ_KEYS:
                        t_i[key].append(float(r[key]))
                    t_i['state'].append(r['state'])
            for i, t_i in tr.items():
                t_i['ref'] = (use[0][1].get('reference_paths') or {}).get(str(i))
            trajs[k] = tr
        except Exception:
            pass
    per = {}
    for r in rows:
        per.setdefault(r['config'], set()).add(r['seed'])
    common = set.intersection(*per.values()) if per else set()
    if per and any(v != common for v in per.values()):
        notes.append('the configurations were not all flown with the same seeds - only the common seeds '
                     f'({", ".join(str(x) for x in sorted(common)) or "none"}) are paired with the baseline')
    return rows, trajs, missing, notes


# ---------------------------------------------------------------------------
# statistics
# ---------------------------------------------------------------------------
def summarize(rows, keys):
    """Mean and SD of every metric per configuration, plus the PAIRED comparison with the
    baseline (same forest layout and seed): mean difference with its 95 % CI, sign-flip
    permutation p-value (Holm-corrected across configurations, per metric) and effect size d_z."""
    summ = {}
    by = {}
    for r in rows:
        by[(r['config'], r.get('layout', 0), r['seed'])] = r
    for k in keys:
        sel = [r for r in rows if r['config'] == k]
        if not sel:
            continue
        s = {'n': len(sel)}
        for mk, *_ in M.ALL:
            v = np.array([r.get(mk) for r in sel if r.get(mk) is not None], float)
            s[mk] = float(np.mean(v)) if len(v) else None
            s[mk + '_sd'] = float(np.std(v, ddof=1)) if len(v) > 1 else (0.0 if len(v) else None)
            s[mk + '_n'] = int(len(v))
        summ[k] = s
    base = summ.get(C.BASELINE)
    if not base:
        return summ
    tests = []
    for k in summ:
        if k == C.BASELINE:
            continue
        for mk, *_, better in M.ALL:
            xs, ys = [], []
            for (kk, L, sd), r in by.items():
                if kk != k:
                    continue
                b = by.get((C.BASELINE, L, sd))
                if b is None or r.get(mk) is None or b.get(mk) is None:
                    continue
                xs.append(r[mk])
                ys.append(b[mk])
            st = ST.paired(xs, ys)
            s = summ[k]
            s[mk + '_npair'] = st['n']
            s[mk + '_diff'] = st['diff']
            s[mk + '_ci'] = st['ci']
            s[mk + '_dz'] = st['dz']
            s[mk + '_p'] = st['p']
            if better is not None and st['n'] >= 2:
                tests.append((k, mk))
    # Holm across the configurations, separately for each metric; only where enough paired
    # missions exist for a significant result to be possible at all (else: 2 % / 2 SE rule)
    for mk, *_ in M.ALL:
        fam = [k for k, m_ in tests if m_ == mk]
        fam = [k for k in fam if ST.testable(summ[k][mk + '_npair'], len(fam))]
        if not fam:
            continue
        adj = ST.holm([summ[k][mk + '_p'] for k in fam])
        for k, pa in zip(fam, adj):
            summ[k][mk + '_padj'] = float(pa)
    return summ


def configs_in(summ):
    return list(summ)


def verdict(summ, k, mk):
    """better / worse / same / None of config k vs the baseline on metric mk.
    With enough paired runs: significant after Holm correction (p < 0.05) AND more than
    2 % of the baseline (0.05 m for the separation margin, which crosses zero). With too few
    runs for a test: more than that and 2 standard errors."""
    base, s = summ.get(C.BASELINE), summ.get(k)
    if not base or not s or k == C.BASELINE:
        return None
    m = M.BY_KEY.get(mk)
    if m is None or m[4] is None or s.get(mk) is None or base.get(mk) is None:
        return None
    pa = s.get(mk + '_padj')
    if pa is None:                                   # too few paired runs for a test
        noise = M.noise_of(s.get(mk + '_sd'), s.get(mk + '_n'), base.get(mk + '_sd'), base.get(mk + '_n'))
        return M.verdict(mk, s.get(mk), base.get(mk), noise=noise)
    diff = s.get(mk + '_diff') or 0.0
    if pa >= ST.ALPHA or abs(diff) <= M.min_diff(mk, base[mk]):
        return 'same'
    if m[4] == 'lower':
        return 'better' if diff < 0 else 'worse'
    return 'better' if diff > 0 else 'worse'


def n_tests(summ):
    """Number of paired significance tests behind the verdicts (0 = too few runs for tests)."""
    return sum(1 for k in summ for f in summ[k] if f.endswith('_padj'))


def counts(summ, k):
    vd = [verdict(summ, k, mk) for mk, *_ in M.ALL]
    return vd.count('better'), vd.count('same'), vd.count('worse')


def pareto(summ, xk, yk):
    pts = [(k, summ[k][xk], summ[k][yk]) for k in configs_in(summ)
           if summ[k].get(xk) is not None and summ[k].get(yk) is not None]
    front = []
    for k, x, y in pts:
        dominated = any((x2 <= x and y2 <= y) and (x2 < x or y2 < y) for k2, x2, y2 in pts if k2 != k)
        if not dominated:
            front.append(k)
    return front


# ---------------------------------------------------------------------------
# terminal tables
# ---------------------------------------------------------------------------
_mfmt = M.fmt_mean          # a MEAN over runs: counts keep their decimals (0.05 is not shown as 0)


def _paint(txt, vd):
    return col(txt, R.C_GREEN) if vd == 'better' else col(txt, R.C_RED) if vd == 'worse' else txt


def _cell(summ, k, mk, width):
    s = summ[k]
    v = s.get(mk)
    if v is None:
        return '-'.rjust(width)
    base = summ.get(C.BASELINE)
    txt = _mfmt(mk, v)
    if base and k != C.BASELINE and base.get(mk) is not None:
        vd = verdict(summ, k, mk)
        txt = f'{txt} ({M.fmt_change(v, base.get(mk), mk)}){M.MARK[vd] if vd in ("better", "worse") else " "}'
        return _paint(txt.rjust(width), vd)
    return (txt + ' ').rjust(width)


KEY_HEADS = [('fleet_energy_kj', 'Energy kJ'), ('fleet_flight_time_s', 'Flight time s'),
             ('fleet_cte_rms_mean_m', 'Path RMS m'), ('fleet_loc_rms_mean_m', 'Position RMS m'),
             ('fleet_min_sep_margin_m', 'Sep. margin m'), ('fleet_mission_time_s', 'Mission s'),
             ('fleet_goals_reached', 'Goals /5'), ('fleet_contacts', 'Tree hits'),
             ('fleet_drone_contacts', 'Drone hits'), ('fleet_wake_time_s', 'Downwash s')]


def _rule(summ):
    nt = n_tests(summ)
    if nt:
        return (' ▲ better / ▼ worse than the baseline: mean paired difference (same forest and seed) significant in a '
                'sign-flip permutation test, Holm-corrected across configurations per metric (p < 0.05), AND more than '
                '2 %; otherwise about the same   |   ◆ baseline')
    return (' ▲ better / ▼ worse than the baseline (difference > 2 % and > 2 standard errors; too few paired runs for '
            'a significance test), otherwise about the same   |   ◆ baseline')


def _fmt_p(p):
    if p is None:
        return '-'
    return '<0.001' if p < 0.001 else f'{p:.3f}'


def tables(summ, keys, source, n_info, notes=(), color=True):
    R.USE_COLOR = color and R.USE_COLOR
    base = summ.get(C.BASELINE)
    present = [k for k in keys if k in summ]
    L = []
    w = 184
    base_name = C.NAV[C.BASELINE].label.replace('Baseline: ', '')
    L.append(col('═' * w, R.C_CYAN))
    L.append(col(f' NAVIGATION STUDY — every configuration vs the BASELINE ({base_name})', R.C_BOLD))
    L.append(f' source: {source}   |   {n_info}   |   the whole fleet is scored: every metric covers all 5 drones')
    L.append(_rule(summ))
    L.append(col('═' * w, R.C_CYAN))
    names = [f'{k} = {C.label_of(k).replace("Baseline: ", "")}' for k in present]
    for a in range(0, len(names), 4):
        L.append((' Configurations: ' if a == 0 else '                 ') + '   '.join(names[a:a + 4]))
    if base is None:
        L.append(col(f' (no baseline in this set - run nav:={C.BASELINE}; changes cannot be computed)', R.C_YELLOW))
    kw = max(14, max(len(k) for k in present) + 3)
    # 1) key metrics, one row per config
    cw = 16
    L.append('')
    L.append(col(' 1) KEY METRICS  mean (change vs baseline), all 5 drones: energy and flight time summed, path and '
                 'position error averaged over the drones, separation margin of the closest pair', R.C_BOLD))
    L.append(col(f" {'config':<{kw}}" + ''.join(f'{t:>{cw}}' for _, t in KEY_HEADS), R.C_GREY))
    for k in present:
        name = k + (' ◆' if k == C.BASELINE else '')
        L.append(f' {name:<{kw}}' + ''.join(_cell(summ, k, mk, cw) for mk, _ in KEY_HEADS))
    # 2) every metric, one column per config
    L.append('')
    L.append(col(' 2) ALL METRICS  (mean over runs)', R.C_BOLD))
    cw2 = max(11, max(len(k) for k in present) + 2)
    L.append(col(f" {'metric':<44}{'unit':>6}" + ''.join(f'{k:>{cw2}}' for k in present), R.C_GREY))
    for sec, ms in M.SECTIONS:
        L.append(col(f' {sec.split(" (")[0]}', R.C_BOLD))
        for mk, lab, unit, *_ in ms:
            vals = [summ[k].get(mk) for k in present]
            if all(v is None for v in vals):
                continue
            row = f' {lab[:44]:<44}{unit:>6}'
            for k, v in zip(present, vals):
                vd = verdict(summ, k, mk)
                cell = (_mfmt(mk, v) + (M.MARK[vd] if vd in ('better', 'worse') else ' ')).rjust(cw2)
                row += _paint(cell, vd)
            L.append(row)
    # 3) verdict counts + pareto
    L.append('')
    L.append(col(' 3) SUMMARY vs BASELINE', R.C_BOLD))
    if base:
        for k in present:
            if k == C.BASELINE:
                continue
            nb, ns, nw = counts(summ, k)
            ch = lambda mk: M.fmt_change(summ[k].get(mk), base.get(mk), mk)
            L.append(f"  {C.label_of(k)[:50]:<50} {col(f'{nb:>2} better', R.C_GREEN)}  {col(f'{nw:>2} worse', R.C_RED)}"
                     f"  {ns:>2} same   | energy {ch('fleet_energy_kj'):>6}  flight time {ch('fleet_flight_time_s'):>6}"
                     f"  path error {ch('fleet_cte_rms_mean_m'):>6}  position error {ch('fleet_loc_rms_mean_m'):>7}")
    L.append('')
    for xk, yk, txt in (('fleet_energy_kj', 'fleet_cte_rms_mean_m', 'energy AND path error'),
                        ('fleet_energy_kj', 'fleet_loc_rms_mean_m', 'energy AND position error')):
        L.append(f'  Pareto-optimal (lowest {txt}): ' + ', '.join(C.label_of(k) for k in pareto(summ, xk, yk)))
    best = lambda mk: min(present, key=lambda k: summ[k].get(mk) if summ[k].get(mk) is not None else 1e18)
    L.append(f"  Lowest energy: {best('fleet_energy_kj')}   |   most accurate paths: {best('fleet_cte_rms_mean_m')}"
             f"   |   best localisation: {best('fleet_loc_rms_mean_m')}")
    # 4) the paired statistics behind the verdicts, for the key metrics
    if base and n_tests(summ):
        L.append('')
        L.append(col(' 4) STATISTICS  paired difference vs baseline: mean ±95 % CI, Holm-corrected p, effect size d_z',
                     R.C_BOLD))
        sw = 33
        L.append(col(f" {'config':<{kw}}" + ''.join(f'{t:>{sw}}' for _, t in KEY_HEADS[:6]), R.C_GREY))
        for k in present:
            if k == C.BASELINE:
                continue
            row = f' {k:<{kw}}'
            for mk, _ in KEY_HEADS[:6]:
                sk = summ[k]
                if sk.get(mk + '_diff') is None:
                    row += '-'.rjust(sw)
                    continue
                ci = sk.get(mk + '_ci')
                dz = sk.get(mk + '_dz')
                txt = (f"{sk[mk + '_diff']:+.3g}"
                       + (f" ±{ci:.2g}" if ci is not None else '')
                       + f" p={_fmt_p(sk.get(mk + '_padj'))}"
                       + (f" d={dz:+.1f}" if dz is not None and math.isfinite(dz) else ''))
                row += _paint(txt.rjust(sw), verdict(summ, k, mk))
            L.append(row)
        L.append(col(f'  ({summ[present[1] if present[0] == C.BASELINE else present[0]].get("fleet_energy_kj_npair", 0)}'
                     ' paired missions per configuration; d_z = mean difference / SD of the differences)', R.C_GREY))
    for n in notes:
        L.append(col(f'  Note: {n}', R.C_YELLOW))
    L.append(col('═' * w, R.C_CYAN))
    return '\n'.join(L)


def write_summary_csv(path, summ):
    """One row per metric: per configuration the mean, SD, % change vs the baseline, and the paired
    statistics (mean difference, 95 % CI half-width, Holm-corrected p, effect size d_z) and the verdict."""
    base = summ.get(C.BASELINE, {})
    ks = list(summ)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        head = ['metric', 'key', 'unit', 'better']
        for tag in ('mean', 'sd', 'change % vs baseline', 'diff vs baseline', 'ci95 half-width', 'p (Holm)', 'd_z',
                    'verdict'):
            head += [f'{k} {tag}' for k in ks]
        w.writerow(head)
        for mk, lab, unit, _, better in M.ALL:
            row = [lab, mk, unit, better or '']
            row += [summ[k].get(mk) for k in ks]
            row += [summ[k].get(mk + '_sd') for k in ks]
            row += [M.change(summ[k].get(mk), base.get(mk)) for k in ks]
            row += [summ[k].get(mk + '_diff') for k in ks]
            row += [summ[k].get(mk + '_ci') for k in ks]
            row += [summ[k].get(mk + '_padj') for k in ks]
            row += [summ[k].get(mk + '_dz') for k in ks]
            row += [verdict(summ, k, mk) or '' for k in ks]
            w.writerow(row)


def _parse_keys(text):
    out = []
    for k in text.split(','):
        k = k.strip()
        if not k:
            continue
        out.append(k if k in C.FLEETS else C.nav_config(k).key)
    return list(dict.fromkeys(out))


def main(argv=None):
    """Run a study; whatever it switched for itself (planner, give-way, camera, wind, ...) is switched
    back afterwards, so nothing carries over to the next tool run in the same process."""
    try:
        return _main(argv)
    finally:
        _reset_overrides()
        layouts.apply(0)


def _main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--gazebo', action='store_true', help='compile your Gazebo runs of each config (no new sims)')
    ap.add_argument('--seeds', type=int, default=10, help='seeds (wind, gusts, sensor noise) per forest layout')
    ap.add_argument('--layouts', type=int, default=1,
                    help='forest layouts: 1 = only the Gazebo forest, N = it plus N-1 random forests')
    ap.add_argument('--configs', default=','.join(c.key for c in C.NAV_CONFIGS),
                    help='comma-separated configurations (or mixed fleets k0..k5, k0_swarm..k5_swarm)')
    ap.add_argument('--mixed', action='store_true', help='study the mixed fleets (k of 5 drones with a camera)')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--cam-fov', type=float, default=0.0, help='override camera coverage in degrees (e.g. 80)')
    ap.add_argument('--wind', type=float, default=None, help='mean wind speed in m/s (default 0.2)')
    ap.add_argument('--no-wind', action='store_true')
    ap.add_argument('--downwash-scale', type=float, default=None,
                    help='sensitivity: multiply the modelled downdraft (e.g. 0.5 = half as strong)')
    ap.add_argument('--layer-gap', type=float, default=None,
                    help='sensitivity: height gap between the fixed altitude layers in m (default 0.6)')
    ap.add_argument('--planner', choices=PLANNERS, default=None,
                    help=f'path planner of every drone (default {C.DEFAULT_PLANNER}; astar = the planner comparison)')
    ap.add_argument('--give-way', choices=GIVE_WAYS, default=None,
                    help=f'swarm give-way (default {C.DEFAULT_GIVE_WAY}: turn trading; priority = the comparison)')
    ap.add_argument('--radio-loss', type=float, default=None,
                    help='control experiment: probability that a radio message does not reach a drone (e.g. 0.2)')
    ap.add_argument('--radio-delay-ms', default=None,
                    help='control experiment: extra radio delay in ms, fixed ("100") or uniform ("50-250"), '
                         f'in steps of {1000 / C.CONTROL_RATE:g} ms')
    ap.add_argument('--radio-range', type=float, default=None,
                    help='control experiment: drones farther apart than this (m) do not hear each other')
    ap.add_argument('--drone-view-deg', type=float, default=None,
                    help='control experiment: how far above / below the horizon drones without the radio see '
                         'other drones (default 45; 90 = straight up and down as well)')
    ap.add_argument('--reactive-buffer', type=float, default=None,
                    help='control experiment: buffer (m) the reactive avoidance keeps on top of the separation '
                         'standard (default 0; the swarm give-way plans with 0.5)')
    ap.add_argument('--ttc-fixed', action='store_true',
                    help='control experiment: turn trading on its fixed schedule (0.5 s rounds, 0.1 s phases) '
                         'without the reactive fallback, as in package 1.8.4 (default: rounds timed to the link)')
    ap.add_argument('--results', default='')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--no-plots', action='store_true')
    ap.add_argument('--no-color', action='store_true')
    a = ap.parse_args(argv)
    if a.no_color or os.environ.get('NO_COLOR'):
        R.USE_COLOR = False
    if a.seeds < 1 or a.layouts < 1:
        ap.error('--seeds and --layouts must be at least 1')
    try:
        # k5 without radio is exactly the baseline (all five drones: 3-pt camera + VIO)
        keys = [f.key for f in C.FLEET_CONFIGS if f.key != 'k5'] if a.mixed else _parse_keys(a.configs)
    except ValueError as e:
        ap.error(str(e))
    if C.BASELINE not in keys:
        keys.insert(0, C.BASELINE)
    res = a.results or default_results_dir()
    ctrl = (a.radio_loss, a.radio_delay_ms, a.radio_range, a.drone_view_deg, a.reactive_buffer,
            a.ttc_fixed or None)
    if a.gazebo and (a.cam_fov or a.no_wind or a.wind is not None or a.layouts > 1 or a.mixed
                     or a.downwash_scale is not None or a.layer_gap is not None
                     or any(x is not None for x in ctrl)):
        print(col(' --cam-fov / --wind / --no-wind / --layouts / --mixed / --downwash-scale / --layer-gap and the'
                  ' control-experiment options only change'
                  ' NEW headless simulations; ignored with --gazebo (--planner and --give-way pick the Gazebo runs'
                  ' to compile)',
                  R.C_YELLOW))
        a.cam_fov, a.no_wind, a.wind, a.layouts = 0.0, False, None, 1
        a.downwash_scale = a.layer_gap = None
        a.radio_loss = a.radio_delay_ms = a.radio_range = a.drone_view_deg = a.reactive_buffer = None
        a.ttc_fixed = False
        keys = [k for k in keys if k in C.NAV]
    delay = None
    if a.radio_delay_ms is not None:
        try:
            ms = [float(x) for x in str(a.radio_delay_ms).split('-')]
        except ValueError:
            ap.error('--radio-delay-ms takes a number or a range like 50-250')
        step = 1000.0 / C.CONTROL_RATE
        delay = (int(round(ms[0] / step)), int(round(ms[-1] / step)))
    overrides = {'cam_fov': a.cam_fov, 'no_wind': a.no_wind, 'wind': a.wind,
                 'downwash_scale': a.downwash_scale, 'layer_gap': a.layer_gap, 'planner': a.planner,
                 'give_way': a.give_way, 'radio_loss': a.radio_loss, 'radio_delay_ticks': delay,
                 'radio_range': a.radio_range, 'drone_view_deg': a.drone_view_deg,
                 'reactive_buffer': a.reactive_buffer, 'ttc_fixed': a.ttc_fixed}
    _apply_overrides(overrides)
    wind_txt = (f'planner {PLANNER_NAMES[C.PLANNER]}, give-way {GIVE_WAY_NAMES[C.GIVE_WAY]}, '
                + ('no wind' if a.no_wind else f'wind {C.WIND.mean_speed:g} m/s'))
    if a.downwash_scale is not None:
        wind_txt += f', downwash x{a.downwash_scale:g}'
    if a.layer_gap is not None:
        wind_txt += f', layer gap {a.layer_gap:g} m'
    if a.radio_loss is not None:
        wind_txt += f', radio loss {100 * a.radio_loss:g} %'
    if delay is not None:
        step = 1000.0 / C.CONTROL_RATE
        wind_txt += f', radio delay +{delay[0] * step:g}' + (f'-{delay[1] * step:g}' if delay[1] != delay[0] else '') + ' ms'
    if a.radio_range is not None:
        wind_txt += f', radio range {a.radio_range:g} m'
    if a.drone_view_deg is not None:
        wind_txt += f', drones seen up to {a.drone_view_deg:g} deg above/below'
    if a.reactive_buffer is not None:
        wind_txt += f', reactive buffer {a.reactive_buffer:g} m'
    if a.ttc_fixed:
        wind_txt += ', turn trading on its fixed schedule'

    notes = []
    if a.gazebo:
        rows, trajs, missing, notes = collect_gazebo(keys, res)
        source = 'Gazebo runs'
        seeds = sorted({r['seed'] for r in rows})
        n_info = (f'{len(rows)} runs of {len({r["config"] for r in rows})} of {len(keys)} configurations '
                  f'(seeds {", ".join(str(x) for x in seeds)}), forest layout 0, from {os.path.basename(os.path.normpath(res))}/')
        for r in rows:
            print(f"   {r['config']:<20} seed {r['seed']:<3} <- {r['run']}")
        if missing:
            print(col(f' Not run in Gazebo yet: {", ".join(missing)}', R.C_YELLOW))
            for k in missing:
                print(f'   ros2 launch swarm_intel swarm_mission.launch.py nav:={k}')
        if not rows:
            print('No Gazebo runs of the study configurations found. Run them first, or use the headless study.')
            return 1
    else:
        layout_ids = list(range(a.layouts))
        n = len(keys) * a.layouts * a.seeds
        print(f'Headless study: {len(keys)} configurations x {a.layouts} forest layout(s) x {a.seeds} seeds = {n} missions'
              f' (speed x{a.speed}, camera {C.SENSORS.cam_fov_deg:.0f} deg, {wind_txt})')
        print('  ' + blas_kernel_note())
        rows, trajs = run_headless_study(keys, layout_ids, a.seeds, a.speed, a.workers, overrides)
        source = 'Headless simulation (same mission code, no GUI)'
        n_info = (f'{a.layouts} forest layout{"s" if a.layouts > 1 else ""} x {a.seeds} seeds x {len(keys)} configs, '
                  f'camera {C.SENSORS.cam_fov_deg:.0f} deg, {wind_txt}, speed x{a.speed}, package {C.MODEL_VERSION}')

    layouts.apply(0)                                  # plots draw the Gazebo forest
    summ = summarize(rows, keys)
    tag = 'gazebo' if a.gazebo else 'headless'
    if a.mixed:
        tag += '_mixed'
    if a.cam_fov:
        tag += f'_cam{a.cam_fov:.0f}'
    if a.no_wind:
        tag += '_nowind'
    elif a.wind is not None:
        tag += f'_wind{a.wind:g}'
    if a.downwash_scale is not None:
        tag += f'_dw{a.downwash_scale:g}'
    if a.layer_gap is not None:
        tag += f'_gap{a.layer_gap:g}'
    if a.radio_loss is not None:
        tag += f'_loss{a.radio_loss:g}'
    if delay is not None:
        tag += f'_delay{delay[0]}-{delay[1]}'
    if a.radio_range is not None:
        tag += f'_range{a.radio_range:g}'
    if a.drone_view_deg is not None:
        tag += f'_view{a.drone_view_deg:g}'
    if a.reactive_buffer is not None:
        tag += f'_rbuf{a.reactive_buffer:g}'
    if a.ttc_fixed:
        tag += '_ttcfixed'
    if a.planner and a.planner != _DEFAULTS['planner']:
        tag += f'_{a.planner}'
    if a.give_way and a.give_way != _DEFAULTS['give_way']:
        tag += f'_{a.give_way}'
    if not a.gazebo and a.layouts > 1:
        tag += f'_L{a.layouts}'                       # several forests: a paper-grade study
    out_dir = os.path.join(res, f"study_{tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out_dir, exist_ok=True)
    write_meta(out_dir, kind='study', source=source, missions=n_info, configurations=keys)
    text = tables(summ, keys, source, n_info, notes, color=R.USE_COLOR)
    print(text)
    use = R.USE_COLOR
    plain = tables(summ, keys, source, n_info, notes, color=False)
    R.USE_COLOR = use
    with open(os.path.join(out_dir, 'report.txt'), 'w') as f:
        f.write(plain + '\n')
    fields = ['config', 'layout', 'seed', 'source', 'run'] + [m[0] for m in M.ALL]
    with open(os.path.join(out_dir, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (keys.index(r['config']), r.get('layout', 0), r['seed'])))
    write_summary_csv(os.path.join(out_dir, 'summary.csv'), summ)
    if not a.no_plots:
        try:
            from .study_plots import figures
            files = figures(summ, keys, trajs, out_dir, source, n_info)
            print('Graphs: ' + ', '.join(os.path.basename(x) for x in files))
            print(col(f'Final comparison graph: {os.path.join(out_dir, "study_comparison.png")}', R.C_BOLD))
        except Exception as e:                                 # plotting must never lose the numbers
            print(f'(graphs skipped: {e})')
    print(f'Saved to {out_dir}  (report.txt, runs.csv, summary.csv)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
