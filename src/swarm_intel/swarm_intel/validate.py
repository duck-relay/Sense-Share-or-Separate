#!/usr/bin/env python3
"""Does the fast headless simulator agree with Gazebo?

  ros2 run swarm_intel validate                    # every Gazebo run of this package version
  ros2 run swarm_intel validate --configs cam3_sensor,all

For every Gazebo run (made with  gazebo_study --seeds N  or by hand) this flies the SAME
mission headless: same configuration, seed (wind, gusts, sensor noise), forest and speed.
Only the plant differs: Gazebo's physics vs the kinematic stand-in. It then reports:

1) per metric, over all runs: the mean of both simulators, their mean and median paired
   difference, Pearson r and Spearman r over the runs (run-by-run agreement) and Spearman rho
   over the configuration means (do they RANK the configurations the same way?)
2) the same without configurations that lose their position in BOTH simulators (mean
   position error > 1 m, e.g. swarm only): their huge values otherwise dominate the means
   and make Pearson r look perfect
3) the EFFECT of every configuration vs the baseline (change in %, paired by seed) in Gazebo
   and headless side by side - this is what the study's conclusions rest on

Writes <results>/validation_<time>/ : report.txt, pairs.csv, effects.csv, validation.png
"""
import argparse
import csv
import math
import os
import sys
from datetime import datetime
from multiprocessing import Pool

import numpy as np

from . import config as C
from . import layouts
from . import metrics as M
from .paths import default_results_dir, runs_of, load_report, study_run
from .runinfo import write_meta
from .grid_planner import PLANNERS
from .ttc import GIVE_WAYS

# every metric covers the whole fleet (all 5 drones)
METRICS = [('fleet_energy_kj', 'Flight energy (kJ)'), ('fleet_flight_time_s', 'Flight time (s)'),
           ('fleet_mission_time_s', 'Mission time (s)'),
           ('fleet_cte_rms_mean_m', 'Path deviation RMS, mean (m)'), ('fleet_track_rms_mean_m', 'Tracking error RMS, mean (m)'),
           ('fleet_loc_rms_mean_m', 'Position error RMS, mean (m)'), ('fleet_landing_err_mean_m', 'Landing error, mean (m)'),
           ('fleet_min_separation_3d_m', 'Closest pair, 3-D (m)'),
           ('fleet_min_sep_margin_m', 'Separation margin (m)'), ('fleet_wake_time_s', 'Time in downwash (s)')]


def _ranks(v):
    v = np.asarray(v, float)
    order = np.argsort(v, kind='mergesort')
    r = np.empty(len(v))
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        r[order[i:j + 1]] = 0.5 * (i + j)
        i = j + 1
    return r


def _pearson(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3 or x.std() < 1e-12 or y.std() < 1e-12:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def _spearman(x, y):
    return _pearson(_ranks(x), _ranks(y)) if len(x) >= 3 else None


EFFECT_METRICS = [('fleet_energy_kj', 'Energy'), ('fleet_flight_time_s', 'Flight time'),
                  ('fleet_track_rms_mean_m', 'Tracking error'), ('fleet_loc_rms_mean_m', 'Position error')]
LOST_M = 1.0          # mean position error above which a configuration is 'lost' (no absolute reference)


def _val(p, side, mk):
    v = p[side].get(mk)
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def lost_configs(pairs):
    """Configurations whose mean position error exceeds LOST_M in BOTH simulators."""
    out = []
    for k in sorted({p['config'] for p in pairs}):
        sel = [p for p in pairs if p['config'] == k]
        g = [v for v in (_val(p, 'g', 'fleet_loc_rms_mean_m') for p in sel) if v is not None]
        h = [v for v in (_val(p, 'h', 'fleet_loc_rms_mean_m') for p in sel) if v is not None]
        if g and h and np.mean(g) > LOST_M and np.mean(h) > LOST_M:
            out.append(k)
    return out


def agreement(pairs, metrics):
    """Per metric: means, mean/median paired difference, Pearson/Spearman over runs, rho over config means."""
    rows = []
    for mk, lab in metrics:
        sel = [p for p in pairs if _val(p, 'g', mk) is not None and _val(p, 'h', mk) is not None]
        if len(sel) < 2:
            continue
        g = np.array([_val(p, 'g', mk) for p in sel])
        h = np.array([_val(p, 'h', mk) for p in sel])
        # a % of a value near zero (separation margin, downwash time) means nothing: absolute difference only
        pct_ok = mk not in M.ABS_TOL
        rel = 100.0 * (g.mean() - h.mean()) / abs(h.mean()) if pct_ok and abs(h.mean()) > 1e-9 else float('nan')
        rr = [100.0 * (a / b - 1) for a, b in zip(g, h) if pct_ok and abs(b) > 1e-9]
        cfgs = sorted({p['config'] for p in sel})
        gm = [np.mean([_val(p, 'g', mk) for p in sel if p['config'] == k]) for k in cfgs]
        hm = [np.mean([_val(p, 'h', mk) for p in sel if p['config'] == k]) for k in cfgs]
        rows.append({'metric': mk, 'label': lab, 'n': len(sel), 'g': float(g.mean()), 'h': float(h.mean()),
                     'diff': float((g - h).mean()), 'rel': rel, 'med_rel': float(np.median(rr)) if rr else float('nan'),
                     'r': _pearson(g, h), 'r_s': _spearman(g, h),
                     'rho': _spearman(gm, hm) if len(cfgs) >= 3 else None})
    return rows


def effects(pairs, metrics=EFFECT_METRICS, base=None):
    """Change of every configuration vs the baseline (%), paired by seed, in both simulators.
    Returns {config: {metric: (gazebo %, headless %, n seeds)}}."""
    base = base or C.BASELINE
    B = {(p['seed']): p for p in pairs if p['config'] == base}
    out = {}
    order = [c.key for c in C.NAV_CONFIGS]
    present = {p['config'] for p in pairs} - {base}
    for k in [k for k in order if k in present] + sorted(present - set(order)):
        sel = [p for p in pairs if p['config'] == k and p['seed'] in B]
        res = {}
        for mk, _ in metrics:
            ok = [(p, B[p['seed']]) for p in sel if None not in (_val(p, 'g', mk), _val(p, 'h', mk),
                                                                  _val(B[p['seed']], 'g', mk), _val(B[p['seed']], 'h', mk))]
            if not ok:
                continue
            gb = np.mean([_val(b, 'g', mk) for _, b in ok])
            hb = np.mean([_val(b, 'h', mk) for _, b in ok])
            if abs(gb) < 1e-9 or abs(hb) < 1e-9:
                continue
            res[mk] = (100.0 * (np.mean([_val(p, 'g', mk) for p, _ in ok]) / gb - 1),
                       100.0 * (np.mean([_val(p, 'h', mk) for p, _ in ok]) / hb - 1), len(ok))
        out[k] = res
    return out


def effect_agreement(eff, keys, metrics=EFFECT_METRICS, min_pct=2.0):
    """Per metric: Spearman of the effects over configurations, median |gazebo - headless| (percentage
    points), and how many effects larger than min_pct in either simulator have the same sign."""
    out = {}
    for mk, _ in metrics:
        v = [eff[k][mk] for k in keys if mk in eff.get(k, {})]
        if not v:
            continue
        g = [a for a, _, _ in v]
        h = [b for _, b, _ in v]
        big = [(a, b) for a, b in zip(g, h) if max(abs(a), abs(b)) > min_pct]
        out[mk] = {'rho': _spearman(g, h), 'med_abs_pp': float(np.median(np.abs(np.subtract(g, h)))),
                   'same_sign': sum(1 for a, b in big if a * b > 0), 'n_big': len(big), 'n': len(v)}
    return out


def read_pairs(path):
    """pairs.csv of a validation folder -> list of {'config', 'seed', 'run', 'g': {...}, 'h': {...}}."""
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            p = {'config': r['config'], 'seed': int(r['seed']), 'run': r.get('gazebo_run', ''), 'g': {}, 'h': {}}
            for key, v in r.items():
                for side, tag in (('g', ' gazebo'), ('h', ' headless')):
                    if key.endswith(tag):
                        p[side][key[:-len(tag)]] = v
            out.append(p)
    return out


def _twin(args):
    key, seed, speed, layout, planner, give_way = args
    from .plant import run_headless
    C.PLANNER = planner
    C.GIVE_WAY = give_way
    layouts.apply(layout)
    rep, _ = run_headless(key, seed=seed, speed_scale=speed)
    return args, M.extract(rep)


def gazebo_runs(res, keys):
    """Newest complete Gazebo-study run (every drone landed) of this package version and planner per (config, seed);
    runs watched from the menu are not used."""
    out = []
    for k in keys:
        by_seed = {}
        for d in runs_of(res, k):
            try:
                rep = load_report(d)
            except Exception:
                continue
            if (rep.get('model_version') != C.MODEL_VERSION or rep.get('planner', 'astar') != C.PLANNER
                    or rep.get('give_way', 'priority') != C.GIVE_WAY or not rep.get('final') or not study_run(rep)):
                continue                       # other version, planner or give-way, not finished, or a watched run
            by_seed[int(rep.get('seed', 0))] = (d, rep)
        for sd in sorted(by_seed):
            out.append((k, sd) + by_seed[sd])
    return out


def main(argv=None):
    """Runs with the chosen planner and give-way, then switches back to the study's own settings."""
    try:
        return _main(argv)
    finally:
        C.PLANNER, C.GIVE_WAY = C.DEFAULT_PLANNER, C.DEFAULT_GIVE_WAY


def _main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default='')
    ap.add_argument('--configs', default=','.join(c.key for c in C.NAV_CONFIGS))
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--planner', choices=PLANNERS, default=C.DEFAULT_PLANNER, help='validate the runs of this planner')
    ap.add_argument('--give-way', choices=GIVE_WAYS, default=C.DEFAULT_GIVE_WAY, help='... and of this give-way')
    a = ap.parse_args(argv)
    C.PLANNER = a.planner
    C.GIVE_WAY = a.give_way
    res = a.results or default_results_dir()
    keys = list(dict.fromkeys(C.nav_config(k).key for k in a.configs.split(',') if k.strip()))
    runs = gazebo_runs(res, keys)
    if not runs:
        print(f'No Gazebo-study runs of package version {C.MODEL_VERSION} with planner {C.PLANNER} and give-way '
              f'{C.GIVE_WAY} found in {res}.\n'
              'Make some first:  ros2 run swarm_intel gazebo_study --seeds 3')
        return 1
    print(f'Validation: {len(runs)} Gazebo runs -> flying the same {len(runs)} missions headless ...')
    jobs = [(k, sd, float(rep.get('speed_scale', 1.0)), int(rep.get('layout', 0)), C.PLANNER, C.GIVE_WAY)
            for k, sd, d, rep in runs]
    with Pool(a.workers or max(1, min(os.cpu_count() or 1, 8))) as pool:
        twins = dict(pool.map(_twin, jobs))
    pairs = []
    for (k, sd, d, rep), job in zip(runs, jobs):
        g = M.extract(rep)
        h = twins[job]
        pairs.append({'config': k, 'seed': sd, 'run': os.path.basename(d), 'g': g, 'h': h})

    out_dir = os.path.join(res, f"validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out_dir, exist_ok=True)
    write_meta(out_dir, kind='validation', gazebo_runs=sorted(p['run'] for p in pairs))
    text, eff = report_text(pairs)
    print(text)
    with open(os.path.join(out_dir, 'report.txt'), 'w') as f:
        f.write(text + '\n')
    with open(os.path.join(out_dir, 'pairs.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['config', 'seed', 'gazebo_run'] + [f'{mk} gazebo' for mk, _ in METRICS]
                   + [f'{mk} headless' for mk, _ in METRICS])
        for p in pairs:
            w.writerow([p['config'], p['seed'], p['run']] + [p['g'].get(mk) for mk, _ in METRICS]
                       + [p['h'].get(mk) for mk, _ in METRICS])
    with open(os.path.join(out_dir, 'effects.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['config', 'seeds'] + [f'{mk} change % {s}' for mk, _ in EFFECT_METRICS for s in ('gazebo', 'headless')])
        for k, d in eff.items():
            n = max((v[2] for v in d.values()), default=0)
            w.writerow([k, n] + [(f'{d[mk][i]:.2f}' if mk in d else '') for mk, _ in EFFECT_METRICS for i in (0, 1)])
    try:
        _plot(pairs, os.path.join(out_dir, 'validation.png'))
    except Exception as e:
        print(f'(graph skipped: {e})')
    print(f'Saved to {out_dir}  (report.txt, pairs.csv, effects.csv, validation.png)')
    return 0


def _pc(v):
    """A change in % for the report: '+12%', or 'x14' for very large changes."""
    if v is None or not math.isfinite(v):
        return '-'
    return f'x{1 + v / 100:.0f}' if v >= 900 else f'{v:+.0f}%'


def _f(v, spec, suffix=''):
    return '-' if v is None or not math.isfinite(v) else format(v, spec) + suffix


def report_text(pairs):
    """The validation report (text) and the per-configuration effects."""
    W = 132
    lost = lost_configs(pairs)
    keep = [p for p in pairs if p['config'] not in lost]
    L = ['=' * W,
         ' VALIDATION: Gazebo vs the headless simulator on the same missions (same configuration, seed, forest, speed)',
         f' {len(pairs)} paired runs of {len({p["config"] for p in pairs})} configurations, package {C.MODEL_VERSION}, planner {C.PLANNER}, give-way {C.GIVE_WAY}',
         '=' * W]

    def block(title, sel):
        L.append(' ' + title)
        L.append(f" {'metric':<34}{'Gazebo':>10}{'headless':>10}{'diff':>10}{'diff %':>9}{'median %':>10}"
                 f"{'Pearson r':>11}{'Spearman r':>12}{'rho configs':>13}")
        for a in agreement(sel, METRICS):
            L.append(f" {a['label']:<34}{a['g']:>10.3f}{a['h']:>10.3f}{a['diff']:>+10.3f}"
                     f"{_f(a['rel'], '+.1f', '%'):>9}{_f(a['med_rel'], '+.1f', '%'):>10}"
                     f"{_f(a['r'], '.2f'):>11}{_f(a['r_s'], '.2f'):>12}{_f(a['rho'], '.2f'):>13}")
        L.append('')

    block('1) ALL RUNS', pairs)
    if lost and keep:
        block(f'2) WITHOUT CONFIGURATIONS THAT LOSE THEIR POSITION IN BOTH SIMULATORS '
              f'(mean position error > {LOST_M:g} m): ' + ', '.join(lost), keep)
    eff = effects(pairs)
    keys = [k for k in eff if k not in lost]
    L.append(' 3) EFFECT OF EACH CONFIGURATION vs THE BASELINE (change in %, paired by seed)   Gazebo | headless')
    L.append(f" {'configuration':<26}{'seeds':>6}" + ''.join(f'{lab:>24}' for _, lab in EFFECT_METRICS))
    for k, d in eff.items():
        n = max((v[2] for v in d.values()), default=0)
        cells = ''.join(f"{(_pc(d[mk][0]) + ' | ' + _pc(d[mk][1])) if mk in d else '-':>24}"
                        for mk, _ in EFFECT_METRICS)
        L.append(f" {k + (' (lost)' if k in lost else ''):<26}{n:>6}{cells}")
    ag = effect_agreement(eff, keys)
    rho_cells, sign_cells = '', ''
    for mk, _ in EFFECT_METRICS:
        a = ag.get(mk)
        rho_cells += f"{('rho ' + _f(a['rho'], '.2f') + ', ' + _f(a['med_abs_pp'], '.1f') + ' pp') if a else '-':>24}"
        sign_cells += f"{(str(a['same_sign']) + ' of ' + str(a['n_big'])) if a else '-':>24}"
    L.append(f" {'agreement' + (' (without lost)' if lost else ''):<32}" + rho_cells)
    L.append(f" {'same sign (effects > 2 %)':<32}" + sign_cells)
    L += ['',
          ' Reading it: diff % of a few % and rho close to 1 mean the fast simulator can stand in for Gazebo.',
          " Section 3 is what the study's conclusions rest on: does Gazebo show the same change vs the baseline?",
          ' pp = median |Gazebo - headless| in percentage points. With few seeds, small effects (< ~5 %) are within noise.',
          '=' * W]
    return '\n'.join(L), eff


def _plot(pairs, path):
    """One scatter per metric (each dot = one mission). Axes become logarithmic when the values span
    more than a factor 20 (e.g. a configuration that loses its position), so the bulk stays readable.
    The numbers in each panel leave out configurations that are lost in both simulators."""
    from . import plots as P
    plt = P.setup()
    lost = set(lost_configs(pairs))
    summary = agreement([p for p in pairs if p['config'] not in lost] or pairs, METRICS)
    n = len(summary)
    cols = 4
    rows = math.ceil(n / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(4.1 * cols, 3.9 * rows + 0.8), squeeze=False)
    fig.subplots_adjust(left=0.06, right=0.98, top=1 - 0.95 / (3.9 * rows + 0.8), bottom=0.08, wspace=0.32, hspace=0.45)
    fig.suptitle('Gazebo vs headless simulator on the same missions (each dot = one mission)', x=0.012, ha='left',
                 fontsize=13, fontweight='bold', color=P.INK)
    for ax, a in zip(axs.ravel(), summary):
        mk = a['metric']
        sel = [p for p in pairs if _val(p, 'g', mk) is not None and _val(p, 'h', mk) is not None]
        x = np.array([_val(p, 'h', mk) for p in sel])
        y = np.array([_val(p, 'g', mk) for p in sel])
        is_lost = np.array([p['config'] in lost for p in sel])
        allv = np.concatenate([x, y])
        log = allv.min() > 0 and allv.max() / allv.min() > 20
        off = 0
        if log:
            lo, hi = allv.min() / 1.3, allv.max() * 1.3
            ax.set_xscale('log')
            ax.set_yscale('log')
        else:
            # axes fit the configurations that keep their position; lost runs beyond them sit on the edge
            core = np.concatenate([x[~is_lost], y[~is_lost]]) if (~is_lost).any() else allv
            lo, hi = core.min(), core.max()
            pad = 0.06 * (hi - lo if hi > lo else 1.0)
            lo, hi = lo - pad, hi + pad
            off = int(np.sum(is_lost & ((x > hi) | (y > hi) | (x < lo) | (y < lo))))
        ax.plot([lo, hi], [lo, hi], color=P.MUTED, lw=1, ls='--', zorder=1)
        ax.scatter(x[~is_lost], y[~is_lost], s=22, color=P.CAT_A, edgecolor=P.SURFACE, linewidth=0.8, zorder=3)
        if is_lost.any():
            ax.scatter(np.clip(x[is_lost], lo, hi), np.clip(y[is_lost], lo, hi), s=26, marker='X', color=P.MUTED,
                       edgecolor=P.SURFACE, linewidth=0.6, zorder=3, clip_on=False)
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.set_title(a['label'] + (' (log axes)' if log else ''), fontsize=10)
        ax.set_xlabel('headless', fontsize=8.5)
        ax.set_ylabel('Gazebo', fontsize=8.5)
        txt = f"diff {_f(a['rel'], '+.1f', '%')}, median {_f(a['med_rel'], '+.1f', '%')}"
        txt += f"\nSpearman r = {_f(a['r_s'], '.2f')}"
        txt += f"\nρ (ranking) = {_f(a['rho'], '.2f')}"
        if off:
            txt += f"\n{off} lost run{'s' if off > 1 else ''} beyond the axes (on the edge)"
        ax.text(0.04, 0.96, txt, transform=ax.transAxes, va='top', fontsize=8, color=P.INK)
    for ax in axs.ravel()[n:]:
        ax.axis('off')
    note = 'Dashed line: perfect agreement. ρ = Spearman rank correlation of the configuration means.'
    if lost:
        note += (f"  Grey ×: {', '.join(sorted(lost))} (loses its position in both simulators; "
                 f"left out of the numbers in each panel).")
    P.footer(fig, note)
    fig.savefig(path, dpi=120)
    plt.close(fig)


if __name__ == '__main__':
    sys.exit(main())
