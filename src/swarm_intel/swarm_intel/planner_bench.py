#!/usr/bin/env python3
"""Planner scaling benchmark: RRT* (the drones' planner) vs A* on larger fields and finer grids.

    ros2 run swarm_intel planner_bench                     # 20/40/80 m fields x 0.2/0.1/0.05 m cells
    ros2 run swarm_intel planner_bench --forests 4 --fields 20,40

The comparison rules were fixed before the benchmark was run, and every result is reported:

* Maps: forests with the density and spacing rules of the study forests (layouts.py: 20 landmark
  trees and 6 pines per 400 m^2, trees >= 1.8 m apart, ...), on square fields of 20, 40 and 80 m.
  Start and goal are opposite corners, 2 m in from the edges (as drone1's pads on the 20 m
  field). A forest is used only if start and goal are connected in the inflated grid (a flood fill:
  this check does not use either planner). Every obstacle is known: this measures the planners,
  not the flight.
* Both planners get the same inflated occupancy grid (drone radius + margins, as in flight), the
  same start and goal, the same line-of-sight pruning afterwards, the same machine and one thread.
  The grid inflation is common to both and reported separately ("map").
* A*: 8-connected, octile heuristic, no corner cutting (grid_planner.astar). RRT*: one tree,
  informed sampling, 1000 samples per 400 m^2 of field (the flight setting, scaled with the area),
  given up after 6000 per 400 m^2; when the regular budget ends without a route it checks once
  whether one exists at all (rrt_star.py); seeded per field and forest.
* Measured per plan: success, planning time (wall clock), route length, and peak memory allocated
  while planning (tracemalloc, in a second, identical run). Per field and cell size, the paired
  ratios RRT* / A* over the forests are tested with a sign-flip permutation test on their
  logarithm, Holm-corrected over the field x cell-size combinations; a planner counts as faster
  (shorter, leaner) only if p < 0.05 and the geometric mean ratio differs from 1 by more than 2 %.
* No route: on each field and cell size one extra map is cut in two by a row of trees between start
  and goal; the time each planner takes to report that there is no route is reported too. (A start
  or goal walled into a small pocket is caught before either planner runs, by the same bounded
  flood fill for both: grid_planner.plan_path.)

Writes results/planner_bench_<time>/ : report.txt, runs.csv, summary.csv, planner_scaling.png/.pdf.
"""
import argparse
import csv
import math
import os
import platform
import sys
import time
import tracemalloc

import numpy as np

from . import config as C
from . import layouts as L
from . import stats
from .grid_planner import GridMap, plan_path, cells_connected, flat_cells
from .paths import default_results_dir
from .runinfo import cpu_name, write_meta
from .rrt_star import DEFAULTS as RRT_DEFAULTS, BUDGET_AREA

PLAN_INFLATION = C.DRONE_RADIUS + C.PLAN_MARGIN + C.PLAN_HYSTERESIS     # = agent.PLAN_INFLATION
PAD_IN = 2.0                 # m: start / goal this far in from the edges (drone1's pads)
EFFECT = 0.02                # a ratio counts only if it differs from 1 by more than this
PLANNERS = ('astar', 'rrtstar')
NAMES = {'astar': 'A*', 'rrtstar': 'RRT*', 'rrtstar2': 'RRT* (two trees)'}


def forest(field, k):
    """Trees and pines of forest k on a field x field m square, with the study's density and spacing."""
    half = field / 2.0
    area = field * field
    n_trees = int(round(L.N_TREES * area / 400.0))
    n_pines = int(round(L.N_PINES * area / 400.0))
    start, goal = np.array([-half + PAD_IN] * 2), np.array([half - PAD_IN] * 2)
    rng = np.random.default_rng([7919, int(field), int(k)])
    edge = half - (10.0 - L.EDGE)
    for _ in range(500):
        trees, pines = [], []
        for _ in range(60 * (n_trees + n_pines)):
            if len(trees) == n_trees and len(pines) == n_pines:
                break
            p = rng.uniform(-edge, edge, 2)
            pine = len(trees) == n_trees
            pads = (L.PAD_GAP_PINE if pine else L.PAD_GAP) + 0.0
            if min(np.linalg.norm(p - start), np.linalg.norm(p - goal)) < pads:
                continue
            gap_t = L.TREE_PINE_GAP if pine else L.TREE_GAP
            if any(np.linalg.norm(p - q) < gap_t for q in trees):
                continue
            if any(np.linalg.norm(p - q) < (L.PINE_GAP if pine else L.TREE_PINE_GAP) for q in pines):
                continue
            (pines if pine else trees).append(p)
        if len(trees) == n_trees and len(pines) == n_pines:
            gm = grid(field, 0.2, trees, pines)
            bl = gm.inflated(PLAN_INFLATION)
            s, g = gm.to_cell(*start), gm.to_cell(*goal)
            if not bl[s] and not bl[g] and cells_connected(flat_cells(bl), gm.n, gm.n, s[0] * gm.n + s[1],
                                                           g[0] * gm.n + g[1]):
                return trees, pines, tuple(start), tuple(goal)
    raise RuntimeError(f'no connected forest found for field {field} m, forest {k}')


def grid(field, res, trees, pines, extra=()):
    gm = GridMap(half=field / 2.0, res=res)
    for (x, y) in trees:
        gm.seed_disk(x, y, C.TREE_CROWN_R)
    for (x, y) in pines:
        gm.seed_disk(x, y, C.PINE_PLAN_R)
    for (x, y) in extra:
        gm.seed_disk(x, y, C.TREE_CROWN_R)
    return gm


def wall_across(field):
    """A closed row of trees along the diagonal x + y = 0, from edge to edge: the field is cut in two
    (start on one side, goal on the other), so no route exists."""
    half = field / 2.0
    n = int(math.ceil(2 * half * math.sqrt(2) / 0.6)) + 1
    return [(-half + 2 * half * i / (n - 1), half - 2 * half * i / (n - 1)) for i in range(n)]


def length(p):
    return float(np.sum(np.linalg.norm(np.diff(np.asarray(p, float), axis=0), axis=1))) if p else float('nan')


def plan_once(gm, planner, start, goal, seed, memory):
    """(path, ms, peak KiB) of one plan on gm (inflation already cached)."""
    kw = {}
    name = planner
    if planner.startswith('rrtstar'):
        kw['rng'] = np.random.default_rng(seed)
        if planner == 'rrtstar2':
            from . import rrt_star as R
            R.DEFAULTS['bidirectional'] = True
        name = 'rrtstar'
    try:
        if memory:
            tracemalloc.start()
            plan_path(gm, start, goal, PLAN_INFLATION, planner=name, **kw)
            peak = tracemalloc.get_traced_memory()[1] / 1024.0
            tracemalloc.stop()
            if planner.startswith('rrtstar'):
                kw['rng'] = np.random.default_rng(seed)
        else:
            peak = float('nan')
        t0 = time.perf_counter()
        path = plan_path(gm, start, goal, PLAN_INFLATION, planner=name, **kw)
        ms = 1000.0 * (time.perf_counter() - t0)
    finally:
        if planner == 'rrtstar2':
            from . import rrt_star as R
            R.DEFAULTS['bidirectional'] = False
    return path, ms, peak


def gmean_ratio(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ok = np.isfinite(a) & np.isfinite(b) & (a > 0) & (b > 0)
    if not ok.any():
        return float('nan'), np.array([])
    d = np.log(a[ok]) - np.log(b[ok])
    return float(np.exp(d.mean())), d


def verdict(ratio, p, better_low=True):
    if not np.isfinite(ratio) or p is None or p >= 0.05 or abs(ratio - 1.0) <= EFFECT:
        return '='
    return 'RRT*' if (ratio < 1.0) == better_low else 'A*'


def summarize(rows, fields, cells, planners):
    """Per field x cell size: success, times, lengths, memory, and the paired RRT* / A* ratios."""
    out = []
    for f in fields:
        for c in cells:
            R = [r for r in rows if r['field'] == f and r['cells'] == c and not r['walled']]
            W = {r['planner']: r for r in rows if r['field'] == f and r['cells'] == c and r['walled']}
            by = {pl: sorted((r for r in R if r['planner'] == pl), key=lambda r: r['forest']) for pl in planners}
            s = {'field': f, 'cells': c, 'grid_cells': int(round(f / c)) ** 2, 'forests': len(by['astar']),
                 'map_ms': float(np.median([r['map_ms'] for r in by['astar']])) if by['astar'] else float('nan')}
            for pl in planners:
                rr = by[pl]
                okv = [r for r in rr if r['ok']]
                s[f'{pl}_ok'] = len(okv)
                s[f'{pl}_ms_med'] = float(np.median([r['ms'] for r in rr])) if rr else float('nan')
                s[f'{pl}_ms_p95'] = float(np.percentile([r['ms'] for r in rr], 95)) if rr else float('nan')
                s[f'{pl}_len_med'] = float(np.median([r['length'] for r in okv])) if okv else float('nan')
                s[f'{pl}_kib_med'] = float(np.nanmedian([r['kib'] for r in rr])) if rr and any(
                    np.isfinite(r['kib']) for r in rr) else float('nan')
                s[f'{pl}_walled_ms'] = W[pl]['ms'] if pl in W else float('nan')
            a, b = by['rrtstar'], by['astar']
            for key, field_name in (('time', 'ms'), ('length', 'length'), ('memory', 'kib')):
                pairs = [(x[field_name], y[field_name]) for x, y in zip(a, b)
                         if (key != 'length' or (x['ok'] and y['ok']))]
                ratio, d = gmean_ratio([p[0] for p in pairs], [p[1] for p in pairs])
                s[f'{key}_ratio'] = ratio
                s[f'{key}_n'] = len(d)
                s[f'{key}_p'] = stats.signflip_p(d) if len(d) >= 2 else None
            out.append(s)
    for key in ('time', 'length', 'memory'):
        ps = [s[f'{key}_p'] if s[f'{key}_p'] is not None else 1.0 for s in out]
        adj = stats.holm(ps)
        for s, pa in zip(out, adj):
            s[f'{key}_p_holm'] = float(pa)
            s[f'{key}_verdict'] = verdict(s[f'{key}_ratio'], pa if s[f'{key}_p'] is not None else None)
    return out


def fmt(v, nd=0, unit=''):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return '-'
    return f'{v:.{nd}f}{unit}'


def report(summary, planners, meta):
    L_ = []
    L_.append('PLANNER SCALING BENCHMARK: RRT* (the drones\' planner) vs A*')
    L_.append(f'  {meta}')
    L_.append('  Same inflated grid, start, goal and line-of-sight pruning for both planners; every obstacle known.')
    L_.append('  Times: one plan, wall clock, one thread (the grid inflation, common to both, is the "map" column).')
    L_.append('  Ratios RRT*/A*: geometric mean over the forests; verdict = the better planner if p (sign-flip, Holm)')
    L_.append(f'  < 0.05 and the ratio differs from 1 by more than {100 * EFFECT:.0f} %, otherwise "=".')
    L_.append('')
    hdr = (f'  {"field":>6} {"cells":>6} {"grid":>9} {"map":>7} | {"A* ok":>5} {"A* ms":>8} {"p95":>8} | '
           f'{"RRT* ok":>7} {"RRT* ms":>8} {"p95":>8} | {"time":>6} {"faster":>6} | {"length":>6} {"shorter":>7} | '
           f'{"memory":>6} {"leaner":>6}')
    L_.append('  1) PLANNING TIME, ROUTE LENGTH AND MEMORY  (ratios RRT*/A*: below 1 = RRT* better)')
    L_.append(hdr)
    for s in summary:
        L_.append(f'  {s["field"]:>4} m {s["cells"]:>5.2f}m {s["grid_cells"]:>9,d} {fmt(s["map_ms"], 0, "ms"):>7} | '
                  f'{s["astar_ok"]:>2}/{s["forests"]:<2} {fmt(s["astar_ms_med"], 0):>8} {fmt(s["astar_ms_p95"], 0):>8} | '
                  f'{s["rrtstar_ok"]:>4}/{s["forests"]:<2} {fmt(s["rrtstar_ms_med"], 0):>8} {fmt(s["rrtstar_ms_p95"], 0):>8} | '
                  f'{fmt(s["time_ratio"], 2):>6} {s["time_verdict"]:>6} | {fmt(s["length_ratio"], 3):>6} '
                  f'{s["length_verdict"]:>7} | {fmt(s["memory_ratio"], 2):>6} {s["memory_verdict"]:>6}')
    L_.append('')
    L_.append('  2) MEDIAN ROUTE LENGTH (m), PEAK MEMORY WHILE PLANNING (KiB), TIME TO REPORT THAT THERE IS NO ROUTE (ms; field cut in two)')
    cols = ''.join(f' {NAMES[p] + " len":>16} {NAMES[p] + " KiB":>16} {NAMES[p] + " no route":>20}' for p in planners)
    L_.append(f'  {"field":>6} {"cells":>6}' + cols)
    for s in summary:
        L_.append(f'  {s["field"]:>4} m {s["cells"]:>5.2f}m' + ''.join(
            f' {fmt(s[f"{p}_len_med"], 2):>16} {fmt(s[f"{p}_kib_med"], 0):>16} {fmt(s[f"{p}_walled_ms"], 0):>20}'
            for p in planners))
    L_.append('')
    t = [s for s in summary if s['time_verdict'] == 'RRT*']
    ta = [s for s in summary if s['time_verdict'] == 'A*']
    ln = [s for s in summary if s['length_verdict'] == 'A*']
    lr = [s for s in summary if s['length_verdict'] == 'RRT*']
    L_.append('  3) IN SHORT')
    L_.append(f'     RRT* plans faster in {len(t)} of {len(summary)} field x cell-size combinations, A* in {len(ta)}.')
    L_.append(f'     A* finds the shorter route in {len(ln)}, RRT* in {len(lr)}; the rest show no meaningful difference.')
    fails = [s for s in summary if s['rrtstar_ok'] < s['astar_ok']]
    if fails:
        L_.append('     RRT* found no route within its sample budget where A* did: ' + ', '.join(
            f'{s["field"]} m / {s["cells"]:.2f} m ({s["astar_ok"] - s["rrtstar_ok"]} of {s["forests"]})' for s in fails))
    return '\n'.join(L_) + '\n'


def _plain_log(ax):
    """Log y axis with plain-number tick labels (5, 10, 20, 50 ... not 10^1)."""
    from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter
    ax.set_yscale('log')
    ax.yaxis.set_major_locator(LogLocator(base=10, subs=(1.0, 2.0, 5.0)))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:,.0f}' if v >= 1 else f'{v:g}'))
    ax.yaxis.set_minor_formatter(NullFormatter())


def plot(summary, rows, fields, cells, out):
    from . import plots as P
    plt = P.setup()
    col = {'astar': P.CAT_B, 'rrtstar': P.CAT_A}
    fig, axes = plt.subplots(3, len(fields), figsize=(3.7 * len(fields), 8.8), squeeze=False)
    xs = list(range(len(cells)))

    def sel(f, c, pl):
        return sorted((r for r in rows if r['field'] == f and r['cells'] == c and r['planner'] == pl
                       and not r['walled']), key=lambda r: r['forest'])

    for k, f in enumerate(fields):
        for row, key, ylab in ((0, 'ms', 'planning time per route (ms)'), (2, 'kib', 'peak memory while planning (KiB)')):
            ax = axes[row][k]
            for pl in PLANNERS:
                v = [[r[key] for r in sel(f, c, pl) if np.isfinite(r[key])] for c in cells]
                if not all(v):
                    continue
                ax.fill_between(xs, [np.percentile(x, 25) for x in v], [np.percentile(x, 75) for x in v],
                                color=col[pl], alpha=0.15, lw=0)
                ax.plot(xs, [np.median(x) for x in v], color=col[pl], marker='o', ms=6, lw=2, label=NAMES[pl])
            _plain_log(ax)
            ax.set_xticks(xs)
            ax.set_xticklabels([f'{c:.2f} m' for c in cells])
            if k == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel('grid cell size')
        axes[0][k].set_title(f'{f} x {f} m field')
        ax = axes[1][k]
        for i, c in enumerate(cells):
            s = next(s for s in summary if s['field'] == f and s['cells'] == c)
            rat = [y['length'] / x['length'] for x, y in zip(sel(f, c, 'astar'), sel(f, c, 'rrtstar'))
                   if x['ok'] and y['ok']]
            if rat:
                ax.plot([i] * len(rat), rat, ls='none', marker='o', ms=4, mfc='none', mec=P.CAT_A, alpha=0.7)
                ax.plot([i - 0.18, i + 0.18], [s['length_ratio']] * 2, color=P.INK, lw=2)
        ax.axhline(1.0, color=P.MUTED, lw=1, ls='--')
        ax.set_xticks(xs)
        ax.set_xticklabels([f'{c:.2f} m' for c in cells])
        if k == 0:
            ax.set_ylabel('route length, RRT* / A*\n(circles: forests, bar: mean)')
    axes[0][0].legend(loc='upper left')
    fig.suptitle('Planner scaling: one corner-to-corner plan, same map for both planners\n'
                 'lines: median, bands: interquartile range over the forests', x=0.01, ha='left', fontsize=10.5,
                 fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(out, f'planner_scaling.{ext}'), dpi=160)
    plt.close(fig)


def replot(folder):
    with open(os.path.join(folder, 'runs.csv')) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r['field'], r['cells'], r['forest'] = int(r['field']), float(r['cells']), int(r['forest'])
        r['walled'], r['ok'] = r['walled'] == 'True', r['ok'] == 'True'
        for k in ('ms', 'length', 'kib', 'map_ms'):
            r[k] = float(r[k])
    fields = sorted({r['field'] for r in rows})
    cells = sorted({r['cells'] for r in rows}, reverse=True)
    planners = tuple(p for p in PLANNERS + ('rrtstar2',) if any(r['planner'] == p for r in rows))
    summary = summarize(rows, fields, cells, planners)
    with open(os.path.join(folder, 'report.txt')) as fh:
        meta = fh.read().split('\n')[1].strip()
    txt = report(summary, planners, meta)
    print(txt)
    with open(os.path.join(folder, 'report.txt'), 'w') as fh:
        fh.write(txt)
    plot(summary, rows, fields, cells, folder)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fields', default='20,40,80', help='field sizes in m (square), e.g. 20,40,80')
    ap.add_argument('--cells', default='0.2,0.1,0.05', help='grid cell sizes in m, e.g. 0.2,0.1,0.05')
    ap.add_argument('--forests', type=int, default=10, help='forests per field size (default 10)')
    ap.add_argument('--two-trees', action='store_true', help='also run the two-tree RRT* (B-RRT*) variant')
    ap.add_argument('--no-memory', action='store_true', help='skip the memory measurement (faster)')
    ap.add_argument('--results', default='', help='results folder (default: the workspace results/)')
    ap.add_argument('--no-plots', action='store_true')
    ap.add_argument('--replot', default='', help='only redraw the report and figure of an existing planner_bench folder')
    a = ap.parse_args(argv)
    if a.replot:
        return replot(a.replot)
    fields = [int(float(x)) for x in a.fields.split(',') if x.strip()]
    cells = [float(x) for x in a.cells.split(',') if x.strip()]
    planners = PLANNERS + (('rrtstar2',) if a.two_trees else ())
    res = a.results or default_results_dir()
    out = os.path.join(res, 'planner_bench_' + time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    write_meta(out, kind='planner_bench', fields_m=fields, cells_m=cells, forests=a.forests)
    meta = (f'package {C.MODEL_VERSION}; fields {", ".join(f"{f} m" for f in fields)}; '
            f'cells {", ".join(f"{c:g} m" for c in cells)}; '
            f'{a.forests} forests per field; RRT* {RRT_DEFAULTS["iterations"]} samples per {BUDGET_AREA:.0f} m^2; '
            f'{cpu_name()}, Python {platform.python_version()}, numpy {np.__version__}')
    print('Planner benchmark: ' + meta, flush=True)
    rows = []
    t_start = time.time()
    for f in fields:
        for k in range(a.forests + 1):                       # the last one: the field cut in two (no route)
            walled = k == a.forests
            trees, pines, start, goal = forest(f, 0 if walled else k)
            for c in cells:
                gm = grid(f, c, trees, pines, wall_across(f) if walled else ())
                t0 = time.perf_counter()
                gm.inflated(PLAN_INFLATION)
                map_ms = 1000.0 * (time.perf_counter() - t0)
                for pl in planners:
                    seed = [int(f), int(k), 0x525254]
                    path, ms, kib = plan_once(gm, pl, start, goal, seed, not a.no_memory and not walled)
                    rows.append({'field': f, 'cells': c, 'forest': k, 'walled': walled, 'planner': pl,
                                 'ok': path is not None, 'ms': ms, 'length': length(path), 'kib': kib,
                                 'map_ms': map_ms})
                    print(f'  {f:>3} m  {c:.2f} m  {"field cut in two" if walled else f"forest {k}":<16} '
                          f'{NAMES[pl]:<17} {"route" if path else "no route":<9} {ms:9.1f} ms  '
                          f'{fmt(length(path), 2, " m"):>9}  {fmt(kib, 0, " KiB"):>10}', flush=True)
    summary = summarize(rows, fields, cells, planners)
    txt = report(summary, planners, meta)
    print('\n' + txt)
    print(f'  ({(time.time() - t_start) / 60:.1f} min) -> {out}')
    with open(os.path.join(out, 'report.txt'), 'w') as fh:
        fh.write(txt)
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    if not a.no_plots:
        plot(summary, rows, fields, cells, out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
