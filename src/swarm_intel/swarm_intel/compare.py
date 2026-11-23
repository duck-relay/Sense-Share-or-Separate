#!/usr/bin/env python3
"""Compare two Gazebo runs side by side (paths, energy vs accuracy, time series).

    ros2 run swarm_intel compare                    # latest 'all' run vs latest baseline run
    ros2 run swarm_intel compare --a cam2_swarm     # any config vs the baseline
    ros2 run swarm_intel compare --a sensor --b cam2   # any two configs
    ros2 run swarm_intel compare --a-dir DIR --b-dir DIR

For ALL configurations at once use:  ros2 run swarm_intel study --gazebo

Reads <run>/summary.json + telemetry.csv written by the mission node and
saves <results>/comparison_<A>_vs_<B>_<time>.png plus a text table. If a speed
sweep exists (<results>/batch/batch_results.csv) its runs are drawn as faint
dots behind the two Gazebo points for context.
"""
import argparse
import csv
import json
import os
from datetime import datetime

import numpy as np

from . import config as C
from . import metrics as M
from .paths import default_results_dir, latest_run


def load_run(d):
    """summary.json and telemetry.csv of a run: every drone's track, and the fleet's series over time
    (energy of all drones, mean path deviation of the drones en route)."""
    with open(os.path.join(d, 'summary.json')) as f:
        rep = json.load(f)
    with open(os.path.join(d, 'telemetry.csv')) as f:
        rows = list(csv.DictReader(f))
    per = {}
    for x in rows:
        per.setdefault(int(x['id']), []).append(x)
    drones = {}
    for i, rs in per.items():
        cols = {k: np.array([float(x[k]) for x in rs]) for k in ('t', 'x', 'y', 'z', 'power_w', 'energy_j', 'cte',
                                                                   'loc_err', 'speed')}
        cols['state'] = np.array([x['state'] for x in rs])
        drones[i] = cols
    e_at, cte_at = {}, {}
    for x in rows:
        t = round(float(x['t']), 3)
        e_at[t] = e_at.get(t, 0.0) + float(x['energy_j'])
        if x['state'] in ('CRUISE', 'ESCAPE'):
            cte_at.setdefault(t, []).append(float(x['cte']))
    ts = np.array(sorted(e_at))
    fleet = {'t': ts, 'energy_j': np.array([e_at[t] for t in ts]),
             'cte_t': np.array(sorted(cte_at)), 'cte': np.array([np.mean(cte_at[t]) for t in sorted(cte_at)])}
    return rep, {'drones': drones, 'fleet': fleet}


TABLE = [('fleet_energy_kj', 'Flight energy, all drones (kJ)'), ('fleet_avg_power_w', 'Average power per drone (W)'),
         ('fleet_flight_time_s', 'Flight time, all drones (s)'), ('fleet_mission_time_s', 'Mission time (s)'),
         ('fleet_cte_rms_mean_m', 'Path deviation RMS, mean (m)'), ('fleet_cte_max_m', 'Path deviation max (m)'),
         ('fleet_on_path_pct_mean', 'Within ±0.5 m of route, mean (%)'),
         ('fleet_loc_rms_mean_m', 'Position error RMS, mean (m)'), ('fleet_landing_err_mean_m', 'Landing error, mean (m)'),
         ('fleet_goals_reached', 'Drones at their goal'), ('fleet_replans', 'Route switches (re-plans)'),
         ('fleet_min_sep_margin_m', 'Separation margin (m)'), ('fleet_wake_time_s', 'Time in downwash (s)'),
         ('fleet_contacts', 'Obstacle contacts'), ('fleet_drone_contacts', 'Drone contacts')]


def metric_rows(a, b):
    """(label, value A, value B, key) for the key-number table."""
    va, vb = M.extract(a), M.extract(b)
    return [(lab, va.get(k), vb.get(k), k) for k, lab in TABLE]


def delta(a, b, key=None):
    return M.fmt_change(a, b, key)


def plot_compare(ra, ca, rb, cb, out_png, batch_csv=None):
    from . import plots as P
    plt = P.setup()
    from matplotlib.gridspec import GridSpec
    fig = plt.figure(figsize=(16.5, 10.2))
    gs = GridSpec(2, 3, figure=fig, left=0.05, right=0.985, top=0.885, bottom=0.07, wspace=0.28, hspace=0.36,
                  width_ratios=[1.15, 1, 1])
    src = 'headless runs' if ra.get('source') == 'headless' else 'Gazebo runs'
    la = C.NAV[ra['nav']].label if ra.get('nav') in C.NAV else ra.get('nav_label', 'A')
    lb = C.NAV[rb['nav']].label if rb.get('nav') in C.NAV else rb.get('nav_label', 'B')
    COL = {'a': P.CAT_A, 'b': P.CAT_B}
    LAB = {'a': la, 'b': lb}
    fig.suptitle(f'{la}  vs  {lb} — the whole fleet (5 drones), {src}',
                 x=0.012, y=0.985, ha='left', fontsize=14.5, fontweight='bold', color=P.INK)
    ea, eb = ra['fleet_energy_kj'], rb['fleet_energy_kj']
    xa, xb = ra['fleet_cte_rms_mean_m'], rb['fleet_cte_rms_mean_m']
    fig.text(0.012, 0.935,
             f"{la}: {ea:.2f} kJ, {xa:.3f} m mean RMS path error   ·   {lb}: {eb:.2f} kJ, {xb:.3f} m   ·   "
             f"energy {delta(ea, eb)}, path error {delta(xa, xb)}",
             color=P.INK2, fontsize=10)

    # A: trajectories of every drone (A solid, B dashed, each in its drone's colour)
    from matplotlib.lines import Line2D
    ax = fig.add_subplot(gs[:, 0])
    P.draw_arena(ax)
    refs = ra.get('reference_paths') or rb.get('reference_paths') or {}
    for d in C.DRONES:
        ref = refs.get(str(d.id))
        if ref:
            ref = np.array(ref)
            ax.plot(ref[:, 0], ref[:, 1], color=P.IDEAL, lw=4, alpha=0.4, solid_capstyle='round', zorder=2)
    for (c, m, ls) in ((cb, 'b', (0, (3, 2))), (ca, 'a', '-')):
        for d in C.DRONES:
            t_d = c['drones'].get(d.id)
            if t_d is None:
                continue
            fl = np.isin(t_d['state'], ['TAKEOFF', 'SCAN', 'CRUISE', 'ESCAPE', 'LANDING'])
            ax.plot(t_d['x'][fl], t_d['y'][fl], color=d.color, lw=1.8 if m == 'a' else 1.3, ls=ls, zorder=4)
    h = P.arena_legend_handles() + [Line2D([], [], color=P.INK2, lw=1.8, label=f"A: {ra.get('nav', 'A')} (solid)"),
                                    Line2D([], [], color=P.INK2, lw=1.3, ls=(0, (3, 2)),
                                           label=f"B: {rb.get('nav', 'B')} (dashed)")]
    ax.legend(handles=h, loc='upper left', fontsize=8.5, bbox_to_anchor=(0.0, -0.09), ncol=2)
    ax.set_anchor('N')
    ax.set_title('A · Flown paths (ground truth, each drone in its colour)\nvs the reference routes (green)', pad=10)

    # B: energy vs accuracy
    ax = fig.add_subplot(gs[0, 1])
    if batch_csv and os.path.isfile(batch_csv):
        with open(batch_csv) as f:
            rows = list(csv.DictReader(f))
        for m, rep in (('a', ra), ('b', rb)):
            sel = [r for r in rows if r.get('config', r.get('mode')) == rep.get('nav')]
            if sel and 'fleet_cte_rms' in sel[0]:
                ax.scatter([float(r['fleet_energy_kj']) for r in sel], [float(r['fleet_cte_rms']) for r in sel],
                           s=9, color=COL[m], alpha=0.18, lw=0, zorder=2)
        ax.text(0.99, 0.02, 'faint dots: headless batch runs (all speeds)', transform=ax.transAxes, ha='right',
                va='bottom', fontsize=7.5, color=P.MUTED)
    for (rep, m) in ((ra, 'a'), (rb, 'b')):
        e, x = rep['fleet_energy_kj'], rep['fleet_cte_rms_mean_m']
        ax.scatter([e], [x], s=110, color=COL[m], edgecolor=P.SURFACE, linewidth=2, zorder=5)
        ax.annotate(f"{rep.get('nav', m)}\n{e:.2f} kJ · {x:.3f} m", (e, x), xytext=(10, 0),
                    textcoords='offset points', va='center', fontsize=8.5, color=P.INK)
    ax.set_xlabel('Flight energy of the fleet (kJ)')
    ax.set_ylabel('Path deviation RMS, mean of the drones (m)')
    ax.set_ylim(0, 1.25 * max(ra['fleet_cte_rms_mean_m'], rb['fleet_cte_rms_mean_m']))
    ax.margins(x=0.25)
    ax.set_title('B · Energy vs accuracy (down-left is better)', pad=10)

    # C: deviation over time
    ax = fig.add_subplot(gs[0, 2])
    for (c, m) in ((cb, 'b'), (ca, 'a')):
        ax.plot(c['fleet']['cte_t'], c['fleet']['cte'], color=COL[m], lw=1.8, label=LAB[m])
    ax.axhline(0.5, color=P.INK2, lw=0.8)
    top = max(0.65, 1.15 * max([float(np.max(c['fleet']['cte'])) for c in (ca, cb) if len(c['fleet']['cte'])] + [0.0]))
    ax.set_ylim(0, top)
    ax.annotate('±0.5 m corridor', (ax.get_xlim()[0], 0.5), xytext=(4, 4), textcoords='offset points',
                fontsize=8, color=P.INK2)
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel('Path deviation, mean of the drones en route (m)')
    ax.legend(loc='upper right')
    ax.set_title('C · Path deviation during cruise', pad=10)

    # D: cumulative energy
    ax = fig.add_subplot(gs[1, 1])
    ends = sorted(((c['fleet']['energy_j'][-1], m, c['fleet']) for (c, m) in ((cb, 'b'), (ca, 'a'))),
                  key=lambda e: e[0])
    for n, (ej, m, c) in enumerate(ends):
        ax.plot(c['t'], c['energy_j'] / 1000.0, color=COL[m], lw=2, label=LAB[m])
        ax.annotate(f"{m.upper()}: {ej / 1000.0:.2f} kJ", (c['t'][-1], ej / 1000.0),
                    xytext=(4, -7 if n == 0 else 7), textcoords='offset points', va='center', fontsize=8.5,
                    color=P.INK)
    ax.set_xlabel('Mission time (s)')
    ax.set_ylabel('Energy used by the fleet (kJ)')
    ax.margins(x=0.12)
    ax.legend(loc='upper left')
    ax.set_title('D · Cumulative energy', pad=10)

    # E: metric table
    ax = fig.add_subplot(gs[1, 2])
    ax.axis('off')
    ax.set_title('E · Key numbers (verdict for A)', pad=10)
    rows = metric_rows(ra, rb)
    cell = [[lbl, M.fmt(k, a), M.fmt(k, b), delta(a, b, k) + ' ' + M.MARK[M.verdict(k, a, b)]] for (lbl, a, b, k) in rows]
    tb = ax.table(cellText=cell, colLabels=['metric', 'A', 'B', 'A vs B'], loc='upper center',
                  cellLoc='right', colLoc='right', colWidths=[0.52, 0.16, 0.16, 0.16], edges='horizontal')
    tb.auto_set_font_size(False)
    tb.set_fontsize(8.5)
    tb.scale(1, 1.32)
    for (r, c_), cl in tb.get_celld().items():
        cl.set_edgecolor(P.GRID)
        cl.set_text_props(color=P.INK if r > 0 else P.INK2)
        if r > 0 and c_ == 3:
            vd = M.verdict(rows[r - 1][3], rows[r - 1][1], rows[r - 1][2])
            if vd in ('better', 'worse'):
                cl.set_text_props(color=P.BETTER if vd == 'better' else P.WORSE, fontweight='bold')
        if c_ == 0:
            cl.set_text_props(ha='left')
            cl._loc = 'left'
        if r == 0:
            cl.set_text_props(fontweight='bold')
    from .report import comparison_notes
    notes = comparison_notes(ra, rb)
    P.footer(fig, f"A = {ra.get('nav', '?')}: {ra.get('_dir', '')}   ·   B = {rb.get('nav', '?')}: {rb.get('_dir', '')}"
                  + (('   ·   Note: ' + '; '.join(notes)) if notes else ''))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default='', help='results folder (default: auto)')
    ap.add_argument('--a', default='all', help='configuration A (default all)')
    ap.add_argument('--b', default=C.BASELINE, help=f'configuration B (default the baseline {C.BASELINE})')
    ap.add_argument('--a-dir', default='', help='explicit run folder for A')
    ap.add_argument('--b-dir', default='', help='explicit run folder for B')
    ap.add_argument('--out', default='', help='output png')
    a = ap.parse_args(argv)
    res = a.results or default_results_dir()
    ka, kb = C.nav_config(a.a).key, C.nav_config(a.b).key
    da = a.a_dir or latest_run(res, ka)
    db = a.b_dir or latest_run(res, kb)
    if not da or not db:
        print(f'Need one finished run of each configuration in {res}\n  {ka}: {da}\n  {kb}: {db}\n'
              f'Run:  ros2 launch swarm_intel swarm_mission.launch.py nav:={ka}   (and nav:={kb})')
        return 1
    ra, ca = load_run(da)
    rb, cb = load_run(db)
    ka, kb = ra.get('nav', ka), rb.get('nav', kb)              # what the folders really contain
    ra['_dir'], rb['_dir'] = os.path.basename(da), os.path.basename(db)
    print(f'A ({ka}): {da}\nB ({kb}): {db}\n')
    print(f"{'metric':<44}{ka:>14}{kb:>14}{'A vs B':>9}   verdict for A")
    print('-' * 96)
    for (lbl, x, y, k) in metric_rows(ra, rb):
        print(f'{lbl:<44}{M.fmt(k, x):>14}{M.fmt(k, y):>14}{delta(x, y, k):>9}   {M.verdict(k, x, y) or ""}')
    from .report import comparison_notes
    for n in comparison_notes(ra, rb):
        print(f'Note: {n}')
    out = a.out or os.path.join(res, f'comparison_{ka}_vs_{kb}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
    plot_compare(ra, ca, rb, cb, out, os.path.join(res, 'batch', 'batch_results.csv'))
    print(f'\nSaved graph: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
