#!/usr/bin/env python3
"""Figures and tables for the paper (IEEE two-column format), from finished studies.

  ros2 run swarm_intel paper_figures                 # newest studies of this model version in the results folder
  ros2 run swarm_intel paper_figures --main <dir> --cam80 <dir> --mixed <dir> --wind <dir> <dir> ...

Reads the runs.csv of each study (the statistics are recomputed exactly as the study does:
paired sign-flip permutation tests of the mean, Holm-corrected per metric, 95 % CIs) and writes to
<results>/paper/:

  fig_scenario.pdf          the Gazebo forest and three of the random forests, with every drone's reference route
  fig_energy_accuracy.pdf   energy vs accuracy of every configuration, mean with 95 % CI on both axes
                            (360 deg ring and 80 deg camera)
  fig_effects.pdf           change vs the baseline with 95 % CI, per metric and configuration
  fig_coordination.pdf      no radio vs fixed altitude layers vs swarm radio (same sensing): energy,
                            separation margin and time in downwash
  fig_mixed_fleet.pdf       how many drones need a camera? (k of 5, with / without swarm radio)
  fig_wind.pdf              accuracy and success vs mean wind speed
  fig_validation.pdf        Gazebo vs headless: change vs the baseline of every configuration in both
                            simulators on the same missions (when a validation exists)
  table_configs.tex, table_main.tex, table_main.csv, table_coordination.tex
  numbers.md                every number quoted in the text, with its statistics (first line: the model version
                            and the computer the main study was computed on)
  meta.json                 which result folders were used, and which were left out

Only result folders of the current model version are used (their meta.json, runinfo.py): results of an earlier
version, or made before meta.json existed, are left out and listed, so the paper never mixes versions.

PDFs use embedded TrueType fonts (no Type 3), as IEEE PDF eXpress requires; a PNG preview
of every figure is written next to it.
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

from . import config as C
from . import layouts
from . import metrics as M
from . import plots as P
from .paths import default_results_dir, stamp_of
from .runinfo import is_current, read_meta, write_meta

SKIPPED = set()      # result folders left out: another model version, or made before meta.json existed
USED = {}            # result folders found by the helpers (for paper/meta.json)


def _current(d):
    """Use a result folder only if it was made with the current model version."""
    if is_current(d):
        return True
    SKIPPED.add(d)
    return False

COL1 = 3.5      # IEEE single column width (in)
COL2 = 7.16     # IEEE double column width (in)

# how each configuration is drawn: sensing set -> marker, coordination -> colour
# (the words of the paper: a marker camera that needs 3 or 2 markers per fix, visual-inertial odometry, the radio)
SENSING = {
    'cam3_sensor': ('3-marker camera + odometry', 'o'), 'cam3_sensor_swarm': ('3-marker camera + odometry', 'o'),
    'cam3_sensor_layers': ('3-marker camera + odometry', 'o'),
    'cam2_sensor': ('2-marker camera + odometry', 's'), 'all': ('2-marker camera + odometry', 's'),
    'cam3': ('3-marker camera only', '^'), 'cam3_swarm': ('3-marker camera only', '^'),
    'cam2': ('2-marker camera only', 'v'), 'cam2_swarm': ('2-marker camera only', 'v'),
    'sensor': ('odometry only', 'D'), 'sensor_swarm': ('odometry only', 'D'),
    'swarm': ('no camera, no odometry', 'X'),
}
COMM_COL = {'none': '#eb6834', 'radio': '#4a3aa7', 'layers': '#1baf7a'}     # validated all-pairs trio
COMM_NAME = {'none': 'no coordination', 'radio': 'radio (turn trading)', 'layers': 'altitude layers'}
SHORT = {'cam3_sensor': 'Baseline', 'cam2': 'C2', 'sensor': 'V', 'swarm': 'W', 'cam2_sensor': 'C2+V',
         'cam2_swarm': 'C2+W', 'sensor_swarm': 'V+W', 'all': 'C2+V+W', 'cam3': 'C3', 'cam3_swarm': 'C3+W',
         'cam3_sensor_swarm': 'C3+V+W', 'cam3_sensor_layers': 'C3+V+L'}
ORDER = ['cam3_sensor', 'cam3', 'cam2', 'cam2_sensor', 'sensor', 'cam3_sensor_layers', 'cam3_sensor_swarm',
         'cam3_swarm', 'cam2_swarm', 'all', 'sensor_swarm', 'swarm']


def comm_of(k):
    n = C.NAV[k]
    return 'radio' if n.swarm else ('layers' if n.layers else 'none')


def setup():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix', 'font.size': 8,
        'axes.titlesize': 8, 'axes.labelsize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7,
        'legend.fontsize': 7, 'axes.linewidth': 0.6, 'lines.linewidth': 1.0, 'grid.linewidth': 0.4,
        'axes.edgecolor': '#8a8984', 'axes.labelcolor': P.INK, 'xtick.color': P.INK2, 'ytick.color': P.INK2,
        'axes.grid': True, 'grid.color': P.GRID, 'axes.spines.top': False, 'axes.spines.right': False,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'axes.titlepad': 4,
        'legend.frameon': False, 'pdf.fonttype': 42, 'ps.fonttype': 42,
        'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02, 'figure.facecolor': 'white',
        'axes.facecolor': 'white', 'savefig.facecolor': 'white',
    })
    return plt


def save(fig, out, name):
    import matplotlib.pyplot as plt
    pdf = os.path.join(out, name + '.pdf')
    fig.savefig(pdf)
    fig.savefig(os.path.join(out, name + '.png'), dpi=300)
    plt.close(fig)
    return pdf


# ---------------------------------------------------------------------------
# data
# ---------------------------------------------------------------------------
def load(study_dir):
    """rows (floats), configuration keys in study order, summary with paired statistics."""
    from .study import summarize
    rows, keys = [], []
    with open(os.path.join(study_dir, 'runs.csv')) as f:
        for r in csv.DictReader(f):
            row = {'config': r['config'], 'layout': int(r.get('layout') or 0), 'seed': int(r['seed'])}
            for mk, *_ in M.ALL:
                v = r.get(mk, '')
                row[mk] = float(v) if v not in ('', 'None', None) else None
            rows.append(row)
            if r['config'] not in keys:
                keys.append(r['config'])
    return rows, keys, summarize(rows, keys)


def find(res, pattern, need=()):
    """Newest study folder whose name matches study_<pattern>_<stamp> (and that flew every configuration in
    `need`: the main study, not a smaller control study with the same name pattern)."""
    cands = [d for d in glob.glob(os.path.join(res, 'study_*'))
             if re.fullmatch(f'study_{pattern}_\\d{{8}}_\\d{{6}}', os.path.basename(d))
             and os.path.exists(os.path.join(d, 'runs.csv')) and _current(d)
             and set(need) <= set((read_meta(d) or {}).get('configurations') or need)]
    return sorted(cands, key=stamp_of)[-1] if cands else None


def mean_ci(rows, k, mk):
    v = np.array([r[mk] for r in rows if r['config'] == k and r.get(mk) is not None], float)
    if not len(v):
        return None, None
    from .stats import t975
    return float(v.mean()), (t975(len(v) - 1) * float(v.std(ddof=1)) / math.sqrt(len(v)) if len(v) > 1 else 0.0)


def pct(summ, k, mk):
    """% change vs baseline of the paired mean difference, with its 95 % CI half-width (%)."""
    base = summ[C.BASELINE].get(mk)
    s = summ.get(k, {})
    if base is None or s.get(mk + '_diff') is None or abs(base) < 1e-12:
        return None, None
    return 100.0 * s[mk + '_diff'] / abs(base), (100.0 * s[mk + '_ci'] / abs(base) if s.get(mk + '_ci') is not None else None)


def sig_mark(summ, k, mk):
    from .study import verdict
    vd = verdict(summ, k, mk)
    return {'better': '▲', 'worse': '▼'}.get(vd, '')


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------
def fig_scenario(out, plt, ids=(0, 1, 2, 3)):
    """The forests with every drone's reference route: A* on what it knows on its pad (the known trees),
    planned exactly as the drone plans it before take-off (agent.DroneAgent._reference_plan)."""
    from .mission import known_map
    from .grid_planner import plan_path
    from .agent import PLAN_INFLATION, INFLATION
    fig, axs = plt.subplots(1, len(ids), figsize=(COL2, COL2 / len(ids) + 0.25))
    for ax, n in zip(axs, ids):
        layouts.apply(n)
        P.draw_arena(ax)
        for t in list(ax.texts):
            t.remove()
        gm = known_map()
        for d in C.DRONES:
            p = (plan_path(gm, d.spawn, d.goal, PLAN_INFLATION, planner='astar')
                 or plan_path(gm, d.spawn, d.goal, INFLATION, planner='astar'))
            if p is None:
                continue
            p = np.asarray(p)
            ax.plot(p[:, 0], p[:, 1], color=d.color, lw=1.0, alpha=0.9, zorder=4)
        ax.set_title('Gazebo forest (layout 0)' if n == 0 else f'random forest {n}', fontsize=7.5)
        ax.set_xlabel('')
        ax.set_ylabel('')
        ax.set_xticks([-10, 0, 10])
        ax.set_yticks([-10, 0, 10])
        ax.tick_params(labelsize=6)
        for ln in ax.lines:
            if ln.get_marker() in ('o', '*') and ln.get_markersize() > 6:
                ln.set_markersize(ln.get_markersize() * 0.6)
    layouts.apply(0)
    from matplotlib.lines import Line2D
    from matplotlib.legend_handler import HandlerTuple
    routes = tuple(Line2D([], [], color=d.color, lw=1.0) for d in C.DRONES)
    start_goal = (Line2D([], [], marker='o', ls='none', ms=5.0, mfc=P.SURFACE, mec=P.MUTED, mew=1.4),
                  Line2D([], [], marker='*', ls='none', ms=8.0, mfc=P.MUTED, mec=P.INK, mew=0.6))
    pine = (Line2D([], [], marker='o', ls='none', ms=5.6, mfc=P.PINE, mec=P.PINE),
            Line2D([], [], marker='x', ls='none', ms=2.6, color='white', mew=0.9))
    h = [routes, start_goal, Line2D([], [], marker='o', ls='none', mfc=P.TREE, mec=P.TREE_EDGE, ms=5), pine]
    labels = ['drones 1-5: reference routes (A* before take-off)', 'start, goal (in the drone\'s colour)',
              'known tree with markers', 'unknown pine']
    fig.legend(h, labels, loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.04), fontsize=6.5, columnspacing=1.4,
               handler_map={routes: HandlerTuple(ndivide=None, pad=0.1), start_goal: HandlerTuple(ndivide=None),
                            pine: HandlerTuple(ndivide=1)})
    fig.subplots_adjust(wspace=0.18, bottom=0.17)
    return save(fig, out, 'fig_scenario')


def _log_axis(ax, values, axis='y'):
    """Log scale with plain-number tick labels (0.05, 0.1, 0.2 ... not 10^-1)."""
    from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter
    lo, hi = min(values), max(values)
    (ax.set_yscale if axis == 'y' else ax.set_xscale)('log')
    a = ax.yaxis if axis == 'y' else ax.xaxis
    a.set_major_locator(LogLocator(base=10, subs=(1.0, 2.0, 5.0) if hi / max(lo, 1e-9) < 60 else (1.0,)))
    a.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
    a.set_minor_formatter(NullFormatter())
    lim = (lo / 1.6, hi * 1.6)
    (ax.set_ylim if axis == 'y' else ax.set_xlim)(*lim)


def _scatter_panel(ax, rows, keys, xk, yk, logy=True, title='', extra_rows=None, label_pos=None):
    """One marker per configuration: mean with its 95 % CI (whiskers on both axes; t interval of the mean over
    the missions); colour = coordination, shape = sensing.
    label_pos: {config: (dx, dy, ha)} offsets in points for labels placed by hand (crowded clusters)."""
    allrows = rows + (extra_rows or [])
    bx = mean_ci(rows, C.BASELINE, xk)[0]
    off = []
    pts = []
    for k in [k for k in ORDER if k in keys or (extra_rows and any(r['config'] == k for r in extra_rows))]:
        x, xc = mean_ci(allrows, k, xk)
        y, yc = mean_ci(allrows, k, yk)
        if x is None or y is None:
            continue
        if bx and x > 1.3 * bx:
            off.append((k, x, y))
            continue
        mk_ = SENSING[k][1]
        col = COMM_COL[comm_of(k)]
        isb = k == C.BASELINE
        ylo = min(yc, 0.95 * y) if logy else yc        # on a log axis a whisker cannot reach zero
        ax.errorbar([x], [y], xerr=[[xc], [xc]], yerr=[[ylo], [yc]], fmt='none', ecolor=col, elinewidth=0.6,
                    capsize=0, alpha=0.55, zorder=3)
        ax.plot([x], [y], marker=mk_, ms=5.2 if not isb else 6.5, mfc=col, mec=P.INK if isb else 'white',
                mew=1.1 if isb else 0.5, ls='none', zorder=4)
        pts.append((k, x, y))
    if logy:
        _log_axis(ax, [y for _, _, y in pts])
    ax.set_title(title)
    ax.margins(x=0.12)
    from .study_plots import _place_labels
    label_pos = label_pos or {}
    fixed = []
    ax.figure.canvas.draw()
    rend = ax.figure.canvas.get_renderer()
    for k, x, y in pts:
        if k in label_pos:
            dx, dy, ha = label_pos[k]
            far = math.hypot(dx, dy) > 9
            t = ax.annotate(SHORT[k], (x, y), xytext=(dx, dy), textcoords='offset points', ha=ha, va='center',
                            fontsize=6.6, color=P.INK2, annotation_clip=False, zorder=6,
                            arrowprops=dict(arrowstyle='-', color='#a3a29b', lw=0.6, shrinkA=1, shrinkB=2.5)
                            if far else None)
            fixed.append(t.get_window_extent(rend).expanded(1.04, 1.15))
    _place_labels(ax, [(x, y, SHORT[k], k == C.BASELINE) for k, x, y in pts if k not in label_pos], fontsize=6.6,
                  scale=0.5, mark_pad=7, skip_own=True, avoid=fixed)
    import matplotlib.patheffects as pe
    for t in ax.texts:                                  # a whisker passing a label goes behind it
        t.set_path_effects([pe.withStroke(linewidth=2.2, foreground=P.SURFACE)])
        t.set_zorder(max(t.get_zorder(), 5))
    return pts, off


# hand-placed labels for crowded clusters: {panel '(b)': {config: (dx, dy, ha)}} in points, with a leader line
# when far from the marker. In the path-accuracy panel C3, C3+W, C2+W, C3+V+W and C2+V+W sit within 0.4 kJ and
# 0.03 m of each other, so their labels are fanned out by hand
LABEL_POS = {'(b)': {'cam3_sensor_swarm': (-22, 13, 'right'), 'all': (-24, -1, 'right'),
                     'cam3': (6, -16, 'left'), 'cam2_swarm': (22, 1, 'left'), 'cam3_swarm': (0, 9, 'center')}}


def fig_energy_accuracy(out, plt, main, cam80=None, name='fig_energy_accuracy', panel_h=2.1):
    rows = main[0]
    panels = [(main, 'fleet_loc_rms_mean_m', 'Position error RMS (m)', '(a) Ring of four cameras: localization'),
              (main, 'fleet_cte_rms_mean_m', 'Path deviation RMS (m)', '(b) Ring of four cameras: path accuracy')]
    if cam80:
        # configurations without a camera do not depend on the camera: reuse their identical missions
        ident = [r for r in rows if r['config'] in ('sensor', 'sensor_swarm', 'swarm')
                 and (r['layout'], r['seed']) in {(q['layout'], q['seed']) for q in cam80[0]}]
        panels += [(cam80, 'fleet_loc_rms_mean_m', 'Position error RMS (m)', '(c) Single 80° camera: localization', ident),
                   (cam80, 'fleet_cte_rms_mean_m', 'Path deviation RMS (m)', '(d) Single 80° camera: path accuracy', ident)]
    nrow = 2 if cam80 else 1
    H = panel_h * nrow + 0.55
    fig, axs = plt.subplots(nrow, 2, figsize=(COL2, H), squeeze=False)
    offs, drawn = [], set()
    for i, (ax, pan) in enumerate(zip(axs.ravel(), panels)):
        data, yk, ylab, title = pan[:4]
        # position errors span two orders of magnitude (log axis); path deviations less than a factor of three,
        # so a linear axis spreads their crowded cluster and keeps every label readable
        pts, off = _scatter_panel(ax, data[0], data[1], 'fleet_energy_kj', yk, logy=yk != 'fleet_cte_rms_mean_m',
                                  title=title, extra_rows=pan[4] if len(pan) > 4 else None,
                                  label_pos=LABEL_POS.get(title[:3]))
        offs += [(title[:3], k, x, y) for k, x, y in off]
        drawn |= {k for k, _, _ in pts}
        if i >= 2 * (nrow - 1):
            ax.set_xlabel('Energy of the team (kJ, 5 drones)')
        ax.set_ylabel(ylab)
    from matplotlib.lines import Line2D
    h1 = [Line2D([], [], marker='o', ls='none', mfc=COMM_COL[c], mec='white', ms=5, label=COMM_NAME[c])
          for c in ('none', 'layers', 'radio')]
    h1.append(Line2D([], [], marker='o', ls='none', mfc='white', mec=P.INK, mew=1.1, ms=6, label='baseline'))
    h1.append(Line2D([], [], color='#9a9993', lw=0.8, label='95 % CI'))
    seen, h2 = [], []
    for k in [k for k in ORDER if k in drawn]:          # a marker that is drawn nowhere gets no legend entry
        lab, m_ = SENSING[k]
        if lab not in seen:
            seen.append(lab)
            h2.append(Line2D([], [], marker=m_, ls='none', mfc='#9a9993', mec='white', ms=5, label=lab))
    fig.legend(handles=h1 + h2, loc='upper center', ncol=5, bbox_to_anchor=(0.5, 1.0), fontsize=7,
               columnspacing=1.2, handletextpad=0.3)
    fig.subplots_adjust(wspace=0.22, hspace=0.42, top=1.0 - 0.66 / H, bottom=0.45 / H)
    # configurations off the axes are listed (not drawn); the paper names them in the caption
    if offs:
        with open(os.path.join(out, f'{name}_offscale.txt'), 'w') as f:
            f.write('Off scale (not drawn): ' + '; '.join(f'{p} {SHORT[k]} {x:.1f} kJ, {y:.2f} m'
                                                         for p, k, x, y in offs) + '\n')
    return save(fig, out, name)


EFFECT_METRICS = [('fleet_energy_kj', 'Energy'), ('fleet_flight_time_s', 'Flight time'),
                  ('fleet_mission_time_s', 'Mission time'), ('fleet_loc_rms_mean_m', 'Position error'),
                  ('fleet_track_rms_mean_m', 'Tracking error'), ('fleet_cte_rms_mean_m', 'Path deviation')]
SAFETY_METRICS = [('fleet_min_sep_margin_m', 'Separation margin (fleet)'),
                  ('fleet_wake_time_s', 'Time in downwash (fleet)'),
                  ('fleet_drone_contacts', 'Drone-drone contacts (fleet)'),
                  ('fleet_contacts', 'Obstacle contacts (fleet)')]


def fig_effects(out, plt, main, name='fig_effects', title=''):
    keys, summ = main[1], main[2]
    from .study import verdict
    others = [k for k in ORDER if k in keys and k != C.BASELINE]
    n = len(EFFECT_METRICS)
    fig, axs = plt.subplots(1, n, figsize=(COL2, 0.16 * len(others) + 0.75), sharey=True)
    ys = np.arange(len(others))[::-1]
    for j, (ax, (mk, lab)) in enumerate(zip(axs, EFFECT_METRICS)):
        vals = [pct(summ, k, mk) for k in others]
        lim = 60.0
        for y, k, (v, ci) in zip(ys, others, vals):
            if v is None:
                continue
            vd = verdict(summ, k, mk)
            col = {'better': P.BETTER, 'worse': P.WORSE}.get(vd, '#9a9993')
            vv = max(-lim, min(lim, v))
            lo, hi = (v - ci, v + ci) if ci is not None else (v, v)
            ax.plot([max(-lim, lo), min(lim, hi)], [y, y], color=col, lw=0.9, solid_capstyle='butt', zorder=2)
            ax.plot([vv], [y], marker='o' if abs(v) <= lim else ('>' if v > 0 else '<'), ms=3.4, color=col, zorder=3)
            if abs(v) > lim:
                ax.text(vv + (-3 if v > 0 else 3), y + 0.32, (f'×{1 + v / 100:.0f}' if v >= 100 else f'{v:+.0f}%'.replace('-', '−')),
                        ha='right' if v > 0 else 'left', va='bottom', fontsize=5.2, color=col)
        ax.axvline(0, color=P.INK, lw=0.6, zorder=1)
        ax.set_xlim(-lim * 1.05, lim * 1.05)
        ax.set_xticks([-50, 0, 50])
        ax.set_title(lab, fontsize=6.8, loc='center')
        ax.grid(axis='y', visible=False)
        ax.tick_params(axis='x', labelsize=6)
        if j == n // 2:
            ax.set_xlabel('change vs baseline (%), mean of paired differences with 95 % CI')
    axs[0].set_yticks(ys)
    axs[0].set_yticklabels([C.label_of(k).replace('All-in-one: ', '') for k in others], fontsize=6.2)
    axs[0].set_ylim(-0.7, len(others) - 0.3)
    if title:
        fig.suptitle(title, x=0.01, ha='left', fontsize=8, fontweight='bold')
    fig.subplots_adjust(wspace=0.12)
    return save(fig, out, name)


def fig_coordination(out, plt, main):
    """The same sensors (C3+V), three ways to keep apart: none, altitude layers, radio (turn trading)."""
    rows, keys, summ = main
    ks = [k for k in ('cam3_sensor', 'cam3_sensor_layers', 'cam3_sensor_swarm') if k in keys]
    if len(ks) < 2:
        return None
    mets = [('fleet_energy_kj', 'Energy, all\n5 drones (kJ)'), ('fleet_flight_time_s', 'Flight time,\nall 5 drones (s)'),
            ('fleet_mission_time_s', 'Mission time\n(last landing, s)'),
            ('fleet_min_sep_margin_m', 'Separation margin,\nclosest pair (m)'),
            ('fleet_wake_time_s', 'Time in downwash,\nsum of 5 drones (s)')]
    fig, axs = plt.subplots(1, len(mets), figsize=(COL2, 1.45))
    names = {'cam3_sensor': 'none', 'cam3_sensor_layers': 'layers', 'cam3_sensor_swarm': 'radio'}   # the caption names them
    fmts = {'fleet_energy_kj': '.1f', 'fleet_flight_time_s': '.0f', 'fleet_mission_time_s': '.1f',
            'fleet_min_sep_margin_m': '+.2f', 'fleet_wake_time_s': '.2f'}
    for ax, (mk, lab) in zip(axs, mets):
        stats = [mean_ci(rows, k, mk) for k in ks]
        lo = min(m - ci for m, ci in stats)
        hi = max(m + ci for m, ci in stats)
        close = 0.12 * max(hi - lo, 1e-9)                 # labels of points this close in value would collide
        for i, k in enumerate(ks):
            m, ci = stats[i]
            col = COMM_COL[comm_of(k)]
            ax.errorbar([i], [m], yerr=[ci], fmt='o', ms=4.2, color=col, mec='white', mew=0.5, ecolor=col,
                        elinewidth=0.8, capsize=2.0, zorder=3)
            mark = sig_mark(summ, k, mk) if k != C.BASELINE else ''
            txt = format(m, fmts.get(mk, '.3g')).replace('-', '−')
            right_free = i == len(ks) - 1 or abs(stats[i + 1][0] - m) > close
            left_free = i > 0 and abs(stats[i - 1][0] - m) > close
            label = f'{txt}{(" " + mark) if mark else ""}'
            if right_free or not left_free:
                ax.annotate(label, (i, m), xytext=(5, 0), textcoords='offset points', ha='left', va='center',
                            fontsize=6.4, color=P.INK)
            else:                                         # neighbour too close on the right: above the error bar
                ax.annotate(label, (i, m + ci), xytext=(0, 2), textcoords='offset points', ha='center',
                            va='bottom', fontsize=6.4, color=P.INK)
        ax.set_xticks(range(len(ks)))
        ax.set_xticklabels([names[k] for k in ks], fontsize=6.4)
        ax.set_xlim(-0.5, len(ks) - 0.1)
        ax.set_title(lab, fontsize=7, loc='center')
        ax.grid(axis='x', visible=False)
        ax.margins(y=0.25)
    fig.subplots_adjust(wspace=0.45)
    return save(fig, out, 'fig_coordination')


def _mixed_panel(ax, mixed, mk, logy):
    """One metric of the mixed fleets vs the number of camera drones, without and with the radio."""
    rows, keys = mixed[0], mixed[1]
    allv = []
    for sw in (False, True):
        xs, ys, cs = [], [], []
        for k in range(6):
            key = f'k{k}' + ('_swarm' if sw else '')
            if key == 'k5' and key not in keys:
                key = C.BASELINE                       # all five with camera, no radio = the baseline
            if key not in keys:
                continue
            m, ci = mean_ci(rows, key, mk)
            if m is None:
                continue
            xs.append(k)
            ys.append(m)
            cs.append(ci)
        if not xs:
            continue
        allv += ys
        ax.errorbar(xs, ys, yerr=[[min(c, 0.95 * y) if logy else c for c, y in zip(cs, ys)], cs],
                    color=COMM_COL['radio' if sw else 'none'], marker='o', ms=3.5, lw=1.0, elinewidth=0.6,
                    capsize=1.5, label='radio' if sw else 'no radio')
    if logy and allv:
        _log_axis(ax, allv)
    ax.set_xlabel('drones with a camera (of 5)')
    ax.set_xticks(range(6))


def fig_mixed(out, plt, mixed):
    fig, axs = plt.subplots(1, 3, figsize=(COL2, 1.75))
    mets = [('fleet_loc_rms_nocam_mean_m', 'Position error of drones\nwithout camera (m)', True),
            ('fleet_loc_rms_mean_m', 'Position error,\nmean of all 5 drones (m)', True),
            ('fleet_energy_kj', 'Energy of all\n5 drones (kJ)', False)]
    for ax, (mk, lab, logy) in zip(axs, mets):
        _mixed_panel(ax, mixed, mk, logy)
        ax.set_title(lab, fontsize=6.8, loc='center')
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 1.17), fontsize=6.5)
    fig.subplots_adjust(wspace=0.42)
    return save(fig, out, 'fig_mixed_fleet')


WIND_KEYS = ['cam3_sensor', 'cam3_sensor_swarm', 'all', 'sensor', 'sensor_swarm', 'swarm']
WIND_STYLE = {'cam3_sensor': ('#0b0b0b', 'o', '-'), 'cam3_sensor_swarm': (COMM_COL['radio'], 'o', '-'),
              'all': (COMM_COL['radio'], 's', '--'), 'sensor': (COMM_COL['none'], 'D', '-'),
              'sensor_swarm': (COMM_COL['radio'], 'D', ':'), 'swarm': ('#9a9993', 'X', '-'),
              'cam3_sensor_layers': (COMM_COL['layers'], 'o', '-.'), 'cam2_sensor': (COMM_COL['none'], 's', '--')}


def _wind_panel(ax, wind_sets, keys, mk, logy):
    """One metric vs the mean wind speed, one line per configuration."""
    allv = []
    for k in keys:
        xs, ys = [], []
        for w, data in wind_sets:
            if k not in data[1]:
                continue
            m, _ = mean_ci(data[0], k, mk)
            if m is not None:
                xs.append(w)
                ys.append(m)
        allv += ys
        col, mk_, ls = WIND_STYLE.get(k, ('#9a9993', 'o', '-'))
        ax.plot(xs, ys, color=col, marker=mk_, ms=3.2, ls=ls, lw=0.9, label=SHORT[k] if k != C.BASELINE
                else 'Baseline (C3+V)')
    if logy and allv:
        _log_axis(ax, allv)
    ax.set_xlabel('mean wind speed (m/s)')
    ax.set_xticks([0, 0.5, 1.0, 1.5])


def fig_wind(out, plt, wind_sets):
    """wind_sets: [(mean wind m/s, (rows, keys, summ)), ...]"""
    wind_sets = sorted(wind_sets, key=lambda t: t[0])
    keys = [k for k in WIND_KEYS if any(k in ws[1][1] for ws in wind_sets)]
    mets = [('fleet_loc_rms_mean_m', 'Position error RMS (m)', True), ('fleet_cte_rms_mean_m', 'Path deviation RMS (m)', True),
            ('fleet_goals_reached', 'Drones that reached\ntheir goal (of 5)', False)]
    fig, axs = plt.subplots(1, 3, figsize=(COL2, 1.8))
    for ax, (mk, lab, logy) in zip(axs, mets):
        _wind_panel(ax, wind_sets, keys, mk, logy)
        ax.set_title(lab, fontsize=6.8, loc='center')
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=len(l), bbox_to_anchor=(0.5, 1.12), fontsize=6.3, columnspacing=1.2)
    fig.subplots_adjust(wspace=0.35)
    return save(fig, out, 'fig_wind')


def fig_mixed_wind(out, plt, mixed, wind_sets):
    """The paper's combined figure: mixed fleets (a, b) and wind (c, d) side by side."""
    wind_sets = sorted(wind_sets, key=lambda t: t[0])
    keys = [k for k in WIND_KEYS if any(k in ws[1][1] for ws in wind_sets)]
    fig, axs = plt.subplots(1, 4, figsize=(COL2, 1.95))
    _mixed_panel(axs[0], mixed, 'fleet_loc_rms_nocam_mean_m', True)
    _mixed_panel(axs[1], mixed, 'fleet_energy_kj', False)
    _wind_panel(axs[2], wind_sets, keys, 'fleet_loc_rms_mean_m', True)
    _wind_panel(axs[3], wind_sets, keys, 'fleet_goals_reached', False)
    titles = ['(a) Mixed teams: position error\nof drones without camera (m)', '(b) Mixed teams: energy\nof all 5 drones (kJ)',
              '(c) Wind: position error\nRMS (m)', '(d) Wind: drones that\nreached their goal (of 5)']
    for ax, lab in zip(axs, titles):
        ax.set_title(lab, fontsize=6.6, loc='center')
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=1, bbox_to_anchor=(0.285, 1.13), fontsize=6.2)
    h, l = axs[2].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=3, bbox_to_anchor=(0.745, 1.13), fontsize=6.2, columnspacing=1.0)
    fig.subplots_adjust(wspace=0.42, top=0.80)
    return save(fig, out, 'fig_mixed_wind')


def validation_data(vdir):
    """Gazebo-vs-headless pairs of a validation folder, the configurations lost in both simulators,
    and the per-configuration effects vs the baseline in both."""
    from . import validate as V
    pairs = V.read_pairs(os.path.join(vdir, 'pairs.csv'))
    if not pairs:
        return None
    lost = V.lost_configs(pairs)
    return pairs, lost, V.effects(pairs)


def fig_validation(out, plt, vdir):
    """Does Gazebo show the same effects as the headless twin? Change vs the baseline of every
    configuration (paired by seed) in Gazebo against the headless twin on the same missions."""
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FuncFormatter, FixedLocator
    from . import validate as V
    data = validation_data(vdir)
    if not data:
        return None
    _, lost, eff = data
    keys = [k for k in ORDER if k in eff and k not in lost]
    fig, axs = plt.subplots(1, 2, figsize=(COL1, 1.95))
    fig.subplots_adjust(left=0.14, right=0.985, bottom=0.2, top=0.8, wspace=0.42)
    panels = [(axs[0], 'Energy, time', [('fleet_energy_kj', 'o'), ('fleet_flight_time_s', 's')], False,
               ('energy', 'flight time')),
              (axs[1], 'Error', [('fleet_track_rms_mean_m', 'o'), ('fleet_loc_rms_mean_m', 's')], True, ('tracking', 'position'))]
    for ax, title, mets, log, names in panels:
        xs, ys = [], []
        for mk, mark in mets:
            for k in keys:
                if mk not in eff[k]:
                    continue
                g, h, _ = eff[k][mk]
                x, y = (1 + h / 100, 1 + g / 100) if log else (h, g)
                xs.append(x)
                ys.append(y)
                col = COMM_COL[comm_of(k)]
                filled = mark == 'o'
                ax.plot([x], [y], mark, ms=3.4 if filled else 3.2, color=col, mfc=col if filled else 'white',
                        mec=col, mew=0.8, zorder=3)
        if not xs:
            continue
        lo, hi = min(xs + ys), max(xs + ys)
        if log:
            lo, hi = lo / 1.25, hi * 1.25
            ax.set_xscale('log')
            ax.set_yscale('log')
            ticks = [t for t in (0.5, 1, 2, 5, 10, 20) if lo <= t <= hi]
            fmt = FuncFormatter(lambda v, _: '0' if abs(v - 1) < 1e-9 else f'{100 * (v - 1):+.0f}'.replace('-', '−'))
            for axis in (ax.xaxis, ax.yaxis):
                axis.set_major_locator(FixedLocator(ticks))
                axis.set_major_formatter(fmt)
                axis.set_minor_formatter(FuncFormatter(lambda v, _: ''))
        else:
            pad = 0.08 * (hi - lo)
            lo, hi = lo - pad, hi + pad
            ax.axhline(0, color='#c9c8c2', lw=0.5, zorder=1)
            ax.axvline(0, color='#c9c8c2', lw=0.5, zorder=1)
        ax.plot([lo, hi], [lo, hi], color='#9a9993', lw=0.6, ls='--', zorder=2)
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.set_aspect('equal', adjustable='box')
        ag = V.effect_agreement(eff, keys, metrics=[(m, m) for m, _ in mets])
        g_all = [eff[k][m][0] for m, _ in mets for k in keys if m in eff[k]]
        h_all = [eff[k][m][1] for m, _ in mets for k in keys if m in eff[k]]
        rho = V._spearman(g_all, h_all)
        same = sum(a['same_sign'] for a in ag.values())
        big = sum(a['n_big'] for a in ag.values())
        ax.text(0.04, 0.97, f'$\\rho$ = {rho:.2f}\nsame sign {same}/{big}', transform=ax.transAxes, va='top',
                fontsize=6.4)
        ax.set_title(title, fontsize=7.5)
        ax.legend(handles=[Line2D([], [], marker='o', ls='none', color='#55544f', ms=3.2, label=names[0]),
                           Line2D([], [], marker='s', ls='none', color='#55544f', mfc='white', ms=3.0, label=names[1])],
                  loc='lower right', fontsize=6.3, handletextpad=0.2, borderaxespad=0.2, labelspacing=0.25)
        ax.set_xlabel('headless change (%)', fontsize=7)
        ax.tick_params(labelsize=6.4)
    axs[0].set_ylabel('Gazebo change (%)', fontsize=7)
    handles = [Line2D([], [], marker='o', ls='none', color=COMM_COL[c], ms=3.4, label=COMM_NAME[c])
               for c in ('none', 'layers', 'radio') if any(comm_of(k) == c for k in keys)]
    fig.legend(handles=handles, loc='upper center', ncol=3, fontsize=6.4, bbox_to_anchor=(0.5, 1.02),
               handletextpad=0.3, columnspacing=0.9)
    return save(fig, out, 'fig_validation')


# ---------------------------------------------------------------------------
# tables + numbers
# ---------------------------------------------------------------------------
def _tex(s):
    return s.replace('%', r'\%').replace('_', r'\_').replace('±', r'$\pm$').replace('×', r'$\times$')


def table_configs(out):
    L = [r'\begin{table}[t]', r'\caption{The configurations, flown by all five drones alike. C$n$: marker camera needing $n$ '
         r'markers per fix; V: visual-inertial odometry; W: radio with turn trading; L: altitude layers. $P_\mathrm{nav}$: '
         r'power of these components, on top of the 6.5\,W every drone draws.}',
         r'\label{tab:configs}', r'\centering\footnotesize', r'\setlength{\tabcolsep}{4pt}',
         r'\begin{tabular}{@{}lcccr@{}}', r'\toprule',
         r'Configuration & Camera & Odometry & Keeping apart & $P_\mathrm{nav}$ (W)\\', r'\midrule']
    from .energy import PowerModel
    for k in ORDER:
        n = C.NAV[k]
        pm = PowerModel(n)
        cam = f'{n.cam_points} markers' if n.cam_points else '--'
        coord = 'radio' if n.swarm else ('altitude layers' if n.layers else '--')
        name = 'C3+V (baseline)' if k == C.BASELINE else SHORT[k]
        L.append(f'{_tex(name)} & {cam} & {"yes" if n.sensor else "--"} & {coord} & {pm.p_nav:.1f}\\\\')
    L += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
    path = os.path.join(out, 'table_configs.tex')
    open(path, 'w').write('\n'.join(L) + '\n')
    return path


TABLE_METRICS = [('fleet_energy_kj', r'$E$ (kJ)', '{:.2f}'), ('dE', r'$\Delta E$ (\%)', None),
                 ('fleet_flight_time_s', r'$t$ (s)', '{:.1f}'), ('fleet_mission_time_s', r'$T$ (s)', '{:.1f}'),
                 ('fleet_loc_rms_mean_m', r'$e_\mathrm{pos}$ (m)', '{:.3f}'), ('fleet_track_rms_mean_m', r'$e_\mathrm{trk}$ (m)', '{:.3f}'),
                 ('fleet_cte_rms_mean_m', r'$e_\mathrm{path}$ (m)', '{:.3f}'),
                 ('fleet_landing_err_mean_m', r'$e_\mathrm{land}$ (m)', '{:.3f}'), ('fleet_goals_reached', 'Goals', '{:.2f}'),
                 ('fleet_min_sep_margin_m', r'$m_\mathrm{sep}$ (m)', '{:+.2f}'),
                 ('fleet_wake_time_s', r'$t_\mathrm{dw}$ (s)', '{:.2f}')]


def table_main(out, main, cam80=None):
    blocks = [('Ring of four cameras', main)] + ([('Single forward 80° camera', cam80)] if cam80 else [])
    L = [r'\begin{table*}[t]', r'\caption{Means per configuration. $\blacktriangle$/$\blacktriangledown$: better/worse '
         r'than the baseline (Section~\ref{sec:design}). $E$: team energy; $\Delta E$: its paired change with 95\,\% CI; $t$: '
         r'flight time (sum of the drones); $T$: mission time; $e_\mathrm{pos}$, $e_\mathrm{trk}$, $e_\mathrm{path}$, '
         r'$e_\mathrm{land}$: position, tracking, path, and landing error (mean of the drones); Goals: drones at their goal '
         r'(of 5); $m_\mathrm{sep}$: separation margin; $t_\mathrm{dw}$: time in downwash (sum). V, V+W, and W are not '
         r'repeated for the single camera.}', r'\label{tab:main}',
         r'\centering\footnotesize\setlength{\tabcolsep}{3.5pt}',
         r'\begin{tabular}{@{}l' + 'r' * len(TABLE_METRICS) + '@{}}', r'\toprule',
         'Configuration & ' + ' & '.join(t for _, t, _ in TABLE_METRICS) + r'\\']
    TM = [m for m in TABLE_METRICS if m[0] != 'dE']
    rows_csv = [['block', 'config'] + [mk for mk, _, _ in TM] + [f'{mk} change %' for mk, _, _ in TM]
                + ['fleet_energy_kj change CI %'] + [f'{mk} verdict' for mk, _, _ in TM]]
    from .study import verdict
    for title, (rows, keys, summ) in blocks:
        n_lay = len({r['layout'] for r in rows})
        n_seed = len({r['seed'] for r in rows})
        L += [r'\midrule', f'\\multicolumn{{{len(TABLE_METRICS) + 1}}}{{@{{}}l}}{{\\textit{{{title}: '
                           f'{n_lay} forests $\\times$ {n_seed} seeds per configuration}}}}\\\\']
        for k in [k for k in ORDER if k in keys]:
            cells, ch, vds = [], [], []
            for mk, _, fmt in TABLE_METRICS:
                if mk == 'dE':                                   # paired energy change with its 95 % CI
                    pc, pci = pct(summ, k, 'fleet_energy_kj') if k != C.BASELINE else (None, None)
                    if pc is None:
                        cells.append('--')
                        continue
                    t = f'{pc:+.1f}'
                    t = '$-$' + t[1:] if t.startswith('-') else t                # a real minus sign
                    cells.append(f'{t}\\,$\\pm$\\,{pci:.1f}')
                    continue
                v = summ[k].get(mk)
                vd = verdict(summ, k, mk)
                mark = {'better': r'$^\blacktriangle$', 'worse': r'$^\blacktriangledown$'}.get(vd, '')
                txt = fmt.format(v) if v is not None else '--'
                txt = '$-$' + txt[1:] if txt.startswith('-') and txt != '--' else txt      # a real minus sign
                cells.append(txt + mark)
                ch.append(pct(summ, k, mk)[0] if k != C.BASELINE else 0.0)
                vds.append(vd or '')
            name = 'C3+V (baseline)' if k == C.BASELINE else SHORT[k]      # the codes of Table I
            L.append(f'{_tex(name)} & ' + ' & '.join(cells) + r'\\')
            e_ci = pct(summ, k, 'fleet_energy_kj')[1] if k != C.BASELINE else 0.0
            rows_csv.append([title, k] + [summ[k].get(mk) for mk, _, _ in TM] + ch + [e_ci] + vds)
    L += [r'\bottomrule', r'\end{tabular}', r'\end{table*}']
    path = os.path.join(out, 'table_main.tex')
    open(path, 'w').write('\n'.join(L) + '\n')
    with open(os.path.join(out, 'table_main.csv'), 'w', newline='') as f:
        csv.writer(f).writerows(rows_csv)
    return path


def numbers(out, sets, vdir=None, robust=None, planner=None, giveway=None, provenance=None):
    """Every number the text needs: key comparisons with mean difference, 95 % CI and p."""
    L = ['# Numbers for the paper', '',
         'Paired differences vs the baseline (3-point camera + VIO): mean [95 % CI], Holm-corrected '
         'sign-flip permutation p (Holm across configurations, per metric), effect size d_z. '
         'Verdict = significant AND > 2 %.', '']
    if provenance:
        pv = provenance
        L += [f"Model {pv.get('model_version')}, package {pv.get('package_version')}; computed on {pv.get('cpu')} "
              f"(Python {pv.get('python')}, numpy {pv.get('numpy')}, OpenBLAS kernel {pv.get('openblas_coretype')}). "
              'Sources: paper/meta.json.', '']
    from .study import verdict, n_tests
    for title, data in sets:
        if not data:
            continue
        rows, keys, summ = data
        L += [f'## {title}', '',
              f'{len(rows)} missions; {len({r["layout"] for r in rows})} forests x {len({r["seed"] for r in rows})} '
              f'seeds x {len(keys)} configurations; {n_tests(summ)} paired tests (Holm-corrected per metric across the configurations).', '',
              '| configuration | metric | baseline | config | change | 95 % CI of change | p (Holm) | d_z | verdict |',
              '|---|---|---|---|---|---|---|---|---|']
        for k in [k for k in ORDER + list(C.FLEETS) if k in keys and k != C.BASELINE]:
            for mk, lab in EFFECT_METRICS + SAFETY_METRICS + [
                    ('fleet_landing_err_mean_m', 'Landing error'), ('fleet_goals_reached', 'Goals'),
                    ('fleet_loc_rms_nocam_mean_m', 'Pos. error, no-camera drones')]:
                s, b = summ[k], summ[C.BASELINE]
                if s.get(mk) is None or b.get(mk) is None or s.get(mk + '_diff') is None:
                    continue
                pa = s.get(mk + '_padj')
                dz = s.get(mk + '_dz')
                unit = M.BY_KEY[mk][2]
                if mk in M.ABS_TOL or abs(b[mk]) < 1e-9:      # crosses zero / zero baseline: absolute change
                    d, ci = s[mk + '_diff'], s.get(mk + '_ci')
                    ch_txt = f'{d:+.2f} {unit}'
                    ci_txt = f'±{ci:.2f} {unit}' if ci is not None else '-'
                else:
                    pc, pci = pct(summ, k, mk)
                    ch_txt = f'{pc:+.1f} %' if pc is not None else '-'
                    ci_txt = f'±{pci:.1f} %' if pci is not None else '-'
                # means over missions: counts keep their decimals (3.9 goals, 0.17 contacts), not '4' and '0'
                L.append(f'| {k} | {lab} | {M.fmt_mean(mk, b[mk])} | {M.fmt_mean(mk, s[mk])} | {ch_txt} | {ci_txt} | '
                         f'{("<0.001" if pa is not None and pa < 0.001 else (f"{pa:.3f}" if pa is not None else "-"))} | '
                         f'{(f"{dz:+.2f}" if dz is not None and math.isfinite(dz) else "-")} | {verdict(summ, k, mk) or "-"} |')
        L.append('')
    # energy split of the baseline: propulsion vs electronics
    main = sets[0][1]
    if main:
        rows = main[0]
        for k in ('cam3_sensor', 'cam3_sensor_swarm'):
            p = [r['fleet_avg_power_w'] for r in rows if r['config'] == k and r.get('fleet_avg_power_w')]
            nav = [r['fleet_nav_power_w'] for r in rows if r['config'] == k and r.get('fleet_nav_power_w') is not None]
            if p:
                from .energy import PowerModel
                base_el = C.ENERGY.p_avionics + C.ENERGY.p_comm_base
                L.append(f'- {k}: average power {np.mean(p):.1f} W, of which navigation electronics '
                         f'{np.mean(nav):.1f} W and other electronics {base_el:.1f} W '
                         f'({100 * (np.mean(nav) + base_el) / np.mean(p):.1f} % of the total); hover rotor power '
                         f'{PowerModel(C.NAV[k]).hover_power() - PowerModel(C.NAV[k]).p_const:.0f} W.')
    if robust:
        L += robustness_numbers(robust)
    if planner:
        L += planner_numbers(planner)
    if giveway:
        L += giveway_numbers(giveway)
    if vdir:
        L += validation_numbers(vdir)
    path = os.path.join(out, 'numbers.md')
    open(path, 'w').write('\n'.join(L) + '\n')
    return path


ROBUST_KEYS = ('cam3_sensor', 'cam3_sensor_layers', 'cam3_sensor_swarm')


def robustness_numbers(sets):
    """numbers.md section: do the coordination findings hold with a weaker downwash or wider layers?
    sets: [(label, (rows, keys, summ))], the first one the default model on the same forests and seeds."""
    from .study import verdict
    L = ['', '## Robustness of the coordination findings (downwash strength, layer spacing)', '',
         'Same forests and seeds in every row. Changes are paired vs the baseline of the same setting; '
         '▲/▼ = significant (Holm) and > 2 %.', '',
         '| setting | configuration | fleet energy | fleet flight time | sep. margin (m) | downwash (s) | '
         'strongest downdraft (m/s) | drone contacts |', '|---|---|---|---|---|---|---|---|']
    for label, (rows, keys, summ) in sets:
        n = len({(r['layout'], r['seed']) for r in rows})
        for k in [k for k in ROBUST_KEYS if k in keys]:
            cells = []
            for mk in ('fleet_energy_kj', 'fleet_flight_time_s'):
                v = summ[k].get(mk)
                txt = f'{v:.1f} kJ' if mk == 'fleet_energy_kj' else f'{v:.0f} s'
                if k != C.BASELINE:
                    pc, _ = pct(summ, k, mk)
                    mark = {'better': ' ▲', 'worse': ' ▼'}.get(verdict(summ, k, mk), '')
                    txt += f' ({pc:+.1f} %{mark})' if pc is not None else ''
                cells.append(txt.replace('-', '−'))
            for mk, f in (('fleet_min_sep_margin_m', '+.2f'), ('fleet_wake_time_s', '.2f'),
                          ('fleet_wake_max_ms', '.2f'), ('fleet_drone_contacts', '.2f')):
                v = summ[k].get(mk)
                cells.append('-' if v is None else format(v, f).replace('-', '−'))
            L.append(f'| {label} ({n} missions) | {SHORT.get(k, k)} | ' + ' | '.join(cells) + ' |')
    return L + ['']


PLANNER_METRICS = [('fleet_energy_kj', 'Energy, fleet (kJ)', '{:.1f}'), ('fleet_flight_time_s', 'Flight time, fleet (s)', '{:.0f}'),
                   ('fleet_mission_time_s', 'Mission time (s)', '{:.1f}'),
                   ('fleet_length_ratio_mean', 'Flown / reference length, mean', '{:.3f}'),
                   ('fleet_plan_time_ms_mean', 'Planning time per route (ms)', '{:.0f}'),
                   ('fleet_replans', 'Route switches, fleet', '{:.1f}'),
                   ('fleet_goals_reached', 'Goals (of 5)', '{:.2f}'), ('fleet_min_sep_margin_m', 'Sep. margin (m)', '{:+.2f}'),
                   ('fleet_wake_time_s', 'Downwash (s)', '{:.2f}'), ('fleet_contacts', 'Obstacle contacts', '{:.2f}'),
                   ('fleet_drone_contacts', 'Drone contacts', '{:.2f}'), ('fleet_loc_rms_mean_m', 'Position error (m)', '{:.3f}'),
                   ('fleet_track_rms_mean_m', 'Tracking error (m)', '{:.3f}'),
                   ('fleet_cte_rms_mean_m', 'Path deviation from the A* reference (m; favours A*)', '{:.3f}')]


GIVEWAY_METRICS = [('fleet_energy_kj', 'Energy, fleet (kJ)', '{:.1f}'), ('fleet_flight_time_s', 'Flight time, fleet (s)', '{:.0f}'),
                   ('fleet_mission_time_s', 'Mission time (s)', '{:.1f}'), ('fleet_yield_time_s', 'Time giving way, fleet (s)', '{:.1f}'),
                   ('fleet_cte_rms_mean_m', 'Path deviation, fleet mean (m)', '{:.3f}'),
                   ('fleet_goals_reached', 'Goals (of 5)', '{:.2f}'), ('fleet_min_sep_margin_m', 'Sep. margin (m)', '{:+.2f}'),
                   ('fleet_wake_time_s', 'Downwash (s)', '{:.2f}'), ('fleet_drone_contacts', 'Drone contacts', '{:.2f}')]


def variant_comparison(res, main_, pattern, metrics):
    """The main study (the default model) vs a variant study (study_<pattern>_<stamp>) on the same missions:
    per configuration and metric, both means and the paired difference main - variant with its 95 % CI and
    the sign-flip p-value, Holm-corrected across the configurations per metric. None without the variant."""
    from . import stats as ST
    d = find(res, pattern)
    if not d or not main_:
        return None
    var = load(d)
    v_by = {(r['config'], r['layout'], r['seed']): r for r in var[0]}
    m_by = {(r['config'], r['layout'], r['seed']): r for r in main_[0]}
    keys = [k for k in var[1] if k in main_[1]]
    out = {'dir': d, 'keys': keys, 'n': {}, 'stats': {}, 'metrics': metrics}
    for k in keys:
        pairs = [(m_by[q], v_by[q]) for q in v_by if q[0] == k and q in m_by]
        out['n'][k] = len(pairs)
        for mk, *_ in metrics:
            xs = [x[mk] for x, y in pairs if x.get(mk) is not None and y.get(mk) is not None]
            ys = [y[mk] for x, y in pairs if x.get(mk) is not None and y.get(mk) is not None]
            st = ST.paired(xs, ys)
            st['main'] = float(np.mean(xs)) if xs else None
            st['var'] = float(np.mean(ys)) if ys else None
            out['stats'][(k, mk)] = st
    for mk, *_ in metrics:
        fam = [k for k in keys if out['stats'][(k, mk)]['n'] >= 2]
        for k, pa in zip(fam, ST.holm([out['stats'][(k, mk)]['p'] for k in fam])):
            out['stats'][(k, mk)]['padj'] = float(pa)
    return out


def planner_comparison(res, main_):
    return variant_comparison(res, main_, 'headless_astar_L\\d+', PLANNER_METRICS)


def giveway_comparison(res, main_):
    return variant_comparison(res, main_, 'headless_priority_L\\d+', GIVEWAY_METRICS)


def variant_numbers(cmp, title, main_name, var_name, note=''):
    """numbers.md section of a variant comparison (variant_comparison)."""
    n = sorted(set(cmp['n'].values()))
    L = ['', f'## {title}', '',
         f'The same missions ({", ".join(map(str, n))} per configuration: the forests and seeds of '
         f'{os.path.basename(cmp["dir"])}, also flown by the main study). Change = {main_name} - {var_name}, '
         'paired by mission, with its 95 % CI; p = sign-flip permutation test, Holm-corrected across the '
         'configurations per metric.' + (' ' + note if note else ''), '',
         f'| configuration | metric | {var_name} | {main_name} | change | 95 % CI | p (Holm) |', '|---|---|---|---|---|---|---|']
    for k in cmp['keys']:
        for mk, lab, f in cmp['metrics']:
            st = cmp['stats'][(k, mk)]
            if st['n'] == 0 or st['var'] is None:
                continue
            ch = st['diff']
            if abs(st['var']) > 1e-9 and mk not in M.ABS_TOL:
                ch_txt = f'{100 * ch / st["var"]:+.1f} %'
                ci_txt = f'±{100 * st["ci"] / abs(st["var"]):.1f} %' if st['ci'] is not None else '-'
            else:
                ch_txt = f'{ch:+.3g}'
                ci_txt = f'±{st["ci"]:.3g}' if st['ci'] is not None else '-'
            pa = st.get('padj')
            p_txt = '-' if pa is None else ('<0.001' if pa < 0.001 else f'{pa:.3f}')
            L.append(f'| {SHORT.get(k, k)} | {lab} | {f.format(st["var"])} | {f.format(st["main"])} | '
                     f'{ch_txt} | {ci_txt} | {p_txt} |')
    return L + ['']


def planner_numbers(cmp):
    return variant_numbers(cmp, "Path planner: RRT* (the drones' planner) vs A* (comparison)", 'RRT*', 'A*',
                           'Path deviation is measured from each drone\'s reference route (A*, planned on board before '
                           'take-off) for both planners. A*\'s first route in flight is nearly that same route, so path '
                           'deviation favours A* by construction; energy, flight time and flown/reference length compare '
                           'the planners on equal terms. Planning times are wall clock and depend on the computer.')


def giveway_numbers(cmp):
    return variant_numbers(cmp, 'Swarm give-way: turn trading (TTC, no priorities) vs fixed priorities (comparison)',
                           'TTC', 'priority', 'Configurations without the radio do not give way and fly the same '
                           'missions in both studies.')


def _pc(v):
    """A change in %, as a factor once it is +100 % or more."""
    return f'×{1 + v / 100:.1f}' if v >= 100 else f'{v:+.1f} %'


def robustness_sets(res, main_):
    """The sensitivity studies (study_headless_dw<x>_..., study_headless_gap<x>_...), newest of each, preceded
    by the main study restricted to the same forests and seeds (the default downwash and layers)."""
    from .study import summarize
    found = {}
    for d in glob.glob(os.path.join(res, 'study_headless_*')):
        m = re.fullmatch(r'study_headless_(dw|gap)([0-9.]+)_L\d+_\d{8}_\d{6}', os.path.basename(d))
        if m and os.path.exists(os.path.join(d, 'runs.csv')) and _current(d):
            tag = (m.group(1), float(m.group(2)))
            if tag not in found or stamp_of(d) > stamp_of(found[tag]):
                found[tag] = d
    USED['robustness'] = sorted(found.values())
    if not found:
        return None
    out = []
    pairs = None
    for (kind, x), d in sorted(found.items()):
        data = load(d)
        pairs = {(r['layout'], r['seed']) for r in data[0]}
        out.append((f'downwash x{x:g}' if kind == 'dw' else f'layer gap {x:g} m', data))
    rows = [r for r in main_[0] if (r['layout'], r['seed']) in pairs and r['config'] in ROBUST_KEYS]
    keys = [k for k in main_[1] if k in ROBUST_KEYS]
    base = ('default model (downwash x1, layer gap 0.6 m)', (rows, keys, summarize(rows, keys)))
    return [base] + out


def validation_numbers(vdir):
    """numbers.md section: Gazebo vs the headless twin on the same missions."""
    from . import validate as V
    data = validation_data(vdir)
    if not data:
        return []
    pairs, lost, eff = data
    keep = [p for p in pairs if p['config'] not in lost] or pairs
    seeds = sorted({p['seed'] for p in pairs})
    L = ['', '## Gazebo validation (same missions flown in Gazebo and headless)', '',
         f'{len(pairs)} paired missions: {len({p["config"] for p in pairs})} configurations x seeds '
         f'{", ".join(map(str, seeds))}, from {os.path.basename(vdir)}.'
         + (f' Lost in both simulators (mean position error > {V.LOST_M:g} m), left out below: {", ".join(lost)}.'
            if lost else ''), '',
         '| metric | Gazebo | headless | mean diff | median paired diff | Spearman r (runs) | rho (config means) |',
         '|---|---|---|---|---|---|---|']
    for a in V.agreement(keep, V.METRICS):
        if math.isfinite(a['rel']):
            d1, d2 = f"{a['rel']:+.1f} %", V._f(a['med_rel'], '+.1f', ' %')
        else:       # near-zero metric (margin, downwash time): absolute difference, no %
            d1, d2 = f"{a['diff']:+.3f} (abs.)", '-'
        L.append(f"| {a['label']} | {a['g']:.3f} | {a['h']:.3f} | {d1} | {d2} | "
                 f"{V._f(a['r_s'], '.2f')} | {V._f(a['rho'], '.2f')} |")
    keys = [k for k in ORDER if k in eff]
    L += ['', 'Change vs the baseline (paired by seed), Gazebo | headless:', '',
          '| configuration | ' + ' | '.join(lab for _, lab in V.EFFECT_METRICS) + ' |',
          '|---|' + '---|' * len(V.EFFECT_METRICS)]
    for k in keys:
        L.append(f'| {k}{" (lost)" if k in lost else ""} | '
                 + ' | '.join((f'{_pc(eff[k][mk][0])} \\| {_pc(eff[k][mk][1])}' if mk in eff[k] else '-')
                              for mk, _ in V.EFFECT_METRICS) + ' |')
    ag = V.effect_agreement(eff, [k for k in keys if k not in lost])
    L.append('| agreement (rho; median abs. diff; same sign of effects > 2 %) | '
             + ' | '.join((f"{V._f(ag[mk]['rho'], '.2f')}; {ag[mk]['med_abs_pp']:.1f} pp; "
                           f"{ag[mk]['same_sign']}/{ag[mk]['n_big']}" if mk in ag else '-')
                          for mk, _ in V.EFFECT_METRICS) + ' |')
    return L + ['']


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default='')
    ap.add_argument('--main', default='', help='main study folder (default: newest study_headless_<time>)')
    ap.add_argument('--cam80', default='')
    ap.add_argument('--mixed', default='')
    ap.add_argument('--wind', nargs='*', default=None, help='wind-sweep study folders (default: all study_headless_wind*)')
    ap.add_argument('--validation', default='')
    ap.add_argument('--out', default='')
    a = ap.parse_args(argv)
    res = a.results or default_results_dir()
    out = a.out or os.path.join(res, 'paper')
    os.makedirs(out, exist_ok=True)
    plt = setup()
    layouts.apply(0)
    d_main = a.main or find(res, 'headless_L\\d+', need=[c.key for c in C.NAV_CONFIGS])
    if not d_main:
        print('No main study found. Run:  ros2 run swarm_intel study --layouts 10 --seeds 10')
        return 1
    d80 = a.cam80 or find(res, 'headless_cam80_L\\d+')
    dmix = a.mixed or find(res, 'headless_mixed_L\\d+')
    print(f'main study : {d_main}\n80° camera : {d80}\nmixed fleet: {dmix}')
    main_ = load(d_main)
    cam80 = load(d80) if d80 else None
    mixed = load(dmix) if dmix else None
    made = [table_configs(out), fig_scenario(out, plt), fig_energy_accuracy(out, plt, main_, cam80),
            fig_energy_accuracy(out, plt, main_, None, name='fig_energy_accuracy_ring', panel_h=1.85),
            fig_effects(out, plt, main_), fig_coordination(out, plt, main_), table_main(out, main_, cam80)]
    if cam80:
        made.append(fig_effects(out, plt, cam80, name='fig_effects_cam80'))
    if mixed:
        made.append(fig_mixed(out, plt, mixed))
    wdirs = a.wind if a.wind is not None else sorted(
        d for d in glob.glob(os.path.join(res, 'study_headless_wind*'))
        if os.path.exists(os.path.join(d, 'runs.csv')) and _current(d))
    wind_sets = []
    for d in wdirs:
        m = re.search(r'_wind([0-9.]+)_(?:L\d+_)?\d{8}', os.path.basename(d))
        if m:
            wind_sets.append((float(m.group(1)), load(d)))
    if wind_sets:
        # the main study is the default wind: use the same forests and seeds as the sweep
        pairs = {(r['layout'], r['seed']) for _, (rows, _, _) in wind_sets for r in rows}
        sub = [r for r in main_[0] if (r['layout'], r['seed']) in pairs]
        wind_sets.append((C.WIND.mean_speed, (sub, main_[1], None)))
        made.append(fig_wind(out, plt, wind_sets))
        if mixed:
            made.append(fig_mixed_wind(out, plt, mixed, wind_sets))
    vd = a.validation or (sorted((d for d in glob.glob(os.path.join(res, 'validation_*')) if _current(d)),
                                 key=stamp_of) or [None])[-1]
    if not (vd and os.path.exists(os.path.join(vd, 'pairs.csv'))):
        vd = None
    if vd:
        made.append(fig_validation(out, plt, vd))
    robust = robustness_sets(res, main_)
    planner = planner_comparison(res, main_)
    giveway = giveway_comparison(res, main_)
    sources = {'main': d_main, 'cam80': d80, 'mixed': dmix, 'wind': list(wdirs), 'validation': vd,
               'robustness': USED.get('robustness') or [],
               'planner': planner['dir'] if planner else None, 'giveway': giveway['dir'] if giveway else None}
    made.append(numbers(out, [('Main study (360° camera ring)', main_), ('Single forward 80° camera', cam80),
                              ('Mixed fleets', mixed)], vdir=vd, robust=robust, planner=planner, giveway=giveway,
                        provenance=read_meta(d_main)))
    write_meta(out, kind='paper', sources={k: (os.path.basename(v) if isinstance(v, str) else
                                               [os.path.basename(x) for x in v] if v else v)
                                           for k, v in sources.items()},
               left_out=sorted(os.path.basename(d) for d in SKIPPED))
    for f in made:
        if f:
            print('  wrote', f)
    if SKIPPED:
        n = len(SKIPPED)
        print(f'  left out ({n} folder{"s" if n > 1 else ""} made with another model version than {C.MODEL_VERSION}, '
              'or before meta.json): ' + ', '.join(sorted(os.path.basename(d) for d in SKIPPED)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
