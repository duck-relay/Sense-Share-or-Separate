#!/usr/bin/env python3
"""Speed sweep: energy-vs-accuracy CURVES for chosen configurations (headless).

    ros2 run swarm_intel batch                                # baseline vs all, 8 seeds x 6 speeds
    ros2 run swarm_intel batch --configs cam3_sensor,cam2_swarm,all --seeds 12
    ros2 run swarm_intel batch --export-runs                  # also writes one headless run folder per
                                                              # config (open with compare --a-dir/--b-dir)

Runs the SAME mission code as the Gazebo node, with a kinematic stand-in for
Gazebo. Faster cruise = less hover time = less energy, but less accuracy:
each configuration gives one curve. For the all-configurations table at the
nominal speed use  ros2 run swarm_intel study.
Output: <results>/batch/batch_results.csv, batch_summary.csv, energy_vs_accuracy.png
"""
import argparse
import csv
import os
import time
from multiprocessing import Pool

import numpy as np

from . import config as C
from .plant import run_headless
from .report import dumps

# every value covers the whole fleet (all 5 drones; metrics.py)
FIELDS = ['config', 'seed', 'speed_scale', 'cruise_speed', 'fleet_energy_kj', 'fleet_cte_rms', 'fleet_cte_max',
          'fleet_on_path_pct', 'fleet_loc_rms', 'fleet_landing_err', 'fleet_flight_time', 'fleet_mission_time',
          'fleet_replans', 'fleet_avoid_s', 'fleet_min_sep', 'fleet_sep_margin', 'fleet_wake_s', 'fleet_contacts',
          'fleet_goals', 'fleet_unknown_known']


def _one(args):
    cfg, seed, sc = args
    rep, _ = run_headless(cfg, seed=seed, speed_scale=sc)
    nan = lambda v: float('nan') if v is None else v
    return {
        'config': cfg, 'seed': seed, 'speed_scale': sc, 'cruise_speed': round(C.CRUISE_SPEED * sc, 3),
        'fleet_energy_kj': rep['fleet_energy_kj'], 'fleet_cte_rms': nan(rep['fleet_cte_rms_mean_m']),
        'fleet_cte_max': nan(rep['fleet_cte_max_m']), 'fleet_on_path_pct': nan(rep['fleet_on_path_pct_mean']),
        'fleet_loc_rms': nan(rep['fleet_loc_rms_mean_m']), 'fleet_landing_err': nan(rep['fleet_landing_err_mean_m']),
        'fleet_flight_time': nan(rep['fleet_flight_time_s']), 'fleet_mission_time': nan(rep['fleet_mission_time_s']),
        'fleet_replans': rep['fleet_replans'], 'fleet_avoid_s': rep['fleet_avoid_time_s'],
        'fleet_min_sep': nan(rep['fleet_min_separation_m']), 'fleet_sep_margin': nan(rep['fleet_min_sep_margin_m']),
        'fleet_wake_s': rep['fleet_wake_time_s'], 'fleet_contacts': rep['fleet_contacts'],
        'fleet_goals': rep['fleet_goals_reached'], 'fleet_unknown_known': nan(rep['fleet_unknown_known_mean']),
    }


DEFAULT_CONFIGS = [C.BASELINE, 'all']


def run_batch(seeds, speeds, workers=None, configs=None):
    configs = list(configs or DEFAULT_CONFIGS)
    jobs = [(m, s, sc) for sc in speeds for m in configs for s in range(seeds)]
    t0 = time.time()
    rows = []
    workers = workers or max(1, min(os.cpu_count() or 1, 8))
    with Pool(workers) as pool:
        for k, r in enumerate(pool.imap_unordered(_one, jobs), 1):
            rows.append(r)
            if k % max(1, len(jobs) // 20) == 0 or k == len(jobs):
                print(f'\r  {k}/{len(jobs)} missions done ({time.time() - t0:.0f}s)', end='', flush=True)
    print()
    rows.sort(key=lambda r: (r['speed_scale'], r['config'], r['seed']))
    return rows


def summarize(rows):
    out = []
    for sc in sorted(set(r['speed_scale'] for r in rows)):
        for m in sorted(set(r['config'] for r in rows), key=lambda k: list(C.NAV).index(k)):
            sel = [r for r in rows if r['speed_scale'] == sc and r['config'] == m]
            if not sel:
                continue
            agg = {'config': m, 'speed_scale': sc, 'cruise_speed': sel[0]['cruise_speed'], 'n': len(sel)}
            for k in FIELDS[4:]:
                v = np.array([r[k] for r in sel], float)
                agg[k + '_mean'] = float(np.nanmean(v))
                agg[k + '_std'] = float(np.nanstd(v))
            out.append(agg)
    return out


def plot_batch(rows, path):
    """Small multiples: one panel per configuration with its energy-vs-accuracy curve over the cruise
    speeds, drawn against the baseline's curve (grey). All panels share their axes, except a configuration
    far off the others (e.g. swarm only), which gets its own scale so it does not squash the rest."""
    import math
    import textwrap
    from matplotlib.lines import Line2D
    from . import plots as P
    plt = P.setup()
    summ = summarize(rows)
    X, Y = 'fleet_energy_kj', 'fleet_cte_rms'
    cfgs = sorted(set(r['config'] for r in rows), key=lambda k: list(C.NAV).index(k))
    ref = C.BASELINE if C.BASELINE in cfgs else cfgs[0]
    others = [k for k in cfgs if k != ref] or [ref]
    curve = lambda k: sorted((s for s in summ if s['config'] == k), key=lambda s: s['cruise_speed'])
    means = lambda k, f: [s[f + '_mean'] for s in curve(k)]
    far = {k for k in others if k != ref and min(means(k, Y)) > 4.0 * max(means(ref, Y))}

    def limits(keys):
        xs, ys = [], []
        for k in keys:
            for s in curve(k):
                xs += [s[X + '_mean'] - s[X + '_std'], s[X + '_mean'] + s[X + '_std']]
                ys += [s[Y + '_mean'] + s[Y + '_std']]
        pad = 0.2 * (max(xs) - min(xs))         # room for the speed labels at both ends
        return (min(xs) - pad, max(xs) + pad), (0.0, 1.12 * max(ys))
    shared = limits([k for k in cfgs if k not in far])

    ncol = min(4, len(others))
    nrow = math.ceil(len(others) / ncol)
    width = max(4.4 * ncol + 0.8, 10.0)
    fig, axs = plt.subplots(nrow, ncol, figsize=(width, 3.9 * nrow + 1.9), squeeze=False)
    top = 1.0 - 1.55 / (3.9 * nrow + 1.9)
    fig.subplots_adjust(left=0.055 * 18 / width, right=0.985, top=top, bottom=0.55 / (3.9 * nrow + 1.9) + 0.06,
                        wspace=0.22, hspace=0.42)
    n_seeds = len(set(r['seed'] for r in rows))
    speeds = sorted(set(r['cruise_speed'] for r in rows))
    fig.suptitle('Energy vs. accuracy of the whole fleet across cruise speeds',
                 x=0.012, y=0.985, ha='left', fontsize=14, fontweight='bold', color=P.INK)
    spd = (f'{len(speeds)} cruise speeds ({speeds[0]:.2f}–{speeds[-1]:.2f} m/s)' if len(speeds) > 1
           else f'1 cruise speed ({speeds[0]:.2f} m/s)')
    sub = (f'{n_seeds} random seed{"s" if n_seeds != 1 else ""} × {spd} per configuration; all 5 drones use it. One panel per '
           'configuration against the baseline (grey). Faster cruise = less energy but more path error; '
           'down-left is better. Dots = mean, whiskers = ±1 SD.')
    fig.text(0.012, 1.0 - 0.62 / (3.9 * nrow + 1.9), textwrap.fill(sub, int(width * 15)), color=P.INK2,
             fontsize=9.5, va='top')
    for ax in axs.ravel()[len(others):]:
        ax.axis('off')

    def draw(ax, k, col, label_ends):
        ss = curve(k)
        x, y = means(k, X), means(k, Y)
        ax.errorbar(x, y, xerr=[s[X + '_std'] for s in ss], yerr=[s[Y + '_std'] for s in ss], fmt='none',
                    ecolor=col, elinewidth=0.9, alpha=0.55, zorder=3)
        ax.plot(x, y, '-', color=col, lw=2, zorder=4)
        ax.scatter(x, y, s=40 if k != ref else 34, marker='o' if k != ref else 'D', color=col,
                   edgecolor=P.SURFACE, linewidth=1.6, zorder=5)
        if label_ends:
            ends = ((int(np.argmax(x)), 8, 'left'), (int(np.argmin(x)), -8, 'right'))
            for i, dx, ha in ends[:1 if len(x) == 1 else 2]:
                ax.annotate(f"{ss[i]['cruise_speed']:.2f} m/s", (x[i], y[i]), xytext=(dx, 8),
                            textcoords='offset points', fontsize=7.5, color=P.INK2, ha=ha, zorder=6,
                            bbox=dict(boxstyle='round,pad=0.12', fc=P.SURFACE, ec='none', alpha=0.85))

    for ax, k in zip(axs.ravel(), others):
        sel = [r for r in rows if r['config'] == k]
        ax.scatter([r[X] for r in sel], [r[Y] for r in sel], s=9, color=P.CAT_A, alpha=0.16, lw=0, zorder=2)
        if k != ref:
            draw(ax, ref, P.SAME, False)
        draw(ax, k, P.CAT_A, True)
        xl, yl = limits([k, ref]) if k in far else shared
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_title(textwrap.fill(C.NAV[k].label, 34), fontsize=10, pad=6)
        # the change vs the baseline at the nominal speed (x1.0), when it was flown
        nom = {s['config']: s for s in summ if abs(s['speed_scale'] - 1.0) < 1e-9}
        if k != ref and k in nom and ref in nom:
            a, b = nom[k], nom[ref]
            from . import metrics as M
            txt = (f"at ×1.0: energy {M.fmt_change(a[X + '_mean'], b[X + '_mean'])}, "
                   f"path RMS {M.fmt_change(a[Y + '_mean'], b[Y + '_mean'])}")
            ax.text(0.98, 0.96, txt, transform=ax.transAxes, ha='right', va='top', fontsize=8, color=P.INK,
                    bbox=dict(boxstyle='round,pad=0.3', fc=P.SURFACE, ec=P.GRID))
        if k in far:
            ax.text(0.98, 0.83, 'own scale: far off the others', transform=ax.transAxes, ha='right', va='top',
                    fontsize=7.5, color=P.WORSE)
        ax.tick_params(labelsize=7.5)
    for ax in axs[-1, :]:
        ax.set_xlabel('Flight energy of the fleet (kJ)', fontsize=8.5)
    for ax in axs[:, 0]:
        ax.set_ylabel('Path deviation RMS, mean (m)', fontsize=8.5)
    for r_ in range(nrow - 1):                 # an empty slot below: label the panel above it
        for c_ in range(ncol):
            if r_ * ncol + c_ < len(others) and (r_ + 1) * ncol + c_ >= len(others):
                axs[r_, c_].set_xlabel('Flight energy of the fleet (kJ)', fontsize=8.5)
    handles = [Line2D([], [], color=P.SAME, marker='D', ms=6, mec=P.SURFACE, lw=2,
                      label=f'Baseline: {C.NAV[ref].label.replace("Baseline: ", "")}'),
               Line2D([], [], color=P.CAT_A, marker='o', ms=7, mec=P.SURFACE, lw=2, label='Configuration of the panel'),
               Line2D([], [], color=P.CAT_A, marker='o', ms=4, lw=0, alpha=0.3, label='Single missions')]
    if ncol >= 3:
        fig.legend(handles=handles, loc='upper right', bbox_to_anchor=(0.985, 0.995), ncol=3, fontsize=8.5)
    else:                                       # narrow figure: the title needs the top row
        axs[-1, -1].legend(handles=handles, loc='lower right', fontsize=8, framealpha=0.9)
    from .energy import PowerModel
    hover = PowerModel(C.NAV[ref]).hover_power()
    P.footer(fig, f'Energy: momentum-theory rotor power model (hover ≈ {hover:.0f} W) incl. navigation electronics. '
                  'Accuracy: ground-truth position vs each drone\'s reference route (A* on board before take-off), '
                  'mean of the 5 drones. Speeds: every drone\'s cruise speed.')
    fig.savefig(path, dpi=130)
    plt.close(fig)


def export_demo_runs(results_dir, configs=None, seed=0, speed=1.0):
    """Write Gazebo-style run folders from headless runs (for `compare --a-dir/--b-dir`).
    They are named <config>_headless_<time> and are NOT counted as Gazebo runs."""
    from .mission import MissionCore
    from .plant import KinematicPlant
    from datetime import datetime
    dirs = {}
    for cfg in (configs or DEFAULT_CONFIGS):
        core = MissionCore(cfg, seed=seed, speed_scale=speed)
        plant = KinematicPlant(core.world)
        d = os.path.join(results_dir, f'{cfg}_headless_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, 'telemetry.csv'), 'w', newline='')
        w = csv.writer(f)
        w.writerow(['t', 'id', 'state', 'x', 'y', 'z', 'est_x', 'est_y', 'est_z', 'vx', 'vy', 'vz', 'speed',
                    'power_w', 'energy_j', 'soc', 'loc_err', 'cte', 'yielding', 'avoiding', 'downdraft'])
        t, dt, k = 0.0, 1.0 / C.CONTROL_RATE, 0
        while t < 300 and core.all_landed_t is None:
            tr = plant.truth()
            cmds = core.step(t, tr)
            for i, a in core.agents.items():
                p, e, v = tr[i]['p'], a.ekf.pos, tr[i]['v']
                w.writerow([round(core.mission_time, 3), i, a.state, *np.round(p, 4), *np.round(e, 4), *np.round(v, 4),
                            round(float(np.linalg.norm(v)), 4), round(a.power, 2), round(a.battery.used_j, 2),
                            round(a.battery.soc, 3), round(a.last_loc_err, 4), round(a.last_cte, 4),
                            int(a.yielding), int(a.avoiding), round(core.downdraft.get(i, 0.0), 3)])
            plant.step(cmds, dt)
            k += 1
            t = k * dt
        f.close()
        rep = core.report()
        rep['final'] = core.all_landed_t is not None
        rep['source'] = 'headless'
        with open(os.path.join(d, 'summary.json'), 'w') as fo:
            fo.write(dumps(rep, indent=2))
        with open(os.path.join(d, 'reference_paths.csv'), 'w') as fo:      # every drone's reference route
            fo.write('id,x,y\n' + ''.join(f'{i},{x:.3f},{y:.3f}\n' for i, a in core.agents.items() for x, y in a.ref))
        dirs[cfg] = d
        print(f'  wrote {d}')
    return dirs


def main(argv=None):
    from .paths import default_results_dir
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seeds', type=int, default=8)
    ap.add_argument('--speeds', default='0.6,0.8,1.0,1.2,1.45,1.7', help='cruise speed multipliers')
    ap.add_argument('--out', default='', help='output folder (default <results>/batch)')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--export-runs', action='store_true', help='also write one headless run folder per config')
    ap.add_argument('--configs', default=f'{C.BASELINE},all', help='comma-separated configurations')
    a = ap.parse_args(argv)
    out = a.out or os.path.join(default_results_dir(), 'batch')
    os.makedirs(out, exist_ok=True)
    from .runinfo import write_meta
    write_meta(out, kind='batch', seeds=a.seeds, speeds=a.speeds, configurations=a.configs)
    speeds = [float(s) for s in a.speeds.split(',') if s.strip()]
    cfgs = list(dict.fromkeys(C.nav_config(k).key for k in a.configs.split(',') if k.strip()))
    print(f'Running {a.seeds} seeds x {len(speeds)} speeds x {len(cfgs)} configs (all 5 drones per run) ...')
    rows = run_batch(a.seeds, speeds, a.workers or None, cfgs)
    with open(os.path.join(out, 'batch_results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    summ = summarize(rows)
    with open(os.path.join(out, 'batch_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0].keys()))
        w.writeheader()
        w.writerows(summ)
    png = os.path.join(out, 'energy_vs_accuracy.png')
    plot_batch(rows, png)
    print(f'\n{"speed":>6} {"config":>13} | {"energy kJ":>10} {"path RMS m":>10} {"path max m":>10} {"on-path %":>9}'
          f' {"flight s":>8} {"mission s":>9} {"closest m":>9} {"contacts":>8}   (all 5 drones)')
    for s in summ:
        print(f"{s['cruise_speed']:6.2f} {s['config']:>13} | {s['fleet_energy_kj_mean']:10.2f} {s['fleet_cte_rms_mean']:10.3f}"
              f" {s['fleet_cte_max_mean']:10.3f} {s['fleet_on_path_pct_mean']:9.1f} {s['fleet_flight_time_mean']:8.1f}"
              f" {s['fleet_mission_time_mean']:9.1f} {s['fleet_min_sep_mean']:9.2f} {s['fleet_contacts_mean']:8.2f}")
    print(f'\nSaved: {png}\n       {out}/batch_results.csv, batch_summary.csv')
    if a.export_runs:
        export_demo_runs(default_results_dir() if not a.out else a.out, cfgs)


if __name__ == '__main__':
    main()
