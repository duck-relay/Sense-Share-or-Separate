"""Graphs of a navigation study (written by `study`).

  study_comparison.png  the detailed final comparison: every metric of every
                        configuration vs the baseline (heatmap) + the key metrics
                        with their run-to-run spread
  study_overview.png    energy vs accuracy, and how many metrics are better / worse
  study_changes.png     % change vs the baseline for 12 metrics
  study_paths.png       every drone's flown path (in its own colour) vs its reference route (green: A* on
                        board before take-off)

Every metric covers the whole fleet (all 5 drones). Colours: green = reference route, each drone's flown
path in its own colour, blue = better than the baseline, red = worse, grey = about the same (difference
within the noise).
"""
import math
import os
import textwrap

import numpy as np

from . import config as C
from . import metrics as M
from . import plots as P


def _verdict(summ, k, mk):
    from .study import verdict
    return verdict(summ, k, mk)


def _rule_text(summ):
    from .study import n_tests
    if n_tests(summ):
        return ('Verdicts: mean paired difference (same forest and seed), significant in a permutation test '
                'after Holm correction, AND more than 2 %.')
    return 'Verdicts: difference > 2 % and > 2 standard errors (too few runs for a significance test).'


def _counts(summ, k):
    from .study import counts
    return counts(summ, k)


VCOL = {'better': P.BETTER, 'worse': P.WORSE, 'same': P.SAME, None: P.MIXED}


def _log_or_linear(ax, values, axis='x'):
    """Log scale with plain-number labels when the values span a decade or more, else linear."""
    from matplotlib.ticker import FuncFormatter, NullFormatter
    v = [x for x in values if x is not None and x > 0]
    if not v or max(v) / min(v) < 10.0:
        return False
    (ax.set_xscale if axis == 'x' else ax.set_yscale)('log')
    a = ax.xaxis if axis == 'x' else ax.yaxis
    a.set_major_formatter(FuncFormatter(lambda x, _: f'{x:g}'))
    a.set_minor_formatter(NullFormatter())
    return True


# ---------------------------------------------------------------------------
# label placement (measured text, never outside the axes, leader lines when far)
# ---------------------------------------------------------------------------
def _place_labels(ax, pts, fontsize=8.5, avoid=(), scale=1.0, line_color='#a3a29b', mark_pad=3, skip_own=False):
    """pts: list of (x, y, text, bold). avoid: display-space boxes to keep clear (e.g. an inset).
    scale: offsets in points are multiplied by this (smaller figures, smaller fonts).
    mark_pad: px around a marker centre a label must keep clear; skip_own: a label may touch its own marker."""
    fig = ax.figure
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    ab = ax.get_window_extent(rend)
    marks = [ax.transData.transform((x, y)) for x, y, *_ in pts]
    placed = list(avoid)
    cands = [(8, 0, 'left'), (-8, 0, 'right'), (8, 9, 'left'), (8, -9, 'left'), (-8, 9, 'right'), (-8, -9, 'right')]
    cands += [(0, 11, 'center'), (0, -11, 'center')]          # long labels in a narrow inset fit only above/below
    for r in (22, 34, 48, 64, 82, 104):
        for ang in range(0, 360, 20):
            dx, dy = r * math.cos(math.radians(ang)), 0.75 * r * math.sin(math.radians(ang))
            cands.append((dx, dy, 'left' if dx >= 0 else 'right'))
        cands += [(0, 0.75 * r, 'center'), (0, -0.75 * r, 'center')]
    cands = [(dx * scale, dy * scale, ha) for dx, dy, ha in cands]
    order = sorted(range(len(pts)), key=lambda i: (not pts[i][3], -marks[i][1]))      # baseline label first
    for i in order:
        x, y, txt, bold = pts[i]
        best = None
        for dx, dy, ha in cands:
            t = ax.annotate(txt, (x, y), xytext=(dx, dy), textcoords='offset points', ha=ha, va='center',
                            fontsize=fontsize, fontweight='bold' if bold else 'normal')
            bb = t.get_window_extent(rend).expanded(1.04, 1.15)
            t.remove()
            inside = bb.x0 >= ab.x0 + 2 and bb.x1 <= ab.x1 - 2 and bb.y0 >= ab.y0 + 2 and bb.y1 <= ab.y1 - 2
            # how badly it collides (area in px^2), so a crowded spot picks the least-bad place, not the first one
            over = sum(max(0.0, min(bb.x1, b.x1) - max(bb.x0, b.x0)) * max(0.0, min(bb.y1, b.y1) - max(bb.y0, b.y0))
                       for b in placed)
            n_marks = sum(1 for j, (mx, my) in enumerate(marks) if not (skip_own and j == i)
                          and bb.x0 - mark_pad < mx < bb.x1 + mark_pad and bb.y0 - mark_pad < my < bb.y1 + mark_pad)
            score = (0 if inside else 1e6) + over + 300 * n_marks
            score = score + 0.5 * math.hypot(dx, dy) if score else 0
            if best is None or score < best[0]:
                best = (score, dx, dy, ha, bb)
            if score == 0:
                break
        _, dx, dy, ha, bb = best
        placed.append(bb)
        far = math.hypot(dx, dy) > 14 * scale
        ax.annotate(txt, (x, y), xytext=(dx, dy), textcoords='offset points', ha=ha, va='center',
                    fontsize=fontsize, color=P.INK if bold else P.INK2, fontweight='bold' if bold else 'normal',
                    annotation_clip=False, zorder=6,
                    arrowprops=dict(arrowstyle='-', color=line_color, lw=0.7 * max(scale, 0.6), shrinkA=2 * scale,
                                    shrinkB=4 * scale) if far else None)


def _pick_corner(ax, xy, zoom_frac):
    """Inset position (axes fraction) that covers no data point and not the zoom box."""
    pts = [ax.transAxes.inverted().transform(ax.transData.transform(p)) for p in xy]
    best = None
    for rect in ([0.42, 0.05, 0.55, 0.47], [0.42, 0.48, 0.55, 0.47], [0.04, 0.48, 0.55, 0.47],
                 [0.04, 0.05, 0.55, 0.47]):
        x0, y0, w, h = rect
        n_pts = sum(1 for (u, v) in pts if x0 - 0.03 < u < x0 + w + 0.03 and y0 - 0.03 < v < y0 + h + 0.03)
        zx0, zy0, zx1, zy1 = zoom_frac
        ov = max(0.0, min(x0 + w, zx1) - max(x0, zx0)) * max(0.0, min(y0 + h, zy1) - max(y0, zy0))
        score = n_pts * 10 + ov * 100
        if best is None or score < best[0]:
            best = (score, rect)
    return best[1]


def _pair_colour(summ, k, yk):
    """Dot colour in the energy-vs-error plots: better / worse on both axes, or a trade-off."""
    if k == C.BASELINE:
        return P.INK
    ve, va = _verdict(summ, k, 'fleet_energy_kj'), _verdict(summ, k, yk)
    vs = {ve, va}
    if vs <= {'better', 'same'} and 'better' in vs:
        return P.BETTER
    if vs <= {'worse', 'same'} and 'worse' in vs:
        return P.WORSE
    if vs == {'same'}:
        return P.SAME
    return P.MIXED


# ---------------------------------------------------------------------------
def _overview(plt, summ, present, base, out_dir, source, n_info):
    from matplotlib.lines import Line2D
    fig, axs = plt.subplots(1, 3, figsize=(18.5, 6.4), gridspec_kw={'width_ratios': [1, 1, 1.1]})
    fig.subplots_adjust(left=0.045, right=0.985, top=0.80, bottom=0.13, wspace=0.42)
    fig.suptitle('Energy vs accuracy of the whole fleet — every configuration against the baseline '
                 '(3-point camera + VIO)', x=0.012, y=0.975, ha='left', fontsize=14, fontweight='bold',
                 color=P.INK)
    cam = ('360° navigation camera ring' if C.SENSORS.cam_fov_deg >= 300
           else f'Single forward {C.SENSORS.cam_fov_deg:.0f}° camera')
    single = max(s['n'] for s in summ.values()) == 1
    fig.text(0.012, 0.905, f'{source}: {n_info}. {cam}. Dots = mean'
             + ('' if single else ', whiskers = ±1 SD') + '. Down-left is better; accuracy axes are logarithmic.',
             color=P.INK2, fontsize=9.5)
    for ax, yk, ylab, title in ((axs[0], 'fleet_cte_rms_mean_m', 'Path deviation RMS, mean of the drones (m)',
                                 'A · Path accuracy vs energy'),
                                (axs[1], 'fleet_loc_rms_mean_m', 'Position error RMS, mean of the drones (m)',
                                 'B · Localisation vs energy')):
        def draw(target, keys_, small=False):
            lab, xy = [], []
            for k in keys_:
                s = summ[k]
                x, y = s.get('fleet_energy_kj'), s.get(yk)
                if x is None or y is None:
                    continue
                isb = k == C.BASELINE
                c = _pair_colour(summ, k, yk)
                target.errorbar([x], [y], xerr=[s.get('fleet_energy_kj_sd') or 0],
                                yerr=[[min(s.get(yk + '_sd') or 0, y * 0.9)], [s.get(yk + '_sd') or 0]],
                                fmt='none', ecolor=c, elinewidth=1, alpha=0.4, zorder=2)
                target.scatter([x], [y], s=(80 if small else 110) if isb else (50 if small else 70), color=c,
                               edgecolor=P.SURFACE, linewidth=1.5 if small else 2, zorder=4, marker='D' if isb else 'o')
                lab.append((x, y, k + (' (baseline)' if isb else ''), isb))
                xy.append((x, y))
            return lab, xy
        lab, xy = draw(ax, present)
        if base and base.get(yk):
            ax.axhline(base[yk], color=P.INK, lw=0.8, alpha=0.45, zorder=1)
        if base and base.get('fleet_energy_kj'):
            ax.axvline(base['fleet_energy_kj'], color=P.INK, lw=0.8, alpha=0.45, zorder=1)
        _log_or_linear(ax, [summ[k].get(yk) for k in present], 'y')
        ax.set_xlabel('Flight energy of the fleet (kJ, all 5 drones)')
        ax.set_ylabel(ylab)
        ax.set_title(title, pad=10)
        ax.margins(x=0.18, y=0.15)
        crowd = []
        if base and base.get(yk) and base.get('fleet_energy_kj'):
            crowd = [k for k in present if summ[k].get(yk) is not None and summ[k].get('fleet_energy_kj') is not None
                     and summ[k][yk] < 2.0 * base[yk]
                     and abs(summ[k]['fleet_energy_kj'] / base['fleet_energy_kj'] - 1) < 0.15]
        if len(crowd) >= 3:
            fig.canvas.draw()
            xs = [summ[k]['fleet_energy_kj'] for k in crowd]
            ys = [summ[k][yk] for k in crowd]
            dx = max(0.05, max(xs) - min(xs))
            dy = max(0.005, max(ys) - min(ys))
            x0, x1 = min(xs) - 0.45 * dx, max(xs) + 1.2 * dx
            y0, y1 = min(ys) - 0.45 * dy, max(ys) + 0.45 * dy
            # keep the zoom box inside the panel (on a log axis y0 can fall below the axis)
            (ax0, ax1), (ay0, ay1) = ax.get_xlim(), ax.get_ylim()
            x0, x1 = max(x0, ax0), min(x1, ax1)
            y0, y1 = max(y0, ay0), min(y1, ay1)
            to_ax = lambda p: ax.transAxes.inverted().transform(ax.transData.transform(p))
            (zx0, zy0), (zx1, zy1) = to_ax((x0, max(y0, 1e-6))), to_ax((x1, y1))
            rect = _pick_corner(ax, xy, (zx0, zy0, zx1, zy1))
            ins = ax.inset_axes(rect)
            ins.set_facecolor(P.SURFACE)
            ilab, _ = draw(ins, crowd, small=True)
            ins.axhline(base[yk], color=P.INK, lw=0.6, alpha=0.4, zorder=1)
            ins.axvline(base['fleet_energy_kj'], color=P.INK, lw=0.6, alpha=0.4, zorder=1)
            ins.set_xlim(x0, x1)
            ins.set_ylim(y0, y1)
            ins.tick_params(labelsize=7)
            ins.set_title('zoom: configurations close to the baseline', fontsize=8, pad=3, color=P.INK2,
                          fontweight='normal')
            for sp in ins.spines.values():
                sp.set_visible(True)
                sp.set_color(P.MUTED)
            ax.indicate_inset_zoom(ins, edgecolor=P.MUTED, alpha=0.6)
            _place_labels(ins, ilab, fontsize=7.5)
            lab = [l_ for l_ in lab if l_[2].split(' ')[0] not in crowd]
            fig.canvas.draw()
            inset_box = ins.get_tightbbox(fig.canvas.get_renderer())
            _place_labels(ax, lab, avoid=[inset_box])
        else:
            _place_labels(ax, lab)
    axs[0].legend(handles=[
        Line2D([], [], marker='D', ls='none', color=P.INK, ms=7, label='baseline'),
        Line2D([], [], marker='o', ls='none', color=P.BETTER, ms=7, label='better (≥ one axis, none worse)'),
        Line2D([], [], marker='o', ls='none', color=P.WORSE, ms=7, label='worse (≥ one axis, none better)'),
        Line2D([], [], marker='o', ls='none', color=P.MIXED, ms=7, label='trade-off (one better, one worse)'),
        Line2D([], [], marker='o', ls='none', color=P.SAME, ms=7, label='about the same')],
        loc='upper left', fontsize=7.5, frameon=True, facecolor=P.SURFACE, edgecolor=P.GRID, framealpha=0.9)
    # C: verdict counts
    ax = axs[2]
    others = [k for k in present if k != C.BASELINE]
    if base and others:
        cnt = np.array([_counts(summ, k) for k in others])          # better, same, worse
        total = int(cnt[0].sum())
        yy = np.arange(len(others))[::-1]
        ax.barh(yy, cnt[:, 0], color=P.BETTER, height=0.62, label='better', edgecolor=P.SURFACE, linewidth=2)
        ax.barh(yy, cnt[:, 1], left=cnt[:, 0], color=P.SAME, height=0.62, label='about the same',
                edgecolor=P.SURFACE, linewidth=2)
        ax.barh(yy, cnt[:, 2], left=cnt[:, 0] + cnt[:, 1], color=P.WORSE, height=0.62, label='worse',
                edgecolor=P.SURFACE, linewidth=2)
        for i, (b_, s_, w_) in enumerate(cnt):
            ax.text(b_ + s_ + w_ + 0.6, yy[i], f'{b_} better · {w_} worse', va='center', fontsize=8.5, color=P.INK2)
        ax.set_yticks(yy)
        ax.set_yticklabels([textwrap.fill(C.label_of(k), 22) for k in others], fontsize=8.5)
        ax.set_title('C · Metrics better / worse than the baseline', pad=10)
        ax.set_xlabel(f'number of metrics (of {total} with a better/worse direction)')
        ax.legend(loc='upper center', bbox_to_anchor=(0.45, -0.12), ncol=3, fontsize=8.5)
        ax.grid(axis='y', visible=False)
        ax.set_xlim(0, total + 14)
        ax.set_xticks(list(range(0, total + 1, 5)))   # the right part holds the counts, not data
        ax.spines['bottom'].set_bounds(0, total)
    P.footer(fig, 'Energy: rotor power model (momentum theory) on the air speed, incl. navigation electronics. '
                  'Accuracy from ground truth. ' + _rule_text(summ))
    f = os.path.join(out_dir, 'study_overview.png')
    fig.savefig(f, dpi=120)
    plt.close(fig)
    return f


def _changes(plt, summ, others, base, out_dir):
    mks = ['fleet_energy_kj', 'fleet_flight_time_s', 'fleet_mission_time_s', 'fleet_energy_per_m_j',
           'fleet_cte_rms_mean_m', 'fleet_cte_max_m', 'fleet_landing_err_mean_m', 'fleet_loc_rms_mean_m',
           'fleet_loc_max_m', 'fleet_safety_interventions', 'fleet_replans', 'fleet_goals_reached']
    fig, axs = plt.subplots(3, 4, figsize=(18.5, 11.8))
    fig.subplots_adjust(left=0.165, right=0.955, top=0.90, bottom=0.06, wspace=0.62, hspace=0.5)
    fig.suptitle('Change vs the baseline (3-point camera + VIO), per fleet metric', x=0.012, y=0.975, ha='left',
                 fontsize=14, fontweight='bold', color=P.INK)
    fig.text(0.012, 0.938, 'Blue = better than the baseline, red = worse, grey = within the run-to-run noise '
             '(direction depends on the metric). Each panel has its own scale; bars beyond ±150 % are clipped (marked »), the label shows the real value.',
             color=P.INK2, fontsize=9.5)
    yy = np.arange(len(others))[::-1]
    for ax, mk in zip(axs.ravel(), mks):
        vals = [M.change(summ[k].get(mk), base.get(mk)) for k in others]
        cols, shown = [], []
        for k, v in zip(others, vals):
            cols.append(VCOL[_verdict(summ, k, mk)] if v is not None else P.SAME)
            shown.append(0.0 if v is None else max(-150.0, min(150.0, v)))
        ax.barh(yy, shown, color=cols, height=0.6, edgecolor=P.SURFACE, linewidth=1.5)
        ax.axvline(0, color=P.INK, lw=0.8)
        # own scale per panel, so a -9 % energy saving is not a sliver next to a ±200 % axis;
        # room is left on both sides for the value labels
        lim = 10.0 * math.ceil((1.45 * max([abs(x) for x in shown] + [0.0]) + 8.0) / 10.0)
        lim = min(max(lim, 30.0), 230.0)
        off = 0.02 * lim
        for i, (k, v) in enumerate(zip(others, vals)):
            txt = 'n/a' if v is None else M.fmt_change(summ[k].get(mk), base.get(mk), mk)
            x = shown[i]
            if v is not None and abs(v) > 150:
                txt = ('» ' if v > 0 else '« ') + txt
            ax.text(x + (off if x >= 0 else -off), yy[i], txt, va='center', ha='left' if x >= 0 else 'right',
                    fontsize=7.5, color=P.INK2)
        ax.set_xlim(-lim, lim)
        ax.set_yticks(yy)
        ax.set_yticklabels(others if ax not in axs[:, 0] else [C.label_of(k) for k in others], fontsize=7.5)
        _, lab, unit, _, _ = M.BY_KEY[mk]
        ax.set_title(textwrap.fill(f"{lab} ({unit})", 34),
                     fontsize=10, pad=6)
        ax.set_xlabel('% change vs baseline', fontsize=8)
        ax.grid(axis='y', visible=False)
        ax.tick_params(axis='x', labelsize=7.5)
    f = os.path.join(out_dir, 'study_changes.png')
    fig.savefig(f, dpi=110)
    plt.close(fig)
    return f


def _paths(plt, summ, present, trajs, out_dir, source):
    import matplotlib.patheffects as pe
    ks = [k for k in present if k in trajs]
    if not ks:
        return None
    ncol = 4
    nrow = int(math.ceil(len(ks) / ncol))
    fig, axs = plt.subplots(nrow, ncol, figsize=(18.5, 4.75 * nrow + 1.3), squeeze=False)
    fig.subplots_adjust(left=0.035, right=0.99, top=0.86 if nrow > 1 else 0.76, bottom=0.03, wspace=0.12,
                        hspace=0.28)
    one = 'seed 0' if source.startswith('Headless') else 'your Gazebo run'
    fig.suptitle(f'The fleet ({one}): where every drone really flew (its own colour) vs its reference route '
                 f'(green: A* on board before take-off)',
                 x=0.012, y=0.985, ha='left', fontsize=14, fontweight='bold', color=P.INK)
    fig.legend(handles=P.arena_legend_handles(flown=True), loc='upper left', bbox_to_anchor=(0.012, 0.955),
               ncol=6, fontsize=8.5)
    for ax, k in zip(axs.ravel(), ks):
        tr = trajs[k]
        P.draw_arena(ax)
        for d in C.DRONES:
            t_d = tr.get(d.id) if isinstance(tr, dict) else None
            if not t_d or not t_d.get('x'):
                continue
            ref = t_d.get('ref')
            if ref is not None:                       # the drone's own reference route (A* before take-off)
                ref = np.array(ref)
                ax.plot(ref[:, 0], ref[:, 1], color=P.IDEAL, lw=3.5, alpha=0.45, solid_capstyle='round', zorder=2)
            ax.plot(t_d['x'], t_d['y'], color=d.color, lw=1.6, zorder=4,
                    path_effects=[pe.Stroke(linewidth=2.6, foreground=P.SURFACE), pe.Normal()])
        s = summ[k]
        ax.set_title(f"{C.label_of(k)}\nmeans: path {M.fmt('fleet_cte_rms_mean_m', s.get('fleet_cte_rms_mean_m'))} m · "
                     f"position {M.fmt('fleet_loc_rms_mean_m', s.get('fleet_loc_rms_mean_m'))} m · "
                     f"fleet {M.fmt('fleet_energy_kj', s.get('fleet_energy_kj'))} kJ",
                     fontsize=9, pad=6)
        ax.set_xlabel('')
        ax.set_ylabel('')
        ax.tick_params(labelsize=7)
    for ax in axs.ravel()[len(ks):]:
        ax.axis('off')
    f = os.path.join(out_dir, 'study_paths.png')
    fig.savefig(f, dpi=100)
    plt.close(fig)
    return f


# ---------------------------------------------------------------------------
# the detailed final comparison
# ---------------------------------------------------------------------------
def _mix(c1, c2, t):
    from matplotlib.colors import to_rgb
    a, b = np.array(to_rgb(c1)), np.array(to_rgb(c2))
    return tuple(a + (b - a) * float(t))


def _comparison(plt, summ, present, base, out_dir, source, n_info):
    from matplotlib.gridspec import GridSpec
    from matplotlib.patches import Rectangle
    others = [k for k in present if k != C.BASELINE]
    # ---- rows of the heatmap: every metric with a direction, grouped by section
    rows = []                                             # (kind, key or section title)
    for sec, ms in M.SECTIONS:
        keys = [m[0] for m in ms if m[4] is not None and base.get(m[0]) is not None
                and any(summ[k].get(m[0]) is not None for k in others)]
        if keys:
            rows.append(('sec', sec.split(' (')[0].title()))
            rows += [('m', mk) for mk in keys]
    n_rows = len(rows) + 2                                # + summary row
    fig = plt.figure(figsize=(22, max(13.5, 0.36 * n_rows + 3.2)))
    gs = GridSpec(3, 3, figure=fig, left=0.165, right=0.962, top=0.895, bottom=0.05, wspace=0.42, hspace=0.42,
                  width_ratios=[2.25, 1, 1])
    fig.suptitle('Final comparison — every configuration against the baseline (3-point camera + VIO)',
                 x=0.012, y=0.985, ha='left', fontsize=15, fontweight='bold', color=P.INK)
    who = ('Each drone flies its fleet\'s mix' if any(k in C.FLEETS for k in others)
           else 'All 5 drones use the configuration')
    fig.text(0.012, 0.952, f'{source}: {n_info}. {who}; every metric covers the whole fleet (all 5 drones).',
             color=P.INK2, fontsize=10)
    fig.text(0.012, 0.932, 'Heatmap: change vs the baseline, coloured by what it means — blue = better, red = worse, '
             'pale = about the same. ' + _rule_text(summ) + ' Right: means with ±1 SD.',
             color=P.INK2, fontsize=10)
    if len(others) > 7:
        fig.text(0.012, 0.912, 'Columns: ' + P.CODE_KEY + '; k<n>: mixed fleet with n camera drones.'
                 if any(k in C.FLEETS for k in others) else 'Columns: ' + P.CODE_KEY + '.', color=P.INK2, fontsize=10)

    # ---- heatmap
    ax = fig.add_subplot(gs[:, 0])
    ax.set_xlim(-1.35, len(others))
    ax.set_ylim(n_rows - 0.5, -1.6)
    ax.axis('off')
    head_y = -1.0
    ax.text(-0.85, head_y, 'baseline\nvalue', ha='center', va='center', fontsize=8.5, color=P.INK2, fontweight='bold')
    compact = len(others) > 7                      # many columns: compact codes, explained in the subtitle
    for j, k in enumerate(others):
        ax.text(j, head_y, P.code(k) if compact else P.short_label(k), ha='center', va='center',
                fontsize=8.5 if compact else 8, color=P.INK, fontweight='bold')
    y = 0
    for kind, val in rows:
        if kind == 'sec':
            ax.text(-1.4, y + 0.1, val.upper(), ha='right', va='center', fontsize=8.5, fontweight='bold', color=P.INK)
            ax.plot([-1.35, len(others) - 0.5], [y + 0.45, y + 0.45], color=P.GRID, lw=0.8)
            y += 1
            continue
        mk = val
        _, lab, unit, _, _ = M.BY_KEY[mk]
        name = lab
        ax.text(-1.4, y, f'{name} ({unit})', ha='right', va='center', fontsize=8.2, color=P.INK2)
        ax.text(-0.85, y, M.fmt_mean(mk, base.get(mk)), ha='center', va='center', fontsize=8.2, color=P.INK)
        for j, k in enumerate(others):
            v = summ[k].get(mk)
            vd = _verdict(summ, k, mk)
            ch = M.change(v, base.get(mk))
            if v is None:
                fc, txt, tc = P.SURFACE, '–', P.MUTED
            else:
                txt = M.fmt_change(v, base.get(mk), mk)
                if vd in ('better', 'worse') and ch is not None:
                    t = 0.25 + 0.75 * (1.0 - math.exp(-abs(ch) / 40.0))
                    fc = _mix(P.NEUTRAL, P.BETTER if vd == 'better' else P.WORSE, t)
                    tc = '#ffffff' if t > 0.62 else P.INK
                elif vd in ('better', 'worse'):            # baseline zero -> no %: full colour
                    fc, tc = (P.BETTER if vd == 'better' else P.WORSE), '#ffffff'
                else:
                    fc, tc = P.NEUTRAL, P.MUTED
            ax.add_patch(Rectangle((j - 0.47, y - 0.43), 0.94, 0.86, facecolor=fc, edgecolor=P.SURFACE, lw=1.5))
            ax.text(j, y, txt, ha='center', va='center', fontsize=8, color=tc)
        y += 1
    # summary row
    y += 0.3
    ax.text(-1.4, y, 'Metrics better / worse', ha='right', va='center', fontsize=8.8, fontweight='bold', color=P.INK)
    for j, k in enumerate(others):
        nb, _, nw = _counts(summ, k)
        ax.text(j, y, f'{nb} ▲\n{nw} ▼' if compact else f'{nb} ▲  {nw} ▼', ha='center', va='center',
                fontsize=8.5 if compact else 9, fontweight='bold', linespacing=1.1,
                color=P.BETTER if nb > nw else P.WORSE if nw > nb else P.INK2)

    # ---- key metrics with spread
    panels = [('fleet_energy_kj', 'Flight energy, all 5 drones', 'kJ', False),
              ('fleet_flight_time_s', 'Flight time, all 5 drones', 's', False),
              ('fleet_cte_rms_mean_m', 'Path deviation RMS, mean of the drones', 'm, log', True),
              ('fleet_loc_rms_mean_m', 'Position error RMS, mean of the drones', 'm, log', True),
              ('fleet_landing_err_mean_m', 'Landing error, mean', 'm, log', True),
              ('fleet_mission_time_s', 'Mission time (until the last drone landed)', 's', False)]
    ys = np.arange(len(present))[::-1]
    for idx, (mk, title, unit, logx) in enumerate(panels):
        a = fig.add_subplot(gs[idx // 2, 1 + idx % 2])
        bv = base.get(mk)
        # linear panels: a far-off configuration (e.g. swarm only) is marked at the edge, not allowed
        # to squash all the others
        far = set()
        if not logx and bv:
            far = {k for k in present if summ[k].get(mk) is not None and summ[k][mk] > 1.35 * bv}
            near = [k for k in present if k not in far and summ[k].get(mk) is not None]
            lo_x = min(summ[k][mk] - (summ[k].get(mk + '_sd') or 0) for k in near)
            hi_x = max(summ[k][mk] + (summ[k].get(mk + '_sd') or 0) for k in near)
            pad = 0.12 * max(hi_x - lo_x, 0.05 * bv)
            a.set_xlim(lo_x - pad, hi_x + pad * (3.0 if far else 1.0))
        for yi, k in zip(ys, present):
            s = summ[k]
            v, sd = s.get(mk), s.get(mk + '_sd') or 0.0
            if v is None:
                continue
            isb = k == C.BASELINE
            c = P.INK if isb else VCOL[_verdict(summ, k, mk)]
            if k in far:
                xr = a.get_xlim()[1]
                a.scatter([xr], [yi], s=60, color=c, marker='>', zorder=4, clip_on=False)
                a.annotate('off scale', (xr, yi), xytext=(-8, 0), textcoords='offset points', ha='right',
                           va='center', fontsize=7, color=P.MUTED)
            else:
                lo = min(sd, v * 0.95) if logx else sd
                a.errorbar([v], [yi], xerr=[[lo], [sd]], fmt='none', ecolor=c, elinewidth=1.4, alpha=0.55, zorder=2)
                a.scatter([v], [yi], s=70 if isb else 48, color=c, marker='D' if isb else 'o', edgecolor=P.SURFACE,
                          linewidth=1.5, zorder=4)
            a.annotate(M.fmt(mk, v), (1.0, yi), xycoords=('axes fraction', 'data'), xytext=(5, 0),
                       textcoords='offset points', va='center', fontsize=7.5, color=P.INK2)
        if bv is not None:
            a.axvline(bv, color=P.INK, lw=0.8, ls=(0, (4, 3)), alpha=0.6, zorder=1)
        if logx and not _log_or_linear(a, [summ[k].get(mk) for k in present]):
            unit = unit.replace(', log', '')
        a.set_yticks(ys)
        a.set_yticklabels([k for k in present], fontsize=8)
        for lbl, k in zip(a.get_yticklabels(), present):
            if k == C.BASELINE:
                lbl.set_fontweight('bold')
        a.set_ylim(-0.7, len(present) - 0.3)
        a.grid(axis='y', visible=False)
        a.set_title(title, fontsize=10.5, pad=6)
        a.set_xlabel(unit, fontsize=8.5)
        a.tick_params(axis='x', labelsize=7.5)
    P.footer(fig, 'Dots: blue = better, red = worse, grey = about the same as the baseline (◆, dashed line). '
                  'Energy from the rotor power model on the air speed, incl. navigation electronics; accuracy from '
                  'ground truth. Full numbers: report.txt and summary.csv.')
    f = os.path.join(out_dir, 'study_comparison.png')
    fig.savefig(f, dpi=110)
    plt.close(fig)
    return f


def figures(summ, keys, trajs, out_dir, source, n_info=''):
    plt = P.setup()
    present = [k for k in keys if k in summ]
    base = summ.get(C.BASELINE)
    files = []
    others = [k for k in present if k != C.BASELINE]
    if base and others:
        files.append(_comparison(plt, summ, present, base, out_dir, source, n_info))
    files.append(_overview(plt, summ, present, base, out_dir, source, n_info))
    if base and others:
        files.append(_changes(plt, summ, others, base, out_dir))
    if trajs:
        f = _paths(plt, summ, present, trajs, out_dir, source)
        if f:
            files.append(f)
    return files
