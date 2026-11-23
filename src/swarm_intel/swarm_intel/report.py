"""Text formatting for the mission overview (terminal) and JSON helpers."""
import json
import math

import numpy as np

USE_COLOR = True
C_RESET = '\033[0m'
C_BOLD = '\033[1m'
C_BLUE = '\033[38;5;39m'
C_RED = '\033[38;5;203m'
C_GREEN = '\033[38;5;77m'
C_YELLOW = '\033[38;5;220m'
C_CYAN = '\033[38;5;51m'
C_GREY = '\033[38;5;245m'


def col(s, c):
    return f'{c}{s}{C_RESET}' if USE_COLOR else str(s)


def to_jsonable(o):
    if isinstance(o, dict):
        return {str(k): to_jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [to_jsonable(v) for v in o]
    if isinstance(o, (np.floating,)):
        v = float(o)
        return None if not math.isfinite(v) else v
    if isinstance(o, float):
        return None if not math.isfinite(o) else o
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return to_jsonable(o.tolist())
    return o


def dumps(o, **kw):
    return json.dumps(to_jsonable(o), **kw)


def _v(x, fmt='{:.2f}', none='  -  '):
    if x is None:
        return none
    try:
        if isinstance(x, float) and not math.isfinite(x):
            return none
    except TypeError:
        pass
    return fmt.format(x)


def _verdict_cell(v):
    if v == 'better':
        return col('▲ better', C_GREEN)
    if v == 'worse':
        return col('▼ worse', C_RED)
    if v == 'same':
        return col('= same', C_GREY)
    return ''


def comparison_notes(rep, baseline):
    """Why a comparison with this baseline may not be like-for-like."""
    from . import config as C
    notes = []
    if rep.get('model_version') != C.MODEL_VERSION:
        notes.append('this run was made with an older version of the package')
    if baseline:
        if baseline.get('model_version') != C.MODEL_VERSION:
            notes.append('the baseline run was made with an older version of the package')
        if baseline.get('give_way', 'priority') != rep.get('give_way', 'priority'):
            notes.append(f"different swarm give-way than the baseline ({rep.get('give_way', 'priority')} vs "
                         f"{baseline.get('give_way', 'priority')})")
        if baseline.get('planner', 'astar') != rep.get('planner', 'astar'):
            notes.append(f"different path planner than the baseline ({rep.get('planner', 'astar')} vs "
                         f"{baseline.get('planner', 'astar')})")
        if abs(float(baseline.get('speed_scale', 1.0)) - float(rep.get('speed_scale', 1.0))) > 1e-6:
            notes.append(f"different cruise speed than the baseline (x{rep.get('speed_scale', 1.0)} vs "
                         f"x{baseline.get('speed_scale', 1.0)})")
        if baseline.get('final') is False:
            notes.append('the baseline run stopped before all its drones landed (its fleet numbers are partial)')
    return notes


def format_report(rep, width=104, baseline=None):
    """Complete mission overview of the whole fleet, printed when every drone has landed.

    baseline: a report dict of a baseline run (3-point camera + VIO) -> adds
    'baseline' and 'change' columns with better / worse verdicts.
    """
    from . import metrics as M
    from . import config as C
    nav = C.NAV.get(rep.get('nav'))                    # current names, also for older result files
    label = nav.label if nav else (rep.get('nav_label') or rep.get('mode', '?'))
    parts = nav.parts if nav else rep.get('nav_parts', '')
    vals = M.extract(rep)
    base = M.extract(baseline) if baseline else None
    is_base = rep.get('nav') == C.BASELINE
    partial = rep.get('final') is False
    L = []
    bar = '═' * (width - 2)
    L.append(col('╔' + bar + '╗', C_CYAN))
    fleet = rep.get('fleet') or []
    n = len(fleet) or 5
    goals = int(rep.get('fleet_goals_reached') or 0)
    landed = sum(1 for f in fleet if f.get('state') == 'LANDED')
    failed = sum(1 for f in fleet if f.get('failed'))
    title = f" MISSION OVERVIEW — FLEET OF {n} — {goals} OF {n} AT THEIR GOAL — {label} "
    L.append(col('║', C_CYAN) + col(title.center(width - 2)[:width - 2], C_BOLD) + col('║', C_CYAN))
    L.append(col('╚' + bar + '╝', C_CYAN))
    ok = goals == n
    res = f'{goals} of {n} drones reached their goal, {landed} landed'
    if failed:
        res += f', {failed} gave up (timeout)'
    if partial:
        res += ' (the run was stopped early)'
    planner = {'rrtstar': 'RRT*', 'astar': 'A*'}.get(rep.get('planner', 'astar'), rep.get('planner'))
    L.append(f"  Navigation        : {parts}   (config '{rep.get('nav', rep.get('mode'))}', all 5 drones,"
             f" planner {planner}, give-way {rep.get('give_way', 'priority')}, wind {rep.get('wind_mean_ms', 0):.2f} m/s,"
             f" seed {rep.get('seed', 0)})")
    L.append(col(f"  Estimator         : one EKF per drone fuses IMU (gyro + heading) + ToF altimeter"
                 f"{' + ' + parts if parts and parts != 'none' else ''}", C_GREY))
    L.append(f"  Result            : {col(res, C_GREEN if ok else C_RED)}"
             f"   fleet energy {_v(rep.get('fleet_energy_kj'), '{:.2f}')} kJ,"
             f" mission time {_v(rep.get('fleet_mission_time_s'), '{:.1f}')} s")
    base_name = C.NAV[C.BASELINE].label.replace('Baseline: ', '')
    if is_base:
        L.append(col(f'  Baseline          : this run IS the baseline ({base_name})', C_GREY))
    elif baseline is None:
        L.append(col(f'  Baseline          : no baseline run found yet - run  nav:={C.BASELINE}  to get the comparison columns',
                     C_YELLOW))
    else:
        L.append(col(f"  Baseline          : {base_name}  ({baseline.get('_run', 'latest run')})", C_GREY))
    for n in comparison_notes(rep, None if is_base else baseline):
        L.append(col(f'  Note              : {n}', C_YELLOW))
    show_base = base is not None and not is_base
    verd = []
    for sec, ms in M.SECTIONS:
        L.append('')
        L.append(col(f"  {sec}{'  (partial: drones still flying)' if partial else ''}", C_BOLD))
        hdr = f"    {'metric':<44}{'unit':>6}{'this run':>12}"
        if show_base:
            hdr += f"{'baseline':>12}{'change':>10}   verdict"
        L.append(col(hdr, C_GREY))
        for key, lab, unit, f, _ in ms:
            v = vals.get(key)
            if v is None and (not show_base or base.get(key) is None):
                continue
            row = f"    {lab:<44}{unit:>6}{M.fmt(key, v):>12}"
            if show_base:
                bv = base.get(key)
                vd = None if partial else M.verdict(key, v, bv)
                verd.append(vd)
                row += f"{M.fmt(key, bv):>12}{M.fmt_change(v, bv, key):>10}   " + _verdict_cell(vd)
            L.append(row)
        if sec.startswith('ENERGY'):
            parts_e = rep.get('fleet_energy_parts_kj', {})
            if parts_e:
                tot = sum(parts_e.values()) or 1.0
                L.append(col('    energy split: ', C_GREY)
                         + ' | '.join(f"{k} {100 * v / tot:.0f}%" for k, v in parts_e.items()))
    if show_base:
        nb, nw = verd.count('better'), verd.count('worse')
        L.append('')
        L.append(f"  vs baseline: {col(str(nb) + ' metrics better', C_GREEN)}, {col(str(nw) + ' worse', C_RED)},"
                 f" {verd.count('same')} about the same")
    L.append('')
    L.append(col('  PER DRONE (the fleet metrics above cover all of them)', C_BOLD))
    L.append(col('    drone   config              state     energy kJ  dist m  path RMS m  landing m  loc RMS m  give-way s'
                 '  downwash s  contacts (obst/drone)', C_GREY))
    for f in fleet:
        L.append(f"    {f['name']:<7} {f.get('nav', ''):<19} {f['state']:<9} {f['energy_kj']:>9.2f}"
                 f"  {f['horizontal_distance_m']:>6.1f}  {_v(f['cte_rms_m']):>10}  {_v(f['landing_err_m']):>9}"
                 f"  {_v(f['loc_rms_m']):>9}  {f['yield_time_s']:>10.1f}  {f.get('wake_time_s', 0.0):>10.1f}"
                 f"  {f['contacts']:>8} / {f.get('drone_contacts', 0)}")
    L.append(col('─' * width, C_CYAN))
    return '\n'.join(L)


SUMMARY_KEYS = [('fleet_energy_kj', 'Flight energy, all drones (kJ)'), ('fleet_flight_time_s', 'Flight time, all drones (s)'),
                ('fleet_cte_rms_mean_m', 'Path deviation RMS, mean (m)'), ('fleet_loc_rms_mean_m', 'Position error RMS, mean (m)'),
                ('fleet_landing_err_mean_m', 'Landing error, mean (m)'), ('fleet_goals_reached', 'Drones at their goal (/5)'),
                ('fleet_contacts', 'Obstacle contacts'), ('fleet_drone_contacts', 'Drone-drone contacts'),
                ('fleet_min_sep_margin_m', 'Separation margin (m)'), ('fleet_wake_time_s', 'Time in downwash (s)')]


def format_summary(rep, baseline=None, width=104):
    """A dozen lines for the live dashboard: the key metrics of this run vs the baseline."""
    from . import metrics as M
    from . import config as C
    vals = M.extract(rep)
    base = M.extract(baseline) if baseline and rep.get('nav') != C.BASELINE else None
    n = len(rep.get('fleet') or []) or 5
    goals = int(rep.get('fleet_goals_reached') or 0)
    L = [col('━' * width, C_CYAN),
         col(f" MISSION COMPLETE - {goals} of {n} drones at their goal", C_BOLD + (C_GREEN if goals == n else C_RED))]
    hdr = f"   {'metric':<36}{'this run':>12}"
    if base:
        hdr += f"{'baseline':>12}{'change':>10}   verdict"
    L.append(col(hdr, C_GREY))
    partial = rep.get('final') is False
    for k, lab in SUMMARY_KEYS:
        row = f"   {lab:<36}{M.fmt(k, vals.get(k)):>12}"
        if base:
            vd = None if partial else M.verdict(k, vals.get(k), base.get(k))
            row += f"{M.fmt(k, base.get(k)):>12}{M.fmt_change(vals.get(k), base.get(k), k):>10}   " + _verdict_cell(vd)
        L.append(row)
    if base is None:
        L.append(col('   (this run is the baseline)' if rep.get('nav') == C.BASELINE
                     else f'   (no baseline yet - run nav:={C.BASELINE})', C_GREY))
    for n in comparison_notes(rep, baseline if base else None):
        L.append(col(f'   Note: {n}', C_YELLOW))
    return '\n'.join(L)
