"""The metric catalogue used by the terminal report, the dashboard and the study.

The study scores the WHOLE FLEET: the five drones are equal, and every metric covers all of them
(summed, averaged, or the worst drone, as the label says). The report also lists every drone.

Every metric is (key, label, unit, format, better) where better is
'lower', 'higher' or None (informational, no better/worse verdict).
fleet_metrics(drones, meta) computes them from the drones' summaries; extract(report) turns a
mission report (summary.json) into a flat dict.
"""
import math

SECTIONS = [
    ("ACCURACY (ground truth vs each drone's reference route: A* on board before take-off, green in Gazebo)", [
        ('fleet_goals_reached', 'Drones that reached their goal', '#/5', '{:.0f}', 'higher'),
        ('fleet_cte_rms_mean_m', 'Path deviation RMS (mean of the drones)', 'm', '{:.3f}', 'lower'),
        ('fleet_track_rms_mean_m', 'Tracking error RMS vs own plan (mean)', 'm', '{:.3f}', 'lower'),
        ('fleet_cte_max_m', 'Path deviation max (worst drone)', 'm', '{:.3f}', 'lower'),
        ('fleet_on_path_pct_mean', 'Cruise within ±0.5 m of route (mean)', '%', '{:.1f}', 'higher'),
        ('fleet_landing_err_mean_m', 'Landing error (mean)', 'm', '{:.3f}', 'lower'),
    ]),
    ('LOCALISATION (estimate vs truth)', [
        ('fleet_loc_rms_mean_m', 'Position error RMS (mean of the drones)', 'm', '{:.3f}', 'lower'),
        ('fleet_loc_max_m', 'Position error max (worst drone)', 'm', '{:.3f}', 'lower'),
        ('fleet_loc_final_mean_m', 'Position error at touchdown (mean)', 'm', '{:.3f}', 'lower'),
        ('fleet_loc_rms_nocam_mean_m', 'Position error RMS, drones without camera', 'm', '{:.3f}', 'lower'),
        ('fleet_sigma_mean_m', 'Reported uncertainty 1-sigma (mean)', 'm', '{:.3f}', None),
        ('fleet_consistency_mean', 'EKF consistency NEES/2 (mean; 1 = honest)', 'x', '{:.2f}', None),
        # a 2-point fix is easier to get than a 3-point fix, so these only describe each config
        ('fleet_cam_fix_pct_mean', 'Camera frames with an N-point fix (mean)', '%', '{:.1f}', None),
        ('fleet_cam_markers_visible_mean', 'Markers visible per frame (mean)', '#', '{:.2f}', None),
        ('fleet_longest_no_fix_s', 'Longest gap without camera fix (any drone)', 's', '{:.1f}', None),
        ('fleet_peer_updates', 'Swarm peer-range updates used', '#', '{:.0f}', None),
        ('fleet_meas_rejected', 'Camera/peer outliers rejected by the EKFs', '#', '{:.0f}', None),
    ]),
    ('ENERGY (all 5 drones)', [
        ('fleet_energy_kj', 'Flight energy, all drones', 'kJ', '{:.3f}', 'lower'),
        ('fleet_energy_per_m_j', 'Energy per metre flown', 'J/m', '{:.1f}', 'lower'),
        ('fleet_avg_power_w', 'Average power per drone', 'W', '{:.1f}', None),     # energy / time, already counted
        ('fleet_peak_power_w', 'Peak power (any drone)', 'W', '{:.1f}', 'lower'),
        ('fleet_nav_power_w', 'Navigation electronics power per drone', 'W', '{:.1f}', None),   # inside the energy
        ('fleet_battery_soc_min', 'Battery left (lowest drone)', '%', '{:.2f}', None),
    ]),
    ('TIME & PATH', [
        ('fleet_flight_time_s', 'Flight time, all drones', 's', '{:.1f}', 'lower'),
        ('fleet_mission_time_s', 'Mission time (until the last drone landed)', 's', '{:.1f}', 'lower'),
        ('fleet_distance_m', 'Distance flown, all drones', 'm', '{:.1f}', 'lower'),
        ('fleet_length_ratio_mean', 'Flown / reference route length (mean)', 'x', '{:.3f}', None),
        ('fleet_max_speed_ms', 'Max speed (any drone)', 'm/s', '{:.2f}', None),
        ('fleet_plans', 'Routes computed in flight (rolling)', '#', '{:.0f}', None),
        # wall clock of the on-board planners: depends on the computer (informational)
        ('fleet_plan_time_ms_mean', 'Planning time per route (on board)', 'ms', '{:.1f}', None),
        ('fleet_plan_time_ms_max', 'Longest planning time', 'ms', '{:.0f}', None),
    ]),
    ('AUTONOMY & SAFETY', [
        ('fleet_replans', 'Route switches in flight (re-plans)', '#', '{:.0f}', None),
        ('fleet_stuck_events', 'Stuck events', '#', '{:.0f}', 'lower'),
        ('fleet_escapes', 'Fail-safe escapes', '#', '{:.0f}', 'lower'),
        ('fleet_avoid_time_s', 'Reactive drone avoidance', 's', '{:.1f}', 'lower'),
        ('fleet_safety_interventions', 'Safety-filter interventions', '#', '{:.0f}', 'lower'),
        ('fleet_contacts', 'Obstacle contacts', '#', '{:.0f}', 'lower'),
        ('fleet_drone_contacts', 'Drone-to-drone contacts', '#', '{:.0f}', 'lower'),
        ('fleet_min_separation_m', 'Closest same-altitude pair', 'm', '{:.2f}', 'higher'),
        ('fleet_min_separation_3d_m', 'Closest pair (3-D, en route)', 'm', '{:.2f}', 'higher'),
        ('fleet_min_sep_margin_m', 'Separation margin (downwash-aware)', 'm', '{:.2f}', 'higher'),
        ('fleet_wake_time_s', "Time in another drone's downwash", 's', '{:.1f}', 'lower'),
        ('fleet_wake_max_ms', 'Strongest downdraft felt', 'm/s', '{:.2f}', 'lower'),
        ('fleet_unknown_known_mean', "Hidden pines in each drone's map (mean)", '/6', '{:.1f}', 'higher'),
    ]),
    ('COORDINATION (give-way, drones with the swarm radio)', [
        ('fleet_yield_time_s', 'Time giving way (all drones)', 's', '{:.1f}', None),
        ('fleet_give_way_hold_s', '  of it holding position', 's', '{:.1f}', None),
        ('fleet_give_way_slow_s', '  of it slowing down', 's', '{:.1f}', None),
        ('fleet_give_way_left_s', '  of it stepping left', 's', '{:.1f}', None),
        ('fleet_give_way_right_s', '  of it stepping right', 's', '{:.1f}', None),
        ('fleet_give_way_up_s', '  of it climbing', 's', '{:.1f}', None),
        ('fleet_give_way_down_s', '  of it descending', 's', '{:.1f}', None),
        ('fleet_ttc_trades', 'Turn-trading cycles executed', '#', '{:.0f}', None),
    ]),
]

ALL = [m for _, ms in SECTIONS for m in ms]
BY_KEY = {m[0]: m for m in ALL}

# fleet metric <- (how, the drones' summary key): summed, averaged, or the worst drone's value
AGG = {
    'fleet_cte_rms_mean_m': ('mean', 'cte_rms_m'), 'fleet_track_rms_mean_m': ('mean', 'track_rms_m'),
    'fleet_cte_max_m': ('max', 'cte_max_m'), 'fleet_on_path_pct_mean': ('mean', 'on_path_pct'),
    'fleet_landing_err_mean_m': ('mean', 'landing_err_m'),
    'fleet_loc_rms_mean_m': ('mean', 'loc_rms_m'), 'fleet_loc_max_m': ('max', 'loc_max_m'),
    'fleet_loc_final_mean_m': ('mean', 'loc_final_m'), 'fleet_sigma_mean_m': ('mean', 'sigma_mean_m'),
    'fleet_consistency_mean': ('mean', 'consistency'), 'fleet_cam_fix_pct_mean': ('mean', 'cam_fix_pct'),
    'fleet_cam_markers_visible_mean': ('mean', 'cam_markers_visible_mean'),
    'fleet_longest_no_fix_s': ('max', 'longest_no_fix_s'), 'fleet_peer_updates': ('sum', 'peer_updates'),
    'fleet_meas_rejected': ('sum', 'meas_rejected'),
    'fleet_energy_kj': ('sum', 'energy_kj'), 'fleet_peak_power_w': ('max', 'peak_power_w'),
    'fleet_nav_power_w': ('mean', 'nav_power_w'), 'fleet_battery_soc_min': ('min', 'battery_soc'),
    'fleet_flight_time_s': ('sum', 'flight_time_s'), 'fleet_distance_m': ('sum', 'horizontal_distance_m'),
    'fleet_length_ratio_mean': ('mean', 'length_ratio'), 'fleet_max_speed_ms': ('max', 'max_speed_ms'),
    'fleet_plans': ('sum', 'plans'), 'fleet_plan_time_ms_max': ('max', 'plan_time_ms_max'),
    'fleet_replans': ('sum', 'replans'), 'fleet_stuck_events': ('sum', 'stuck_events'),
    'fleet_escapes': ('sum', 'escapes'), 'fleet_avoid_time_s': ('sum', 'avoid_time_s'),
    'fleet_safety_interventions': ('sum', 'safety_interventions'), 'fleet_contacts': ('sum', 'contacts'),
    'fleet_min_separation_m': ('min', 'min_separation_m'), 'fleet_min_separation_3d_m': ('min', 'min_separation_3d_m'),
    'fleet_min_sep_margin_m': ('min', 'min_sep_margin_m'), 'fleet_wake_time_s': ('sum', 'wake_time_s'),
    'fleet_wake_max_ms': ('max', 'wake_max_ms'), 'fleet_unknown_known_mean': ('mean', 'unknown_known'),
    'fleet_yield_time_s': ('sum', 'yield_time_s'), 'fleet_ttc_trades': ('sum', 'ttc_trades'),
}


def _agg(how, vals):
    v = [float(x) for x in vals if x is not None and not (isinstance(x, float) and not math.isfinite(x))]
    if not v:
        return None
    return {'sum': sum, 'max': max, 'min': min}[how](v) if how != 'mean' else sum(v) / len(v)


def fleet_metrics(drones, nav_of=None, mission_time=None, give_way_kinds=()):
    """Every fleet metric from the drones' summaries (each a dict as DroneAgent.summary() plus the
    world's per-drone safety scores). nav_of(drone) -> its NavConfig (for the no-camera mean)."""
    out = {k: _agg(how, [d.get(src) for d in drones]) for k, (how, src) in AGG.items()}
    out['fleet_goals_reached'] = sum(int(bool(d.get('goal_reached'))) for d in drones)
    out['fleet_drone_contacts'] = sum(int(d.get('drone_contacts') or 0) for d in drones) // 2   # each counted twice
    out['fleet_ttc_trades'] = round(out['fleet_ttc_trades'] or 0.0, 6)
    e = sum(d.get('energy_kj') or 0.0 for d in drones) * 1000.0
    dist = sum(d.get('horizontal_distance_m') or 0.0 for d in drones)
    ft = sum(d.get('flight_time_s') or 0.0 for d in drones)
    out['fleet_energy_per_m_j'] = e / dist if dist > 0 else None
    out['fleet_avg_power_w'] = e / ft if ft > 0 else None
    n = sum(d.get('plans') or 0 for d in drones)
    out['fleet_plan_time_ms_mean'] = (sum((d.get('plan_time_ms_mean') or 0.0) * (d.get('plans') or 0) for d in drones)
                                      / n) if n else None
    out['fleet_mission_time_s'] = mission_time
    if nav_of is not None:
        out['fleet_loc_rms_nocam_mean_m'] = _agg('mean', [d.get('loc_rms_m') for d in drones
                                                          if not nav_of(d).cam_points])
    for k in give_way_kinds:
        out[f'fleet_give_way_{k}_s'] = sum((d.get('give_way_s') or {}).get(k, 0.0) for d in drones)
    parts = {}
    for d in drones:
        for k, v in (d.get('energy_parts_kj') or {}).items():
            parts[k] = parts.get(k, 0.0) + v
    out['fleet_energy_parts_kj'] = parts
    return out


def _num(v):
    if v is None:
        return None
    if isinstance(v, bool):
        return float(v)
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def extract(rep):
    """Flat {metric_key: float|None} of the fleet metrics of a mission report."""
    return {key: _num(rep.get(key)) for key, *_ in ALL}


def change(value, base):
    """Relative change in % (None if not computable)."""
    if value is None or base is None:
        return None
    if abs(base) < 1e-9:
        return 0.0 if abs(value) < 1e-9 else None
    return 100.0 * (value - base) / abs(base)


def verdict(key, value, base, tol_pct=2.0, noise=0.0):
    """'better' / 'worse' / 'same' / None, using the metric's direction.
    A difference within tol_pct of the baseline, or within `noise` (2 x the standard
    error of the difference, when means of several runs are compared), counts as 'same'."""
    m = BY_KEY.get(key)
    if m is None or m[4] is None or value is None or base is None:
        return None
    if abs(value - base) <= max(min_diff(key, base, tol_pct), noise):
        return 'same'
    if m[4] == 'lower':
        return 'better' if value < base else 'worse'
    return 'better' if value > base else 'worse'


def noise_of(sd_a, n_a, sd_b, n_b):
    """2 x standard error of the difference of two means (about a 95 % band)."""
    if not n_a or not n_b or sd_a is None or sd_b is None:
        return 0.0
    return 2.0 * math.sqrt(sd_a ** 2 / n_a + sd_b ** 2 / n_b)


COUNT_UNITS = ('#', '#/5', '/6')

# metrics that cross zero (a margin) or are usually zero (downwash): changes are shown as absolute
# differences, and a difference smaller than this is 'about the same' (a % of a value near zero means nothing)
ABS_TOL = {'fleet_min_sep_margin_m': 0.05, 'fleet_wake_time_s': 0.05, 'fleet_wake_max_ms': 0.2}


def min_diff(key, base, tol_pct=2.0):
    """Smallest difference from the baseline that counts as a change."""
    return max(1e-9, abs(base) * tol_pct / 100.0, ABS_TOL.get(key, 0.0))


def is_count(key):
    """Whole-number metric of one run (contacts, re-plans, ...)."""
    return key in BY_KEY and BY_KEY[key][2] in COUNT_UNITS and BY_KEY[key][3] == '{:.0f}'


def _num_text(v, signed=False):
    """0.05 -> '0.05', 0.4 -> '0.40', 3 -> '3', 1.9 -> '1.9' (counts and their differences)."""
    s = '+' if signed else ''
    if abs(v - round(v)) < 1e-9:
        return format(round(v), s + 'd') if not signed or round(v) else '±0'
    return format(v, s + ('.2f' if abs(v) < 1 else '.1f'))


def fmt_mean(key, v):
    """Format a MEAN over runs: counts get decimals so 0.05 is not shown as 0."""
    if v is None:
        return '-'
    if key in BY_KEY and BY_KEY[key][3] == '{:.0f}':     # whole numbers per run (counts, goal 0/1)
        return _num_text(v)
    return fmt(key, v)


def fmt_change(value, base, key=None):
    """Change vs baseline as text: '±0%', '+12%', '×15' (one formatter everywhere).
    Small counts (baseline below 1, e.g. 0.05 contacts per run) get the absolute difference,
    because a % of almost nothing (+200 %) says little."""
    if value is not None and base is not None and key in ABS_TOL:
        d = value - base
        return '±0' if abs(d) < 0.005 else f'{d:+.2f}'
    if value is not None and base is not None and (abs(base) < 1e-9 or (is_count(key) and abs(base) < 1)):
        d = value - base
        return '±0' if abs(d) < 1e-9 else _num_text(d, signed=True)
    ch = change(value, base)
    if ch is None:
        return '-'
    if abs(ch) < 0.5:
        return '±0%'
    if abs(ch) >= 1000 and base:
        return f'×{value / base:.0f}'
    return f'{ch:+.0f}%'


MARK = {'better': '▲', 'worse': '▼', 'same': '=', None: ' '}


def fmt(key, v):
    if v is None:
        return '-'
    return BY_KEY[key][3].format(v) if key in BY_KEY else f'{v:.3g}'
