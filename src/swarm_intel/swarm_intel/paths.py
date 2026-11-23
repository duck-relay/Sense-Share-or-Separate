"""Where results are written (shared by the ROS nodes and the offline tools)."""
import glob
import json
import os
import re


def default_results_dir():
    """$SWARM_RESULTS_DIR, else <workspace>/results (/media/sf_swarm_ws or ~/swarm_ws), else ~/swarm_results."""
    env = os.environ.get('SWARM_RESULTS_DIR')
    if env:
        return env
    for base in ('/media/sf_swarm_ws', os.path.expanduser('~/swarm_ws')):
        if os.path.isdir(base) and os.access(base, os.W_OK):
            return os.path.join(base, 'results')
    return os.path.expanduser('~/swarm_results')


def stamp_of(path):
    """The YYYYmmdd_HHMMSS stamp in a run / study folder name ('' if none)."""
    m = re.search(r'\d{8}_\d{6}', os.path.basename(path.rstrip('/')))
    return m.group(0) if m else ''


def _is_run_of(d, key):
    """Folder <key>_<YYYYmmdd>_<HHMMSS> (so 'cam2' does not match 'cam2_sensor_...')."""
    name = os.path.basename(d.rstrip('/'))
    return re.fullmatch(re.escape(key) + r'_\d{8}_\d{6}', name) is not None


def _summary(d):
    try:
        with open(os.path.join(d, 'summary.json')) as f:
            return json.load(f)
    except Exception:
        return None


def runs_of(results_dir, key):
    """Gazebo runs of one configuration in which every drone landed, oldest first. Headless demo
    exports, runs stopped before the fleet had landed and runs marked invalid (two simulations at
    once) are ignored; runs of older package versions are kept (the reports flag them)."""
    out = []
    for d in glob.glob(os.path.join(results_dir, f'{key}_*')):
        if not _is_run_of(d, key):
            continue
        rep = _summary(d)
        if rep and rep.get('nav') == key and rep.get('source') == 'gazebo' and rep.get('all_landed_at_s') is not None \
                and not rep.get('invalid'):
            out.append(d)
    return sorted(out, key=stamp_of)


def study_run(rep):
    """Made by the Gazebo study (gazebo_study)? Only such runs enter the Gazebo study and the validation:
    a run watched from the menu (1-12, always seed 0) or launched by hand is not one, so watching never
    replaces a study run (with the GUI, Gazebo runs slower, which changes the drones' real planning delay
    in simulated time)."""
    return bool(rep and rep.get('study_run'))


def latest_run(results_dir, key, exclude=None):
    """Newest complete run (every drone landed) of a configuration, else the newest run."""
    runs = [d for d in runs_of(results_dir, key)
            if not exclude or os.path.abspath(d) != os.path.abspath(exclude)]
    final = [d for d in runs if (_summary(d) or {}).get('final')]
    return (final or runs)[-1] if runs else None


def load_report(run_dir):
    with open(os.path.join(run_dir, 'summary.json')) as f:
        rep = json.load(f)
    rep['_run'] = os.path.basename(run_dir.rstrip('/'))
    return rep


def baseline_report(results_dir, exclude=None, planner=None, give_way=None):
    """Latest Gazebo run of the baseline (3-point camera + VIO), slimmed for embedding. With planner /
    give_way: the latest one flown with them (else the latest one; the report then flags the difference)."""
    from . import config as C
    d = latest_run(results_dir, C.BASELINE, exclude=exclude)
    if planner or give_way:
        runs = [r for r in runs_of(results_dir, C.BASELINE)
                if not exclude or os.path.abspath(r) != os.path.abspath(exclude)]
        same = [r for r in runs if (not planner or (_summary(r) or {}).get('planner', 'astar') == planner)
                and (not give_way or (_summary(r) or {}).get('give_way', 'priority') == give_way)]
        final = [r for r in same if (_summary(r) or {}).get('final')]
        if same:
            d = (final or same)[-1]
    if not d:
        return None
    try:
        rep = load_report(d)
    except Exception:
        return None
    return {k: v for k, v in rep.items() if k not in ('events', 'ideal_path', 'baseline', 'fleet')}
