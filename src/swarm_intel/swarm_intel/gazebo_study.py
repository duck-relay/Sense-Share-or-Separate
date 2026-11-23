#!/usr/bin/env python3
"""Run every navigation configuration in REAL Gazebo, no GUI, one after another,
show each result in the terminal, then print the combined comparison.
In every run all 5 drones use the configuration; the whole fleet is scored.

  ros2 run swarm_intel gazebo_study                      # all 12 configs, seed 0
  ros2 run swarm_intel gazebo_study --seeds 3            # seeds 0, 1, 2 of every config (for the paper)
  ros2 run swarm_intel gazebo_study --seeds 3 --validate # ... then compare each run with its headless twin
  ros2 run swarm_intel gazebo_study --configs cam3_sensor,cam2,all
  ros2 run swarm_intel gazebo_study --gui                # watch each run (slower)
  ros2 run swarm_intel gazebo_study --planner astar      # the A* comparison instead of RRT*
  ros2 run swarm_intel gazebo_study --give-way priority  # the fixed-priority give-way instead of turn trading

Each run is a normal  ros2 launch swarm_intel swarm_mission.launch.py nav:=<cfg> seed:=<s>
gui:=false exit_when_done:=true study_run:=true  - its console output goes to
<results>/gazebo_logs/<cfg>_seed<s>.log. Afterwards this calls  study --gazebo  (and
validate with --validate). Study runs already made with this package version are skipped,
so an interrupted study can simply be started again. Only study runs (study_run:=true) enter the
Gazebo study and the validation: a run watched from the menu (always seed 0) never replaces one.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

from . import config as C
from . import report as R
from .report import col, format_report
from .paths import default_results_dir, load_report, baseline_report, runs_of, stamp_of, study_run
from .grid_planner import PLANNERS
from .ttc import GIVE_WAYS


def _kill_group(p):
    """Stop the whole process group of a launch (Gazebo server, bridge, node), even if
    the launch process itself has already exited (its group id is its pid)."""
    for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return                                   # nothing left in the group
        except Exception:
            pass
        try:
            p.wait(timeout=wait)
        except Exception:
            pass


def main(argv=None):
    """Runs with the chosen planner and give-way, then switches back to the study's own settings."""
    try:
        return _main(argv)
    finally:
        C.PLANNER, C.GIVE_WAY = C.DEFAULT_PLANNER, C.DEFAULT_GIVE_WAY


def _main(argv=None):
    if os.environ.get('NO_COLOR'):
        R.USE_COLOR = False
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--configs', default=','.join(c.key for c in C.NAV_CONFIGS))
    ap.add_argument('--seed', type=int, default=0, help='first seed')
    ap.add_argument('--seeds', type=int, default=1, help='number of seeds per configuration')
    ap.add_argument('--validate', action='store_true', help='afterwards compare every run with its headless twin')
    ap.add_argument('--redo', action='store_true', help='run again even if a run of this version exists')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--gui', action='store_true', help='show the Gazebo window for every run')
    ap.add_argument('--timeout', type=float, default=1200.0, help='wall-clock seconds per run before giving up')
    ap.add_argument('--planner', choices=PLANNERS, default=C.DEFAULT_PLANNER, help='path planner of every drone')
    ap.add_argument('--give-way', choices=GIVE_WAYS, default=C.DEFAULT_GIVE_WAY, help='swarm give-way')
    ap.add_argument('--results', default='')
    a = ap.parse_args(argv)
    C.PLANNER = a.planner
    C.GIVE_WAY = a.give_way
    keys = list(dict.fromkeys(C.nav_config(k).key for k in a.configs.split(',') if k.strip()))
    if C.BASELINE in keys:                       # baseline first, so every later run can compare against it
        keys.remove(C.BASELINE)
        keys.insert(0, C.BASELINE)
    res = a.results or default_results_dir()
    logs = os.path.join(res, 'gazebo_logs')
    os.makedirs(logs, exist_ok=True)
    seeds = list(range(a.seed, a.seed + max(1, a.seeds)))
    jobs = [(k, sd) for sd in seeds for k in keys]          # every config of a seed, baseline first
    print(col(f'Gazebo study: {len(keys)} configurations x {len(seeds)} seed(s) = {len(jobs)} runs (all 5 drones '
              f'each), {"GUI" if a.gui else "no GUI"}. Logs: {logs}', R.C_BOLD))
    done = []
    for n, (k, sd) in enumerate(jobs, 1):
        nav = C.NAV[k]
        if not a.redo and _have_run(res, k, sd, a.speed):
            print(col(f'\n[{n}/{len(jobs)}] {nav.label}, seed {sd}: already done with this version - skipped', R.C_GREY))
            done.append(k)
            continue
        before = set(runs_of(res, k))
        cmd = ['ros2', 'launch', 'swarm_intel', 'swarm_mission.launch.py', f'nav:={k}', f'seed:={sd}',
               f'speed:={a.speed}', f'planner:={a.planner}', f'give_way:={a.give_way}',
               f'gui:={"true" if a.gui else "false"}', 'paused:=false',
               'exit_when_done:=true', 'study_run:=true', f'results_dir:={res}']
        print(col(f'\n[{n}/{len(jobs)}] {nav.label}  ({nav.parts}), seed {sd}', R.C_CYAN + R.C_BOLD))
        print(col('    ' + ' '.join(cmd), R.C_GREY))
        t0 = time.time()
        with open(os.path.join(logs, f'{k}_seed{sd}.log'), 'w') as log:
            p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                while p.poll() is None:
                    time.sleep(2.0)
                    el = time.time() - t0
                    print(f'\r    running ... {el:5.0f} s wall', end='', flush=True)
                    if el > a.timeout:
                        print(col(f'\n    timeout after {a.timeout:.0f} s - stopping this run', R.C_RED))
                        _kill_group(p)
                        break
            except KeyboardInterrupt:
                print(col('\n    interrupted - stopping', R.C_YELLOW))
                _kill_group(p)
                return 130
        _kill_group(p)                           # make sure no Gazebo server is left behind
        time.sleep(2.0)
        new = sorted(set(runs_of(res, k)) - before, key=stamp_of)
        after = new[-1] if new else None
        if not after:
            print(col(f'\n    no usable result (not every drone landed, or nothing was written) - see '
                      f'{logs}/{k}_seed{sd}.log', R.C_RED))
            continue
        rep = load_report(after)
        done.append(k)
        print(f'\n    finished in {time.time() - t0:.0f} s wall  ->  {after}')
        print(format_report(rep, baseline=baseline_report(res, exclude=after, planner=a.planner,
                                                         give_way=a.give_way)))
    if not done:
        print(col('No run finished. Check the logs above (is the workspace sourced? does Gazebo start?).', R.C_RED))
        return 1
    print(col('\nAll runs done - combined comparison of the Gazebo runs of every configuration:', R.C_BOLD))
    from . import study
    rc = study.main(['--gazebo', '--results', res, '--configs', ','.join(keys), '--planner', a.planner,
                     '--give-way', a.give_way])
    if a.validate:
        from . import validate
        rc = validate.main(['--results', res, '--configs', ','.join(keys), '--planner', a.planner,
                            '--give-way', a.give_way])
    return rc


def _have_run(res, k, seed, speed):
    """A finished Gazebo-study run of this package version, planner and give-way with this seed and speed exists
    (runs watched from the menu do not count)."""
    for d in runs_of(res, k):
        try:
            rep = load_report(d)
        except Exception:
            continue
        if (rep.get('model_version') == C.MODEL_VERSION and rep.get('planner', 'astar') == C.PLANNER
                and rep.get('give_way', 'priority') == C.GIVE_WAY and int(rep.get('seed', -1)) == seed
                and abs(float(rep.get('speed_scale', 1.0)) - speed) < 1e-6 and rep.get('final') and study_run(rep)):
            return True
    return False


if __name__ == '__main__':
    sys.exit(main())
