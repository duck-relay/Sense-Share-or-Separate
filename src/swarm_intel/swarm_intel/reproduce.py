"""Reproduce every headless result of the paper, then its figures, tables and numbers.

  ros2 run swarm_intel reproduce                    # everything (about 16 h on 2 CPU cores, 6.5 h on 8)
  ros2 run swarm_intel reproduce --only main,wind   # some steps only
  ros2 run swarm_intel reproduce --list             # what each step runs

Steps (in this order; each writes its own study_headless_<...>_<time> folder into results/):
  main     12 configurations x 10 forests x 10 seeds (Gazebo forest + 9 random forests)
  mixed    mixed fleets: k of 5 drones with a camera, with and without swarm radio, 10 x 6
  cam80    the single forward 80 deg camera: the 9 camera configurations, 10 x 8
  wind     wind sweep 0 / 0.5 / 1.0 / 1.5 m/s (0.2 m/s is the main study), 5 forests x 4 seeds
  robust   sensitivity of the coordination findings: downwash half as strong, layers 1.0 m and 1.5 m
           apart (baseline, fixed layers, swarm give-way; 10 x 6)
  speed    energy vs accuracy over 4 cruise speeds (Gazebo forest, 4 seeds)
  planner  the planner comparison: the drones plan with A* instead of RRT* (8 configurations,
           10 forests x 4 seeds, the same missions as in the main study)
  giveway  the give-way comparison: fixed priorities (in drone order) instead of turn trading
           (the baseline and the 6 radio configurations, 10 forests x 4 seeds, the main study's missions)
  controls robustness checks, 10 forests x 4 seeds each: the radio with lost, late and range-limited
           messages (turn trading timed to the link, and on its fixed schedule); the layers with drones seen
           up to 90 deg above/below and with the radio; a 0.5 m buffer on the reactive avoidance (also with
           the layers seen up to 90 deg); then
           `controls` pairs them with their references (controls_<time>/)
  scaling  planner scaling benchmark: RRT* vs A* on 20 / 40 / 80 m fields with 0.2 / 0.1 / 0.05 m grid
           cells, 10 forests each (planning only, no flight; planner_bench, about 15 min). Planning
           times are wall clock and depend on the CPU, unlike the missions.
  figures  paper/: figures, LaTeX tables and numbers.md (the newest folder of each study made with this
           model version; every folder records its version, settings and computer in meta.json)

The Gazebo check needs ROS 2 + Gazebo and is run separately (about 1 h):
  ros2 run swarm_intel gazebo_study --seeds 3 --validate
Run `reproduce --only figures` afterwards to add it to paper/.

Every mission is deterministic: on the same computer and setup it repeats exactly. The reference machine
is an Intel Core i7-10750H (VirtualBox VM, Python 3.10.12, numpy 1.26.4, OpenBLAS's generic Prescott
kernel, which the package selects on every x86-64 CPU); every results folder records its own computer
and software in meta.json. On another computer some missions can differ in their details, above all
those with the swarm radio: compare the statistics, not single missions (README, section 1).
"""
import argparse
import os
import subprocess
import sys
import time

from . import blas_kernel_note
from .paths import default_results_dir

WIND_CFGS = 'cam3_sensor,cam2_sensor,cam3_sensor_swarm,all,sensor,sensor_swarm,swarm,cam3_sensor_layers'
CAM80_CFGS = 'cam3_sensor,cam2,cam2_sensor,cam2_swarm,all,cam3,cam3_swarm,cam3_sensor_swarm,cam3_sensor_layers'
ROBUST_CFGS = 'cam3_sensor,cam3_sensor_layers,cam3_sensor_swarm'
SPEED_CFGS = 'cam3_sensor,cam2_sensor,cam3_sensor_swarm,all,cam3_sensor_layers'
PLANNER_CFGS = WIND_CFGS
GIVEWAY_CFGS = 'cam3_sensor,cam3_sensor_swarm,all,cam3_swarm,cam2_swarm,sensor_swarm,swarm'
# control studies of the robustness checks (10 forests x 4 seeds each; the baseline is added to every study):
# a reference with the layers + radio, the radio with lost / late / range-limited messages (turn trading timed
# to the link, and on its fixed schedule as in package 1.8.4), drones that see each other up to 90 deg above
# and below, and the reactive avoidance with a 0.5 m buffer
RADIO_LINKS = [('--radio-loss', '0.1'), ('--radio-loss', '0.3'), ('--radio-delay-ms', '100-250'),
               ('--radio-loss', '0.2', '--radio-delay-ms', '50-200', '--radio-range', '8')]
CONTROL_RUNS = ([('--configs', 'cam3_sensor,cam3_sensor_swarm,cam3_sensor_layers,cam3_sensor_layers_swarm')]
                + [('--configs', 'cam3_sensor_swarm') + link + fx for fx in ((), ('--ttc-fixed',)) for link in RADIO_LINKS]
                + [('--configs', 'cam3_sensor_layers', '--drone-view-deg', '90'),
                   ('--configs', 'cam3_sensor_layers', '--reactive-buffer', '0.5'),
                   ('--configs', 'cam3_sensor_layers', '--drone-view-deg', '90', '--reactive-buffer', '0.5')])


def steps(res, workers):
    w = ['--workers', str(workers)] if workers else []
    r = ['--results', res, '--no-color']
    study = ('study', )
    return [
        ('main', [study + tuple(['--layouts', '10', '--seeds', '10'] + r + w)], 1200),
        ('mixed', [study + tuple(['--mixed', '--layouts', '10', '--seeds', '6'] + r + w)], 720),
        ('cam80', [study + tuple(['--cam-fov', '80', '--layouts', '10', '--seeds', '8', '--configs', CAM80_CFGS] + r + w)],
         720),
        ('wind', [study + tuple(['--wind', str(v), '--layouts', '5', '--seeds', '4', '--configs', WIND_CFGS] + r + w)
                  for v in (0, 0.5, 1.0, 1.5)], 640),
        ('robust', [study + tuple(['--downwash-scale', '0.5', '--layouts', '10', '--seeds', '6', '--configs', ROBUST_CFGS]
                                  + r + w)]
                   + [study + tuple(['--layer-gap', str(g), '--layouts', '10', '--seeds', '6', '--configs', ROBUST_CFGS]
                                    + r + w) for g in (1.0, 1.5)], 540),
        ('speed', [('batch', '--seeds', '4', '--speeds', '0.7,1.0,1.3,1.6', '--configs', SPEED_CFGS,
                    '--out', os.path.join(res, 'speed_sweep'), *w)], 80),
        ('planner', [study + tuple(['--planner', 'astar', '--layouts', '10', '--seeds', '4', '--configs', PLANNER_CFGS]
                                   + r + w)], 320),
        ('giveway', [study + tuple(['--give-way', 'priority', '--layouts', '10', '--seeds', '4', '--configs',
                                    GIVEWAY_CFGS] + r + w)], 280),
        ('controls', [study + tuple(['--layouts', '10', '--seeds', '4'] + list(x) + r + w) for x in CONTROL_RUNS]
                     + [('controls', '--results', res)], 1040),
        ('scaling', [('planner_bench', '--results', res)], 0),
        ('figures', [('paper_figures', '--results', res)], 0),
    ]


def _run(cmd):
    """Each step in its own Python process: no setting of one study can leak into the next."""
    name, args = cmd[0], list(cmd[1:])
    print(f'\n$ ros2 run swarm_intel {name} ' + ' '.join(args), flush=True)
    rc = subprocess.call([sys.executable, '-m', f'swarm_intel.{name}', *args])
    if rc != 0:
        raise SystemExit(f'{name} failed (exit code {rc})')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', default='', help='results folder (default: the workspace results/)')
    ap.add_argument('--workers', type=int, default=0, help='parallel processes (default: all CPU cores)')
    ap.add_argument('--only', default='', help='comma-separated steps, e.g. main,figures')
    ap.add_argument('--list', action='store_true', help='print the commands and exit')
    a = ap.parse_args(argv)
    res = a.results or default_results_dir()
    plan = steps(res, a.workers)
    names = [s[0] for s in plan]
    want = [x.strip() for x in a.only.split(',') if x.strip()] or names
    bad = [x for x in want if x not in names]
    if bad:
        ap.error(f'unknown step(s) {", ".join(bad)}; choose from {", ".join(names)}')
    todo = [s for s in plan if s[0] in want]
    total = sum(n for _, _, n in todo)
    print(f'Reproducing: {", ".join(s[0] for s in todo)}  ({total} missions) -> {res}')
    print('  ' + blas_kernel_note())
    if a.list:
        for name, cmds, n in todo:
            for c in cmds:
                print(f'  [{name}] ros2 run swarm_intel {c[0]} ' + ' '.join(c[1:]))
        return 0
    t0 = time.time()
    for name, cmds, n in todo:
        print(f'\n=== {name} ({n} missions) - {time.strftime("%H:%M:%S")} ===', flush=True)
        for c in cmds:
            _run(c)
    print(f'\nDone in {(time.time() - t0) / 3600:.1f} h. Paper material: {os.path.join(res, "paper")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
