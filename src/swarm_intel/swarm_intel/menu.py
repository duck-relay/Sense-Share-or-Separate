#!/usr/bin/env python3
"""Interactive menu for the navigation study.   ros2 run swarm_intel menu

  1-12  watch one configuration in Gazebo (with the GUI); p / g switch the planner (RRT* / A*) and the
        give-way (turn trading / fixed priorities) for these runs and for the Gazebo study
  13    all studies: every headless study of the paper, then its figures (ros2 run swarm_intel reproduce)
  14    all in Gazebo: every configuration x 3 seeds without the GUI, the validation, the paper figures

Every single tool runs on its own too: ros2 run swarm_intel <study | planner_bench | batch | compare |
validate | paper_figures | selftest | ...> (README, sections 2-5).
"""
import os
import shutil
import signal
import subprocess
import sys

from . import config as C
from .paths import default_results_dir, latest_run, stamp_of
from .grid_planner import PLANNER_NAMES
from .ttc import GIVE_WAY_NAMES


def _ask(prompt):
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return 'q'


def _run_launch(cmd):
    """Run ros2 launch; on Ctrl+C give it time for an orderly shutdown (Gazebo, bridge, node)."""
    p = subprocess.Popen(cmd)
    while True:
        try:
            return p.wait()
        except KeyboardInterrupt:
            try:
                p.send_signal(signal.SIGINT)
                return p.wait(timeout=20)
            except KeyboardInterrupt:
                continue
            except subprocess.TimeoutExpired:
                p.kill()
                return p.wait()


def _describe(c):
    """Menu wording of a configuration: its sensing and coordination, the baseline marked."""
    parts = c.parts.replace('swarm', 'swarm radio')
    if c.layers:
        parts = parts.replace(' + fixed altitude layers', ', fixed altitude layers')
    if parts in ('VIO', 'swarm radio'):
        parts += ' only'
    parts = parts[0].upper() + parts[1:]
    return parts + (' (baseline)' if c.key == C.BASELINE else '')


def _when(run_dir):
    """'2026-10-16 12:55' from a run folder name."""
    s = stamp_of(run_dir)
    return f'{s[0:4]}-{s[4:6]}-{s[6:8]} {s[9:11]}:{s[11:13]}' if s else os.path.basename(run_dir)


def _run_tool(name, *args):
    """Run one tool of the package (study, batch, ...) in its own Python process, as `ros2 run swarm_intel
    <name>` does: whatever a study switches for itself (planner, give-way, camera, wind, forest) can never
    carry over to the next choice in this menu. Ctrl+C stops the tool and returns here."""
    cmd = [sys.executable, '-m', f'swarm_intel.{name}', *args]
    print('  $ ros2 run swarm_intel ' + ' '.join([name, *args]))
    p = subprocess.Popen(cmd)
    while True:
        try:
            return p.wait()
        except KeyboardInterrupt:           # the terminal sends Ctrl+C to the tool as well: wait for it
            try:
                return p.wait(timeout=20)
            except KeyboardInterrupt:
                continue
            except subprocess.TimeoutExpired:
                p.kill()
                return p.wait()


def main(argv=None):
    if os.environ.get('NO_COLOR'):
        from . import report as _R
        _R.USE_COLOR = False
    from .plots import CODES
    res = default_results_dir()
    n = len(C.NAV_CONFIGS)
    o_all, o_gz = n + 1, n + 2
    base_name = C.NAV[C.BASELINE].label.replace('Baseline: ', '')
    rule = '=' * 104
    planner, give_way = C.DEFAULT_PLANNER, C.DEFAULT_GIVE_WAY     # for the Gazebo runs (p / g switch them)
    other = {'rrtstar': 'astar', 'astar': 'rrtstar', 'ttc': 'priority', 'priority': 'ttc'}
    while True:
        variant = []                                 # launch / tool arguments of the chosen variant
        if planner != C.DEFAULT_PLANNER:
            variant.append(('planner', planner))
        if give_way != C.DEFAULT_GIVE_WAY:
            variant.append(('give_way', give_way))
        flags = [x for k_, v in variant for x in (f'--{k_.replace("_", "-")}', v)]
        print('\n' + rule)
        print(f"  {'Multi-Drone Navigation Study':<30}{'baseline: ' + base_name:>72}")
        gz = f'Gazebo: planner {PLANNER_NAMES[planner]}, give-way {GIVE_WAY_NAMES[give_way]}'
        print(f"  {'':<30}{gz:>72}")
        print(rule)
        print('\n  Watch one configuration in Gazebo  (all five drones fly it; the whole fleet is scored)')
        print('  Run 1 first: every later run is compared with the baseline flown with the same planner and give-way.\n')
        for i, c in enumerate(C.NAV_CONFIGS, 1):
            last = latest_run(res, c.key)
            when = f'last run {_when(last)}' if last else 'not run yet'
            print(f'  {i:>3}  {CODES.get(c.key, ""):<8}{_describe(c):<46}{c.key:<20}{when}')
        print('\n  All runs\n')
        rows = [(o_all, 'All studies', 'every headless study of the paper, then its figures, tables and numbers',
                 '~6-15 h'),
                (o_gz, 'All in Gazebo', f'the {n} configurations x 3 seeds in Gazebo without the GUI (p and g apply),',
                 '~1-3 h'),
                ('', '', 'then Gazebo vs headless (validation) and the paper figures', '')]
        for num, name, what, dur in rows:
            print(f'  {num!s:>3}  {name:<16}{what:<78}{dur}'.rstrip())
        print(f'\n    p  Planner in Gazebo: {PLANNER_NAMES[planner]} (switch to {PLANNER_NAMES[other[planner]]})')
        print(f'    g  Give-way in Gazebo: {GIVE_WAY_NAMES[give_way]} (switch to {GIVE_WAY_NAMES[other[give_way]]})')
        print('    q  Quit\n')
        ch = _ask('  Select: ')
        if ch in ('q', 'quit', 'exit'):
            return 0
        if ch == '':
            continue
        if ch == 'p':
            planner = other[planner]
            continue
        if ch == 'g':
            give_way = other[give_way]
            continue
        if not ch.isdigit():
            print('  Invalid choice.')
            continue
        k = int(ch)
        try:
            if 1 <= k <= n:
                c = C.NAV_CONFIGS[k - 1]
                cmd = ['ros2', 'launch', 'swarm_intel', 'swarm_mission.launch.py', f'nav:={c.key}']
                cmd += [f'{k_}:={v}' for k_, v in variant]
                if shutil.which('xterm'):
                    cmd.append('dashboard:=xterm')
                    print('  The live dashboard opens in a separate window.')
                else:
                    print('  For the live dashboard, run in a second terminal:  ros2 run swarm_intel dashboard')
                print('  Gazebo starts paused. Press Play to begin: every drone plans its reference route (green, A*)')
                print(f'  on its pad, then flies, re-planning with {PLANNER_NAMES[planner]} every second; the route it has')
                print('  flown is drawn behind it in its own colour. When all drones have landed, press Ctrl+C to return.')
                print('  $ ' + ' '.join(cmd))
                _run_launch(cmd)
            elif k == o_all:
                _run_tool('reproduce')
            elif k == o_gz:
                if _run_tool('gazebo_study', '--seeds', '3', '--validate', *flags) == 0 and not variant:
                    _run_tool('paper_figures', '--results', res)     # adds the Gazebo check to the paper
            else:
                print('  Invalid choice.')
        except KeyboardInterrupt:
            print('\n  Interrupted. Returning to the menu.')


if __name__ == '__main__':
    sys.exit(main())
