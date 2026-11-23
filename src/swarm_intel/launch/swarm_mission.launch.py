"""Drone navigation study in Gazebo Fortress - one configuration per launch, used by ALL 5 drones.

Nodes: Gazebo, the ros_gz bridge, the world node (world_node: physics, scoring, drawing, report) and
ONE NODE PER DRONE (drone_node: the drone's on-board computer - EKF, map, A* / RRT* planning, turn
trading, control). Every route is planned on the drones while the mission runs; nothing is computed
beforehand. In flight every drone re-plans every second, in a planner process next to its flight loop.
Gazebo shows every drone's reference route (A* on the pad before take-off) in GREEN, from its pad up,
along the route and down to its landing pad, and the route it has actually flown as a trail in the
drone's own colour, drawn as it flies, from its take-off pad to its landing pad.

  ros2 launch swarm_intel swarm_mission.launch.py nav:=cam3_sensor  # BASELINE: 3-point camera + VIO
  ros2 launch swarm_intel swarm_mission.launch.py nav:=cam2         # 2-point camera
  ros2 launch swarm_intel swarm_mission.launch.py nav:=sensor       # VIO only
  ros2 launch swarm_intel swarm_mission.launch.py nav:=swarm        # swarm only
  ros2 launch swarm_intel swarm_mission.launch.py nav:=cam2_sensor
  ros2 launch swarm_intel swarm_mission.launch.py nav:=cam2_swarm
  ros2 launch swarm_intel swarm_mission.launch.py nav:=sensor_swarm
  ros2 launch swarm_intel swarm_mission.launch.py nav:=all          # 2-point camera + VIO + swarm

Arguments
  nav         see above (default: the baseline cam3_sensor)
  exit_when_done true | false       shut everything down when all drones have landed
  gui         true | false          false = headless server (fastest)
  paused      true | false | auto   start paused (auto = paused when gui:=true); press Play to start
  lines       true | false | auto   draw every drone's routes: green = reference route (A*, before
                                    take-off), drone colour = the route flown, drawn as it flies
                                    (auto = on when gui:=true)
  shadows     true | false          prettier, slower with software rendering
  seed        int                   noise seed (use the same seed for every configuration)
  planner     rrtstar | astar       path planner of every drone (default rrtstar = RRT*; astar = the
                                    A* comparison)
  give_way    ttc | priority        swarm give-way (default ttc = turn trading for the fleet's lowest
                                    energy, no priorities; priority = the fixed-priority comparison)
  speed       float                 cruise speed multiplier (1.0 = 1.2 m/s, every drone)
  results_dir path                  default: $SWARM_RESULTS_DIR, else the workspace results/
                                    folder (/media/sf_swarm_ws/results), else ~/swarm_results
  software_gl true | false          LIBGL_ALWAYS_SOFTWARE=1 for VirtualBox (default true)
  dashboard   none | xterm          open the dashboard in an xterm window (sudo apt install xterm)
  kill_stale  true | false          stop simulation processes left over from an earlier run first
                                    (a leftover Gazebo publishing the same topics ruins a run)
  study_run   true | false          set by the Gazebo study (gazebo_study): only its runs enter the
                                    Gazebo study and the validation; runs you watch stay separate

Every launch runs in its own Gazebo transport partition (IGN_PARTITION), so even a leftover
Gazebo server cannot feed this run's bridge.
"""
import os
import signal
import subprocess
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo,
                            RegisterEventHandler, EmitEvent, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


STALE = r'ign gazebo|ign-gazebo|gz sim|parameter_bridge.*swarm_bridge|lib/swarm_intel/(world|drone|mission)_node'


def _stale_processes():
    """(pid, command) of simulation processes still running from an earlier launch."""
    try:
        out = subprocess.run(['pgrep', '-af', STALE], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    mine = {os.getpid(), os.getppid()}
    procs = []
    for line in out.splitlines():
        pid, _, cmd = line.strip().partition(' ')
        if pid.isdigit() and int(pid) not in mine and 'pgrep' not in cmd:
            procs.append((int(pid), cmd))
    return procs


def _kill(procs):
    for sig, wait in ((signal.SIGINT, 3.0), (signal.SIGKILL, 1.0)):
        alive = []
        for pid, cmd in procs:
            try:
                os.kill(pid, sig)
                alive.append((pid, cmd))
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        if not alive:
            return
        time.sleep(wait)
        procs = [(pid, cmd) for pid, cmd in alive if os.path.exists(f'/proc/{pid}')]


def _setup(context, *args, **kwargs):
    from swarm_intel import config as C
    from swarm_intel.world_gen import generate_world

    nav = C.nav_config(LaunchConfiguration('nav').perform(context).strip().lower() or C.BASELINE)
    exit_when_done = _truthy(LaunchConfiguration('exit_when_done').perform(context))
    gui = _truthy(LaunchConfiguration('gui').perform(context))
    paused_arg = LaunchConfiguration('paused').perform(context).strip().lower()
    paused = gui if paused_arg in ('', 'auto') else _truthy(paused_arg)
    if paused and not gui:
        paused = False                   # without a GUI there is no Play button: never start paused
    lines_arg = LaunchConfiguration('lines').perform(context).strip().lower()
    lines = gui if lines_arg in ('', 'auto') else (_truthy(lines_arg) and gui)
    shadows = _truthy(LaunchConfiguration('shadows').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    speed = float(LaunchConfiguration('speed').perform(context))
    planner = LaunchConfiguration('planner').perform(context).strip().lower()
    give_way = LaunchConfiguration('give_way').perform(context).strip().lower()
    results_dir = LaunchConfiguration('results_dir').perform(context)
    software_gl = _truthy(LaunchConfiguration('software_gl').perform(context))
    dashboard = LaunchConfiguration('dashboard').perform(context).strip().lower()
    kill_stale = _truthy(LaunchConfiguration('kill_stale').perform(context))
    study_run = _truthy(LaunchConfiguration('study_run').perform(context))

    notes = []
    stale = _stale_processes()
    if stale:
        listing = '; '.join(f'{pid}: {cmd[:70]}' for pid, cmd in stale)
        if kill_stale:
            _kill(stale)
            notes.append(LogInfo(msg=f'[swarm_intel] stopped {len(stale)} simulation process(es) left over from an '
                                     f'earlier run: {listing}'))
        else:
            notes.append(LogInfo(msg=f'[swarm_intel] WARNING: {len(stale)} simulation process(es) from an earlier '
                                     f'run are still running ({listing}). Stop them (pkill -f "ign gazebo") or use '
                                     'kill_stale:=true.'))
    partition = f'swarm_intel_{os.getpid()}'

    out_dir = os.path.expanduser('~/.ros/swarm_intel')
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, f'swarm_arena_{nav.key}.world')
    with open(world_path, 'w') as f:
        f.write(generate_world(nav.key, shadows=shadows))

    gz_cmd = ['ign', 'gazebo', world_path]
    if not paused:
        gz_cmd.insert(2, '-r')
    if not gui:
        gz_cmd.insert(2, '-s')
    env = {'LIBGL_ALWAYS_SOFTWARE': '1'} if software_gl else {}

    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen', additional_env=env, name='gazebo')
    actions = notes + [
        # this run's own Gazebo transport partition: nothing else can publish into it
        SetEnvironmentVariable('IGN_PARTITION', partition),
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg=f'[swarm_intel] {nav.label} [{nav.parts}] for all 5 drones  planner={planner}  give_way={give_way}  world={world_path}'
                    f'  gui={gui}  paused={paused}  lines={lines}  partition={partition}'),
        gazebo,
        # closing the Gazebo window ends the whole run (bridge, world and drone nodes, dashboard)
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[
            LogInfo(msg='[swarm_intel] Gazebo closed - shutting down'), EmitEvent(event=Shutdown())])),
    ]
    if paused:
        actions.append(LogInfo(msg='[swarm_intel] Gazebo starts PAUSED: press the orange Play button (bottom-left) '
                                   'to start the mission. On the pad every drone plans its reference route with A* '
                                   '(GREEN); in flight the route it has flown is drawn behind it in its own colour.'))

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    for d in C.DRONES:
        bridge_args.append(f'/model/{d.name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist')
        bridge_args.append(f'/model/{d.name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry')
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='swarm_bridge',
                        arguments=bridge_args, output='screen'))

    world = Node(package='swarm_intel', executable='world_node', name='swarm_world', output='screen',
                 parameters=[{'use_sim_time': True, 'nav': nav.key, 'seed': seed, 'planner': planner,
                              'give_way': give_way, 'speed_scale': speed, 'results_dir': results_dir,
                              'exit_when_done': exit_when_done, 'lines': lines, 'gui': gui,
                              'study_run': study_run}])
    actions.append(world)
    for d in C.DRONES:                      # one on-board computer (edge node) per drone
        actions.append(Node(package='swarm_intel', executable='drone_node', name=d.name, output='screen',
                            parameters=[{'use_sim_time': True, 'drone_id': d.id, 'nav': nav.key, 'seed': seed,
                                         'planner': planner, 'give_way': give_way, 'speed_scale': speed}]))
    if exit_when_done:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=world,
            on_exit=[LogInfo(msg='[swarm_intel] mission finished - shutting down'), EmitEvent(event=Shutdown())])))
    if dashboard == 'xterm':
        actions.append(Node(package='swarm_intel', executable='dashboard', name='swarm_dashboard', output='screen',
                            prefix='xterm -fa Monospace -fs 10 -geometry 150x48 -sl 5000 -title SwarmDashboard -e'))
    return actions


def generate_launch_description():
    from swarm_intel import config as C
    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value=C.BASELINE,
                              description=' | '.join(c.key for c in C.NAV_CONFIGS)),
        DeclareLaunchArgument('exit_when_done', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='auto', description='auto = paused when gui:=true'),
        DeclareLaunchArgument('lines', default_value='auto',
                              description='draw every drone\'s routes (green: A* before take-off, drone colour: route flown); '
                                          'auto = with the GUI'),
        DeclareLaunchArgument('shadows', default_value='false'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('planner', default_value=C.PLANNER, description='rrtstar (RRT*) | astar (A*)'),
        DeclareLaunchArgument('give_way', default_value=C.GIVE_WAY, description='ttc (turn trading) | priority'),
        DeclareLaunchArgument('results_dir', default_value=''),
        DeclareLaunchArgument('software_gl', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='none', description='none | xterm'),
        DeclareLaunchArgument('kill_stale', default_value='true',
                              description='stop simulation processes left over from an earlier run first'),
        DeclareLaunchArgument('study_run', default_value='false',
                              description='set by gazebo_study: the run enters the Gazebo study and the validation'),
        OpaqueFunction(function=_setup),
    ])
