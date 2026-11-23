"""
Scenario + model configuration for the navigation study.

Everything the simulation needs lives here, so the Gazebo world, the ideal
path, the ROS mission node and the headless batch runner all agree.
Edit values here, re-build (colcon build) and re-launch.
"""
from dataclasses import dataclass
from typing import List, Tuple

MODEL_VERSION = '2026.11-fleet.3'   # written into every result; runs of other versions are flagged in comparisons

# ---------------------------------------------------------------------------
# Arena
# ---------------------------------------------------------------------------
ARENA_HALF = 10.0            # green area is [-10, 10] x [-10, 10] m
BOUNDARY_KEEP_IN = 0.9       # drones never plan closer than this to the edge
GRID_RES = 0.2               # planning / mapping grid resolution (m)

# ---------------------------------------------------------------------------
# Obstacles
# ---------------------------------------------------------------------------
# 20 KNOWN landmark trees (same layout as the original arena.world). Each trunk carries
# ArUco markers on 4 sides (so a marker faces every direction) at ANCHOR_Z.
# Crown = sphere r=0.5 m centred z=1.5 m.
LANDMARK_TREES: List[Tuple[float, float]] = [
    (-7.0, -5.0), (-6.0, -3.0), (-4.0, -8.0), (-7.5, 2.0), (-3.0, 7.0),
    (-5.5, 7.5), (-1.0, -5.5), (1.5, -2.0), (-2.5, 0.5), (3.0, 3.0),
    (3.5, -7.0), (7.0, -3.5), (8.5, 1.0), (4.0, 7.0), (4.5, 4.0),
    (0.0, 8.5), (-8.5, -1.0), (2.0, -8.5), (7.0, 5.0), (-8.0, 8.5),
]
TREE_TRUNK_R = 0.06
TREE_TRUNK_H = 1.2
TREE_CROWN_R = 0.5
TREE_CROWN_Z = 1.5
ANCHOR_Z = 0.75              # ArUco marker height on the trunk

# UNKNOWN obstacles ("pine trees"): NOT known to any drone at start. They must
# be discovered with the on-board depth sensor. Tiered cone, 3.4 m tall.
UNKNOWN_PINES: List[Tuple[float, float]] = [
    (-4.8, -4.6), (-2.2, -2.0), (1.3, 2.3), (2.4, 5.6), (3.2, -3.6), (-6.0, 1.2),
]
PINE_TIERS = [  # (z_bottom, z_top, radius)
    (0.0, 0.55, 0.10),   # trunk
    (0.55, 1.45, 0.62),
    (1.45, 2.25, 0.46),
    (2.25, 3.00, 0.30),
    (3.00, 3.40, 0.12),
]
PINE_PLAN_R = 0.62           # planning radius of an unknown pine
OBSTACLE_TOP = max(PINE_TIERS[-1][1], TREE_CROWN_Z + TREE_CROWN_R + 0.32)   # tallest obstacle (m)

# ---------------------------------------------------------------------------
# Drones
# ---------------------------------------------------------------------------
DRONE_RADIUS = 0.52          # centre-to-rotor-tip radius of the (75 %) quad model (its rotors: ENERGY.rotor_radius)
DRONE_COLLISION_R = 0.50     # collision radius: Gazebo collision disc, headless plant and contact metric
CONTACT_DZ = 0.6             # rotor discs can touch when two drones are closer than this in height
SEP_MARGIN = 0.25            # m kept on top of the rotor / downwash footprint (separation standard)
                             # -> downwash.required_separation(): ~1.3 m level, 1.5-1.9 m one above the other
PLAN_MARGIN = 0.18           # extra clearance on top of drone radius (collision check)
PLAN_HYSTERESIS = 0.15       # paths are PLANNED with this much more clearance, so tiny
                             # map updates do not trigger endless re-planning
PLANNER = 'rrtstar'          # path planner of every drone: 'rrtstar' (RRT*, rrt_star.py) or 'astar' (A* on
                             # the grid, the planner comparison: study --planner astar, launch planner:=astar)
GIVE_WAY = 'ttc'             # swarm give-way: 'ttc' (the drones trade passage turns for the fleet's lowest
                             # energy, ttc.py; no drone has priority) or 'priority' (fixed priorities in drone
                             # order: the comparison, study --give-way priority, launch give_way:=priority)
DEFAULT_PLANNER = PLANNER    # the study's own settings: tools that switch PLANNER / GIVE_WAY for a comparison
DEFAULT_GIVE_WAY = GIVE_WAY  # (study --planner astar, --give-way priority) return to these
REST_Z = 0.127               # body-centre height when standing on its legs
# vertical extent of the drone (same shapes as the Gazebo collision model): rotor disc
# +-DRONE_DISC_DZ around the body centre, and 4 feet REST_Z below it, 0.13 m off the centre
DRONE_DISC_DZ = 0.05
DRONE_FEET_R = 0.14          # feet ring (feet 0.127 m from the centre + 0.012 m foot radius)
DRONE_FEET_H = 0.024         # foot sphere diameter
OBSTACLE_V_CLEAR = 0.25      # vertical clearance kept over obstacle tops: the depth camera (vertical field
                             # of view) reports anything this close below the feet or above the rotors


@dataclass
class DroneSpec:
    """One drone of the fleet. All five are equal: the same airframe, sensors, speed and software;
    they differ only in their pads and their colour. The study scores the whole fleet."""
    id: int
    name: str
    color: str               # display colour (Gazebo body, plots), only to tell the drones apart
    spawn: Tuple[float, float]
    goal: Tuple[float, float]
    cruise_alt: float
    cruise_speed: float
    priority: int            # drone order (0 first): the layer order of the fixed-layer configuration and
                             # the fixed priorities of the comparison give-way (GIVE_WAY = 'priority')
    layer_alt: float         # own altitude layer (fixed-layer configuration only)


# display colours: a categorical palette checked for colour-vision deficiencies (all pairs), and none
# of them close to the green of the reference routes (each drone's trail in Gazebo is in its colour)
DRONE_COLORS = {1: '#eb6834', 2: '#6250d6', 3: '#1baf7a', 4: '#9b2d5b', 5: '#0e8fc0'}
DRONE_COLOR_NAMES = {1: 'orange', 2: 'violet', 3: 'aqua', 4: 'plum', 5: 'cerulean'}
CRUISE_SPEED = 1.2           # m/s, every drone

DRONES: List[DroneSpec] = [
    DroneSpec(1, 'drone1', DRONE_COLORS[1], (-8.0, -8.0), (8.0, 8.0), 1.5, CRUISE_SPEED, 0, 1.5),
    DroneSpec(2, 'drone2', DRONE_COLORS[2], (-5.0, 5.0), (6.0, -9.0), 1.5, CRUISE_SPEED, 1, 2.30),
    DroneSpec(3, 'drone3', DRONE_COLORS[3], (5.0, -5.0), (-6.0, 4.5), 1.5, CRUISE_SPEED, 2, 2.90),
    DroneSpec(4, 'drone4', DRONE_COLORS[4], (0.0, 0.0), (-8.8, -6.8), 1.5, CRUISE_SPEED, 3, 3.50),
    DroneSpec(5, 'drone5', DRONE_COLORS[5], (6.0, 6.0), (-1.5, -8.8), 1.5, CRUISE_SPEED, 4, 4.10),
]

# ---------------------------------------------------------------------------
# Mission timing (SIM time, so it is independent of Gazebo's real-time factor)
# ---------------------------------------------------------------------------
PREVIEW_TIME = 5.0           # s on the pad: each drone plans its reference route (A*) before take-off
TAKEOFF_SPEED = 0.8
SCAN_TIME = 2.6              # 360 deg yaw scan after take-off
SCAN_YAW_RATE = 2.5
LAND_SPEED = 0.45
CONTROL_RATE = 20.0          # Hz
MISSION_TIMEOUT = 240.0      # sim seconds before a drone gives up
# rolling re-planning in flight (agent.py): every REPLAN_PERIOD s each drone plans again on board from where
# it is to its pad, and at once when a newly found obstacle blocks its route. It switches to the new route
# only if that is shorter than what is left of the current one by REPLAN_GAIN (and at least REPLAN_GAIN_M),
# or the current route is blocked, so the random variation of RRT* never makes a drone wander
REPLAN_PERIOD = 1.0          # s
REPLAN_GAIN = 0.03
REPLAN_GAIN_M = 0.25         # m

# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
LOOKAHEAD = 0.9
ACC_MAX_H = 2.2              # m/s^2 horizontal acceleration limit
ACC_MAX_V = 1.5
ACC_MAX_WIND = 6.0           # m/s^2 the wind feed-forward may change (tilt authority of the airframe)
V_MAX = 2.6
KP_Z = 1.6
KI_Z = 0.4
YAW_RATE_MAX = 1.6
ESCAPE_ALT = 4.7             # above every obstacle (pines 3.4 m)

# ---------------------------------------------------------------------------
# Sensors (synthetic, derived from Gazebo ground truth + noise). Every
# measurement is fused by the drone's EKF (estimator.py) before it is used.
# ---------------------------------------------------------------------------
@dataclass
class SensorParams:
    # --- always on: IMU + ToF ------------------------------------------------
    gyro_sigma: float = 0.02       # yaw-rate noise (rad/s)
    attitude_sigma_deg: float = 0.3
    compass_sigma_deg: float = 2.0  # magnetometer heading noise
    compass_rate: float = 10.0     # Hz
    tof_sigma: float = 0.02        # down-looking ToF altimeter (m)
    # --- VIO: IMU + visual-inertial odometry (measured velocity) -----------
    odom_sigma: float = 0.07       # measured velocity noise (m/s)
    odom_bias_rw: float = 0.005    # bias random walk (m/s/sqrt(s))
    odom_bias_init: float = 0.015  # initial bias std (m/s); the drone calibrates it on the pad
    # --- swarm: UWB peer ranging ---------------------------------------------
    uwb_sigma: float = 0.10        # UWB ranging noise (m)
    uwb_rate: float = 5.0          # Hz
    nlos_bias_min: float = 0.25    # positive bias per blocking obstacle (m)
    nlos_bias_max: float = 0.85
    multipath_prob: float = 0.05
    peer_max_range: float = 8.0
    peer_sigma_per_m: float = 0.03  # peer ranging noise grows with distance
    # --- camera landmark navigation (ArUco on the 20 known trees) -----------
    # Navigation camera rig: 4 x 90 deg cameras (360 deg ring), 640x480, fx = 320 px.
    # Set cam_fov_deg = 80 to study your single forward RGB-D camera instead:
    # it sees 3 markers at once in about 1 % of its frames in this forest.
    cam_fov_deg: float = 360.0     # horizontal coverage of the navigation camera(s)
    cam_vfov_deg: float = 74.0     # vertical field of view of each camera
    marker_size: float = 0.30      # ArUco board edge (m), 4 boards per trunk
    cam_range: float = 6.0         # 0.30 m marker at fx=320 px -> 16 px at 6 m
    cam_sure_range: float = 4.0    # detection prob. falls from 0.95 here ...
    cam_p_near: float = 0.95
    cam_p_far: float = 0.45        # ... to this at cam_range
    cam_rate: float = 10.0         # marker detections processed per second
    cam_bearing_sigma_deg: float = 1.0   # pixel error (heading is a separate EKF state)
    cam_range_sigma0: float = 0.03       # PnP range error = s0 + k * d^2
    cam_range_sigma_k: float = 0.025
    cam_outlier_prob: float = 0.01       # wrong marker / bad corner (gated)
    cam_look_offset_deg: float = 30.0    # narrow camera only (cam_fov_deg < 300): yaw up to this far off
                                         # the travel direction to keep landmarks in view
    # --- EKF motion model ----------------------------------------------------
    track_sigma: float = 0.034           # how well the airframe follows its air-velocity command (m/sqrt(s))
    wind_rw: float = 0.10                # wind estimate process noise (gusts) (m/s/sqrt(s)), 30 s correlation
    wind_init: float = 0.30              # wind is unknown at take-off (m/s)
    # --- forward depth camera (obstacles + nearby drones) --------------------
    depth_range: float = 4.5       # forward depth camera range (m)
    depth_fov_deg: float = 90.0
    depth_rays: int = 25
    depth_rate: float = 5.0
    depth_sigma: float = 0.03
    ring_range: float = 2.0        # short-range ToF ring, 360 deg around: obstacles beside and behind the drone
    ring_rays: int = 24            # (the forward depth camera does not see them while the drone flies round a tree)
    ring_sigma: float = 0.03
    proximity_range: float = 3.0   # the depth camera + ToF ring see other drones this close (horizontally)
    proximity_elev_deg: float = 45.0  # ... and up to this far above / below the horizon (not straight up / down)


SENSORS = SensorParams()

# ---------------------------------------------------------------------------
# Energy model  (momentum theory + blade profile, Zeng et al. 2019 form)
# ---------------------------------------------------------------------------
@dataclass
class EnergyParams:
    mass: float = 1.5              # kg
    rotor_radius: float = 0.135    # m (4 rotors)
    n_rotors: int = 4
    rho: float = 1.225
    eta_induced: float = 0.60      # figure of merit x motor x ESC efficiency
    profile_frac: float = 0.25     # blade-profile power as fraction of induced hover
    u_tip: float = 85.0            # blade tip speed (m/s)
    flat_plate_area: float = 0.035 # m^2 (fuselage drag)
    eta_climb: float = 0.60
    descent_recovery: float = 0.25
    p_avionics: float = 6.0        # FC + IMU + ToF + RGB-D camera + computer, every config (W)
    p_comm_base: float = 0.5       # telemetry link, every config (W)
    p_cam_nav: float = 2.5         # 4 nav cameras + ArUco detection/PnP at 10 Hz (camera configs)
    p_sensor_nav: float = 1.0      # sensor-data processing: VIO / optical flow (sensor-data configs)
    p_comm_swarm: float = 2.5      # mesh radio + UWB peer ranging + map sharing (swarm configs)
    battery_wh: float = 74.0       # 4S 5000 mAh


ENERGY = EnergyParams()

# ---------------------------------------------------------------------------
# Wind. VelocityControl in Gazebo follows commands perfectly, so MissionCore
# adds the wind to every drone's command before it goes to Gazebo (the same
# for every configuration). A drone can only hold its track by estimating the
# wind in its EKF (from VIO, camera fixes or peer ranges) and flying
# against it; with nothing to estimate it from, it drifts with the air.
# ---------------------------------------------------------------------------
@dataclass
class WindParams:
    mean_speed: float = 0.20       # m/s under the canopy, random direction per seed
    gust_sigma: float = 0.10       # m/s, per-drone turbulence (Ornstein-Uhlenbeck)
    gust_tau: float = 3.0          # s


WIND = WindParams()

# ---------------------------------------------------------------------------
# Rotor downwash (PHYSICS, downwash.py): the wake of a drone pushes the drones below it
# down and shakes them sideways. Applied like the wind, so Gazebo and headless agree.
# ---------------------------------------------------------------------------
@dataclass
class DownwashParams:
    on: bool = True
    b0: float = 0.52               # 1/e radius of the wake at the rotors = vehicle radius DRONE_RADIUS (m)
    spread: float = 0.15           # wake radius growth per metre below the drone (turbulent jet)
    ramp_depth: float = 0.25       # wake still forming this close below the rotor plane (m)
    max_depth: float = 8.0         # ignored further down (m)
    w_tol: float = 1.0             # largest downdraft the altitude loop can cancel (its 1 m/s climb
                                   # limit): the separation standard keeps drones out of stronger wake
    turb: float = 0.25             # sideways turbulence in the wake: 1-sigma as a fraction of the downdraft
    turb_tau: float = 0.4          # s
    scale: float = 1.0             # sensitivity studies only: multiplies the downdraft (1 = the jet model above)


DOWNWASH = DownwashParams()

# ---------------------------------------------------------------------------
# Radio link of the HEADLESS twin (mission.Radio). The default is the ideal link of the study: no message is
# lost, every message arrives one control step later, every drone hears every other. The control experiments
# make it realistic (study --radio-loss / --radio-delay-ms / --radio-range); Gazebo uses ROS 2 topics instead.
# ---------------------------------------------------------------------------
@dataclass
class RadioParams:
    loss: float = 0.0                       # probability that one message does not reach one receiver
    delay_ticks: Tuple[int, int] = (0, 0)   # extra delay in control steps, drawn uniformly from [min, max]
    range_m: float = 0.0                    # receivers farther away than this hear nothing (0 = unlimited)


RADIO = RadioParams()

# Reactive avoidance of drones without the radio (agent._peer_avoidance and the safety barrier): extra
# distance kept on top of the separation standard. 0 in the study; the control experiment gives the
# reactive drones the same 0.5 m buffer the swarm give-way plans with (study --reactive-buffer 0.5).
REACTIVE_BUFFER = 0.0

# Turn trading on a real link (ttc.py, package 1.8.5): the rounds are timed to the largest message delay the
# drones measure, a drone avoids reactively (with the give-way buffer) every drone whose round message did not
# arrive in time, positions in late messages are moved on by their age, and messages older than
# RADIO_MAX_AGE s are not used. On an ideal link all of this changes nothing. False (study --ttc-fixed): the
# fixed 0.5 s schedule with 0.1 s phases and no fallback, as in package 1.8.4.
TTC_ADAPTIVE = True
RADIO_MAX_AGE = 1.0          # s

# ---------------------------------------------------------------------------
# Navigation configurations compared in the study
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class NavConfig:
    key: str            # used on the command line:  nav:=<key>
    label: str
    cam_points: int     # camera landmark fixes with N markers (0 = not used)
    sensor: bool        # VIO: on-board IMU + visual-inertial odometry (measured velocity)
    swarm: bool         # swarm intelligence: peer UWB + shared map + give-way (turn trading)
    layers: bool = False  # no radio: every drone always cruises on its own fixed altitude layer

    @property
    def parts(self):
        p = []
        if self.cam_points:
            p.append(f'{self.cam_points}-point camera')
        if self.sensor:
            p.append('VIO')
        if self.swarm:
            p.append('swarm')
        if self.layers:
            p.append('fixed altitude layers')
        return ' + '.join(p) if p else 'none'


NAV_CONFIGS: List[NavConfig] = [
    NavConfig('cam3_sensor', 'Baseline: 3-point camera + VIO', 3, True, False),
    NavConfig('cam2', '2-point camera', 2, False, False),
    NavConfig('sensor', 'VIO only', 0, True, False),
    NavConfig('swarm', 'Swarm only', 0, False, True),
    NavConfig('cam2_sensor', '2-point camera + VIO', 2, True, False),
    NavConfig('cam2_swarm', '2-point camera + swarm', 2, False, True),
    NavConfig('sensor_swarm', 'VIO + swarm', 0, True, True),
    NavConfig('all', 'All-in-one: 2-point camera + VIO + swarm', 2, True, True),
    # completing the comparison: the 3-point camera on its own and with swarm, so the camera type,
    # VIO and swarm can each be compared with everything else held the same
    NavConfig('cam3', '3-point camera', 3, False, False),
    NavConfig('cam3_swarm', '3-point camera + swarm', 3, False, True),
    NavConfig('cam3_sensor_swarm', '3-point camera + VIO + swarm', 3, True, True),
    # the classic way to separate drones WITHOUT communication: every drone cruises on its own altitude
    # layer (1.5 m, then 2.3 m and up, 0.6 m apart). Tests whether separation in height works once
    # downwash is modelled
    NavConfig('cam3_sensor_layers', 'Baseline + fixed altitude layers (no radio)', 3, True, False, True),
]
# Extra configurations for control experiments (not part of the default study; name them with --configs):
# the fixed altitude layers WITH the swarm radio - layers as airspace designs use them, together with shared
# positions, which separates the effect of the layers from the blind spot of drones without the radio
NAV_EXTRA: List[NavConfig] = [
    NavConfig('cam3_sensor_layers_swarm', 'Baseline + fixed altitude layers + swarm radio', 3, True, True, True),
]
BASELINE = 'cam3_sensor'
NAV = {c.key: c for c in NAV_CONFIGS + NAV_EXTRA}
NAV_ALIASES = {'baseline': 'cam3_sensor', '3point': 'cam3_sensor', '2point': 'cam2',
               'sensor_data': 'sensor', 'sensors': 'sensor', 'vio': 'sensor', 'swarm_only': 'swarm',
               'all_in_one': 'all', 'full': 'all',
               # older names, still accepted on the command line
               'vel': 'sensor', 'velocity': 'sensor', 'cam2_vel': 'cam2_sensor', 'vel_swarm': 'sensor_swarm',
               'solo': 'cam2_sensor', 'layers': 'cam3_sensor_layers', 'baseline_layers': 'cam3_sensor_layers'}


# ---------------------------------------------------------------------------
# Mixed fleets (headless study only): each drone has its own configuration, in the
# order drone1 .. drone5. k drones carry the 3-point camera (drone1 .. drone k), the others
# VIO only.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FleetConfig:
    key: str
    label: str
    navs: Tuple[str, str, str, str, str]

    @property
    def n_cameras(self):
        return sum(1 for k in self.navs if NAV[k].cam_points)

    @property
    def swarm(self):
        return any(NAV[k].swarm for k in self.navs)


def _k_fleet(k, swarm):
    cam, cheap = ('cam3_sensor_swarm', 'sensor_swarm') if swarm else ('cam3_sensor', 'sensor')
    order = [1, 2, 3, 4, 5]                       # who gets a camera first (drone order)
    with_cam = set(order[:k])
    navs = tuple(cam if i in with_cam else cheap for i in (1, 2, 3, 4, 5))
    return FleetConfig(f"k{k}{'_swarm' if swarm else ''}",
                       f"{k} of 5 drones with camera{', swarm' if swarm else ', no radio'}", navs)


FLEET_CONFIGS: List[FleetConfig] = [_k_fleet(k, s) for s in (False, True) for k in range(6)]
FLEETS = {f.key: f for f in FLEET_CONFIGS}


def label_of(key):
    """Display name of a configuration or mixed-fleet key."""
    return FLEETS[key].label if key in FLEETS else NAV[key].label


def swarm_of(key):
    return FLEETS[key].swarm if key in FLEETS else NAV[key].swarm


def nav_config(key):
    """NavConfig of a configuration key (or one of its aliases)."""
    k = str(key).strip().lower().replace('+', '_').replace('-', '_')
    k = NAV_ALIASES.get(k, k)
    if k not in NAV:
        raise ValueError(f"unknown nav config '{key}'. Choose one of: {', '.join(NAV)}")
    return NAV[k]

