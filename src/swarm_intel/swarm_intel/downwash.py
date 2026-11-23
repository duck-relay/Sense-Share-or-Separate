"""Rotor downwash: the wake of one drone acting on the drones below it.

PHYSICS (used by MissionCore for BOTH simulators, like the wind): a hovering multirotor
pushes a column of air down. Far enough below the rotors the four slipstreams merge into
one round turbulent jet, modelled here with the textbook free-jet assumptions:

  * Gaussian velocity profile    w(dz, r) = u_c(dz) * exp(-(r / b(dz))^2)
  * linear spreading              b(dz)    = b0 + k * dz            (b0 = vehicle radius, centre to rotor tip)
  * momentum flux = thrust        T = rho * integral(w^2 dA) = rho * pi * b^2 * u_c^2 / 2
                                  ->  u_c(dz) = sqrt(2 T / (rho * pi * b(dz)^2))

T and rho are those of the energy model (momentum theory, 1.5 kg quad). Below a hovering
drone this gives a 4.5 m/s downdraft 0.6 m down, 3.7 m/s at 1.5 m and 2.9 m/s at 3 m: the
wake stays strong for metres, which is why stacking drones a few decimetres apart in
height (altitude layers) does not separate them. The wake trails behind a moving drone
(it is shed where the drone was) and drifts with the wind; directly next to the rotor
plane (|dz| < 0.25 m) it is still forming, so it is ramped in.

SEPARATION STANDARD (used by every drone's avoidance logic and by the safety metrics):
two drones must keep a horizontal centre distance of at least
  required_separation(dz) = max(rotor contact, wake) + margin
  rotor contact : 2 x vehicle radius               (|dz| < 0.6 m)
  wake          : wake radius where the downdraft falls to w_tol (what the altitude loop
                  can still cancel) + the lower drone's own vehicle radius
i.e. about 1.3 m at the same height and 1.5-1.9 m when one drone is above the other.
"""
import math

from . import config as C

G = 9.81


def _thrust(load=1.0):
    return C.ENERGY.mass * G * load


def wake_width(dz):
    """1/e radius of the wake (m) at dz metres below the drone."""
    p = C.DOWNWASH
    return p.b0 + p.spread * max(0.0, dz)


def centre_speed(dz, load=1.0):
    """Downdraft on the wake axis (m/s) at dz metres below a drone flying with this load factor."""
    p = C.DOWNWASH
    if dz <= 0.0 or dz > p.max_depth:
        return 0.0
    b = wake_width(dz)
    u = math.sqrt(2.0 * _thrust(load) / (C.ENERGY.rho * math.pi * b * b))
    return p.scale * u * min(1.0, dz / p.ramp_depth)


def speed(dz, r, load=1.0):
    """Downdraft (m/s) at dz below the drone and r metres from the wake axis."""
    u = centre_speed(dz, load)
    if u <= 0.0:
        return 0.0
    b = wake_width(dz)
    return u * math.exp(-(r / b) ** 2)


def wake_radius(dz, w=None):
    """Distance from the wake axis (m) where the downdraft falls to w (default: w_tol)."""
    w = C.DOWNWASH.w_tol if w is None else w
    u = centre_speed(abs(dz))
    if u <= w:
        return 0.0
    return wake_width(abs(dz)) * math.sqrt(math.log(u / w))


def required_separation(dz):
    """Horizontal centre distance (m) two drones at a height difference dz must keep."""
    dz = abs(dz)
    r = C.DRONE_RADIUS
    contact = 2.0 * r if dz < C.CONTACT_DZ else 0.0
    wake = wake_radius(dz) + r if C.DOWNWASH.on else 0.0
    return max(contact, wake) + C.SEP_MARGIN


def axis_offset(dz, v_air_xy):
    """Where the wake axis is at depth dz relative to the drone that shed it: the air that
    reaches dz left the rotors dz / (mean downdraft) seconds ago, when the drone was further
    back along its (air-relative) track."""
    u = max(1.0, 0.5 * (centre_speed(min(dz, C.DOWNWASH.ramp_depth) + 1e-3) + centre_speed(dz)))
    tau = dz / u
    return -tau * v_air_xy[0], -tau * v_air_xy[1]
