"""Rotary-wing power model (per drone).

P = P_profile(V) + P_induced(V, n) + P_parasite(V) + P_climb(vz) + P_electronics

P_electronics = avionics + telemetry + the navigation components switched on
                (camera marker processing, sensor-data processing, swarm radio) - config.ENERGY

  P_profile  = P0 * (1 + 3 V^2 / U_tip^2)                     blade profile drag
  P_induced  = Pi * n^1.5 * sqrt( sqrt(1 + V^4/(4 vn^4)) - V^2/(2 vn^2) ),  vn = v0 sqrt(n)
  P_parasite = 0.5 * rho * S_fp * V^3                          fuselage drag
  P_climb    = W * vz / eta_climb   (vz > 0),  -k * W * |vz| when descending
  n          = |a + g| / g          load factor: accelerating, braking and
                                    turning all need extra thrust, so jerky
                                    avoidance manoeuvres cost real energy.

V is the AIR speed (the drone's air-velocity command), so flying against the
wind costs energy. Pi and v0 come from momentum theory for the model's
4 x 0.135 m rotors, divided by an overall efficiency, P0 is a fraction of Pi
(Zeng et al., 2019). For the 1.5 kg quad this gives ~157 W of rotor power in
hover (+6.5-12.5 W electronics, depending on the configuration), which is typical.
"""
import math

from . import config as C

G = 9.81


class PowerModel:
    def __init__(self, nav, ep=None):
        self.ep = ep or C.ENERGY
        ep = self.ep
        self.W = ep.mass * G
        area = ep.n_rotors * math.pi * ep.rotor_radius ** 2
        self.v0 = math.sqrt(self.W / (2.0 * ep.rho * area))
        self.Pi = self.W * self.v0 / ep.eta_induced
        self.P0 = ep.profile_frac * self.Pi
        self.p_nav = ((ep.p_cam_nav if nav.cam_points else 0.0) + (ep.p_sensor_nav if nav.sensor else 0.0)
                      + (ep.p_comm_swarm if nav.swarm else 0.0))
        self.p_const = ep.p_avionics + ep.p_comm_base + self.p_nav

    def hover_power(self):
        return self.P0 + self.Pi + self.p_const

    def power(self, v_world, a_world):
        ep = self.ep
        V = math.hypot(v_world[0], v_world[1])
        vz = v_world[2]
        n = math.sqrt(a_world[0] ** 2 + a_world[1] ** 2 + (G + a_world[2]) ** 2) / G
        n = max(0.3, n)
        p_prof = self.P0 * (1.0 + 3.0 * V * V / ep.u_tip ** 2)
        r = V * V / (2.0 * self.v0 ** 2 * n)          # induced velocity grows with thrust
        ind_factor = math.sqrt(max(1e-6, math.sqrt(1.0 + r * r) - r))
        p_ind = self.Pi * n ** 1.5 * ind_factor
        p_par = 0.5 * ep.rho * ep.flat_plate_area * V ** 3
        if vz >= 0:
            p_climb = self.W * vz / ep.eta_climb
        else:
            p_climb = -ep.descent_recovery * self.W * abs(vz)
        p = p_prof + p_ind + p_par + p_climb + self.p_const
        floor = 0.4 * self.hover_power()                 # rotors never idle in flight
        if p < floor:
            p_climb += floor - p
            p = floor
        return p, {'profile': p_prof, 'induced': p_ind, 'parasite': p_par,
                   'climb': p_climb, 'electronics': self.p_const - self.p_nav, 'navigation': self.p_nav}


class Battery:
    def __init__(self, wh=None):
        self.capacity_j = (wh or C.ENERGY.battery_wh) * 3600.0
        self.used_j = 0.0

    def draw(self, p, dt):
        self.used_j += p * dt

    @property
    def soc(self):
        return max(0.0, 100.0 * (1.0 - self.used_j / self.capacity_j))
