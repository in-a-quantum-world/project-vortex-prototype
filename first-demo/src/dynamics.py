"""3-DOF point-mass flight dynamics with RK4 integration.

State (SI, radians):
    x    East position   [m]
    y    North position  [m]
    h    altitude (up)   [m]
    psi  heading, from +x toward +y  [rad]
    Va   airspeed        [m/s]

Controls:
    bank        commanded bank angle [rad] (drives coordinated-turn rate)
    climb_rate  commanded air-relative climb rate [m/s] (0 for level flight)
    Va_cmd      commanded airspeed [m/s] (first-order tracked, time const tau)

Wind w=(wx,wy,wz) is added to the air-relative velocity to get ground
velocity; the vertical ground rate includes wz.

NOTE (flagged): the spec lists "throttle" as a control. In this slice the
vertical channel is commanded directly via `climb_rate` and the required
electrical power (the throttle's physical effect) is recovered by energy.py
from the flight condition. For the fixed-altitude baseline climb_rate=0, so
the two formulations are equivalent. A true throttle->climb dynamic is left
for the energy-aware planner (SPEC §6).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .aircraft import G
from .wind import WindField

# State vector indices
X, Y, H, PSI, VA = 0, 1, 2, 3, 4


@dataclass
class Controls:
    bank: float = 0.0        # rad
    climb_rate: float = 0.0  # m/s, air-relative
    Va_cmd: float = 0.0      # m/s


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def deriv(state: np.ndarray, u: Controls, wind: WindField, t: float,
          airspeed_tau: float) -> np.ndarray:
    """State derivative given controls, wind field, time, airspeed time const."""
    x, y, h, psi, Va = state
    wx, wy, wz = wind.at(x, y, h, t)

    # Air-relative horizontal velocity along heading, plus wind => ground vel.
    xdot = Va * np.cos(psi) + wx
    ydot = Va * np.sin(psi) + wy
    hdot = u.climb_rate + wz

    # Coordinated-turn heading rate from bank angle.
    psidot = G * np.tan(u.bank) / Va if Va > 1e-6 else 0.0

    # First-order airspeed tracking toward commanded airspeed.
    Vadot = (u.Va_cmd - Va) / airspeed_tau

    return np.array([xdot, ydot, hdot, psidot, Vadot])


def rk4_step(state: np.ndarray, u: Controls, wind: WindField, t: float,
             dt: float, airspeed_tau: float) -> np.ndarray:
    """Advance the state one RK4 step of dt. Controls held constant over step."""
    k1 = deriv(state, u, wind, t, airspeed_tau)
    k2 = deriv(state + 0.5 * dt * k1, u, wind, t + 0.5 * dt, airspeed_tau)
    k3 = deriv(state + 0.5 * dt * k2, u, wind, t + 0.5 * dt, airspeed_tau)
    k4 = deriv(state + dt * k3, u, wind, t + dt, airspeed_tau)
    new = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    new[PSI] = wrap_to_pi(new[PSI])
    return new
