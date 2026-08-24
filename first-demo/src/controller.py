"""Baseline waypoint autopilot: fly a rectangular circuit at fixed altitude
and fixed commanded airspeed. Cross-track (line-following) guidance keeps the
aircraft on each leg; proportional heading control produces the bank command.

No energy awareness -- this is the experimental control group.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize_scalar

from .aircraft import Aircraft
from .dynamics import Controls, X, Y, PSI, wrap_to_pi
from .energy import electrical_power
from .wind import WindField


@dataclass
class GuidanceInfo:
    target_idx: int      # index into the target-waypoint sequence
    leg_idx: int         # which of the 4 circuit legs (0..3) is active
    cross_track: float   # signed perpendicular distance to active leg [m]
    dist_to_target: float
    done: bool


class CircuitController:
    """Flies `laps` circuits of a rectangle length x width at fixed altitude.

    Legs (starting from the origin corner):
        0: (0,0)      -> (L,0)      long, along +x   <-- wind-aligned leg
        1: (L,0)      -> (L,W)      short, along +y
        2: (L,W)      -> (0,W)      long, along -x
        3: (0,W)      -> (0,0)      short, along -y
    """

    def __init__(self, circuit: dict, ctrl: dict, laps: int = 1):
        self.L = float(circuit["length"])
        self.W = float(circuit["width"])
        self.altitude = float(circuit["altitude"])
        self.airspeed = float(circuit["airspeed"])

        self.k_heading = float(ctrl.get("k_heading", 1.0))
        self.k_ct = float(ctrl.get("k_cross_track", 0.03))
        self.capture_radius = float(ctrl.get("waypoint_radius", 30.0))
        self.max_bank = math.radians(float(ctrl.get("max_bank_deg", 30.0)))
        self.max_ct_corr = math.radians(float(ctrl.get("max_ct_correction_deg", 60.0)))

        corners = [(0.0, 0.0), (self.L, 0.0), (self.L, self.W), (0.0, self.W)]
        # Target sequence: WP1, WP2, WP3, WP0 repeated per lap.
        one_lap = [corners[1], corners[2], corners[3], corners[0]]
        self.targets = np.array(one_lap * int(laps))
        self.start = np.array(corners[0])

        self.target_idx = 0

    # ------------------------------------------------------------------ #
    def _leg_endpoints(self):
        """Return (A, B): previous waypoint and current target."""
        B = self.targets[self.target_idx]
        A = self.start if self.target_idx == 0 else self.targets[self.target_idx - 1]
        return np.asarray(A, dtype=float), np.asarray(B, dtype=float)

    @property
    def done(self) -> bool:
        return self.target_idx >= len(self.targets)

    def command(self, state: np.ndarray):
        """Return (Controls, GuidanceInfo) for the current state."""
        if self.done:
            # Hold last heading, level, at cruise speed.
            info = GuidanceInfo(self.target_idx, 3, 0.0, 0.0, True)
            return Controls(0.0, 0.0, self.airspeed), info

        p = np.array([state[X], state[Y]])
        A, B = self._leg_endpoints()
        d = B - A
        leg_len = np.hypot(d[0], d[1])
        u = d / leg_len  # unit along-leg
        ap = p - A
        along = float(np.dot(ap, u))                # distance along leg
        cross = float(u[0] * ap[1] - u[1] * ap[0])  # signed left-positive
        dist_to_target = float(np.hypot(*(B - p)))

        # Advance waypoint on capture OR once we've passed the target along-track.
        if dist_to_target < self.capture_radius or along > leg_len:
            self.target_idx += 1
            if self.done:
                info = GuidanceInfo(self.target_idx, 3, cross, dist_to_target, True)
                return Controls(0.0, 0.0, self.airspeed), info
            A, B = self._leg_endpoints()
            d = B - A
            leg_len = np.hypot(d[0], d[1])
            u = d / leg_len
            ap = p - A
            cross = float(u[0] * ap[1] - u[1] * ap[0])
            dist_to_target = float(np.hypot(*(B - p)))

        leg_bearing = math.atan2(d[1], d[0])
        # Cross-track correction: steer back toward the leg line.
        corr = np.clip(-self.k_ct * cross, -self.max_ct_corr, self.max_ct_corr)
        desired_heading = leg_bearing + corr

        heading_err = wrap_to_pi(desired_heading - state[PSI])
        bank = float(np.clip(self.k_heading * heading_err, -self.max_bank, self.max_bank))

        leg_idx = self.target_idx % 4
        info = GuidanceInfo(self.target_idx, leg_idx, cross, dist_to_target, False)
        return Controls(bank=bank, climb_rate=0.0, Va_cmd=self.airspeed), info


# ---------------------------------------------------------------------- #
# Energy-aware speed scheduling (SPEC §6)
# ---------------------------------------------------------------------- #
def optimal_airspeed(ac: Aircraft, w_along: float,
                     va_min: float | None = None,
                     va_max: float | None = None) -> float:
    """Airspeed [m/s] minimising electrical energy per ground km on a leg.

    Solves  Va* = argmin  P_elec(Va) / Vg,   Vg = Va + w_along,
    where `w_along` is the signed along-track wind component (positive =
    tailwind). This is the electric analogue of MacCready speed-to-fly for the
    zero-lift (no-thermal) case: dropping airmass sink to zero, the optimum
    trades airspeed against exposure time to the along-track wind.

    Bounded to [va_min, va_max]; defaults to [1.1*V_stall, Va_max] (a 10%
    stall margin -- see note in the module / config).
    """
    lo = va_min if va_min is not None else 1.1 * ac.V_stall
    hi = va_max if va_max is not None else ac.Va_max

    def cost(Va: float) -> float:
        Vg = Va + w_along
        if Vg <= 1e-6:            # cannot make ground progress -> infeasible
            return float("inf")
        return electrical_power(ac, Va) / Vg

    res = minimize_scalar(cost, bounds=(lo, hi), method="bounded")
    return float(res.x)


class SpeedToFlyController(CircuitController):
    """Baseline circuit guidance, but the commanded airspeed on each leg is the
    energy-per-ground-km optimum for that leg's along-track wind.

    Uses the *known mean* wind (via WindField.mean_at) for now; a wind
    estimator replaces this later (SPEC §3). The per-leg solution is cached.
    """

    def __init__(self, circuit: dict, ctrl: dict, laps: int,
                 ac: Aircraft, wind: WindField):
        super().__init__(circuit, ctrl, laps=laps)
        self.ac = ac
        self.wind = wind
        self.altitude = float(circuit["altitude"])
        self.va_min = 1.1 * ac.V_stall
        self.va_max = ac.Va_max
        self._cache: dict[int, float] = {}

    def leg_airspeed(self, target_idx: int) -> float:
        """Cached optimal airspeed for the leg ending at `target_idx`."""
        if target_idx in self._cache:
            return self._cache[target_idx]
        A = self.start if target_idx == 0 else self.targets[target_idx - 1]
        B = self.targets[target_idx]
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        d = B - A
        u = d / np.hypot(d[0], d[1])
        mx = 0.5 * (A[0] + B[0])
        my = 0.5 * (A[1] + B[1])
        wx, wy, _ = self.wind.mean_at(mx, my, self.altitude, 0.0)
        w_along = float(wx * u[0] + wy * u[1])   # + tailwind, - headwind
        va = optimal_airspeed(self.ac, w_along, self.va_min, self.va_max)
        self._cache[target_idx] = va
        return va

    def command(self, state: np.ndarray):
        u, info = super().command(state)
        if info.done:
            return u, info
        u.Va_cmd = self.leg_airspeed(info.target_idx)
        return u, info
