"""Electrical energy model.

Steady-state force balance gives the thrust required to hold a flight
condition (airspeed, bank, climb rate). Electrical power follows from the
propulsion chain efficiency:

    P_elec = T * Va / (eta_prop * eta_motor)

Energy is integrated in Joules internally and reported in Wh at boundaries.
"""
from __future__ import annotations

import math

from .aircraft import Aircraft


def thrust_required(ac: Aircraft, Va: float, bank: float = 0.0,
                    climb_rate: float = 0.0) -> float:
    """Steady-state thrust [N] to sustain (Va, bank, climb_rate).

    Flight-path angle gamma from the air-relative climb: sin(gamma)=climb/Va.
    Force balance along the path: T = D + W*sin(gamma). Lift supports the
    banked, climbing weight: L = n * W * cos(gamma), n = 1/cos(bank).
    """
    gamma = math.asin(max(-1.0, min(1.0, climb_rate / Va)))
    n = 1.0 / math.cos(bank)
    CL = 2.0 * n * ac.weight * math.cos(gamma) / (ac.rho * Va * Va * ac.S)
    D = 0.5 * ac.rho * Va * Va * ac.S * ac.CD(CL)
    return D + ac.weight * math.sin(gamma)


def electrical_power(ac: Aircraft, Va: float, bank: float = 0.0,
                     climb_rate: float = 0.0) -> float:
    """Electrical power draw [W]. Clamped to >= 0 (no regeneration modelled)."""
    T = thrust_required(ac, Va, bank, climb_rate)
    P_prop = T * Va
    P_elec = P_prop / (ac.eta_prop * ac.eta_motor)
    return max(P_elec, 0.0)


class EnergyTracker:
    """Integrates electrical energy and battery state over a run."""

    def __init__(self, ac: Aircraft):
        self.ac = ac
        self.energy_J = 0.0
        self.battery_Wh = ac.battery_Wh

    def step(self, Va: float, bank: float, climb_rate: float, dt: float) -> float:
        """Advance energy by one step; return power [W] drawn this step."""
        P = electrical_power(self.ac, Va, bank, climb_rate)
        self.energy_J += P * dt
        self.battery_Wh -= P * dt / 3600.0
        return P

    @property
    def energy_Wh(self) -> float:
        return self.energy_J / 3600.0


def wh_per_km(energy_Wh: float, ground_distance_m: float) -> float:
    """Specific energy consumption [Wh/km]. Returns inf for zero distance."""
    if ground_distance_m <= 0.0:
        return float("inf")
    return energy_Wh / (ground_distance_m / 1000.0)
