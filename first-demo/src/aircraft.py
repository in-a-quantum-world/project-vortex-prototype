"""Point-mass fixed-wing aircraft: parameters, drag polar, and glide analytics.

SI units throughout; angles in radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import yaml

G = 9.81  # should this be made variable with altitude for precision?


@dataclass(frozen=True)
class Aircraft:
    mass: float        # kg
    S: float           # wing reference area, m^2
    AR: float          # aspect ratio
    CD0: float         # zero-lift drag coefficient
    e: float           # Oswald efficiency factor
    rho: float         # air density, kg/m^3
    eta_prop: float    # propeller efficiency
    eta_motor: float   # motor + ESC efficiency
    battery_Wh: float  # usable battery capacity, Wh
    V_stall: float = 0.0   # stall airspeed, m/s
    Va_max: float = 18.0   # max commandable airspeed, m/s

    @classmethod
    def from_yaml(cls, path: str) -> "Aircraft":
        with open(path) as f:
            d = yaml.safe_load(f)
        required = ("mass", "S", "AR", "CD0", "e", "rho",
                    "eta_prop", "eta_motor", "battery_Wh")
        kwargs = {k: float(d[k]) for k in required}
        for opt in ("V_stall", "Va_max"):
            if opt in d:
                kwargs[opt] = float(d[opt])
        return cls(**kwargs)

    # ------------------------------------------------------------------ #
    # Derived quantities
    # ------------------------------------------------------------------ #
    @property
    def k(self) -> float:
        """Induced-drag factor, k = 1 / (pi * e * AR)."""
        return 1.0 / (math.pi * self.e * self.AR)

    @property
    def weight(self) -> float:
        """Weight, N."""
        return self.mass * G

    def CD(self, CL: float) -> float:
        """Drag polar: CD = CD0 + k * CL^2."""
        return self.CD0 + self.k * CL * CL

    def CL_for_level(self, Va: float, bank: float = 0.0) -> float:
        """Lift coefficient required to sustain flight at airspeed Va and bank.

        Load factor n = 1/cos(bank); L = n * W.
        """
        n = 1.0 / math.cos(bank)
        return 2.0 * n * self.weight / (self.rho * Va * Va * self.S)

    def drag(self, Va: float, bank: float = 0.0) -> float:
        """Aerodynamic drag force [N] to sustain level flight at (Va, bank)."""
        CL = self.CL_for_level(Va, bank)
        return 0.5 * self.rho * Va * Va * self.S * self.CD(CL)

    def sink_rate(self, Va: float, bank: float = 0.0) -> float:
        """Steady (unpowered) sink rate [m/s] from the drag polar.

        Energy balance in a glide: W * w_sink = D * Va  =>  w_sink = D*Va/W.
        Equivalent to power-required / weight.
        """
        return self.drag(Va, bank) * Va / self.weight

    # ------------------------------------------------------------------ #
    # Analytic best-glide (max L/D) results
    # ------------------------------------------------------------------ #
    def best_LD(self) -> float:
        """Analytic maximum lift-to-drag ratio, 1 / (2*sqrt(CD0*k))."""
        return 1.0 / (2.0 * math.sqrt(self.CD0 * self.k))

    def best_glide_CL(self) -> float:
        """Lift coefficient at max L/D: CL* = sqrt(CD0 / k)."""
        return math.sqrt(self.CD0 / self.k)

    def best_glide_speed(self) -> float:
        """Airspeed [m/s] at max L/D in level (small-angle) glide.

        Va* = sqrt( 2*W / (rho*S*CL*) ).
        """
        CL = self.best_glide_CL()
        return math.sqrt(2.0 * self.weight / (self.rho * self.S * CL))
