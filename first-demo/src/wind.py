"""Wind field interface and implementations.

Frame: x = East, y = North, h = up. A wind vector w = (wx, wy, wz) is the
velocity of the air mass; wz > 0 is an updraft.

Direction convention for UniformWind: `direction_deg` is the direction the
wind blows *toward*, measured from +x (East) toward +y (North). So
direction_deg=0 -> wind toward +x, direction_deg=90 -> wind toward +y,
direction_deg=180 -> wind toward -x.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod

import numpy as np


class WindField(ABC):
    """Abstract wind field. Returns air velocity at a point and time."""

    @abstractmethod
    def at(self, x: float, y: float, h: float, t: float):
        """Return (wx, wy, wz) [m/s] at position (x, y, h) and time t."""
        raise NotImplementedError

    def mean_at(self, x: float, y: float, h: float, t: float):
        """Return the *mean* (gust-free) wind vector at a point.

        Used by planners/schedulers that should react to the resolved wind,
        not instantaneous turbulence. Defaults to `at`; overridden where a
        stochastic component is present.
        """
        return self.at(x, y, h, t)


class UniformWind(WindField):
    """Spatially uniform horizontal wind with optional seeded Gaussian gusts.

    The mean wind is constant in space and time. If gust_std > 0, an
    isotropic zero-mean Gaussian gust is added to the horizontal components,
    drawn from a seeded RNG so runs are reproducible.
    """

    def __init__(self, speed: float, direction_deg: float,
                 gust_std: float = 0.0, seed: int = 0):
        self.speed = float(speed)
        self.direction_deg = float(direction_deg)
        self.gust_std = float(gust_std)
        theta = math.radians(self.direction_deg)
        self._wx = self.speed * math.cos(theta)
        self._wy = self.speed * math.sin(theta)
        self._rng = np.random.default_rng(seed)

    def at(self, x: float, y: float, h: float, t: float):
        wx, wy = self._wx, self._wy
        if self.gust_std > 0.0:
            gx, gy = self._rng.normal(0.0, self.gust_std, size=2)
            wx += gx
            wy += gy
        return (wx, wy, 0.0)

    def mean_at(self, x: float, y: float, h: float, t: float):
        return (self._wx, self._wy, 0.0)


# ---------------------------------------------------------------------- #
# Stubs for later sections (see SPEC.md). Not implemented in this slice.
# ---------------------------------------------------------------------- #
class ThermalWind(WindField):
    """§3 Thermal model. TODO."""

    def at(self, x, y, h, t):
        raise NotImplementedError("ThermalWind not implemented yet (SPEC §3)")


class RidgeWind(WindField):
    """§4 Orographic lift. TODO."""

    def at(self, x, y, h, t):
        raise NotImplementedError("RidgeWind not implemented yet (SPEC §4)")


class ShearWind(WindField):
    """§5 Wind shear. TODO."""

    def at(self, x, y, h, t):
        raise NotImplementedError("ShearWind not implemented yet (SPEC §5)")


class DrydenTurbulence(WindField):
    """§5 Dryden turbulence model. TODO."""

    def at(self, x, y, h, t):
        raise NotImplementedError("DrydenTurbulence not implemented yet (SPEC §5)")


def make_wind(cfg: dict) -> WindField:
    """Factory: build a WindField from a config dict.

    Expects at least {'kind': 'uniform', ...}. A 'seed' key may be injected
    by the caller for reproducible gusts.
    """
    kind = cfg.get("kind", "uniform")
    if kind == "uniform":
        return UniformWind(
            speed=cfg.get("speed", 0.0),
            direction_deg=cfg.get("direction_deg", 0.0),
            gust_std=cfg.get("gust_std", 0.0),
            seed=cfg.get("seed", 0),
        )
    raise NotImplementedError(f"Wind kind '{kind}' not implemented (SPEC §2)")
