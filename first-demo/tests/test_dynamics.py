"""Dynamics tests: ground velocity = air velocity + wind; turn kinematics."""
import math

import numpy as np
import pytest

from src.aircraft import G
from src.dynamics import Controls, rk4_step, wrap_to_pi, X, Y, H, PSI, VA
from src.wind import UniformWind


def test_straight_still_air():
    # Heading +x, no wind, no bank: pure eastward translation at Va.
    state = np.array([0.0, 0.0, 100.0, 0.0, 10.0])
    u = Controls(bank=0.0, climb_rate=0.0, Va_cmd=10.0)
    wind = UniformWind(0.0, 0.0)
    s = rk4_step(state, u, wind, 0.0, 0.1, airspeed_tau=2.0)
    assert s[X] == pytest.approx(1.0, rel=1e-6)   # 10 m/s * 0.1 s
    assert s[Y] == pytest.approx(0.0, abs=1e-9)
    assert s[H] == pytest.approx(100.0, abs=1e-9)
    assert s[VA] == pytest.approx(10.0, rel=1e-9)


def test_ground_velocity_includes_wind():
    # Ground velocity = air velocity + wind vector.
    state = np.array([0.0, 0.0, 100.0, 0.0, 10.0])
    u = Controls(bank=0.0, climb_rate=0.0, Va_cmd=10.0)
    wind = UniformWind(3.0, 90.0)  # 3 m/s toward +y
    dt = 0.1
    s = rk4_step(state, u, wind, 0.0, dt, airspeed_tau=2.0)
    assert (s[X] - 0.0) / dt == pytest.approx(10.0, rel=1e-6)  # air x
    assert (s[Y] - 0.0) / dt == pytest.approx(3.0, rel=1e-6)   # wind y


def test_vertical_wind_sets_climb():
    state = np.array([0.0, 0.0, 100.0, 0.0, 10.0])
    u = Controls(bank=0.0, climb_rate=0.0, Va_cmd=10.0)

    class Updraft(UniformWind):
        def at(self, x, y, h, t):
            return (0.0, 0.0, 2.0)

    s = rk4_step(state, u, Updraft(0.0, 0.0), 0.0, 0.1, airspeed_tau=2.0)
    assert (s[H] - 100.0) / 0.1 == pytest.approx(2.0, rel=1e-6)


def test_coordinated_turn_rate():
    # psi_dot = g tan(bank) / Va, checked over a short step.
    Va, bank = 10.0, math.radians(20.0)
    state = np.array([0.0, 0.0, 100.0, 0.0, Va])
    u = Controls(bank=bank, climb_rate=0.0, Va_cmd=Va)
    dt = 1e-3
    s = rk4_step(state, u, UniformWind(0.0, 0.0), 0.0, dt, airspeed_tau=1e6)
    expected = G * math.tan(bank) / Va
    assert wrap_to_pi(s[PSI]) / dt == pytest.approx(expected, rel=1e-3)


def test_airspeed_first_order_tracking():
    state = np.array([0.0, 0.0, 100.0, 0.0, 8.0])
    u = Controls(bank=0.0, climb_rate=0.0, Va_cmd=12.0)
    tau = 2.0
    dt = 1e-4  # small step so the RK4 average ~ instantaneous derivative
    s = rk4_step(state, u, UniformWind(0.0, 0.0), 0.0, dt, airspeed_tau=tau)
    # dVa/dt = (12 - 8)/2 = 2 m/s^2 initially.
    assert (s[VA] - 8.0) / dt == pytest.approx(2.0, rel=1e-3)


def test_wrap_to_pi():
    # 3*pi and -3*pi both represent a heading of pi (wraps to the [-pi, pi)
    # branch as -pi); check magnitude.
    assert abs(wrap_to_pi(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert abs(wrap_to_pi(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_to_pi(0.5) == pytest.approx(0.5)
    assert wrap_to_pi(0.0) == pytest.approx(0.0)
