"""Speed-to-fly (SPEC §6.1) tests.

Source concept: MacCready speed-to-fly, zero-lift case. Analytic targets are
derived in-test from the polar, not hardcoded.
"""
import math

import pytest
import yaml

from src.aircraft import Aircraft
from src.controller import SpeedToFlyController, optimal_airspeed
from src.simulate import run_case
from src.wind import UniformWind

AC_CFG = "configs/default_aircraft.yaml"
EXP_CFG = "configs/experiment_baseline.yaml"


@pytest.fixture(scope="module")
def ac():
    return Aircraft.from_yaml(AC_CFG)


@pytest.fixture(scope="module")
def env():
    with open(EXP_CFG) as f:
        cfg = yaml.safe_load(f)
    ac = Aircraft.from_yaml(cfg["aircraft"])
    ctrl = dict(cfg.get("controller", {}))
    ctrl.setdefault("airspeed_tau", cfg.get("airspeed_tau", 2.0))
    # Large t_max so even strong-headwind circuits finish before the cap.
    return ac, cfg["circuit"], ctrl, 0.1, 2000.0, 1


# ---- #1: still-air optimum == analytic max-range (min-drag) speed ---------- #
def test_still_air_is_max_range_speed(ac):
    # For electric prop, still-air Wh/km = D/eta, so the optimum is the
    # minimum-drag speed Va* = (B/A)^(1/4).
    A = 0.5 * ac.rho * ac.S * ac.CD0
    B = 2.0 * ac.k * ac.weight ** 2 / (ac.rho * ac.S)
    va_analytic = (B / A) ** 0.25
    va_star = optimal_airspeed(ac, 0.0)
    assert va_star == pytest.approx(va_analytic, rel=0.01)
    # ... which is exactly the best-glide (max L/D) speed.
    assert va_star == pytest.approx(ac.best_glide_speed(), rel=0.01)


# ---- #2: headwind => Va* strictly increasing; 5 m/s headwind in [11,13] ---- #
def test_headwind_monotonic_increasing(ac):
    # w_along < 0 is headwind.
    vas = [optimal_airspeed(ac, -w) for w in range(0, 8)]
    assert all(vas[i + 1] > vas[i] + 1e-3 for i in range(len(vas) - 1))
    assert 11.0 <= optimal_airspeed(ac, -5.0) <= 13.0


# ---- #3: tailwind => Va* decreasing, never below 1.1*V_stall --------------- #
def test_tailwind_monotonic_decreasing_bounded(ac):
    lo = 1.1 * ac.V_stall
    vas = [optimal_airspeed(ac, float(w)) for w in range(0, 9)]
    assert all(vas[i + 1] < vas[i] + 1e-9 for i in range(len(vas) - 1))
    assert vas[-1] < vas[0]                 # strictly lower at strong tailwind
    assert all(v >= lo - 1e-6 for v in vas)


# ---- #4: bank/sink analytic (pre-Phase-1 requirement) ---------------------- #
def test_sink_rate_ratio_45deg_bank(ac):
    vbg = ac.best_glide_speed()
    ratio = ac.sink_rate(vbg, math.radians(45.0)) / ac.sink_rate(vbg, 0.0)
    assert ratio == pytest.approx(1.5, rel=0.01)


# ---- #5: circuit-level saving ---------------------------------------------- #
def _run_pair(env, wind_speed):
    ac, circuit, ctrl, dt, t_max, laps = env
    base = run_case(ac, UniformWind(wind_speed, 0.0), circuit, ctrl, dt, t_max,
                    laps, "base")
    wind = UniformWind(wind_speed, 0.0)
    stf = run_case(ac, wind, circuit, ctrl, dt, t_max, laps, "stf",
                   controller=SpeedToFlyController(circuit, ctrl, laps, ac, wind))
    return base.metrics["wh_per_km"], stf.metrics["wh_per_km"]


def test_still_air_stf_equals_baseline(env):
    b, s = _run_pair(env, 0.0)
    assert abs(s - b) / b < 0.005


@pytest.mark.parametrize("wind_speed", [3.0, 5.0, 7.0])
def test_windy_stf_beats_baseline(env, wind_speed):
    b, s = _run_pair(env, wind_speed)
    assert s < b
