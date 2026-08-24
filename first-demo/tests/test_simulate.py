"""Integration acceptance tests.

Covers #4 (zero-wind track error < 5 m, monotone energy),
#5 (tailwind < still < headwind aligned-leg Wh/km),
#6 (determinism: same seed => identical metrics)."""
import numpy as np
import pytest
import yaml

from src.aircraft import Aircraft
from src.simulate import run_case
from src.wind import UniformWind

CONFIG = "configs/experiment_baseline.yaml"


@pytest.fixture(scope="module")
def env():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    ac = Aircraft.from_yaml(cfg["aircraft"])
    ctrl = dict(cfg.get("controller", {}))
    ctrl.setdefault("airspeed_tau", cfg.get("airspeed_tau", 2.0))
    return ac, cfg["circuit"], ctrl, float(cfg["dt"]), float(cfg["t_max"]), int(cfg["laps"])


def test_zero_wind_track_and_energy(env):
    ac, circuit, ctrl, dt, t_max, laps = env
    wind = UniformWind(0.0, 0.0)
    r = run_case(ac, wind, circuit, ctrl, dt, t_max, laps, "still")
    # #4a: circuit completes and ground-track error on legs < 5 m.
    assert r.completed
    assert r.metrics["max_crosstrack_on_leg_m"] < 5.0
    # #4b: energy strictly monotonically consumed (battery strictly decreasing).
    assert np.all(np.diff(r.battery) < 0.0)
    assert r.energy_Wh > 0.0


def test_still_air_aligned_leg_matches_analytic(env):
    ac, circuit, ctrl, dt, t_max, laps = env
    r = run_case(ac, UniformWind(0.0, 0.0), circuit, ctrl, dt, t_max, laps, "still")
    # Sanity vs acceptance #3: still-air aligned leg ~0.85 Wh/km.
    assert r.metrics["wh_per_km_aligned_leg"] == pytest.approx(0.85, rel=0.10)
    assert r.metrics["mean_power_W"] == pytest.approx(31.0, rel=0.15)


def test_wind_ordering(env):
    # #5: tailwind leg < still air < headwind leg (aligned-leg Wh/km).
    ac, circuit, ctrl, dt, t_max, laps = env
    still = run_case(ac, UniformWind(0.0, 0.0), circuit, ctrl, dt, t_max, laps, "still")
    head = run_case(ac, UniformWind(5.0, 180.0), circuit, ctrl, dt, t_max, laps, "head")
    tail = run_case(ac, UniformWind(5.0, 0.0), circuit, ctrl, dt, t_max, laps, "tail")
    wt = tail.metrics["wh_per_km_aligned_leg"]
    ws = still.metrics["wh_per_km_aligned_leg"]
    wh = head.metrics["wh_per_km_aligned_leg"]
    assert wt < ws < wh


def test_determinism_same_seed(env):
    # #6: identical seed => identical metrics (use gusts to exercise the RNG).
    ac, circuit, ctrl, dt, t_max, laps = env
    w1 = UniformWind(5.0, 0.0, gust_std=1.0, seed=999)
    w2 = UniformWind(5.0, 0.0, gust_std=1.0, seed=999)
    r1 = run_case(ac, w1, circuit, ctrl, dt, t_max, laps, "a")
    r2 = run_case(ac, w2, circuit, ctrl, dt, t_max, laps, "b")
    assert r1.metrics == r2.metrics
    assert np.array_equal(r1.x, r2.x)
    assert np.array_equal(r1.battery, r2.battery)
