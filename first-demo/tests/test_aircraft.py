"""Analytic-truth tests for the aircraft aerodynamic model.

Covers acceptance criteria #1 (glide ratio at best-glide speed == best L/D)
and #2 (best-glide speed ~10 m/s, best L/D ~13.3).
"""
import math

import pytest

from src.aircraft import Aircraft

CFG = "configs/default_aircraft.yaml"


@pytest.fixture
def ac():
    return Aircraft.from_yaml(CFG)


def test_induced_drag_factor(ac):
    # k = 1 / (pi * e * AR)
    assert ac.k == pytest.approx(1.0 / (math.pi * 0.85 * 8.0), rel=1e-12)


def test_drag_polar_parabolic(ac):
    # CD(0) == CD0 and the induced term is quadratic in CL.
    assert ac.CD(0.0) == pytest.approx(ac.CD0, rel=1e-12)
    assert ac.CD(1.0) == pytest.approx(ac.CD0 + ac.k, rel=1e-12)
    assert ac.CD(2.0) - ac.CD0 == pytest.approx(4.0 * (ac.CD(1.0) - ac.CD0), rel=1e-12)


def test_best_glide_analytics_match_targets(ac):
    # Acceptance #2: within 1% of the specified targets.
    assert ac.best_glide_speed() == pytest.approx(10.0, rel=0.01)
    assert ac.best_LD() == pytest.approx(13.3, rel=0.01)


def test_glide_ratio_at_best_speed_equals_best_LD(ac):
    # Acceptance #1: simulated glide ratio (Va / sink_rate) at the analytic
    # best-glide speed matches analytic best L/D within 2%.
    Va = ac.best_glide_speed()
    glide_ratio = Va / ac.sink_rate(Va, bank=0.0)
    assert glide_ratio == pytest.approx(ac.best_LD(), rel=0.02)


def test_best_glide_speed_minimizes_nothing_worse(ac):
    # The glide ratio at best-glide speed should be >= glide ratio at nearby
    # speeds (it is the maximum L/D point).
    Va = ac.best_glide_speed()
    gr = Va / ac.sink_rate(Va)
    for dv in (-2.0, -1.0, 1.0, 2.0):
        assert gr >= (Va + dv) / ac.sink_rate(Va + dv) - 1e-9
