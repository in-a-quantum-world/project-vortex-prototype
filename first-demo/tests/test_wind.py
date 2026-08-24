"""Wind-field tests: vector convention, stubs, gust determinism."""
import numpy as np
import pytest

from src.wind import (DrydenTurbulence, RidgeWind, ShearWind, ThermalWind,
                      UniformWind, make_wind)


def test_uniform_direction_convention():
    # direction_deg is the direction the wind blows *toward*, from +x to +y.
    w0 = UniformWind(5.0, 0.0)      # toward +x
    assert w0.at(0, 0, 0, 0) == pytest.approx((5.0, 0.0, 0.0))
    w90 = UniformWind(5.0, 90.0)    # toward +y
    wx, wy, wz = w90.at(0, 0, 0, 0)
    assert (wx, wy, wz) == pytest.approx((0.0, 5.0, 0.0), abs=1e-12)
    w180 = UniformWind(5.0, 180.0)  # toward -x
    assert w180.at(0, 0, 0, 0)[0] == pytest.approx(-5.0)


def test_uniform_zero_vertical():
    w = UniformWind(7.0, 45.0)
    assert w.at(10, 20, 30, 5)[2] == 0.0


def test_gust_determinism_same_seed():
    a = UniformWind(5.0, 0.0, gust_std=1.5, seed=42)
    b = UniformWind(5.0, 0.0, gust_std=1.5, seed=42)
    sa = [a.at(0, 0, 0, t) for t in range(20)]
    sb = [b.at(0, 0, 0, t) for t in range(20)]
    assert np.allclose(sa, sb)


def test_gust_differs_with_different_seed():
    a = UniformWind(5.0, 0.0, gust_std=1.5, seed=1)
    b = UniformWind(5.0, 0.0, gust_std=1.5, seed=2)
    sa = [a.at(0, 0, 0, t) for t in range(20)]
    sb = [b.at(0, 0, 0, t) for t in range(20)]
    assert not np.allclose(sa, sb)


def test_gust_mean_near_specified_wind():
    w = UniformWind(5.0, 0.0, gust_std=1.0, seed=7)
    samples = np.array([w.at(0, 0, 0, t)[:2] for t in range(5000)])
    assert samples[:, 0].mean() == pytest.approx(5.0, abs=0.1)
    assert samples[:, 1].mean() == pytest.approx(0.0, abs=0.1)


@pytest.mark.parametrize("cls", [ThermalWind, RidgeWind, ShearWind, DrydenTurbulence])
def test_stubs_raise(cls):
    with pytest.raises(NotImplementedError):
        cls().at(0, 0, 0, 0)


def test_factory_unknown_kind_raises():
    with pytest.raises(NotImplementedError):
        make_wind({"kind": "thermal"})
