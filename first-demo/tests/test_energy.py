"""Energy-model tests. Covers acceptance criterion #3 (cruise power ~31 W,
Wh/km ~0.85 at 10 m/s in still air)."""
import pytest

from src.aircraft import Aircraft
from src.energy import electrical_power, thrust_required, wh_per_km

CFG = "configs/default_aircraft.yaml"


@pytest.fixture
def ac():
    return Aircraft.from_yaml(CFG)


def test_level_thrust_equals_drag(ac):
    # In level, unbanked flight thrust must equal aerodynamic drag.
    Va = 10.0
    assert thrust_required(ac, Va, 0.0, 0.0) == pytest.approx(ac.drag(Va, 0.0), rel=1e-9)


def test_cruise_power_10ms(ac):
    # Acceptance #3a: electrical power ~= 31 W (+/-10%).
    P = electrical_power(ac, 10.0, 0.0, 0.0)
    assert P == pytest.approx(31.0, rel=0.10)


def test_cruise_whkm_10ms(ac):
    # Acceptance #3b: Wh/km ~= 0.85 (+/-10%) at 10 m/s in still air.
    Va = 10.0
    P = electrical_power(ac, Va, 0.0, 0.0)
    # Still air => ground speed == airspeed; energy over 1 km:
    dist_m = 1000.0
    energy_Wh = P * (dist_m / Va) / 3600.0
    assert wh_per_km(energy_Wh, dist_m) == pytest.approx(0.85, rel=0.10)


def test_climb_costs_more_than_level(ac):
    # Climbing requires more thrust/power than level flight at the same speed.
    assert electrical_power(ac, 10.0, 0.0, 1.0) > electrical_power(ac, 10.0, 0.0, 0.0)


def test_bank_costs_more_than_level(ac):
    import math
    assert electrical_power(ac, 10.0, math.radians(30), 0.0) > \
        electrical_power(ac, 10.0, 0.0, 0.0)
