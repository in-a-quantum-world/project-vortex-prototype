"""Controller tests: waypoint sequencing and cross-track sign convention."""
import numpy as np

from src.controller import CircuitController
from src.dynamics import VA

CIRCUIT = {"length": 1000.0, "width": 500.0, "altitude": 100.0, "airspeed": 10.0}
CTRL = {"k_heading": 1.0, "waypoint_radius": 30.0, "max_bank_deg": 30.0}


def _state(x, y, psi=0.0):
    return np.array([x, y, 100.0, psi, 10.0])


def test_first_leg_is_x_aligned():
    c = CircuitController(CIRCUIT, CTRL, laps=1)
    u, info = c.command(_state(0.0, 0.0))
    assert info.target_idx == 0
    assert info.cross_track == 0.0     # starts on the line
    assert u.Va_cmd == 10.0


def test_cross_track_sign_left_positive():
    # North of the +x leg (left when facing +x) => positive cross-track.
    c = CircuitController(CIRCUIT, CTRL, laps=1)
    _, info = c.command(_state(500.0, 20.0))
    assert info.cross_track > 0
    # Steering correction should command a negative (rightward) bank to return.
    u, _ = c.command(_state(500.0, 20.0, psi=0.0))
    assert u.bank < 0


def test_waypoint_advances_on_capture():
    c = CircuitController(CIRCUIT, CTRL, laps=1)
    c.command(_state(0.0, 0.0))
    assert c.target_idx == 0
    # Near the far corner (within capture radius) -> advance.
    c.command(_state(995.0, 0.0))
    assert c.target_idx == 1


def test_completes_after_one_lap():
    c = CircuitController(CIRCUIT, CTRL, laps=1)
    # Walk through all four corners.
    for (x, y) in [(1000, 0), (1000, 500), (0, 500), (0, 0)]:
        c.command(_state(float(x), float(y)))
    assert c.done
