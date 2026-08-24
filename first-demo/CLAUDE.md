# Project Vortex — Conventions

Fixed-wing UAV atmospheric-energy simulation. These are **permanent conventions**;
do not violate them without explicit instruction.

## Units
- **SI units everywhere** (kg, m, s, N, W, J). Energy reported in Wh only at
  reporting boundaries (metrics, plots); integrate in Joules internally.
- **Angles in radians internally.** Convert to/from degrees only at I/O
  boundaries (YAML configs, plots, human-facing text).

## Testing
- Every physics module ships with a pytest unit test against **analytic ground
  truth** (closed-form solution, conservation law, or limiting case).
- **Tests must pass before any commit.** No exceptions.
- Tests live in `tests/` mirroring the `src/` layout.

## Experiments
- Experiments are **config-driven** (YAML in `configs/`), never hardcoded.
- Each run writes plots + a `metrics.json` to `results/<experiment_name>/`.
- Runs must be **deterministic**: same seed => identical `metrics.json`.

## Dependencies
- Allowed: `numpy`, `scipy`, `matplotlib`, `pyyaml`, `pytest`. **Nothing else.**
- **No ML libraries yet** (no torch, tensorflow, jax, sklearn, gym).

## Layout
- `src/`: `dynamics.py`, `aircraft.py`, `wind.py`, `controller.py`,
  `energy.py`, `simulate.py`, `plotting.py`.
- `tests/`: mirrors `src/`.
- `configs/`: YAML experiment + aircraft definitions.
- `results/`: generated output (gitignored).

## Running
- Use the project venv: `uv venv && uv pip install -e .` (or install the five
  deps directly).
- This machine has ROS on the global `PYTHONPATH`, whose pytest plugins fail to
  import and abort collection. Run tests/sim with a clean env:
  `env -u PYTHONPATH .venv/bin/python -m pytest -q`
  `env -u PYTHONPATH .venv/bin/python -m src.simulate configs/experiment_baseline.yaml`

## Physics conventions
- Coordinate frame: x = East, y = North, h = altitude (up positive).
- Heading measured from +x (East) toward +y (North), radians.
- Wind is a vector `w = (wx, wy, wz)` in the same frame; `wz > 0` is updraft.
- Ground velocity = air-relative velocity + wind.
- Gravity `g = 9.81 m/s^2`.
