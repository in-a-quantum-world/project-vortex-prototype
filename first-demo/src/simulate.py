"""Simulation driver and baseline experiment entry point.

Usage:
    python -m src.simulate configs/experiment_baseline.yaml

Runs each wind case defined in the config, writes trajectory / timeline /
bar-chart plots and a metrics.json to results/<experiment_name>/.
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import yaml

from .aircraft import Aircraft
from .controller import CircuitController, SpeedToFlyController
from .dynamics import Controls, rk4_step, X, Y, H, PSI, VA
from .energy import EnergyTracker, wh_per_km
from .wind import WindField, make_wind

CORNER_BUFFER = 60.0  # m; cross-track excluded within this of a corner


@dataclass
class SimResult:
    name: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    h: np.ndarray
    Va: np.ndarray
    va_cmd: np.ndarray
    bank: np.ndarray
    power: np.ndarray
    battery: np.ndarray
    cross_track: np.ndarray
    established: np.ndarray      # bool: on a straight leg, away from corners
    ground_distance: float       # total horizontal ground track [m]
    energy_Wh: float
    completed: bool              # circuit(s) finished within t_max
    metrics: dict = field(default_factory=dict)


def run_case(ac: Aircraft, wind: WindField, circuit: dict, ctrl: dict,
             dt: float, t_max: float, laps: int, name: str = "case",
             controller=None) -> SimResult:
    """Integrate one wind case and return time series + metrics.

    If `controller` is None a fixed-airspeed baseline CircuitController is used;
    pass a SpeedToFlyController (or any drop-in) to override.
    """
    if controller is None:
        controller = CircuitController(circuit, ctrl, laps=laps)
    tracker = EnergyTracker(ac)
    airspeed_tau = float(ctrl.get("airspeed_tau", 2.0))

    state = np.array([0.0, 0.0, circuit["altitude"], 0.0, circuit["airspeed"]])

    ts, xs, ys, hs, Vas, vacs, banks, powers, batts, cts, ests = ([] for _ in range(11))
    ground_dist = 0.0
    aligned_energy_J = 0.0
    aligned_dist = 0.0

    t = 0.0
    n_steps = int(round(t_max / dt))
    for _ in range(n_steps):
        u, info = controller.command(state)
        if info.done:
            break

        P = tracker.step(state[VA], u.bank, u.climb_rate, dt)

        # Distance from the active leg's start corner, for corner exclusion.
        along = _along_leg(controller, state)
        leg_len = _leg_len(controller)
        established = (info.dist_to_target > CORNER_BUFFER
                       and along > CORNER_BUFFER
                       and along < leg_len - CORNER_BUFFER)

        ts.append(t); xs.append(state[X]); ys.append(state[Y]); hs.append(state[H])
        Vas.append(state[VA]); vacs.append(u.Va_cmd); banks.append(u.bank)
        powers.append(P); batts.append(tracker.battery_Wh)
        cts.append(info.cross_track); ests.append(established)

        new_state = rk4_step(state, u, wind, t, dt, airspeed_tau)
        step_dist = float(np.hypot(new_state[X] - state[X], new_state[Y] - state[Y]))
        ground_dist += step_dist
        if info.target_idx == 0:  # wind-aligned first leg
            aligned_energy_J += P * dt
            aligned_dist += step_dist

        state = new_state
        t += dt

    completed = controller.done
    energy_Wh = tracker.energy_Wh
    wpk = wh_per_km(energy_Wh, ground_dist)
    wpk_aligned = wh_per_km(aligned_energy_J / 3600.0, aligned_dist)

    res = SimResult(
        name=name,
        t=np.array(ts), x=np.array(xs), y=np.array(ys), h=np.array(hs),
        Va=np.array(Vas), va_cmd=np.array(vacs), bank=np.array(banks),
        power=np.array(powers),
        battery=np.array(batts), cross_track=np.array(cts),
        established=np.array(ests, dtype=bool),
        ground_distance=ground_dist, energy_Wh=energy_Wh, completed=completed,
    )
    est_ct = res.cross_track[res.established] if res.established.any() else np.array([0.0])
    res.metrics = {
        "energy_Wh": round(energy_Wh, 4),
        "ground_distance_m": round(ground_dist, 2),
        "wh_per_km": round(wpk, 4),
        "wh_per_km_aligned_leg": round(wpk_aligned, 4),
        "mean_power_W": round(float(np.mean(res.power)), 3) if len(res.power) else 0.0,
        "max_crosstrack_on_leg_m": round(float(np.max(np.abs(est_ct))), 4),
        "battery_Wh_remaining": round(tracker.battery_Wh, 4),
        "duration_s": round(t, 2),
        "completed_circuit": bool(completed),
    }
    return res


def _along_leg(controller: CircuitController, state: np.ndarray) -> float:
    if controller.done:
        return 0.0
    A, B = controller._leg_endpoints()
    d = B - A
    u = d / np.hypot(d[0], d[1])
    ap = np.array([state[X] - A[0], state[Y] - A[1]])
    return float(np.dot(ap, u))


def _leg_len(controller: CircuitController) -> float:
    if controller.done:
        return 1.0
    A, B = controller._leg_endpoints()
    return float(np.hypot(B[0] - A[0], B[1] - A[1]))


# ---------------------------------------------------------------------- #
# Experiment entry point
# ---------------------------------------------------------------------- #
def run_experiment(config_path: str) -> dict:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    ac = Aircraft.from_yaml(cfg["aircraft"])
    seed = int(cfg.get("seed", 0))
    dt = float(cfg["dt"])
    t_max = float(cfg["t_max"])
    laps = int(cfg.get("laps", 1))
    circuit = cfg["circuit"]
    ctrl = dict(cfg.get("controller", {}))
    ctrl.setdefault("airspeed_tau", cfg.get("airspeed_tau", 2.0))

    results = []
    for case in cfg["cases"]:
        wind_cfg = dict(case["wind"])
        wind_cfg.setdefault("seed", seed)
        wind = make_wind(wind_cfg)
        res = run_case(ac, wind, circuit, ctrl, dt, t_max, laps, name=case["name"])
        results.append(res)

    out_dir = os.path.join("results", cfg["experiment_name"])
    os.makedirs(out_dir, exist_ok=True)

    metrics = {r.name: r.metrics for r in results}
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    # Plots (imported lazily so tests need not import matplotlib).
    from . import plotting
    plotting.plot_trajectories(results, os.path.join(out_dir, "trajectories.png"),
                               circuit)
    plotting.plot_timelines(results, os.path.join(out_dir, "timelines.png"))
    plotting.plot_whkm_bar(results, os.path.join(out_dir, "whkm_bar.png"))

    _print_table(results)
    print(f"\nWrote plots + metrics.json to {out_dir}/")
    return metrics


def _print_table(results):
    print("\n  Wh/km summary (aligned = wind-aligned outbound leg)")
    print("  " + "-" * 66)
    print(f"  {'case':<16}{'Wh/km (leg)':>14}{'Wh/km (circuit)':>18}{'mean P [W]':>14}")
    print("  " + "-" * 66)
    for r in results:
        m = r.metrics
        print(f"  {r.name:<16}{m['wh_per_km_aligned_leg']:>14.4f}"
              f"{m['wh_per_km']:>18.4f}{m['mean_power_W']:>14.2f}")
    print("  " + "-" * 66)


# ---------------------------------------------------------------------- #
# Speed-to-fly sweep experiment (baseline vs energy-aware)
# ---------------------------------------------------------------------- #
def _circuit_whkm(ac, wind, circuit, ctrl, dt, t_max, laps, use_stf):
    """Run one case and return (circuit Wh/km, SimResult)."""
    controller = (SpeedToFlyController(circuit, ctrl, laps, ac, wind)
                  if use_stf else None)
    res = run_case(ac, wind, circuit, ctrl, dt, t_max, laps,
                   name=("stf" if use_stf else "baseline"), controller=controller)
    return res.metrics["wh_per_km"], res


def run_speed_to_fly_sweep(config_path: str) -> dict:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    ac = Aircraft.from_yaml(cfg["aircraft"])
    dt = float(cfg["dt"])
    t_max = float(cfg["t_max"])
    laps = int(cfg.get("laps", 1))
    circuit = cfg["circuit"]
    ctrl = dict(cfg.get("controller", {}))
    ctrl.setdefault("airspeed_tau", cfg.get("airspeed_tau", 2.0))

    wind_speeds = list(cfg["wind_speeds"])
    direction = float(cfg.get("wind_direction_deg", 0.0))
    gust_std = float(cfg.get("gust_std", 0.0))
    seeds = list(cfg["seeds"])
    highlight = float(cfg.get("highlight_wind", 5.0))

    def wind(speed, seed):
        return make_wind({"kind": "uniform", "speed": speed,
                          "direction_deg": direction, "gust_std": gust_std,
                          "seed": seed})

    sweep = {}                 # wind_speed -> aggregated stats
    va_timeline = {}           # SimResults for the highlighted wind speed
    for ws in wind_speeds:
        base_vals, stf_vals = [], []
        for seed in seeds:
            b_wpk, b_res = _circuit_whkm(ac, wind(ws, seed), circuit, ctrl,
                                         dt, t_max, laps, use_stf=False)
            s_wpk, s_res = _circuit_whkm(ac, wind(ws, seed), circuit, ctrl,
                                         dt, t_max, laps, use_stf=True)
            base_vals.append(b_wpk)
            stf_vals.append(s_wpk)
            if abs(ws - highlight) < 1e-9 and seed == seeds[0]:
                va_timeline = {"baseline": b_res, "stf": s_res}
        bm, sm = float(np.mean(base_vals)), float(np.mean(stf_vals))
        sweep[ws] = {
            "baseline_whkm_mean": round(bm, 5),
            "baseline_whkm_min": round(float(np.min(base_vals)), 5),
            "baseline_whkm_max": round(float(np.max(base_vals)), 5),
            "stf_whkm_mean": round(sm, 5),
            "stf_whkm_min": round(float(np.min(stf_vals)), 5),
            "stf_whkm_max": round(float(np.max(stf_vals)), 5),
            "saving_pct": round(100.0 * (bm - sm) / bm, 3) if bm > 0 else 0.0,
        }

    out_dir = os.path.join("results", cfg["experiment_name"])
    os.makedirs(out_dir, exist_ok=True)
    metrics = {"source_concept": "MacCready speed-to-fly, zero-lift case",
               "sweep": {str(k): v for k, v in sweep.items()}}
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    from . import plotting
    plotting.plot_whkm_vs_wind(sweep, os.path.join(out_dir, "whkm_vs_wind.png"))
    plotting.plot_saving_vs_wind(sweep, os.path.join(out_dir, "saving_vs_wind.png"))
    if va_timeline:
        plotting.plot_va_cmd_timeline(
            va_timeline, os.path.join(out_dir, "va_cmd_timeline.png"), highlight)

    _print_sweep_table(sweep, highlight)
    print(f"\nWrote plots + metrics.json to {out_dir}/")
    return metrics


def _print_sweep_table(sweep, highlight):
    print("\n  Speed-to-fly sweep: circuit Wh/km, baseline vs energy-aware")
    print("  " + "-" * 62)
    print(f"  {'wind [m/s]':>10}{'baseline':>14}{'speed-to-fly':>16}{'saving %':>12}")
    print("  " + "-" * 62)
    for ws in sorted(sweep):
        s = sweep[ws]
        mark = "  <--" if abs(ws - highlight) < 1e-9 else ""
        print(f"  {ws:>10.1f}{s['baseline_whkm_mean']:>14.4f}"
              f"{s['stf_whkm_mean']:>16.4f}{s['saving_pct']:>12.2f}{mark}")
    print("  " + "-" * 62)


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("usage: python -m src.simulate <config.yaml>")
        return 1
    with open(argv[0]) as f:
        mode = (yaml.safe_load(f) or {}).get("mode", "baseline")
    if mode == "speed_to_fly_sweep":
        run_speed_to_fly_sweep(argv[0])
    else:
        run_experiment(argv[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
