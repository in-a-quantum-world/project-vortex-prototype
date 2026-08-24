"""Matplotlib plotting for baseline experiment outputs.

All figures are written to PNG; nothing is shown interactively (Agg backend).
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _circuit_outline(circuit: dict):
    L, W = float(circuit["length"]), float(circuit["width"])
    xs = [0, L, L, 0, 0]
    ys = [0, 0, W, W, 0]
    return xs, ys


def plot_trajectories(results, path: str, circuit: dict):
    fig, ax = plt.subplots(figsize=(8, 5))
    cx, cy = _circuit_outline(circuit)
    ax.plot(cx, cy, "k--", lw=1, alpha=0.5, label="commanded circuit")
    for r in results:
        ax.plot(r.x, r.y, lw=1.5, label=r.name)
    ax.set_xlabel("x East [m]")
    ax.set_ylabel("y North [m]")
    ax.set_title("Ground trajectories")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_timelines(results, path: str):
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for r in results:
        axes[0].plot(r.t, r.h, lw=1.2, label=r.name)
        axes[1].plot(r.t, r.Va, lw=1.2, label=r.name)
        axes[2].plot(r.t, r.power, lw=1.2, label=r.name)
    axes[0].set_ylabel("altitude [m]")
    axes[1].set_ylabel("airspeed [m/s]")
    axes[2].set_ylabel("power [W]")
    axes[2].set_xlabel("time [s]")
    axes[0].set_title("Altitude / airspeed / electrical power")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_whkm_bar(results, path: str):
    names = [r.name for r in results]
    aligned = [r.metrics["wh_per_km_aligned_leg"] for r in results]
    circuit = [r.metrics["wh_per_km"] for r in results]
    x = range(len(names))
    w = 0.38
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.bar([i - w / 2 for i in x], aligned, w, label="aligned leg")
    ax.bar([i + w / 2 for i in x], circuit, w, label="full circuit")
    ax.set_xticks(list(x))
    ax.set_xticklabels(names)
    ax.set_ylabel("Wh/km")
    ax.set_title("Specific energy consumption per case")
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)
    for i, v in enumerate(aligned):
        ax.text(i - w / 2, v, f"{v:.3f}", ha="center", va="bottom", fontsize=7)
    for i, v in enumerate(circuit):
        ax.text(i + w / 2, v, f"{v:.3f}", ha="center", va="bottom", fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------- #
# Speed-to-fly sweep plots
# ---------------------------------------------------------------------- #
def plot_whkm_vs_wind(sweep: dict, path: str):
    ws = sorted(sweep)
    b_mean = [sweep[w]["baseline_whkm_mean"] for w in ws]
    b_lo = [sweep[w]["baseline_whkm_min"] for w in ws]
    b_hi = [sweep[w]["baseline_whkm_max"] for w in ws]
    s_mean = [sweep[w]["stf_whkm_mean"] for w in ws]
    s_lo = [sweep[w]["stf_whkm_min"] for w in ws]
    s_hi = [sweep[w]["stf_whkm_max"] for w in ws]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ws, b_mean, "-o", color="tab:orange", label="baseline (fixed Va)")
    ax.fill_between(ws, b_lo, b_hi, color="tab:orange", alpha=0.2)
    ax.plot(ws, s_mean, "-o", color="tab:blue", label="speed-to-fly")
    ax.fill_between(ws, s_lo, s_hi, color="tab:blue", alpha=0.2)
    ax.set_xlabel("wind speed [m/s] (along long legs)")
    ax.set_ylabel("circuit Wh/km")
    ax.set_title("Energy consumption vs wind (shaded = seed spread)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_saving_vs_wind(sweep: dict, path: str):
    ws = sorted(sweep)
    saving = [sweep[w]["saving_pct"] for w in ws]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(ws, saving, "-o", color="tab:green")
    ax.axhspan(4, 6, color="gray", alpha=0.15, label="expected 4-6% @ 5 m/s")
    ax.set_xlabel("wind speed [m/s] (along long legs)")
    ax.set_ylabel("energy saving [%]")
    ax.set_title("Speed-to-fly saving vs baseline")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    for w, v in zip(ws, saving):
        ax.text(w, v, f"{v:.1f}", ha="center", va="bottom", fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_va_cmd_timeline(results: dict, path: str, wind_speed: float):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    base, stf = results["baseline"], results["stf"]
    ax.plot(base.t, base.va_cmd, color="tab:orange", lw=1.5,
            label="baseline commanded Va")
    ax.plot(stf.t, stf.va_cmd, color="tab:blue", lw=1.5,
            label="speed-to-fly commanded Va")
    ax.plot(stf.t, stf.Va, color="tab:blue", lw=0.8, ls="--", alpha=0.6,
            label="speed-to-fly actual Va")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("airspeed [m/s]")
    ax.set_title(f"Commanded airspeed vs time ({wind_speed:.0f} m/s wind)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
