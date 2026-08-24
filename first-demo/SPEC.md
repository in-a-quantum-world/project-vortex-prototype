# Project Vortex — Technical Specification

> Section headers only for now. Equations to be filled in.

## §1 Airframe & energy model

## §2 Wind field interface

## §3 Thermal model + EKF

## §4 Orographic lift

## §5 Shear + Dryden

## §6 Planner

### §6.1 Airspeed scheduling (speed-to-fly)

Source concept: **MacCready speed-to-fly, zero-lift case.**

On each leg, command the airspeed that minimises electrical energy per unit
ground distance:

    Va* = argmin_Va  P_elec(Va) / Vg,     Vg = Va + w_along

where `w_along` is the signed along-track wind component (positive tailwind,
negative headwind) and `P_elec(Va) = (A·Va³ + B/Va) / (eta_prop·eta_motor)`
with `A = 0.5·rho·S·CD0`, `B = 2·k·W²/(rho·S)`.

- **Still air** (`w_along = 0`): the objective reduces to minimising drag
  `D = P_elec·eta/Va`, so `Va* = (B/A)^(1/4)`, the minimum-drag / best-glide
  speed (~10 m/s) — the propeller max-range speed.
- **Headwind** (`w_along < 0`): `Va*` increases (spend less time exposed to the
  opposing airmass).
- **Tailwind** (`w_along > 0`): `Va*` decreases, bounded below by a stall margin
  `1.1·V_stall`; upper bound `Va_max`.

Solved numerically (`scipy.optimize.minimize_scalar`, bounded) and cached per
leg. Uses the known mean wind for now; the wind estimator (§3) replaces it.
The classic MacCready construction adds airmass vertical velocity (thermal /
sink) as a vertical offset on the polar; that "lift" term is deferred — this is
the zero-lift specialisation.

## §7 Metrics
