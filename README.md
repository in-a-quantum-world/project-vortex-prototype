# Project Vortex: Atmospheric Energy Harvesting UAV

> **Researching whether autonomous UAVs can exploit naturally occurring atmospheric energy to extend flight endurance and enable new classes of long-duration missions.**

## Overview

This repository contains the research, analysis, modelling, technical exploration, and product development work behind an **atmospheric energy harvesting UAV concept**.
The central question behind the project is:

> **Can an autonomous aircraft intelligently exploit wind gradients, turbulence, thermals, and other naturally occurring atmospheric phenomena to recover energy during flight and substantially extend its endurance?**

Rather than treating the atmosphere purely as a disturbance to overcome, this project investigates whether it can be treated as an **energy source**. The concept draws inspiration from phenomena such as **dynamic soaring**, used by certain seabirds to extract energy from wind gradients, while exploring how modern sensing, control, optimisation and autonomy could allow a UAV to perform analogous manoeuvres deliberately and reliably.

The long-term goal is not simply to build a drone that can stay airborne slightly longer. The ambition is to investigate whether atmospheric energy harvesting could enable **persistent, low-logistics autonomous flight** in scenarios where conventional battery-powered or fuel-dependent UAVs face severe endurance constraints.

---

# The Core Problem Seen

UAV endurance remains one of the fundamental constraints on autonomous aerial systems.

Current approaches generally rely on:

* Larger or better batteries
* Internal combustion or hybrid propulsion
* Solar power
* More efficient motors and propellers
* Reduced payload
* Frequent recharging, refuelling or vehicle replacement

Each approach involves trade-offs.

Battery-powered UAVs are limited by energy density. Increasing battery capacity increases mass, which can itself increase the energy required to fly. Fuel-based systems improve endurance but introduce additional logistical, mechanical and environmental constraints. Solar systems are dependent on environmental conditions, surface area and time of day.

This project explores a different possibility:

**Instead of carrying all of the energy required for a mission, can a UAV periodically recover usable energy from the environment while airborne?**

If viable, this could change the endurance problem from purely an onboard energy-storage problem into a **real-time energy acquisition and trajectory optimisation problem**.

---

# The Core Hypothesis

The project is based on the hypothesis that, in certain atmospheric environments, an autonomous UAV could:

1. **Detect exploitable atmospheric energy**
2. **Estimate the local structure and evolution of that energy**
3. **Plan trajectories that maximise net energy gain**
4. **Execute those trajectories autonomously**
5. **Return to an efficient mission trajectory once sufficient energy has been recovered**

The objective is therefore not necessarily continuous energy generation.

Instead, a vehicle could potentially alternate between:

> **Mission operation → atmospheric energy recovery → mission operation**

The key metric is **net energy gain**.

A manoeuvre is only useful if the energy extracted from the environment exceeds the additional energy lost through drag, control effort, propulsion and other inefficiencies.

---

# Potential Energy Sources

The research explores several possible atmospheric mechanisms.

## Dynamic Soaring

Dynamic soaring extracts energy from differences in wind velocity across space.

An aircraft repeatedly crosses regions of different wind speed and direction, using carefully controlled manoeuvres to increase its airspeed and recover energy from the wind gradient.

This is one of the strongest sources of inspiration for the project because:

* It has been demonstrated extensively in nature
* It is physically established rather than speculative
* It can potentially operate without requiring a large external energy-harvesting surface
* It creates an interesting autonomy and control problem

A major research question is whether dynamic soaring can be made sufficiently **autonomous, robust and generalisable** for practical UAV operations.

---

## Thermals and Vertical Air Movement

Rising air can potentially provide energy in a manner analogous to conventional gliding.

Autonomous detection and exploitation of:

* Thermals
* Updrafts
* Ridge lift
* Orographic effects
* Local vertical wind structures

could allow an aircraft to gain altitude with reduced or zero propulsion expenditure.

This raises the possibility of combining conventional gliding strategies with more advanced atmospheric sensing and energy-aware trajectory planning.

---

## Turbulence and Unsteady Atmospheric Flow

Turbulence is normally treated as a disturbance.

However, turbulence also represents kinetic energy distributed across changing flow structures.

A more speculative area of this research is whether an aircraft could identify and exploit favourable transient structures rather than simply rejecting all disturbances.

This is significantly more difficult than dynamic soaring or thermal exploitation because the environment is:

* Highly variable
* Difficult to predict
* Potentially stochastic
* Difficult to model accurately
* Safety-critical

The aim is therefore not to assume that arbitrary turbulence can be converted into useful energy. Instead, this work investigates whether specific **predictable or statistically exploitable atmospheric structures** could create net-energy-positive flight opportunities.

---

# The Technical Challenge

The difficult part of this idea is not proving that atmospheric energy exists.

It is proving that a practical UAV can extract enough of it **reliably, autonomously and with a positive net energy balance**.

This creates several interconnected problems.

## 1. Atmospheric State Estimation

The UAV must estimate variables such as:

* Local wind velocity
* Wind gradients
* Vertical air movement
* Turbulence intensity
* Energy availability
* Measurement uncertainty

These estimates may need to be generated using a combination of:

* IMU data
* GPS/GNSS
* Airspeed sensors
* Pressure sensors
* Vision
* Learned environmental models
* External weather information where available

---

## 2. Energy-Aware Trajectory Planning

The aircraft must determine not simply:

> **What trajectory reaches the destination?**

but:

> **What trajectory maximises mission utility while maintaining or increasing available energy?**

This transforms the problem into an optimisation or control problem involving potentially competing objectives:

* Energy expenditure
* Energy recovery
* Mission progress
* Safety
* Time
* Vehicle constraints
* Environmental uncertainty

Possible approaches include:

* Model Predictive Control
* Optimal control
* Trajectory optimisation
* Reinforcement learning
* Hybrid model-based and learned controllers

---

## 3. Real-Time Control

Even if an optimal manoeuvre can be calculated offline, the real atmosphere will not behave exactly like a simulation.

A practical system must therefore tolerate:

* Sensor noise
* Imperfect environmental models
* Changing wind conditions
* Actuator limitations
* Unexpected disturbances

The controller must operate fast enough to respond to the atmosphere while remaining stable and safe.

---

## 4. Energy Accounting

One of the most important parts of this project is rigorous energy accounting.

It is easy to demonstrate an aircraft gaining altitude or increasing airspeed.

That does **not** automatically mean useful energy has been harvested.

The system must account for:

* Propulsive energy consumed
* Electrical energy recovered or saved
* Kinetic energy changes
* Potential energy changes
* Aerodynamic losses
* Drag
* Control effort

The fundamental success criterion is:

> **Does exploiting the atmospheric manoeuvre improve total mission endurance compared with the best conventional flight strategy?**

---

# Proposed System Architecture

The eventual system can be thought of as four major layers:

```text
┌─────────────────────────────────────────────┐
│              Mission Objective              │
│     Where does the UAV need to operate?     │
└──────────────────────┬──────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────┐
│          Atmospheric Energy Manager         │
│ Detects whether energy recovery is valuable │
└──────────────────────┬──────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────┐
│        State Estimation + Environment       │
│   Wind, gradients, turbulence, uncertainty  │
└──────────────────────┬──────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────┐
│         Trajectory Planning + Control       │
│     Executes safe energy-aware manoeuvres   │
└──────────────────────┬──────────────────────┘
                       │
                       ▼
                 Physical UAV
                       │
                       ▼
                Sensor feedback
```

The long-term objective is a **closed-loop system** in which the UAV continuously:

1. Observes the atmosphere
2. Estimates energy opportunities
3. Predicts the value of exploiting them
4. Selects an action
5. Executes the manoeuvre
6. Measures the resulting energy state
7. Updates its model

---

# Why This Could Matter

The strongest value proposition is likely to exist where **endurance is significantly more valuable than speed or payload capacity**.

Potential high-demand scenarios could include:

## Persistent Surveillance and Monitoring

Examples include:

* Border monitoring
* Maritime surveillance
* Infrastructure inspection
* Environmental monitoring
* Search operations
* Disaster-zone observation

Many of these missions are constrained by the need to repeatedly land, recharge, refuel or replace aircraft.

An aircraft capable of significantly extending its operational duration could reduce:

* Operational interruptions
* Fleet requirements
* Human intervention
* Logistics requirements

---

## Remote and Logistically Difficult Environments

The technology could be particularly valuable where recharging or recovery is difficult.

Examples may include:

* Oceans
* Mountainous regions
* Large remote areas
* Disaster zones
* Areas with limited infrastructure

In these scenarios, the value of endurance may be disproportionately high.

---

## Defence and Security

A potential long-term application is persistent intelligence, surveillance and reconnaissance.

This is attractive because endurance can directly influence:

* Area coverage
* Time on station
* Number of vehicles required
* Frequency of recovery operations
* Operational logistics

However, defence is **not the only possible application**, and the project should not depend on defence adoption alone.

---

# What Makes This Different?

There is already significant research into:

* Dynamic soaring
* Autonomous soaring
* Thermal exploitation
* Wind-aware path planning
* Energy-efficient UAV flight
* Bio-inspired flight
* Atmospheric sensing

That does **not** automatically make this project redundant.

The potential opportunity lies in the gap between scattered research results and an integrated system capable of answering:

> **Can atmospheric energy exploitation become a practical, autonomous endurance technology rather than an isolated research demonstration?**

The potential differentiator is therefore not necessarily the discovery of an entirely new physical phenomenon.

It may instead be the development of a system that integrates:

* Atmospheric perception
* Energy opportunity detection
* Predictive modelling
* Real-time decision-making
* Trajectory optimisation
* Robust control

into a practical platform.

The key question for the project is whether this integration produces a **large enough real-world endurance advantage** to justify a new product.

---

# Research Questions

The project is currently investigating questions including:

### Physics and Feasibility

* Under what atmospheric conditions is net energy extraction possible?
* How much energy can realistically be recovered?
* What vehicle characteristics maximise energy recovery?
* How sensitive is performance to atmospheric uncertainty?

### Control

* Can a UAV autonomously detect favourable energy structures?
* What control strategies work best under uncertainty?
* Can energy-recovery manoeuvres be performed reliably in real time?

### Machine Learning

* Can learned models improve prediction of exploitable atmospheric structures?
* Can reinforcement learning discover strategies that conventional controllers miss?
* How can learning-based systems be constrained to remain safe and physically valid?

### Product Viability

* How much additional endurance is required to create significant commercial value?
* Which missions experience the greatest cost from current endurance limitations?
* What existing technologies compete with this approach?
* Is the eventual product a vehicle, an autonomy stack, or a technology that can be integrated into existing UAV platforms?

---

# Current Development Approach

The project follows a staged approach.

## Phase 1 — Literature and Competitive Research

* [x] Establish the initial concept
* [x] Investigate existing work on dynamic soaring and atmospheric energy extraction
* [x] Explore related research areas
* [x] Begin competitive and market analysis
* [x] Identify potential applications
* [ ] Map the research landscape systematically
* [ ] Identify the most defensible technical gap

## Phase 2 — Mathematical and Physical Modelling

* [ ] Develop simplified atmospheric models
* [ ] Model aircraft dynamics
* [ ] Define rigorous energy accounting
* [ ] Simulate energy-recovery manoeuvres
* [ ] Identify theoretically favourable operating conditions

## Phase 3 — Simulation and Control

* [ ] Build a simulation environment
* [ ] Implement baseline controllers
* [ ] Implement energy-aware trajectory planning
* [ ] Compare conventional flight against atmospheric-energy strategies
* [ ] Investigate optimisation and learning-based approaches

## Phase 4 — Prototype Validation

* [ ] Define a minimum viable airframe
* [ ] Select sensing requirements
* [ ] Validate algorithms in increasingly realistic environments
* [ ] Test whether simulated energy gains survive real-world uncertainty

## Phase 5 — Product Development

* [ ] Identify the strongest initial market
* [ ] Define the minimum valuable endurance improvement
* [ ] Determine the product architecture
* [ ] Develop a defensible technical and commercial thesis

---

# Repository Structure

The repository is intended to organise work across research, modelling, software and product development.

```text
.
├── research/
│   ├── literature/
│   ├── dynamic_soaring/
│   ├── atmospheric_energy/
│   └── competitive_landscape/
│
├── modelling/
│   ├── aircraft_dynamics/
│   ├── atmospheric_models/
│   └── energy_analysis/
│
├── simulation/
│   ├── environments/
│   ├── controllers/
│   ├── optimisation/
│   └── experiments/
│
├── ml/
│   ├── state_estimation/
│   ├── prediction/
│   └── reinforcement_learning/
│
├── hardware/
│   ├── airframe/
│   ├── sensors/
│   └── electronics/
│
├── business/
│   ├── market_research/
│   ├── applications/
│   ├── competition/
│   └── pitch/
│
└── docs/
    ├── decisions/
    ├── ideas/
    └── roadmap/
```

The exact structure may evolve as the project becomes more technically defined.

---

# Key Principle

This repository should not become a collection of ideas that merely sound interesting.

Every major claim should eventually be tested against one of the following:

* **Physics**
* **Simulation**
* **Experiment**
* **Market reality**

The project succeeds only if it can demonstrate a meaningful answer to:

> **Can an autonomous UAV exploit the atmosphere in a way that produces a repeatable, measurable and commercially valuable endurance advantage?**

---

# Long-Term Vision

The long-term vision is to move beyond UAVs that simply **endure the atmosphere**.

The goal is to investigate autonomous aerial systems that:

> **Perceive the environment, understand its energy structure, and actively use the atmosphere as part of their energy system.**

If successful, this could represent a different approach to autonomous flight:

**The environment is not just something the aircraft flies through. It becomes part of the aircraft's energy strategy.**

---

## Status

**Early-stage research and development.**

The project is currently focused on validating the underlying technical and commercial thesis before committing to a specific vehicle architecture or product direction.

---

*This repository documents an evolving research project. Assumptions, designs and conclusions should be treated as hypotheses until supported by analysis, simulation or experimental evidence.*
