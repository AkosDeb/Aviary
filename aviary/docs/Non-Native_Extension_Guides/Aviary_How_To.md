# NASA Aviary — Overview

> **Related:** For full detail on OpenMDAO optimization, solvers, and outputs → [OpenMDAO & MDAO Detail](https://app.notion.com/p/tytan-technologies/Methods-368667b889f080cfaae6fa5dffdd873d) Or in the file "OpenMDAO_How_To.md in Non-Native_Extension_Guides

---

## What Aviary Is

Aviary is NASA's open-source Python tool for aircraft **analysis, design, and optimization**. It wraps two legacy NASA sizing codes (GASP and FLOPS) in a modern framework and uses Dymos for trajectory optimization. Everything runs inside an OpenMDAO problem, so the aircraft design and the flight mission are optimized simultaneously as one coupled system.

---

## 1. The Three Interface Levels

| Level | How you interact | Use when |
|---|---|---|
| **Level 1** | CSV + `phase_info.py`, run via CLI | Quick sizing studies, no custom code |
| **Level 2** | Python script with `AviaryProblem` API | Custom subsystems, design vars, constraints |
| **Level 3** | Raw OpenMDAO, build the problem yourself | Full control, research, non-standard configs |

---

## 2. Project Folder Structure

```text
aviary/models/aircraft/my_uav/
  my_uav.csv          ← aircraft definition (geometry, mass, settings)
  phase_info.py       ← Dymos flight phases (climb, cruise, descent)
  run_my_uav.py       ← builds and runs the AviaryProblem
```

---

## 3. Aircraft CSV — Defining the Vehicle

Format: `variable_name, value, unit`

```text
aircraft:design:gross_mass,         15.0,   kg
aircraft:design:empty_mass,          3.5,   kg
aircraft:wing:area,                  0.45,  m**2
aircraft:wing:span,                  1.8,   m
aircraft:fuselage:length,            2.0,   m
aircraft:fuselage:max_width,         0.30,  m
aircraft:fuselage:max_height,        0.25,  m
aircraft:horizontal_tail:area,       0.15,  m**2
aircraft:fuel:total_capacity,        8.0,   kg
settings:aerodynamics_method,        FLOPS
settings:mass_method,                FLOPS
settings:problem_type,               fallout
```

---

## 4. Choosing FLOPS vs. GASP

```text
settings:aerodynamics_method,   FLOPS   # or GASP
settings:mass_method,           FLOPS   # or GASP
```

### FLOPS — NASA Langley
- Designed for **large conventional transports** (tube-and-wing jets)
- Six modules: weights, aerodynamics, propulsion scaling, mission performance, takeoff/landing, cost
- Weights from statistical regression per structural group (wing bending, fuselage, tails, landing gear, avionics)
- Mission uses **height-energy equations of motion**
- Best for: jets, transports, conventional airliner-type shapes

### GASP — NASA Ames
- Designed for **small fixed-wing aircraft** (piston → turboprop → business jet)
- Supports hybrid wing body and truss-braced wing
- Same empirical regression approach calibrated to lighter/smaller aircraft
- Mission uses **2-DOF equations of motion** (lift, drag, weight, thrust)
-

### ⚠️ What FLOPS and GASP Do NOT Model

This is critical to understand before trusting results:

Both FLOPS and GASP are **point-mass aerodynamic models**. They compute only:
- **Lift (CL)** and **Drag (CD)** as functions of Mach and altitude

They do **not** include:
- **Static stability** — no pitch/roll/yaw stability derivatives (Cmα, Cnβ, etc.)
- **Trim analysis** — the aircraft is assumed to fly trimmed with no computation of elevator/throttle deflections needed
- **Control authority** — no check that the tail/control surfaces can actually achieve the required moments
- **Dynamic stability** — no phugoid, short-period, dutch roll, spiral, or roll mode analysis
- **Angle of attack** — treated implicitly; not a tracked state in energy-state method
- **Structural loads from maneuvers** — no g-load coupling to structural sizing

For a UAV where stability and control authority are real design constraints, you need to add these as external subsystems or post-process the Aviary result separately (e.g. with AVL, OpenVSP, or a custom stability module). The optimizer will happily produce a design that is aerodynamically efficient but statically unstable or untrimmable unless you constrain it explicitly.

A common approach is to add a static margin constraint:
```python
prob.model.add_constraint(
    av.Aircraft.Design.STATIC_MARGIN,
    lower=0.05, upper=0.15, ref=0.10,
)
```
But this is a crude proxy — it does not replace a full stability analysis.

---

## 5. Equations of Motion — Choosing a Mission Model

Aviary offers three EOM approaches, set via the transcription in `phase_info`:

### Height-Energy (FLOPS)
- Treats the aircraft as a point mass with a single energy state
- No flight path angle — altitude and speed can be exchanged instantaneously
- Simpler, fewer variables, faster to solve
- Good for cruise-dominated missions where the climb/descent shape is not critical

```
h = (E - ½V²) / g
State: energy E
Control: Mach (speed)
```

### 2-DOF (GASP)
- Full force balance in the vertical plane
- Tracks altitude, speed, flight path angle γ simultaneously
- Physically more accurate, especially for steep climbs/descents

```
m·V̇ = T·cos(α) − D − W·sin(γ)
m·V·γ̇ = L + T·sin(α) − W·cos(γ)
ḣ  = V·sin(γ)
ẋ  = V·cos(γ)
States: altitude h, velocity V, flight path angle γ
Controls: throttle, angle of attack α
```

### SOLVED_2DOF
- Like 2-DOF but a nonlinear solver handles the defect constraints internally rather than the optimizer
- Useful for simple propagation (simulation, not optimization)
- Slower per iteration but can converge from worse initial guesses

---

## 6. Dymos — Segments, Order, and Collocation

Dymos turns the mission trajectory into a large nonlinear system solved simultaneously — not by stepping forward in time.

### What `num_segments` controls

Each phase is divided into `num_segments` polynomial pieces. Each segment is solved independently and continuity constraints enforce that the state is continuous at segment boundaries.

```
Phase: climb
|--seg 1--|--seg 2--|--seg 3--|--seg 4--|
 t0                                    tf
```

- **More segments** = finer resolution of rapidly-changing behavior (steep climb, sharp Mach change)
- **Fewer segments** = faster solve, less accurate for complex profiles
- Start with `num_segments=4`. Increase to 6–8 if the trajectory shape looks wrong

### What `order` controls

Within each segment, states are represented as a **polynomial of this order**. The order determines how many collocation (evaluation) nodes exist within the segment.

- `order=3` → 3 nodes per segment (minimum, default)
- `order=5` → 5 nodes per segment (more accurate, more expensive)
- Higher order is good for smoothly varying states; multiple lower-order segments is better for trajectories with distinct sub-phases

Total collocation nodes in a phase ≈ `num_segments × order`

### Transcription methods

| Transcription | How it works | Use when |
|---|---|---|
| `GaussLobatto` | Nodes at segment endpoints + interior; interpolates between state and collocation nodes | Default, most missions |
| `Radau` | All nodes are design variables; no interpolation step needed | Better initial guess tolerance; bang-bang controls |
| `Birkhoff` | Single large segment, integral form of defects; better conditioning | Research; slow but high-quality solutions |

```python
from dymos import Transcription

# In phase_info user_options:
'transcription': Transcription.COLLOCATION,      # GaussLobatto
'transcription': Transcription.RADAU_PS,         # Radau
```

**GaussLobatto vs Radau in practice:**
GaussLobatto results in a smaller NLP (fewer design variables) because state values are only required at segment endpoints. Radau requires state values at every node, making it larger but more robust when your initial guess is poor — the interpolation step in GaussLobatto can produce wild out-of-range values if the guess is far off, which breaks nested solvers.

---

## 7. Flight Phases — Full `phase_info` Reference

### Structure of a phase

```python
phase_info = {
    'pre_mission': { ... },
    'climb':       { 'user_options': { ... } },
    'cruise':      { 'user_options': { ... } },
    'descent':     { 'user_options': { ... } },
    'post_mission':{ ... },
}
```

### `pre_mission` and `post_mission`

```python
'pre_mission': {
    'include_takeoff': False,   # True = use Dymos takeoff phase (2DOF only)
    'optimize_mass':   False,   # True = let optimizer choose takeoff gross mass
},
'post_mission': {
    'include_landing':    False,
    'constrain_range':    False,  # True = enforce a target range
    'target_range':       (500.0, 'km'),  # used if constrain_range=True
},
```

> For detailed FLOPS-based takeoff and landing modelling see → [Aviary: FLOPS Detailed Takeoff and Landing](https://openmdao.github.io/Aviary/user_guide/FLOPS_based_detailed_takeoff_and_landing.html)

### Core `user_options` for a flight phase

```python
'climb': {
    'user_options': {
        # --- Dymos discretisation ---
        'num_segments': 4,
        'order':        3,
        'transcription': Transcription.COLLOCATION,   # GaussLobatto

        # --- Altitude ---
        'altitude_initial':      (50.0,   'm'),   # fixed start (constraint added)
        'altitude_final':        (5000.0, 'm'),   # fixed end   (constraint added)
        'altitude_bounds':       ((0.0, 6000.0), 'm'),  # hard limits throughout phase
        'altitude_optimize':     True,             # False = fix altitude at initial_conditions values
        'altitude_ref':          (1000.0, 'm'),   # scaling: value/ref ≈ 1.0
        'no_descent':            True,             # prevent unexpected descent during climb
        'no_climb':              False,

        # --- Mach ---
        'mach_initial':          (0.12, 'unitless'),
        'mach_final':            (0.25, 'unitless'),
        'mach_bounds':           ((0.05, 0.40), 'unitless'),
        'mach_optimize':         True,
        'mach_ref':              (0.3,  'unitless'),

        # --- Throttle ---
        'throttle_enforcement':  'path_constraint',  # 'bounded', 'boundary_constraint', 'control'
        'throttle_bounds':       ((0.0, 1.0), 'unitless'),
        'throttle_optimize':     True,

        # --- Mass ---
        'mass_initial':          (None, 'kg'),   # None = connected from upstream phase
        'mass_final':            (None, 'kg'),
        'mass_bounds':           ((0.0, None), 'kg'),
        'mass_ref':              (10.0, 'kg'),   # scale for UAV; use (70000.0, 'kg') for airliner

        # --- Misc ---
        'ground_roll':           False,   # True only during takeoff roll
        'reserve':               False,   # True = this is a reserve phase
        'required_available_climb_rate': (None, 'ft/s'),  # minimum climb capability
    },
},
```

### Optimizing vs. fixing altitude and Mach

The key distinction between `_optimize=True` and `_optimize=False`:

**`altitude_optimize: True`** (default)
- Altitude at every collocation node is a design variable
- The optimizer freely chooses the altitude profile across the phase, subject to `altitude_bounds`
- Only `altitude_initial` and `altitude_final` pin the endpoints if specified
- Use this for climb/descent where you want the optimal profile

**`altitude_optimize: False`**
- The optimizer cannot change altitude; you must supply values in `initial_conditions`
- Use this to enforce a fixed cruise altitude or a pre-specified profile

**Example — optimal climb to fixed cruise altitude:**

```python
'climb': {
    'user_options': {
        'num_segments': 5,
        'order': 3,
        'altitude_initial': (50.0,   'm'),
        'altitude_final':   (4000.0, 'm'),
        'altitude_bounds':  ((0.0, 5000.0), 'm'),
        'altitude_optimize': True,       # optimizer picks the climb profile
        'altitude_ref':     (1000.0, 'm'),
        'mach_initial':     (0.12, 'unitless'),
        'mach_final':       (0.25, 'unitless'),
        'mach_optimize':    True,        # optimizer picks speed schedule too
        'mach_ref':         (0.3, 'unitless'),
        'no_descent': True,
    },
},
'cruise': {
    'user_options': {
        'num_segments': 6,
        'order': 3,
        'altitude_initial': (4000.0, 'm'),
        'altitude_final':   (4000.0, 'm'),
        'altitude_optimize': False,      # fixed cruise altitude
        'mach_initial': (0.25, 'unitless'),
        'mach_final':   (0.25, 'unitless'),
        'mach_optimize': False,          # fixed cruise Mach
    },
},
```

### Phase types (2-DOF only)

For 2-DOF missions, `phase_type` lets you pick a specialized sub-model:

| `phase_type` | Description |
|---|---|
| `PhaseType.DEFAULT` | Standard flight phase (use for climb/descent) |
| `PhaseType.BREGUET_RANGE` | Analytic Breguet range cruise — fast but assumes constant CL/CD and SFC |
| `PhaseType.SIMPLE_CRUISE` | Simplified cruise, less assumptions than Breguet |

```python
'cruise': {
    'user_options': {
        'phase_type': PhaseType.BREGUET_RANGE,
        ...
    }
}
```

### Custom path constraints within a phase

You can add any variable as a path or boundary constraint directly in `user_options`:

```python
'constraints': {
    'flight_path_angle': {
        'equals': None,
        'lower': -10.0,
        'upper':  15.0,
        'loc':    'path',       # 'path' = at every node; 'initial'/'final' = boundary
        'units':  'deg',
        'type':   'path',
    },
    'throttle': {
        'lower': 0.0,
        'upper': 1.0,
        'loc':   'path',
        'units': 'unitless',
        'type':  'path',
    },
},
```

---

## 8. Fuel Mass Budget

Aviary's built-in fuel capacity check enforces:
```
fuel used ≤ aircraft:fuel:total_capacity
```

This is a **volumetric/tank limit** only. It does not check whether the aircraft can physically carry that fuel given the mass of everything else on board.

For a full mass budget, you need to close the loop explicitly:

```
available fuel = gross mass − empty mass − payload mass − all subsystem masses
fuel budget margin = available fuel − mission fuel used
```

Any subsystem whose mass is being sized by the optimizer (propulsion, structure, battery, etc.) competes for the same fuel budget. The optimizer cannot know about this competition unless you add a component that computes the margin and constrain it to be non-negative.

The general pattern:

```python
# 1. Create a component that computes the margin
#    Inputs: gross_mass, empty_mass, payload_mass, subsystem_masses..., mission_fuel_used
#    Output: fuel_budget_margin

# 2. Connect any optimized subsystem masses into it
prob.model.connect('pre_mission.subsystem_a:mass', 'fuel_budget.subsystem_a_mass')
prob.model.connect('pre_mission.subsystem_b:mass', 'fuel_budget.subsystem_b_mass')

# 3. Constrain margin ≥ 0
prob.model.add_constraint(
    'fuel_budget.margin',
    lower=0.0,
    units='kg',
    ref=5.0,   # scale to order-of-magnitude 1.0
)
```

This turns the fuel budget into an active trade-off: any subsystem that gets heavier (larger wing, bigger motor, more battery) directly reduces the available fuel, and the optimizer must balance all of them simultaneously. Without this constraint, the optimizer can freely increase subsystem masses without penalty, producing a design that has no room for the fuel it needs to fly the mission.

Keep Aviary's native tank capacity constraint active alongside this — the solution should be bounded by both the physical tank volume **and** the overall mass budget.

---

## 9. Building the AviaryProblem (Level 2)

```python
import aviary.api as av

prob = av.AviaryProblem()
prob.load_inputs('my_uav.csv', phase_info)
prob.load_external_subsystems([SmallTurbojetModel()])

prob.check_and_preprocess_inputs()
prob.add_pre_mission_systems()
prob.add_phases()
prob.add_post_mission_systems()
prob.link_phases()

prob.add_driver('IPOPT', max_iter=500)

prob.model.add_design_var(av.Aircraft.Wing.SPAN, lower=1.2, upper=2.5, units='m', ref=1.8)
prob.model.add_constraint('fuel_budget.margin', lower=0.0, units='kg', ref=5.0)

prob.add_objective()
prob.setup()
prob.set_initial_guesses()
prob.run_aviary_problem()
prob.cleanup()
```

---

## 10. External Subsystems

Use when built-in empirical equations don't fit your aircraft (custom propulsion, novel structure, stability model).

```python
from aviary.subsystem_builder import SubsystemBuilderBase

class MyPropBuilder(SubsystemBuilderBase):
    def build_pre_mission(self, aviary_inputs):
        return MyPropComponent()   # OpenMDAO Component or Group

prob.load_external_subsystems([MyPropBuilder()])
```

The external component must be wrapped as an OpenMDAO component — the underlying code does not have to be written in OpenMDAO, only the wrapper. For how to write components and provide analytic derivatives see → [OpenMDAO & MDAO Detail](<LINK_TO_OPENMDAO_PAGE>)

---

## 11. Problem Types

Set in CSV with `settings:problem_type`:

| Type | What it does |
|---|---|
| `fallout` | Fix mission definition, compute resulting range/fuel — optimizer maximizes range |
| `sizing` | Fix range target, size the aircraft to meet it |
| `alternate` | Off-design: fly a different mission with a pre-sized aircraft |

---

## 12. Running and Results

```powershell
# Run
& C:/Software/Anaconda/envs/aviary/python.exe aviary/models/aircraft/my_uav/run_my_uav.py

# Open dashboard
aviary dashboard outputs/run_my_uav_out
```

Output folder contents:

| File | What it contains |
|---|---|
| `n2.html` | Interactive data-flow diagram of the full model — use this first when debugging |
| `opt_report.html` | Optimizer convergence history, active constraints, objective value |
| `traj_results_report.html` | Altitude, Mach, fuel burn, throttle vs. time for all phases |
| `mass_breakdown.html` | Component weight breakdown |
| `IPOPT.out` | Raw solver log — watch `inf_pr` (constraint violation) and `inf_du` (optimality) |
| `aircraft_for_flight_simulator.csv` | Final sized aircraft in Aviary CSV format |

---

## Sources

- [Aviary GitHub](https://github.com/OpenMDAO/Aviary)
- [Aviary Documentation](https://openmdao.github.io/Aviary)
- [phase_info Options Reference](https://openmdao.github.io/Aviary/source_docs/phase_info_detailed.html)
- [FLOPS Detailed Takeoff and Landing](https://openmdao.github.io/Aviary/user_guide/FLOPS_based_detailed_takeoff_and_landing.html)
- [Dymos: What is Collocation?](https://openmdao.github.io/dymos/getting_started/collocation.html)
- [Aviary: Mission Analysis Theory](https://openmdao.github.io/Aviary/theory_guide/mission.html)
- [FLOPS Weights Method (NASA TM-2017)](https://ntrs.nasa.gov/citations/20170005851)
- [GASP Volume 1: Theoretical Development](https://ntrs.nasa.gov/citations/19810010562)
