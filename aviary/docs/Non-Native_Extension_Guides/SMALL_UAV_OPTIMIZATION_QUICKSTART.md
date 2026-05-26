# Small UAV Optimization Quickstart

This guide shows the minimum pieces you need to create and run a simple Aviary
optimization like `horizontal_small_uav`.

## 1. Create The Aircraft Folder

Make one folder for the aircraft:

```text
aviary/models/aircraft/my_small_uav/
  my_small_uav.csv
  phase_info.py
  run_my_small_uav.py
  README.md
```

Use the CSV for aircraft data, `phase_info.py` for Dymos flight phases, and the run
script for building and running the Aviary problem.

## 2. Fill The CSV

The CSV is where you define the aircraft:

```text
aircraft:design:gross_mass,15.0,kg
aircraft:design:empty_mass,3.5,kg
aircraft:fuselage:length,2.0,m
aircraft:fuselage:max_width,0.30,m
aircraft:fuselage:max_height,0.25,m
aircraft:wing:area,0.45,m**2
aircraft:wing:span,1.8,m
aircraft:horizontal_tail:area,0.15,m**2
aircraft:fuel:total_capacity,8.0,kg
settings:aerodynamics_method,FLOPS
settings:mass_method,FLOPS
settings:problem_type,fallout
```

Change geometry by editing the matching `aircraft:*` variables. For example, to make
the wing larger, change `aircraft:wing:area` or `aircraft:wing:span`.

## 3. Put Flight Phases In `phase_info.py`

Keep the Dymos phase dictionary separate from the runner:

```python
from aviary.variable_info.enums import Transcription

phase_info = {
    'pre_mission': {'include_takeoff': False, 'optimize_mass': False},
    'climb': {
        'user_options': {
            'num_segments': 4,
            'order': 3,
            'mach_initial': (0.12, 'unitless'),
            'mach_final': (0.25, 'unitless'),
            'altitude_initial': (50.0, 'm'),
            'altitude_final': (5000.0, 'm'),
            'transcription': Transcription.COLLOCATION,
        },
    },
    'cruise': {...},
    'post_mission': {'include_landing': False, 'constrain_range': False},
}
```

This keeps the run script easier to read.

## 4. Build The Aviary Problem

The runner usually follows this pattern:

```python
prob = av.AviaryProblem(problem_type=av.ProblemType.FALLOUT)
prob.load_inputs(aircraft_data=AIRCRAFT_DATA, phase_info=phase_info)
prob.load_external_subsystems([SmallTurbojetModel()])

prob.check_and_preprocess_inputs()
prob.add_pre_mission_systems()
prob.add_phases()
prob.add_post_mission_systems()
prob.link_phases()
```

Then add the optimizer:

```python
prob.add_driver('IPOPT', max_iter=500)
```

IPOPT requires `pyoptsparse`. Use the Aviary conda environment if plain `python` does
not have it.

## 5. Add A New Subsystem

Import the subsystem class and pass it into `load_external_subsystems`:

```python
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetModel

prob.load_external_subsystems([SmallTurbojetModel()])
```

For a new custom subsystem, create an OpenMDAO component or group, wrap it in an Aviary
builder/model class, then load it the same way.

## 6. Add Design Variables

A design variable is something the optimizer can change:

```python
prob.model.add_design_var(
    av.Aircraft.Wing.SPAN,
    lower=1.34,
    upper=2.0,
    units='m',
    ref=1.8,
)
```

Use bounds to keep the optimizer in a reasonable design space.

External subsystems can also provide design variables through their builder's
`get_design_vars()` method.

## 7. Add Constraints

A constraint is a rule the optimizer must satisfy:

```python
prob.model.add_constraint(
    'pre_mission.propulsion.small_turbojet:mass',
    upper=5.0,
    units='kg',
    ref=5.0,
)
```

Common constraints are max engine mass, minimum range, fuel capacity, wing loading,
thrust-to-weight ratio, or stability margins.

## 8. Add A Fuel-Mass Budget

Aviary's built-in fuel capacity constraint checks tank capacity:

```text
fuel used <= aircraft:fuel:total_capacity
```

For a small UAV, you may also want fuel to compete with payload and engine mass:

```text
available fuel = gross mass - empty mass - payload mass - engine mass
fuel budget margin = available fuel - mission fuel
```

Add a small OpenMDAO component that computes those two values, connect the optimized
engine mass into it, and constrain the margin:

```python
prob.model.connect(
    'pre_mission.propulsion.small_turbojet:mass',
    'fuel_budget_estimate.engine_mass',
)

prob.model.add_constraint(
    'horizontal_small_uav:fuel_budget_margin',
    lower=0.0,
    units='kg',
    ref=8.0,
)
```

This makes the optimizer choose between a heavier engine with less fuel, or a lighter
engine with more fuel. Keep the normal Aviary fuel-capacity constraint too, so the
final fuel is limited by both tank volume and aircraft mass budget.

## 9. Add A Soft Target

A soft target is a preferred value that the optimizer should try to stay near, without
making the run fail if it is not exact. The simplest method is to use a relaxed
constraint band and print the target error.

Example: prefer `5%` static margin, but accept `3%` to `10%`:

```python
STATIC_MARGIN_TARGET = 0.05
STATIC_MARGIN_BOUNDS = (0.03, 0.10)

prob.model.add_constraint(
    av.Aircraft.Design.STATIC_MARGIN,
    lower=STATIC_MARGIN_BOUNDS[0],
    upper=STATIC_MARGIN_BOUNDS[1],
    ref=0.10,
)
```

Then report how close the final design came to the target:

```python
static_margin = prob.get_val(av.Aircraft.Design.STATIC_MARGIN).item()
print(f'Static Margin        = {100.0 * static_margin:.2f} %')
print(f'Static Margin Target = {100.0 * STATIC_MARGIN_TARGET:.2f} %')
print(f'Static Margin Error  = {100.0 * (static_margin - STATIC_MARGIN_TARGET):.2f} %')
```

This is not a true penalty objective. It is a practical first step: the optimizer must
stay inside the acceptable band, and the report tells you whether it landed near the
preferred value. A stricter soft target can be built later with a custom combined
objective, such as maximizing range while lightly penalizing static-margin error.

## 10. Add The Objective And Run

For a fallout mission, Aviary can add the range objective:

```python
prob.add_objective()
prob.setup()
prob.set_initial_guesses()
prob.run_aviary_problem()
prob.cleanup()
```

Run from the repository root:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/my_small_uav/run_my_small_uav.py
```

## 11. Open Results In The Dashboard

After a successful run, Aviary writes an output folder like:

```text
run_my_small_uav_out/
```

Open the dashboard with:

```powershell
aviary dashboard run_my_small_uav
```

If the command is not on your PATH, use the environment Python/Scripts version:

```powershell
& C:/Software/Anaconda/envs/aviary/Scripts/aviary.exe dashboard run_my_small_uav
```

The dashboard lets you inspect reports, N2, optimizer output, trajectory results, and
mission summaries.
