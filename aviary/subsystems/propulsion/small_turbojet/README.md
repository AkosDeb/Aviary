# Adding a Regression-Based Engine Model to Aviary

This guide explains the pattern used by the `small_turbojet` example. The goal is to add a new
engine model whose behavior comes from regression equations rather than an engine deck table.

The same pattern can be used for any new engine model that maps design variables and mission
conditions into thrust, fuel flow, temperatures, emissions, or other propulsion outputs.

## Mental Model

Aviary separates engine modeling into two parts:

1. Pre-mission engine design calculations.
2. Mission engine performance calculations.

For a regression-based engine, the usual flow is:

```text
engine design variables
    -> pre-mission regressions
        -> static engine properties
            -> mission performance model
                -> thrust, fuel flow, temperatures
```

For the small turbojet example:

```text
diameter, length
    -> max thrust, mass, EGT, SFC
        -> throttle, max thrust, EGT, SFC
            -> thrust, thrust_max, fuel_flow_negative, T4
```

OpenMDAO and Aviary then use this computational chain during optimization. The optimizer changes
design variables, Aviary recomputes pre-mission quantities, Dymos flies the mission, and the
objective and constraints determine the next design update.

## File Layout

The example is organized as:

```text
aviary/subsystems/propulsion/small_turbojet/
  __init__.py
  README.md
  variables.py
  small_turbojet_builder.py
  model/
    __init__.py
    small_turbojet_premission.py
    small_turbojet_mission.py
  test/
    __init__.py
    test_small_turbojet.py
```

Use this layout for a new engine family because it keeps the Aviary wrapper, variable names,
OpenMDAO components, and tests separated.

## Variables

Custom variables are defined once in `variables.py`:

```python
class SmallTurbojetVariables:
    DIAMETER = 'small_turbojet:diameter'
    LENGTH = 'small_turbojet:length'
    MASS = 'small_turbojet:mass'
    EGT = 'small_turbojet:egt'
    SFC = 'small_turbojet:sfc'
```

Every other file imports these names. Do not redefine them in multiple files.

For this example, all custom variables use SI units:

```text
diameter: m
length: m
mass: kg
EGT: K
SFC: kg/(N*s)
```

Mission outputs can also be declared in SI units. OpenMDAO handles compatible unit conversion when
Aviary connects components.

## EngineModel Wrapper

The builder file contains the Aviary-facing wrapper:

```python
class SmallTurbojetModel(EngineModel):
    _default_name = 'small_turbojet'
    compute_max_values = True
```

This class does not contain the regression equations. It tells Aviary how to build and expose the
engine.

The important methods are:

```python
def build_pre_mission(self, aviary_inputs, subsystem_options=None):
    return SmallTurbojetPreMission()
```

This creates the OpenMDAO component that computes static engine design properties before mission
evaluation.

```python
def build_mission(self, num_nodes, aviary_inputs, user_options, subsystem_options):
    return SmallTurbojetMission(num_nodes=num_nodes)
```

This creates the OpenMDAO component that computes per-node mission performance.

```python
def get_design_vars(self, aviary_inputs=None):
    return {
        SmallTurbojetVariables.DIAMETER: {...},
        SmallTurbojetVariables.LENGTH: {...},
    }
```

This tells Aviary which variables the optimizer may change.

```python
def get_parameters(self, aviary_inputs=None, user_options=None, subsystem_options=None):
    return {
        SmallTurbojetVariables.DIAMETER: {...},
        SmallTurbojetVariables.LENGTH: {...},
        Aircraft.Engine.SCALED_SLS_THRUST: {...},
        SmallTurbojetVariables.EGT: {...},
        SmallTurbojetVariables.SFC: {...},
    }
```

This tells Aviary which fixed phase parameters should be available to the mission model.

```python
def get_timeseries(self, aviary_inputs=None, user_options=None, subsystem_options=None):
    return [
        Dynamic.Vehicle.Propulsion.THRUST,
        Dynamic.Vehicle.Propulsion.THRUST_MAX,
        Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
        Dynamic.Vehicle.Propulsion.TEMPERATURE_T4,
    ]
```

This asks Aviary/Dymos to record these mission outputs.

## Pre-Mission Component

`SmallTurbojetPreMission` is an OpenMDAO `ExplicitComponent`.

Its job is to turn design variables into static engine properties:

```text
diameter, length -> max thrust, mass, EGT, SFC
```

The current equations are placeholders:

```python
inlet_area = 0.25 * np.pi * diameter**2
engine_volume = inlet_area * length

outputs[Aircraft.Engine.SCALED_SLS_THRUST] = 20000.0 * inlet_area
outputs[SmallTurbojetVariables.MASS] = 1200.0 * engine_volume
outputs[SmallTurbojetVariables.EGT] = 850.0 + 250.0 * diameter + 50.0 * length
outputs[SmallTurbojetVariables.SFC] = 3.2e-5 - 1.5e-5 * diameter + 2.0e-6 * length
```

Replace these with trained regression equations when the real model is ready.

Good regression outputs for pre-mission are usually:

```text
max thrust
engine mass
EGT or T4 limit
design SFC
```

Avoid making engine mass an independent optimizer input unless that is physically intentional.
Mass should usually be predicted from engine geometry or sizing variables.

## Mission Component

`SmallTurbojetMission` is also an OpenMDAO `ExplicitComponent`.

Its job is to compute performance at each Dymos mission node:

```text
throttle, max thrust, EGT, SFC -> thrust, thrust_max, fuel flow, T4
```

The simplified model assumes the same performance at every altitude and Mach number:

```python
thrust = throttle * max_thrust
fuel_flow_negative = -thrust * sfc
temperature_T4 = egt
```

This is intentionally simple. Later, the mission regression can include:

```text
Mach
altitude
ambient temperature
throttle
diameter
length
max thrust
EGT
```

The component must remain smooth if it will be used in gradient-based optimization. Avoid hard
clipping, discontinuous conditionals, and regression extrapolation outside the training range.

## Derivatives

OpenMDAO optimizers need derivatives such as:

```text
d(thrust)/d(throttle)
d(fuel_flow)/d(SFC)
d(max_thrust)/d(diameter)
```

There are three common choices:

1. Write analytic derivatives in `compute_partials`.
2. Use finite difference with `declare_partials(..., method='fd')`.
3. Use complex step with `declare_partials(..., method='cs')`.

Analytic derivatives are fastest and most reliable for final optimization. Finite difference or
complex step can be useful while the regression equations are still changing.

The small turbojet example uses analytic derivatives so the test can verify the component
sensitivities.

## Using the Engine in Aviary

The engine is loaded as an external subsystem:

```python
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetModel

prob.load_external_subsystems([SmallTurbojetModel()])
```

Then Aviary can include its design variables, pre-mission component, mission component, parameters,
and timeseries outputs in the aircraft problem.

## Testing

The unit test is:

```text
aviary/subsystems/propulsion/small_turbojet/test/test_small_turbojet.py
```

It checks two things:

1. The pre-mission regression component produces expected static engine properties.
2. A coupled premission-to-mission model produces expected thrust, max thrust, fuel flow, and T4.

Run it with:

```text
python -m unittest aviary.subsystems.propulsion.small_turbojet.test.test_small_turbojet
```

In this workspace, the tested command was:

```text
C:/Software/Anaconda/envs/aviary/python.exe -m unittest aviary.subsystems.propulsion.small_turbojet.test.test_small_turbojet
```

## Implementation Checklist

For a new regression-based engine model:

1. Create a package under `aviary/subsystems/propulsion/<engine_name>/`.
2. Define custom variable names once in `variables.py`.
3. Create an `EngineModel` subclass in `<engine_name>_builder.py`.
4. Implement `build_pre_mission` if static design regressions are needed.
5. Implement `build_mission`; this is required.
6. Add design variables in `get_design_vars`.
7. Add static mission parameters in `get_parameters`.
8. Return useful mission outputs in `get_timeseries`.
9. Write component tests and derivative checks.
10. Only modify core Aviary propulsion files if the new engine needs outputs that
    `PropulsionMission` does not currently collect.

For most regression engine models, no changes to `propulsion_builder.py`,
`propulsion_premission.py`, or `propulsion_mission.py` are needed.
