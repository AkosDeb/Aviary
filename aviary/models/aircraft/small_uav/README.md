# Small UAV Aviary Example

This directory contains a lightweight exploratory Aviary model for a small turbojet UAV.
It is intended for learning and experimentation with Aviary/OpenMDAO optimization, not
for validated aircraft design.

The current setup runs a **fallout mission**: the aircraft has a fixed gross mass and
available fuel, and Aviary optimizes the trajectory/design variables to maximize range.

---

## Files

```text
small_uav/
  small_uav.csv              Aircraft and mission input deck
  run_small_uav_mission.py   Range optimization runner
  phase_info.py              Standalone phase-info variant
  README.md                  This guide
```

Generated run outputs are written near the repository root, usually:

```text
run_small_uav_mission_out/
```

---

## Aircraft Concept

Baseline aircraft inputs are defined in `small_uav.csv`.

Key values:

```text
Gross mass       12.0 kg
Payload          2.5 kg misc cargo
Wing area        0.45 m^2
Initial span     1.8 m
Fuselage length  2.0 m
Cruise altitude  5000 m
Cruise Mach      0.25
Fuel capacity    6.0 kg
Engine           1 scaled turbojet deck
```

The model uses:

```text
settings:aerodynamics_method,FLOPS
settings:mass_method,FLOPS
settings:problem_type,fallout
```

FLOPS was not developed for this very small scale. Treat the output as a sandbox for
understanding coupling, derivatives, and optimization behavior.

---

## Geometry Coupling

For the FLOPS geometry path, wing aspect ratio should be computed from wing span and
wing area:

```text
aspect_ratio = span^2 / area
```

For this reason, `aircraft:wing:aspect_ratio` is commented out in `small_uav.csv`.
If it is provided as a fixed aircraft-data value, Aviary treats it as an override and
disconnects the computed aspect ratio from the optimization variable.

Current intended geometry chain:

```text
wing span + fixed wing area
  -> computed aspect ratio
  -> aerodynamics and wing mass
  -> mission range
```

The wing wetted area is also left to FLOPS geometry so parasite-drag-related quantities
can respond to geometry changes.

---

## Optimization Setup

The main runner is:

```text
run_small_uav_mission.py
```

It creates:

```python
av.AviaryProblem(problem_type=av.ProblemType.FALLOUT)
```

The current manually added design variables are:

```text
aircraft:wing:span
  lower = 1.0 m
  upper = 2.0 m
  ref   = 1.8 m

aircraft:engine:scale_factor
  lower = 0.0002
  upper = 0.005
  ref   = 0.0008
```

The objective is Aviary's fallout range objective, added through:

```python
prob.add_objective()
```

The climb and cruise phases use collocation transcription with fixed Mach and altitude
endpoints. Mass bounds are relaxed down to `6 kg` so the fallout mission can burn nearly
all available fuel without hitting an artificial climb mass floor.

---

## Running The Optimization

From the repository root:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/small_uav/run_small_uav_mission.py
```

Typical successful output includes:

```text
Optimization terminated successfully
Wing Span
Wing Area
Wing Aspect Ratio
Range
Total Fuel
Gross Mass
Empty Mass
Wing Loading
T/W Ratio
```

The script also writes a simple payload/range CSV report:

```text
run_small_uav_mission_out/reports/payload_range_data.csv
```

---

## XDSM Generation

The repository includes `universal_xdsm_creator.py`, which can infer a high-level XDSM
from this optimization script.

Generate `.tex` and `.tikz` only:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm `
  --no-pdf
```

Generate `.tex`, `.tikz`, and PDF if Docker Desktop is running:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm
```

Outputs are written to:

```text
xdsm_outputs/small_uav_auto_xdsm.tex
xdsm_outputs/small_uav_auto_xdsm.tikz
xdsm_outputs/small_uav_auto_xdsm.pdf
```

The PDF is only produced if Docker can run the TeXLive container.

---

## Important Warnings

### FLOPS Scale Mismatch

This aircraft is much smaller than the aircraft FLOPS correlations were designed for.
Mass, drag, nacelle, and fuel-system estimates can be unrealistic.

### Engine Scaling

The engine is scaled from a much larger reference engine deck. The scale factor changes
available thrust, engine-related mass, and propulsion performance. Very small scale
factors may push the engine map outside its physically meaningful range.

### Fuel Capacity Constraint

`aircraft:fuel:ignore_fuel_capacity_constraint` is set to `True` for this experiment.
That avoids an optimizer issue where the excess fuel capacity constraint is effectively
constant with respect to the active design variables. Fuel use is still reported, but
the fuel-capacity constraint is not enforced.

### Bounds Matter

The optimizer may prefer low span because this toy FLOPS model can penalize wetted area
and mass more strongly than it rewards induced-drag reduction. If you want a more
wing-like UAV design space, set the lower span bound based on a minimum aspect ratio.

For example, with fixed wing area `0.45 m^2`:

```text
AR >= 4  ->  span >= sqrt(4 * 0.45) = 1.34 m
AR >= 6  ->  span >= sqrt(6 * 0.45) = 1.64 m
```

---

## Recommended Experiments

1. Run the baseline optimization and record range, span, AR, and T/W.
2. Increase the lower span bound to enforce `AR >= 4` or `AR >= 6`.
3. Try narrower engine-scale bounds around the current optimum.
4. Compare fixed engine scale versus optimized engine scale.
5. Replace the scaled large-engine deck with a custom small turbojet deck if real data is available.
6. Add constraints for climb rate, minimum thrust margin, or minimum aspect ratio.

---

## Related Documentation

```text
aviary/docs/Non-Native_Extension_Guides/UNIVERSAL_XDSM_DOCUMENTATION.md
universal_xdsm_creator.py
aviary/variable_info/variables.py
aviary/models/engines/
```

---

## Status

This example is an experimental MDAO sandbox for understanding Aviary coupling and
derivatives on a small UAV-like configuration.

TODO 
- change the mission set up, add more mission phases so see how it performs and if we generate any unknonwn error between phase to phase interraction
- add more design variables to check if new error arises with more complexitiy
- Look more into the partial derivative calculation backend of the code as OpenMDAO fully rely on this and for the optimiziation to work fast and well does need to be understood fully 


