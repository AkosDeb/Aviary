# Small UAV Optimization: Convergence Fixes

This document explains the changes made to resolve IPOPT non-convergence in the small UAV
range optimization (`aviary/models/aircraft/small_uav/run_small_uav_mission.py`) and the
engine design variable scaling improvements in the small turbojet builder.

---

## Problem

After 380+ IPOPT iterations the optimization failed to converge:

- Dual infeasibility grew continuously from ~1e1 to ~1e7.
- Nearly every iteration produced `Warning: Cutting back alpha due to evaluation error`,
  forcing step sizes as small as 7e-4 (10–11 line-search backtracks per step).
- The objective value barely moved throughout the entire run.

The evaluation errors meant the model was returning `NaN` or `Inf` at most trial points,
so IPOPT could not find a valid descent direction.

---

## Root Causes and Fixes

### 1. Cruise Mach set above the aerodynamics valid range

**File:** `run_small_uav_mission.py` — cruise phase `user_options`

**Before:**
```python
"mach_optimize": False,
"mach_final": (0.7, "unitless"),
```

**After:**
```python
"mach_optimize": False,
"mach_initial": (0.25, "unitless"),
"mach_final":   (0.25, "unitless"),
```

**Why this was the primary cause:**
The aircraft CSV specifies:

```text
aircraft:design:cruise_mach,0.25,unitless
mission:constraints:max_mach,0.6,unitless
```

Setting `mach_final=0.7` caused the cruise phase to interpolate through Mach numbers above
both the design cruise Mach and the `max_mach` constraint. The FLOPS computed aerodynamics
model evaluates lift and drag from subsonic correlations calibrated for this aircraft;
Mach values beyond the valid range produced `NaN`, triggering evaluation errors at almost
every IPOPT trial point.

Adding `mach_initial=0.25` also ensures Dymos sees a flat, consistent Mach profile for
the cruise phase, avoiding any interpolation ambiguity when the phase is linked to the
climb output.

---

### 2. Cruise phase missing `mass_ref`

**File:** `run_small_uav_mission.py` — cruise phase `user_options`

**Added:**
```python
"mass_ref": (MAX_TAKEOFF_MASS_KG, "kg"),
```

**Why:**
IPOPT works best when design variables and state variables are scaled to O(1) internally.
The climb phase already had `mass_ref`, but cruise did not. Without a reference value, the
mass state has no normalization hint for the optimizer, degrading gradient quality as the
fuel burns during cruise and mass drifts away from its initial value.

---

### 3. Cruise phase missing `throttle_enforcement`

**File:** `run_small_uav_mission.py` — cruise phase `user_options`

**Added:**
```python
"throttle_enforcement": "path_constraint",
```

**Why:**
The climb phase declared throttle enforcement as a path constraint. Without the same
declaration in cruise, IPOPT treated the throttle constraint differently between the two
phases. This asymmetry can introduce discontinuities in the KKT conditions at the phase
boundary, making it harder for IPOPT to satisfy dual feasibility.

---

### 4. `parallel_phases=True` without MPI installed

**File:** `run_small_uav_mission.py` — `build_problem()`

**Before:**
```python
prob.add_phases(parallel_phases=True)
```

**After:**
```python
prob.add_phases()
```

**Why:**
`parallel_phases=True` wraps the trajectory phases in an OpenMDAO `ParallelGroup`. Without
`mpi4py` installed, this falls back to serial execution but still changes how OpenMDAO
assembles the Jacobian. In some OpenMDAO/Dymos versions, the ParallelGroup wrapper can
cause the gradient assembly to be less efficient or to miss some cross-phase connections
when only one MPI rank is present.

Check if `mpi4py` is available before enabling parallel phases:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe -c "import mpi4py; print(mpi4py.__version__)"
```

If the command fails, keep `parallel_phases` at its default (off).

---

### 5. Engine design variables missing `ref` scaling

**File:** `aviary/subsystems/propulsion/small_turbojet/small_turbojet_builder.py`
— `get_design_vars()`

**Before:**
```python
f'pre_mission.propulsion.{SmallTurbojetVariables.DIAMETER}': {
    'units': 'm',
    'lower': 0.05,
    'upper': 0.272,
},
f'pre_mission.propulsion.{SmallTurbojetVariables.LENGTH}': {
    'units': 'm',
    'lower': 0.150,
    'upper': 0.75,
},
```

**After:**
```python
f'pre_mission.propulsion.{SmallTurbojetVariables.DIAMETER}': {
    'units': 'm',
    'lower': 0.05,
    'upper': 0.272,
    'ref': 0.15,
},
f'pre_mission.propulsion.{SmallTurbojetVariables.LENGTH}': {
    'units': 'm',
    'lower': 0.150,
    'upper': 0.75,
    'ref': 0.45,
},
```

**Why:**
IPOPT internally scales each design variable to O(1) by dividing by `ref`. Without `ref`,
OpenMDAO defaults to `ref=1.0`. Because diameter and length are in metres with values well
below 1.0 (0.05–0.272 m and 0.15–0.75 m), IPOPT was working with variables effectively
100× to 10× smaller than unit scale. This degraded gradient magnitudes for the engine
regression outputs and made the optimizer over-sensitive to small changes in engine geometry.

The `ref` values chosen are the nominal training-data centroids:

```text
diameter ref = 0.15 m  (training mean ≈ 0.138 m)
length   ref = 0.45 m  (training mean ≈ 0.340 m)
```

---

## Summary Table

| File | Change | Effect |
|------|--------|--------|
| `run_small_uav_mission.py` | Cruise `mach_final` 0.7 → 0.25 | Eliminates NaN from aerodynamics at invalid Mach |
| `run_small_uav_mission.py` | Added cruise `mach_initial=0.25` | Flat, consistent Mach profile; continuity with climb |
| `run_small_uav_mission.py` | Added cruise `mass_ref` | Normalizes mass state to O(1) for IPOPT |
| `run_small_uav_mission.py` | Added cruise `throttle_enforcement` | Consistent constraint form across both phases |
| `run_small_uav_mission.py` | `parallel_phases=True` → `add_phases()` | Avoids Jacobian assembly issues without MPI |
| `small_turbojet_builder.py` | `ref=0.15` on diameter DV | Scales engine diameter to O(1) for IPOPT |
| `small_turbojet_builder.py` | `ref=0.45` on length DV | Scales engine length to O(1) for IPOPT |

---

## General Advice for IPOPT Non-Convergence in Aviary

1. **Check Mach values against `max_mach` and `cruise_mach` in the CSV.**
   Phase `mach_final` values that exceed `mission:constraints:max_mach` will cause the
   aerodynamics model to evaluate outside its valid range.

2. **Add `mass_ref` to every phase.**
   Use the maximum takeoff mass as the reference: `mass_ref = (MTOW, "kg")`.

3. **Add `throttle_enforcement` consistently across phases.**
   Mixing enforced and unenforced throttle constraints between phases creates KKT
   discontinuities at phase boundaries.

4. **Set `ref` on all design variables.**
   For variables in metres or kilograms with values well below 1.0, set `ref` to a
   representative nominal value. For wing span, `ref=1.8` (the initial span). For engine
   geometry, use the training-data mean.

5. **Disable parallel phases if MPI is not installed.**
   Use `prob.add_phases()` (no arguments) unless `mpi4py` is confirmed available.

6. **Monitor `inf_du` (dual infeasibility) in IPOPT output.**
   If `inf_du` grows rather than shrinks, the problem usually has a scaling or
   constraint-formulation issue, not just a bad initial guess.

---

## Related Files

```text
aviary/models/aircraft/small_uav/run_small_uav_mission.py
aviary/models/aircraft/small_uav/small_uav.csv
aviary/subsystems/propulsion/small_turbojet/small_turbojet_builder.py
aviary/docs/Non-Native_Extension_Guides/UNIVERSAL_XDSM_DOCUMENTATION.md
```
