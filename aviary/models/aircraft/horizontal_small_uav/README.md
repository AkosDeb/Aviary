# SpaJeti v1.0.0 — H-wing UAV Range Optimisation

Conceptual fixed-wing UAV: straight wing with twin-VTP H-tail endplates and a small
turbojet. The layout replaces the conventional single vertical stabilizer with two
winglet-mounted VTPs that simultaneously provide lateral stability and act as
endplates, improving the wing's effective aspect ratio.

---

## Configuration

| Parameter | Value |
|---|---|
| Layout | Wing + H-tail (twin-VTP endplates) + fuselage pod |
| Propulsion | Single small turbojet (scaled from regression model) |
| Gross mass | 15.0 kg (fixed) |
| Empty mass | 7.0 kg (fixed structural budget) |
| Payload | 0.5 kg |
| Fuel tank | 8.0 kg capacity (fuselage) |
| Cruise altitude | 5 000 m ISA |
| EOM | Height-energy (FLOPS/Dymos) |
| Problem type | Fallout — maximize range |

---

## Files

```text
horizontal_small_uav.csv        Aircraft input deck (FLOPS variables)
phase_info.py                   Dymos phase definitions (accel_to_dash, climb, cruise, dash)
run_horizontal_small_uav.py     Level-2 AviaryProblem builder and runner
README.md                       This file
```

---

## Run

From the repository root:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

Outputs → `outputs/run_horizontal_small_uav_out/`

### Open dashboard

```powershell
$env:PYTHONPATH = (Get-Location).Path
& C:/Software/Anaconda/envs/aviary/Scripts/aviary.exe dashboard outputs/run_horizontal_small_uav_out
```

---

## Detail Print

The run script has a top-level flag that controls whether a full aero breakdown is
printed after the optimisation completes:

```python
# run_horizontal_small_uav.py  (line ~88)
PRINT_AERO_DETAIL = True   # set False to suppress the wing/VTP/rudder aero breakdown
```

| Value | Effect |
|-------|--------|
| `True` | Prints a 9-section breakdown: wing geometry, endplate AR correction (Scholz steps), Polhamus CL_alpha, K_wf, CY_beta/CY_delta_r, Ny/Nz reproduced from formula, engine sizing, fuel budget |
| `False` | Only the solver exit message and Aviary summary table are printed |

To toggle from the command line without editing the file:

```powershell
# Suppress detail
$env:PYTHONPATH = (Get-Location).Path
& C:/Software/Anaconda/envs/aviary/python.exe -c "
import run_horizontal_small_uav as m; m.PRINT_AERO_DETAIL = False; m.main()
" 2>&1
```

Or simply edit the constant at the top of [run_horizontal_small_uav.py](run_horizontal_small_uav.py)
before running.

---

## External Subsystems

| Subsystem | Module | Purpose |
|---|---|---|
| `SmallTurbojetModel` | `aviary/subsystems/propulsion/small_turbojet` | Regression-based turbojet sizing and SFC |
| `HTailGeometry` | `aviary/subsystems/geometry/flops_based/htail_geometry.py` | VTP span → area, AR, root chord |
| `ScholzWingletARCorrection` | `aviary/subsystems/aerodynamics/flops_based/lift_curve_slope.py` | Scholz (INCAS 2018) winglet AR correction → AR_eff, k_h |
| `LiftCurveSlopePolhamus` | `aviary/subsystems/aerodynamics/flops_based/lift_curve_slope.py` | Polhamus CL_alpha (wing and VTP) |
| `CyBetaVtp` / `CyDeltaRudder` | `aviary/subsystems/aerodynamics/flops_based/cy_beta_vtp.py` | Side-force derivatives for Ny constraint |
| `LateralLoadFactor` | `aviary/subsystems/aerodynamics/flops_based/lateral_load_factor.py` | Ny = CY_beta*beta + CY_delta_r*delta_r (q*S) / W |
| `LongitudinalLoadFactor` | `aviary/subsystems/aerodynamics/flops_based/lateral_load_factor.py` | Nz = CL_alpha*alpha_max (q*S) / W |
| `MACGeometryComp` | `aviary/subsystems/aerodynamics/flops_based/surface_geometry.py` | MAC chord, spanwise station, and body-frame position (x, z) |
| `MachCriticalComp` | `aviary/subsystems/aerodynamics/flops_based/mach_critical.py` | Weisshaar M_DD / M_crit; constrained M_crit >= DASH_MACH + 0.05 |
| `CGEstimatorGroup` / `CGComputeComp` | `aviary/subsystems/geometry/flops_based/cg_estimator.py` | Weighted-average aircraft CG from component mass list; bypass mode available |
| `FuelBudgetEstimate` | `run_horizontal_small_uav.py` (inline) | Available fuel = gross − empty − payload − engine |

Component-level formulas, I/O tables, and worked examples are in:

- [README_htail_geometry.md](../../geometry/flops_based/README_htail_geometry.md)
- [README_lift_curve_slope.md](../../aerodynamics/flops_based/README_lift_curve_slope.md)
- [README_lateral_stability.md](../../aerodynamics/flops_based/README_lateral_stability.md)

---

## Aircraft Reference Frame

Body-fixed coordinates, origin at the **nose tip**:

| Axis | Direction | Units |
|------|-----------|-------|
| x | Positive AFT (fuselage station from nose) | m |
| y | Positive STARBOARD | m |
| z | Positive DOWN | m |

Note: z-down differs from FLOPS (which uses z-up).  All component apex positions
in the model are x-stations measured aft from the nose; all values are positive.

---

## Wing Mean Aerodynamic Chord

Computed live by `MACGeometryComp` (`surface_geometry.py`) so MAC position updates
automatically when span or area change as design variables.

### Formulas

```
c_r    = 2 * S / (b * (1 + lambda))              root chord from trapezoidal area

c_mac  = (2/3) * c_r * (1 + lambda + lambda^2)   mean aerodynamic chord
                      / (1 + lambda)

y_mac  = (b / 6) * (1 + 2*lambda) / (1 + lambda) spanwise BL of MAC from centreline

tan(Lambda_LE) = tan(Lambda_c4) + c_r*(1 - lambda)/b    LE sweep from c/4 sweep

x_mac_le = x_apex + y_mac * tan(Lambda_LE)       MAC LE x-station from nose
z_mac_le = z_apex - y_mac * tan(dihedral)         MAC LE z-station (- because up-dihedral
                                                   lifts MAC above root; z positive down)
x_mac_c4 = x_mac_le + c_mac / 4                  MAC quarter-chord (aero centre x)
```

### Inputs / Outputs

| Variable | Symbol | Units | Source |
|---|---|---|---|
| `surface_area` | S | m² | `aircraft:wing:area` (DV) |
| `surface_span` | b | m | `aircraft:wing:span` (DV) |
| `surface_taper` | lambda | — | CSV |
| `surface_sweep_c4` | Lambda_c4 | deg | CSV |
| `dihedral_deg` | Gamma | deg | CSV |
| `wing_x_apex` | x_apex | m | `load_cond` (geometric input) |
| `wing_z_apex` | z_apex | m | `load_cond` (geometric input) |
| **`wing_root_chord`** | c_r | m | output |
| **`wing_c_mac`** | c_mac | m | output |
| **`wing_y_mac`** | y_mac | m | output |
| **`wing_x_mac_le`** | x_mac_le | m | output |
| **`wing_z_mac_le`** | z_mac_le | m | output |
| **`wing_x_mac_c4`** | x_mac_c4 | m | output |

### SpaJeti Numerical Example

Input baseline: b = 1.8 m, S = 0.45 m², lambda = 0.6, sweep_c4 = 0 deg,
dihedral = 3 deg, x_apex = 0.80 m, z_apex = 0.00 m.

```
c_r      = 2 * 0.45 / (1.8 * 1.6)         = 0.313 m
c_mac    = (2/3) * 0.313 * 1.96 / 1.6     = 0.255 m
y_mac    = (1.8/6) * 2.2 / 1.6            = 0.413 m  (from CL; 46% of half-span)
tan_LE   = tan(0) + 0.313 * 0.4 / 1.8     = 0.0694  =>  Lambda_LE = 3.97 deg
x_mac_le = 0.80 + 0.413 * 0.0694          = 0.829 m
z_mac_le = 0.00 - 0.413 * tan(3 deg)      = -0.022 m (MAC is above nose datum)
x_mac_c4 = 0.829 + 0.255 / 4              = 0.893 m  (44.6 % of 2.0 m fuselage)
```

The MAC quarter-chord falls at 44.6 % of the fuselage length aft of the nose.
This is the x-reference for static margin and CG travel calculations.

---

## Centre of Gravity Estimator

Computed by `CGEstimatorGroup` (`cg_estimator.py`) using the standard component
weighted sum:

```
x_cg = sum(m_i * x_i) / sum(m_i)    (and likewise for y_cg, z_cg)
```

### Bypass mode

Set `CG_BYPASS = True` at the top of the run file to skip estimation entirely.
The group exposes the same four output names (`x_cg`, `y_cg`, `z_cg`,
`total_mass`) from an `IndepVarComp`; all downstream consumers are unaffected.

```python
CG_BYPASS         = False   # flip to True to go manual
CG_X_MANUAL_M     = 0.91   # [m]  x_cg override
CG_Y_MANUAL_M     = 0.00   # [m]  y_cg override
CG_Z_MANUAL_M     = 0.05   # [m]  z_cg override
CG_MASS_MANUAL_KG = 15.0   # [kg] total mass override
```

### add_component API

```python
# Fixed mass + position (known hardware)
cg_est.add_component('camera', mass=0.15, x=0.30, y=0.05, z=0.08)

# Variable mass from an OpenMDAO model-scope variable, fixed position
cg_est.add_component('fuel', mass=av.Mission.TOTAL_FUEL, x=0.90, y=0.0, z=0.05)

# Variable mass via explicit connect (non-model-scope path)
cg_est.add_component('engine', mass='cg_engine_mass', x=1.55, y=0.0, z=0.0)
# ... then:
prob.model.connect(premission_propulsion_var(...), 'cg_est.cg_engine_mass')
```

### SpaJeti component list (baseline estimates)

All structural masses are **placeholders** — replace with FLOPS component mass
outputs when the weight breakdown subsystem is added.

| Component | Mass | x [m] | y [m] | z [m] | Notes |
|---|---|---|---|---|---|
| wing_struct | 1.50 kg [P] | 0.80 | 0.0 | 0.00 | WING_X_APEX_M |
| fuselage | 1.00 kg [P] | 1.00 | 0.0 | 0.00 | mid-body |
| empennage | 0.25 kg [P] | 1.85 | 0.0 | 0.00 | HTP + VTP mounts |
| vtp_pair | 0.15 kg [P] | 1.75 | 0.0 | −0.15 | above wing tips |
| landing_gear | 0.20 kg [P] | 0.95 | 0.0 | 0.15 | below fuselage |
| avionics | 0.25 kg [P] | 0.45 | 0.0 | 0.00 | nose bay |
| **engine** | *optimizer* | 1.55 | 0.0 | 0.00 | SmallTurbojet mass |
| **payload** | *optimizer* | 0.85 | 0.0 | 0.05 | CrewPayload variable |
| **fuel** | *optimizer* | 0.90 | 0.0 | 0.05 | Mission.TOTAL_FUEL |

[P] = placeholder mass.  Bold = live OpenMDAO variable.

### Outputs

| Variable | Units | Description |
|---|---|---|
| `x_cg` | m | CG x-station from nose (positive aft) |
| `y_cg` | m | CG lateral offset (0 for symmetric) |
| `z_cg` | m | CG z-station from nose datum (positive down) |
| `total_mass` | kg | sum of registered component masses |

An approximate static margin (wing AC only) is printed in the results summary:

```
SM_approx = (wing_x_mac_c4 - x_cg) / wing_c_mac
```

Positive = CG forward of wing aerodynamic centre = stable.  A full multi-surface
neutral-point calculation will be added in `StaticMarginComp` (see TOOD.md).

---

## Optimisation Problem

### Design Variables

| Variable | Lower | Upper | Ref | Units |
|---|---|---|---|---|
| `aircraft:wing:span` | 1.34 | 2.0 | 1.8 | m |
| `aircraft:vertical_tail:span` (per VTP) | 0.15 | 0.60 | 0.31 | m |
| `aircraft:engine:scaled_sls_thrust` | — | — | — | N (Aviary DV) |
| Phase Mach (climb, cruise, dash) | 0.30 | 0.60 | — | — (Aviary DVs) |

### Constraints

| Constraint | Bound | Design point | Reference |
|---|---|---|---|
| Lateral load factor Ny | ≥ 7.0 | 550 km/h, 5 km ISA | see [README_lateral_stability.md] |
| Longitudinal load factor Nz | ≥ 7.0 | 550 km/h, 5 km ISA | see [README_lateral_stability.md] |
| T/W at SLS | ≥ 1.5 | SLS, sea level | `scaled_sls_thrust ≥ 1.5 * m_gross * g` |
| Engine mass | ≤ 5.0 kg | — | regression upper bound |
| Fuel budget margin | ≥ 0 kg | — | see formula below |

#### Fuel Budget Formula

```
available_fuel = gross_mass − empty_mass − payload_mass − engine_mass
fuel_budget_margin = available_fuel − mission_fuel_used  ≥ 0
```

| Input | Value (fixed) |
|---|---|
| `gross_mass` | 15.0 kg |
| `empty_mass` | 7.0 kg |
| `payload_mass` | 0.5 kg |
| `engine_mass` | optimised |
| `mission_fuel_used` | optimised |

**Note:** Aviary's native `excess_fuel_capacity` constraint is disabled
(`IGNORE_FUEL_CAPACITY_CONSTRAINT = True`) because it has zero gradient w.r.t.
the design variables. `fuel_budget_margin` covers the same check with correct
derivatives.

### Objective

Minimise `−range` (Aviary fallout: fixed design, computed range).

---

## Baseline Result — 2026-06-08

> **Note:** This baseline was run with k_WL = 2.0 (theoretical optimum).
> Current code uses k_WL = 2.45 (experimental average). AR_eff and range
> will differ on re-run; update this section after the next run.

Run converged to acceptable level (IPOPT, 37 iterations, 39 s).

### Geometry

| Output | Value |
|---|---|
| Wing span | 1.357 m |
| Wing area | 0.450 m² (fixed) |
| Wing AR (geometric) | 4.09 |
| Wing AR_eff (endplate) | 7.04 |
| Endplate factor k_h | 1.720 |
| Fuselage factor K_wf | 0.999 |
| VTP span (per endplate) | 0.507 m |
| VTP area | 0.080 m² |
| VTP AR | 1.20 |
| H-tail area | 0.150 m² |
| Fuselage length | 2.000 m |

### Load Factors (550 km/h, 5 km ISA, q = 8 581 Pa)

| Output | Value | Limit |
|---|---|---|
| Wing CL_alpha (endplate + K_wf) | 5.196 /rad | — |
| VTP CL_alpha_v | 2.649 /rad | — |
| CY_beta_vtp | −1.095 /rad | — |
| CY_delta_r | −0.517 /rad | — |
| Ny (lateral) | 14.77 | ≥ 7.0 ✓ |
| Nz (vertical) | 28.57 | ≥ 7.0 ✓ |
| SLS Thrust | 272.5 N | — |
| T/W at SLS | 1.853 | ≥ 1.5 ✓ |

### Propulsion

| Output | Value |
|---|---|
| Engine diameter | 0.126 m |
| Engine max RPM | 110 173 rpm |
| Engine mass | 2.971 kg |
| SFC | 4.373 × 10⁻⁵ kg/(N·s) |

### Mission Performance

| Output | Value |
|---|---|
| Range | **74.1 km** |
| Total fuel used | 4.493 kg |
| Fuel capacity | 8.000 kg |
| Mass-budget fuel limit | 4.529 kg |
| Fuel budget margin | 0.035 kg (nearly active) |
| Payload | 0.500 kg |

### Mass Breakdown

| Item | Mass |
|---|---|
| Gross mass | 15.000 kg |
| Empty mass (structural) | 7.000 kg |
| Engine | 2.971 kg |
| Payload | 0.500 kg |
| Fuel used | 4.493 kg |

The fuel budget constraint is the binding constraint — the optimizer exhausted
nearly all available fuel to achieve 74 km. The climb Mach lower bound (0.30)
was also active.

---

## Known Limitations

- FLOPS height-energy EOM: no angle-of-attack tracking; pitch trim not checked.
- Static margin set to 0.10 in CSV but not actively constrained by the optimizer.
- VTP wetted area zeroed out — no FLOPS drag contribution from the H-tail endplates
  (drag is accounted for through induced-drag reduction via AR_eff instead).
- Landing and takeoff phases are excluded (`include_takeoff = False`).
