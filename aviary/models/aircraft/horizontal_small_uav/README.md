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
cd C:\Software\Repository\Aviary_clean
python aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

Outputs → `outputs/run_horizontal_small_uav_try_v1_v1.29.0_out/`

### Open dashboard (after opt completes)

```powershell
aviary dashboard run_horizontal_small_uav_try_v1_v1.29.0
```

### One-liner: run opt then launch dashboard

```powershell
python aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py; aviary dashboard run_horizontal_small_uav_try_v1_v1.29.0
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
| `True` | Prints a 9-section breakdown: wing geometry, endplate AR correction (Scholz steps), Polhamus CL_alpha, M_crit evaluation, CY_beta/CY_delta_r, Ny/Nz reproduced from formula, engine sizing, fuel budget |
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
| `ScholzWingletARCorrection` | `aviary/subsystems/aerodynamics/SpaJeti_based/lift_curve_slope.py` | Scholz (INCAS 2018) winglet AR correction → AR_eff, k_h |
| `LiftCurveSlopePolhamus` | `aviary/subsystems/aerodynamics/SpaJeti_based/lift_curve_slope.py` | Wing-alone Polhamus CL_alpha; c_l_alpha computed from t/c via Abbott & von Doenhoff |
| `CyBetaVtp` / `CyDeltaRudder` | `aviary/subsystems/aerodynamics/SpaJeti_based/cy_beta_vtp.py` | Side-force derivatives for Ny constraint |
| `LateralLoadFactor` | `aviary/subsystems/aerodynamics/SpaJeti_based/lateral_load_factor.py` | Ny = CY_beta*beta + CY_delta_r*delta_r (q*S) / W |
| `LongitudinalLoadFactor` | `aviary/subsystems/aerodynamics/SpaJeti_based/lateral_load_factor.py` | Nz = CL_alpha*alpha_max (q*S) / W |
| `MACGeometryComp` | `aviary/subsystems/aerodynamics/SpaJeti_based/surface_geometry.py` | MAC chord, spanwise station, and body-frame position (x, z) |
| `MachCriticalComp` | `aviary/subsystems/aerodynamics/SpaJeti_based/mach_critical.py` | Weisshaar M_DD / M_crit; CL derived from Nz_min requirement; constrained M_crit >= DASH_MACH + M_CRIT_SAFETY_MARGIN |
| `SpaJetiMassGroup` | `aviary/subsystems/mass/spajeti_based/mass_group.py` | Physics-based structural mass plus aircraft mass/CG outputs |
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
dihedral = 0 deg, x_apex = 0.80 m, z_apex = 0.00 m.

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

## Centre of Gravity and Structural Mass

The legacy `CGEstimatorGroup` has been removed from this aircraft case.  Current
mass and CG reporting comes from `SpaJetiMassGroup` in
`aviary/subsystems/mass/spajeti_based/`.

The group combines live wing, fuselage, and tail structural mass estimates with
engine/fuel/payload point masses.  Its aircraft-level outputs are:

| Variable | Units | Description |
|---|---|---|
| `aircraft_x_cg` | m | Aircraft CG x-position in the SpaJeti mass frame |
| `aircraft_z_cg` | m | Aircraft CG z-position in the SpaJeti mass frame |
| `aircraft_empty_mass` | kg | Structural empty mass plus fixed installed items |
| `aircraft_total_mass` | kg | Empty mass plus payload and fuel |

Static-margin reporting is intentionally disabled until the CG x-axis convention
is unified with the MAC geometry x-axis.

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
| M_crit margin | ≥ 0 | DASH_MACH + 0.05 = 0.57 | CL based on Nz_min; tighten via `M_CRIT_SAFETY_MARGIN` |
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

## Aero Physics Changes (2026-06-15)

Three corrections applied since the 2026-06-08 baseline:

### 1. Wing CL_alpha — fuselage K_wf removed

`LiftCurveSlopePolhamus` now outputs the **wing-alone** CL_alpha.
`K_wf = 1 + 0.025(d_f/b) − 0.25(d_f/b)²` is a wing-body correction that belongs
in the total lift buildup, not in the wing's own CL_alpha. It has been removed from
the component entirely.

### 2. Section lift slope — Abbott & von Doenhoff t/c correction

The component no longer takes `section_lift_slope` as an external input.
It now accepts `thickness_to_chord` (t/c) and computes c_l_alpha internally:

```
c_l_alpha = 2π (1 + 0.77 · t/c)   [Abbott & von Doenhoff]
```

At t/c = 0.12 this gives 6.86 /rad vs. the thin-airfoil 6.28 /rad (+9 %).
The t/c is wired from `section_tc` (AirfoilConstantsComp) so it tracks the
design variable automatically.

### 3. M_crit — CL based on Nz requirement, not alpha_max

Previously `MachCriticalComp` used `CL_alpha × alpha_max` ≈ 1.42 in the
Weisshaar formula — the stall CL, not the dash CL (~0.03).
This was 5× too conservative and forced the optimizer to paper-thin wings.

CL is now derived from the Nz_min load-factor requirement:

```
CL = Nz_min × m × g / (q × S)   ≈ 0.24  at the design point
```

The `M_CRIT_SAFETY_MARGIN = 0.05` (mach_upper_bound = DASH_MACH + 0.05)
provides all required safety — no additional multiplier on CL.

At t/c = 0.12, sweep = 0: M_crit ≈ 0.63 (was ~0.53), constraint satisfied with 0.06 margin.

| | Before | After |
|---|---|---|
| CL into Weisshaar | 1.42 (CL_alpha × α_max) | 0.24 (Nz_min × mg/qS) |
| M_crit at t/c = 0.12 | ~0.53 — violated | ~0.63 — OK |
| Optimizer t/c result | 0.058 (forced thin) | TBD after re-run |

---

## Baseline Result — stale, pending re-run

> **Stale since 2026-06-15 aero physics corrections.** Re-run to update.

Pre-fix run (original code, K_wf in CL_alpha, CL_max in M_crit):
IPOPT EXIT — converged to point of local infeasibility. Range = 159 km.
Root cause: M_crit constraint driven by CL_max ≈ 1.42 forced t/c → 0.058,
making the constraint effectively infeasible at any reasonable wing geometry.

---

## Known Limitations

- FLOPS height-energy EOM: no angle-of-attack tracking; pitch trim not checked.
- Static margin set to 0.10 in CSV but not actively constrained by the optimizer.
- VTP wetted area zeroed out — no FLOPS drag contribution from the H-tail endplates
  (drag is accounted for through induced-drag reduction via AR_eff instead).
- Landing and takeoff phases are excluded (`include_takeoff = False`).
