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
| `FuelBudgetEstimate` | `run_horizontal_small_uav.py` (inline) | Available fuel = gross − empty − payload − engine |

Component-level formulas, I/O tables, and worked examples are in:

- [README_htail_geometry.md](../../geometry/flops_based/README_htail_geometry.md)
- [README_lift_curve_slope.md](../../aerodynamics/flops_based/README_lift_curve_slope.md)
- [README_lateral_stability.md](../../aerodynamics/flops_based/README_lateral_stability.md)

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
