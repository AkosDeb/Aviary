# Roskam Wing Induced Drag

This document covers the local Roskam induced-drag implementation in
`induced_drag.py`. It follows the same coefficient-only convention as the
parasite-drag module: output `CDi`, not drag force in Newtons.

## Equation

`RoskamInducedDragComp` implements the first, no-twist term of Roskam Part VI
Section 4.2.1.2, Eq. 4.8:

```text
CDi = C_Lw^2 / (pi * AR_eff * e)
```

Current convention:

```text
C_Lw = CL
```

Roskam Eq. 4.11 gives a preliminary trim allowance of `C_Lw = 1.05*CL` for an
aircraft without a canard. That multiplier is intentionally not applied here.
Use the actual `CL`; add trim explicitly later only if a tail-load/trim analysis
computes a real wing lift coefficient.

Span efficiency uses Roskam Eq. 4.12:

```text
e = 1.1 * (CL_alpha_w / AR_eff)
    / (R * (CL_alpha_w / AR_eff) + (1 - R) * pi)
```

`R` is the Roskam leading-edge suction parameter from Part VI Figure 4.7. The
component can either use a fixed preliminary value
(`default_leading_edge_suction_parameter = 0.98`) or compute `R` from live wing
geometry with `compute_leading_edge_suction=True`.

The Figure 4.7 helper path is:

```python
leading_edge_radius_ratio_naca_4_digit(t_c)
leading_edge_reynolds_number_from_mach(M, p, T, r_LE)
leading_edge_suction_parameter_roskam(Re_LER, M, Lambda_LE, AR, taper)
```

The leading-edge Reynolds number is:

```text
Re_LER = rho * U * r_LE / mu
```

where `r_LE` is the airfoil leading-edge radius, not the chord. If airfoil data
stores `r_LE/c`, use `r_LE = (r_LE/c) * chord`.

For the current NACA 4-digit workflow, `r_LE/c` is not fixed inside the induced
drag component. It is recomputed from the live section thickness ratio:

```text
r_LE / c = 1.1019 * (t/c)^2
r_LE     = (r_LE/c) * MAC
```

That means an optimizer change to `wing_section_tc` changes `r_LE` and `Re_LER`
immediately. In the currently enabled high-x inset of Figure 4.7, `R` itself is a
function of `AR_eff * taper / cos(Lambda_LE)` only, so `CDi` may not change with
`t/c` while the case remains in that inset. When the lower-x main chart is
properly digitized, `Re_LER` will also feed through to `R` and then to `CDi`.

Current implementation status:

```text
x = Re_LER * cot(Lambda_LE) * sqrt(1 - M^2*cos^2(Lambda_LE))
```

Only the Figure 4.7 high-x inset branch is enabled. If `x < 1.3e5`, the helper
raises `NotImplementedError` because the main-chart digitization has not been
properly checked. If `x >= 1.3e5`, the helper returns the inset interpolation but
emits a `RuntimeWarning`: the inset values are a first-pass placeholder from the
supplied image, not a verified perfect digitization of the original chart.

## Excluded Terms

The full Roskam Eq. 4.8 includes two twist terms:

```text
+ 2*pi * C_Lw * eps_t * V
+ 4*pi^2 * eps_t^2 * w
```

They are zero for an untwisted wing and are intentionally excluded. When twist is
added, `V` and `w` require a spanwise lift distribution or Fourier loading model.

## Fuselage Drag Due To Lift

`FuselageLiftInducedDragComp` implements Roskam Part VI Section 4.3.1.2,
Eq. 4.33:

```text
CDi_fus = 2 * alpha^2 * S_b_fus / S
        + eta * c_d_c * alpha^3 * S_plf_fus / S
```

Current convention:

| Symbol | Current model source |
|--------|----------------------|
| `alpha` | aircraft alpha from `alpha_max_deg`; same scalar design-point alpha used by the wing load-factor/Mach-critical path |
| `S_b_fus` | `fuselage_base_area` from `SuperellipseFuselageGeometry` |
| `S_plf_fus` | `fuselage_planform_area` from `SuperellipseFuselageGeometry` |
| `S` | wing reference area |
| `eta` | scalar input `eta_finite_cylinder`; chart interpolation pending |
| `c_d_c` | scalar input `crossflow_drag_coefficient`; chart interpolation pending |

The component outputs:

| Output | Meaning |
|--------|---------|
| `CDi_fus` | total fuselage drag coefficient due to lift |
| `CDi_fus_base_area_term` | `2 * alpha^2 * S_b_fus / S` |
| `CDi_fus_planform_term` | `eta * c_d_c * alpha^3 * S_plf_fus / S` |

Base drag is not included in this component. Implement it later as a separate
optional term after the aft-body/exhaust-base assumption is checked.

The SpaJeti model wires this component as report-only. It is not added to the
mission drag polar yet.

## AR Convention

Use `AR_eff` from `ScholzWingletARCorrection`, not geometric AR, when computing
wing induced drag for the H-tail/endplate configuration. `CL_alpha_w` should be
the 3-D Polhamus lift-curve slope evaluated at that same `AR_eff`. The Figure
4.7 helper's `aspect_ratio` argument should also receive `AR_eff` for this
configuration.

The built-in FLOPS `InducedDrag` component still computes from its own mission
lift, geometric AR, and span-efficiency inputs; it does not apply the 1.05 trim
factor either.

## OpenMDAO Interface

Inputs:

| Variable | Units | Description |
|----------|-------|-------------|
| `CL` | unitless | Aircraft lift coefficient used directly |
| `AR_eff` | unitless | Effective aspect ratio from the Scholz endplate correction |
| `CL_alpha_w` | 1/rad | 3-D wing lift-curve slope at `AR_eff` |
| `mach` | unitless | Mach number for `Re_LER` and Figure 4.7 |
| `static_pressure` | Pa | Static pressure for `Re_LER` |
| `temperature` | K | Static temperature for `Re_LER` |
| `mean_aerodynamic_chord` | m | Wing MAC used to convert `r_LE/c` to `r_LE` |
| `thickness_to_chord` | unitless | Live section `t/c` used by the NACA radius estimate |
| `leading_edge_sweep` | deg | Wing leading-edge sweep `Lambda_LE` |
| `taper_ratio` | unitless | Wing taper ratio `lambda` |

Outputs:

| Variable | Units | Description |
|----------|-------|-------------|
| `CDi` | unitless | Induced drag coefficient |
| `e_oswald` | unitless | Oswald span efficiency factor |
| `leading_edge_radius` | m | Computed `r_LE` |
| `leading_edge_reynolds_number` | unitless | Computed `Re_LER` |
| `leading_edge_suction_parameter` | unitless | `R` used in Eq. 4.12 |

Options:

| Option | Default | Description |
|--------|---------|-------------|
| `compute_leading_edge_suction` | `False` | Compute `R` from live geometry and atmosphere |
| `default_leading_edge_suction_parameter` | `0.98` | Fixed fallback `R` when chart computation is disabled |

Safety guards:

| Condition | Action |
|-----------|--------|
| `AR_eff <= 0` | `ValueError` |
| `CL_alpha_w <= 0` | `ValueError` |
| Figure 4.7 helper with `M >= 0.8` | `RuntimeWarning`; chart is marked M < 0.8 |
| Figure 4.7 helper with `x < 1.3e5` | `NotImplementedError`; main-chart digitization not verified |
| Figure 4.7 helper with `x >= 1.3e5` | `RuntimeWarning`; inset digitization is first-pass placeholder data |

## Current Status

`RoskamInducedDragComp` is wired into the SpaJeti load-factor/reporting model
with live `r_LE`, `Re_LER`, and `R` diagnostics. The FLOPS mission drag polar is
still using Aviary's built-in induced-drag path. Wiring the local `CDi` into
mission fuel burn remains a TODO. `FuselageLiftInducedDragComp` is also wired
as report-only and is not yet included in mission fuel burn.
