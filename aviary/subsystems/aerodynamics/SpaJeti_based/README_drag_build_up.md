# Roskam/DATCOM Parasite Drag Build-Up

This document covers the local clean drag build-up implementation:

| File | Purpose |
|------|---------|
| `aviary/subsystems/aerodynamics/aero_utils.py` | Shared atmosphere, Reynolds, roughness cutoff, skin-friction, and form-factor helpers |
| `parasite_drag.py` | `RoskamParasiteDragBuildUp`, an OpenMDAO component for component-wise `CD0` build-up |
| `test/test_parasite_drag.py` | Pytest coverage for the helper formulas and component outputs |

The module is intentionally coefficient-only. It outputs dimensionless drag
coefficients (`CD0`, component `CD0_component`). It does not output force in
Newtons. If force is needed, compute it in a separate component:

```text
D = q * Sref * CD
```

Later induced and total drag modules should follow the same convention:
`CDi` for induced drag coefficient and `CD` for total drag coefficient.

---

## Component Equation

The build-up uses three separate interference paths:

**Fuselage-attached lifting surfaces** (wing, HTP — `component_kind = 'lifting_surface'`):

```text
CD0 = Cf * FF * R_LS * R_wf * (1 + K_LP) * Swet / Sref
```

**H-wing VTP panels — wingtip-attached** (`component_kind = 'vtp'`):

```text
CD0 = Cf * FF * R_LS * R_tail * (1 + K_LP) * Swet / Sref
```

The current X-tail uses `R_tail = 1.04`. A future H-tail backend should publish
the same factor used for the wing endplate / wingtip-tail junction.

**Fuselage** (`component_kind = 'fuselage'` or `'raymer_fuselage'`):

```text
CD0 = Cf_fus * FF_fus * R_wf * (1 + K_LP) * Swet_fus / Sref
```

(Roskam Part VI Eq. 4.30; base-pressure drag CD0_base is intentionally excluded —
the SpaJeti fuselage is a jet-exhaust body, so the base is filled by engine outflow
and does not contribute a separated-wake base drag term.)

**External bodies / nacelles** (`component_kind = 'body'`):

```text
CD0 = Cf * FF * Q * (1 + K_LP) * Swet / Sref
```

| Symbol | Meaning | Source |
|--------|---------|--------|
| `Cf` | flat-plate skin-friction coefficient (mixed lam/turb) | Raymer formula, Re-based |
| `FF` | form factor | DATCOM thickness formula (surfaces) or body formula (fuselages) |
| `R_LS` | lifting-surface compressibility correction | DATCOM table: Mach × cos(Λ_t/c); 1.0 for bodies |
| `R_wf` | wing-fuselage interference factor | DATCOM table: Mach × Re_fus (driven by `fuselage_length`); applied to fuselage-attached lifting surfaces and fuselage (Roskam Eq. 4.30) |
| `R_tail` | tail junction/layout factor | `tail_drag_interference_factor`; 1.04 for the current X-tail |
| `Q` | interference factor for bodies/nacelles | user input `interference_factor`; **ignored for lifting surfaces, vtp, and fuselage** |
| `K_LP` | leakage and protuberance fraction | user input `leakage_protuberance_factor` |
| `Swet` | **exposed** component wetted area | see Exposed Wetted Area Convention |
| `Sref` | aircraft reference area | user input `reference_area` |

The component also outputs `CD0_component`, an `(num_nodes, num_components)` array,
so the drag contribution from each component can be printed and audited.

---

## Reusable Helpers

### Atmosphere and Reynolds Number

`aero_utils.py` provides:

```python
dynamic_viscosity_sutherland(temperature_K)
speed_of_sound(temperature_K)
air_density_ideal_gas(static_pressure_Pa, temperature_K)
reynolds_number(rho, velocity, characteristic_length, dynamic_viscosity)
reynolds_number_from_mach(mach, static_pressure_Pa, temperature_K, characteristic_length_m)
```

These functions are framework-agnostic and should be reused by drag,
aeroelasticity, stability, and reporting code instead of duplicating Reynolds
number logic.

### Roughness Cut-Off Reynolds Number

For rough surfaces, Raymer introduces an upper limit on the Reynolds number used
for skin-friction drag:

Subsonic:

```text
Re_co = 38 * (l/k)^1.053
```

Transonic and supersonic:

```text
Re_co = 44 * (l/k)^1.053 * M^1.16
```

The helper is:

```python
roughness_cutoff_reynolds(characteristic_length_m, roughness_m, mach=0.0)
apply_roughness_cutoff(reynolds, characteristic_length_m, roughness_m, mach=0.0)
```

Roughness-height presets:

| Surface | `k` [m] |
|---------|---------|
| `aluminum` | `10.0e-6` |
| `flat_coating` | `6.3e-6` |
| `unpolished_sheet` | `4.1e-6` |
| `polished_sheet` | `1.5e-6` |
| `flat_cfk` | `0.5e-6` |

Reference: Raymer, *Aircraft Design: A Conceptual Approach*, 6th ed.,
Section 12.5.3, Eq. 12.28 and 12.29.

### Flat-Plate Skin Friction

The turbulent flat-plate coefficient uses the common compressible Raymer fit:

```text
Cf_turb = 0.455 / [log10(Re)^2.58 * (1 + 0.144*M^2)^0.65]
```

The laminar flat-plate coefficient is:

```text
Cf_lam = 1.328 / sqrt(Re)
```

The helper mixes laminar and turbulent values with `laminar_fraction`:

```python
flat_plate_skin_friction_coeff(reynolds, mach=0.0, laminar_fraction=0.0)
```

---

## Form Factors

### Lifting Surfaces

For wings, HTPs, VTPs, fins, and similar airfoil-based surfaces:

```text
CD0_w = R_wf * R_LS * Cf_w * [1 + L'*(t/c) + 100*(t/c)^4] * Swet_w/Sref
```

where:

| Symbol | Meaning |
|--------|---------|
| `R_wf` | wing/fuselage interference factor |
| `R_LS` | lifting-surface correction factor |
| `(t/c)` | thickness-to-chord ratio |
| `(x/c)_m` | chordwise location of maximum airfoil thickness |
| `L'` | airfoil thickness-location parameter |
| `Swet_w` | lifting-surface wetted area |

In the code, the thickness form-factor helper returns only:

```text
FF = 1 + L'*(t/c) + 100*(t/c)^4
```

Per the current project convention requested for this implementation:

| Condition | `L'` |
|-----------|------|
| `(x/c)_m <= 0.30` | `1.2` |
| `(x/c)_m > 0.30` | `2.0` |

Important: `(x/c)_m` is a real airfoil property, not a numerical safety factor.
For example:

| Airfoil family | Typical `(x/c)_m` |
|----------------|-------------------|
| NACA 4-digit | about `0.30` |
| NACA 6-series / NACA 64A | about `0.40` |

The current `AirfoilData` catalog stores this as `max_thickness_location`.

`R_LS` is provided by:

```python
lifting_surface_correction_factor(mach, cos_sweep_at_max_thickness)
```

It is a linearized digitization of Roskam Part VI, Figure 4.2. The interpolation
is linear in Mach and `cos(Lambda_(t/c)_max)`, with values outside the digitized
range clipped to the nearest tabulated edge.

`R_wf` is provided by:

```python
wing_fuselage_interference_factor(fuselage_reynolds_number, mach)
```

It is a linearized digitization of Roskam Part VI, Figure 4.1. The interpolation
is linear in Mach and `log10(Re_fus)`. In `RoskamParasiteDragBuildUp`, R_wf is
computed automatically from `fuselage_length` and applied to all lifting-surface
and fuselage components. The `interference_factor` input is only for
`component_kind='body'` (nacelles, external pods).

### Fuselage and Bodies

Default fuselage/body form factor uses the DATCOM-style streamlined-body form:

```text
FF_body_DATCOM = 1 + 60/f^3 + f/400
```

where `f = l/d` is the fineness ratio.

The Raymer fuselage form is also implemented for comparison:

```text
FF_fuselage_Raymer = 0.9 + 5/f^1.5 + f/400
```

In `RoskamParasiteDragBuildUp`, `component_kind='fuselage'` uses the DATCOM form
by default. Use `component_kind='raymer_fuselage'` only when deliberately comparing
against the Raymer alternate.

---

## OpenMDAO Component

`RoskamParasiteDragBuildUp` inputs:

| Input | Units | Shape | Description |
|-------|-------|-------|-------------|
| `mach` | unitless | `num_nodes` | Flight Mach number |
| `static_pressure` | Pa | `num_nodes` | Atmospheric static pressure |
| `temperature` | K | `num_nodes` | Atmospheric static temperature |
| `reference_area` | m^2 | scalar | Aircraft reference area |
| `wetted_area` | m^2 | `num_components` | Exposed component wetted areas (see convention above) |
| `characteristic_length` | m | `num_components` | MAC for wings/tails; total length for bodies |
| `fuselage_length` | m | scalar | Fuselage length for R_wf Re_fus computation |
| `roughness` | m | `num_components` | Surface roughness height; zero disables cutoff |
| `laminar_fraction` | unitless | `num_components` | Fraction of laminar flow |
| `thickness_to_chord` | unitless | `num_components` | Lifting-surface `t/c` |
| `max_thickness_location_over_chord` | unitless | `num_components` | `(x/c)_m` |
| `sweep_at_max_thickness` | deg | `num_components` | Sweep at max-thickness line |
| `fineness_ratio` | unitless | `num_components` | Body/fuselage fineness ratio |
| `interference_factor` | unitless | `num_components` | `Q` for bodies/nacelles only; ignored for lifting surfaces |
| `leakage_protuberance_factor` | unitless | `num_components` | leakage and protuberance fraction |

Outputs:

| Output | Units | Shape | Description |
|--------|-------|-------|-------------|
| `reynolds_number` | unitless | `(num_nodes, num_components)` | Reynolds number after roughness cutoff |
| `skin_friction_coefficient` | unitless | `(num_nodes, num_components)` | `Cf` |
| `form_factor` | unitless | `(num_nodes, num_components)` | `FF` |
| `lifting_surface_correction_factor` | unitless | `(num_nodes, num_components)` | `R_LS`; 1.0 for body/fuselage components |
| `R_wf` | unitless | `num_nodes` | Wing-fuselage interference factor (DATCOM table) |
| `CD0_component` | unitless | `(num_nodes, num_components)` | Per-component parasite drag coefficient |
| `CD0` | unitless | `num_nodes` | Total parasite drag coefficient |

Supported `component_kinds`:

| Kind | Form-factor path | Interference |
|------|------------------|-------------|
| `lifting_surface` | DATCOM/Roskam lifting-surface thickness form factor + `R_LS` | `R_wf` |
| `vtp` | same as `lifting_surface` | `R_tail` |
| `fuselage` | DATCOM streamlined-body form factor | `R_wf` |
| `body` | DATCOM streamlined-body form factor | user `Q` |
| `raymer_fuselage` | Raymer fuselage alternate | `R_wf` |

---

## Lifting-Surface Characteristic Lengths

All lifting-surface Reynolds-number characteristic lengths use trapezoidal mean
aerodynamic chord, not the simple area/span average:

```text
c_MAC = (2/3) * c_root * (1 + lambda + lambda^2) / (1 + lambda)
```

The main wing path consumes the live `wing_c_mac` output from `MACGeometryComp`.

## Tail Drag Contract - Physical Panel MAC

The characteristic length fed into `_GeomArrayAssembler` for the tail component
comes from the formal `TailGeometryGroup` drag contract. It is the **physical
panel mean aerodynamic chord**, not the projected geometry.

```
tail_drag_wetted_area = tail_physical_wetted_area
tail_drag_characteristic_length = (2/3) * c_root * (1 + lambda + lambda^2) / (1 + lambda)
tail_drag_interference_factor = 1.04    # current X-tail backend
```

For the X-tail backend, `tail_physical_wetted_area` is exposed external skin:

```text
tail_drag_wetted_area = 2 * N_panels * (S_panel - S_panel_buried_in_fuselage)
```

The matching fuselage skin holes are published as `tail_fuselage_cutout_area`:

```text
tail_fuselage_cutout_area = N_panels * K_tail * (t/c)_tail * c_root^2
```

`FuselageExposedWettedAreaComp` subtracts this tail cutout in addition to the
existing two wing airfoil cutouts.

Parasite-drag consumers should use `tail_drag_*` names so future tail backends
can keep the same API.

**Why not use the projected chord?**

For a canted X-tail, projected aerodynamic area is intentionally different from
true material geometry. Reynolds-number-based skin friction should use the real
panel MAC, not a horizontal or vertical projection.

**UAV numerical example** (single panel, c_root = 0.20 m, lambda = 0.50):

```
tail_drag_characteristic_length = (2/3) * 0.20 * (1 + 0.50 + 0.50^2) / (1 + 0.50)
                                  = 0.1556 m
```

Using the physical panel MAC removes cant-angle and panel-count projection factors
from the Reynolds number calculation.

---

## Exposed Wetted Area Convention

The `wetted_area` input must be the **exposed** wetted area — the portion of the
surface that is actually in contact with the external flow.  For a lifting surface
(wing, HTP, VTP) that passes through a fuselage the convention is:

```text
Swet_exposed = (S_planform - S_inside_fuselage) * 2
```

Reusable helper:

```python
exposed_wetted_area_lifting_surface(
    planform_area,
    buried_planform_area=0.0,
    sides=2.0,
)
```

where:

| Symbol | Meaning |
|--------|---------|
| `S_planform` | Total one-sided planform area (including the buried panel) |
| `S_inside_fuselage` | One-sided planform area of the panel that sits inside the fuselage body |
| factor `* 2` | Both upper and lower surfaces of the exposed panel |

For bodies and fuselages use the full outer wetted surface area (no factor of 2).

This convention applies **everywhere** S_wet is used in the project — parasite drag,
wetted-area-based mass models, and any future aeroelastic components.

### UAV numerical example

Wing: S_planform = 0.450 m², fuselage width = 0.160 m, root chord = 0.313 m.
Buried panel area ≈ (0.160/2) × 0.313 = 0.025 m² (one-sided half-width × root chord).
Swet_exposed = (0.450 - 0.025) × 2 = 0.850 m²

---

## Safety Guards and Input Validation

The following guards are active.  Unless noted they emit a ``RuntimeWarning`` and
continue with a clamped value.

### `aero_utils.py` helpers

| Condition | Action |
|-----------|--------|
| `exposed_wetted_area_lifting_surface`: `planform_area <= 0` | `ValueError` |
| `exposed_wetted_area_lifting_surface`: `buried_planform_area < 0` | `ValueError` |
| `exposed_wetted_area_lifting_surface`: `buried_planform_area > planform_area` | `ValueError`; buried area cannot exceed the full planform |
| `exposed_wetted_area_lifting_surface`: `sides <= 0` | `ValueError` |
| Reynolds number <= 0 in `flat_plate_skin_friction_coeff` | `RuntimeWarning`; Cf clamped to near-zero-Re value |
| `form_factor_lifting_surface`: t/c < 0 | `RuntimeWarning` |
| `form_factor_lifting_surface`: t/c > 0.50 | `RuntimeWarning` (DATCOM calibrated for t/c <= 0.30) |
| `form_factor_datcom_body`: fineness ratio < 1.0 | `RuntimeWarning`; 60/f³ diverges below f = 1 |
| `form_factor_raymer_fuselage`: fineness ratio < 1.0 | `RuntimeWarning`; 5/f^1.5 diverges below f = 1 |
| `lifting_surface_correction_factor` (R_LS): Mach > 0.90 | `RuntimeWarning`; DATCOM not reliable in transonic regime |
| `lifting_surface_correction_factor` (R_LS): Mach < 0.25 | silent edge-held (low-speed limit is stable) |
| `lifting_surface_correction_factor` (R_LS): cos(sweep) outside [0.50, 1.00] | `RuntimeWarning`; edge-held |
| `wing_fuselage_interference_factor` (R_wf): fuselage Re <= 0 | `ValueError`; non-physical input |
| `wing_fuselage_interference_factor` (R_wf): fuselage Re outside table range | `ValueError`; no reliable extrapolation |
| `wing_fuselage_interference_factor` (R_wf): Mach > 0.90 | `ValueError`; table not valid in transonic regime |
| `wing_fuselage_interference_factor` (R_wf): Mach < 0.25 | silent edge-hold (low-speed limit is stable) |

### `RoskamParasiteDragBuildUp` component

| Condition | Action |
|-----------|--------|
| `reference_area` <= 0 | `ValueError` (division by zero) |
| any `mach` < 0 | `ValueError` (physically impossible) |
| any `mach` >= 1.0 | `RuntimeWarning` (correlations calibrated for subsonic) |
| any `wetted_area` < 0 | `RuntimeWarning` (indicates wrong exposed-area calculation) |

---

## Current Status

This component is implemented and tested, but not yet wired into the SpaJeti
mission drag polar. Next steps:

1. Map live wing/VTP/HTP/fuselage/nacelle geometry into the component arrays.
2. Feed `thickness_to_chord` and `(x/c)_m` from `AirfoilData` / `AirfoilConstantsComp`.
3. Enable nonzero VTP wetted area so H-tail parasite drag is no longer suppressed.
4. Print `CD0_component` breakdown in `run_horizontal_small_uav.py`.
5. Only after parasite drag is audited, implement `CDi` and total `CD`.
