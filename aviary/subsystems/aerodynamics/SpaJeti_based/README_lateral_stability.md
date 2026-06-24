# Lateral and Longitudinal Load Factors

This document covers the OpenMDAO components in `cy_beta_vtp.py` and
`lateral_load_factor.py` that compute the side-force and lift-force
derivatives for the tail geometry equivalent surfaces and convert them to structural
load factors `Ny` (lateral) and `Nz` (longitudinal/vertical).

```text
XTailGeometry  (geometry/spajeti_based/tail_geometry.py)
    ↓  tail_physical_panel_area, tail_panel_ar, tail_cant_angle,
    ↓  tail_panel_count, wing_ref_area

VTPSurface  (lifting_surface.py)
    LiftCurveSlopePolhamus(AR=tail_panel_ar) → surface_CL_alpha   (physical panel)
    TailCantRotation → CY_beta_tail          (body-axis side-force slope)
                     → CL_alpha_tail         (body-axis lift slope)

LateralLoadFactor → CY_beta_total, CY, side_force, Ny   (fixed tail, no rudder)

WingSurface  (lifting_surface.py)
    EndplateARCorrection → AR_eff
    LiftCurveSlopePolhamus → wing_CL_alpha   (wing, endplate-corrected)

total_cl_alpha (ExecComp)
    CL_alpha_total = wing_CL_alpha + CL_alpha_tail

LongitudinalLoadFactor → CL, lift, Nz

─── Excluded (pending full integration) ───────────────────────────
CyBetaWing     → CY_beta_wing  (main wing — zero for unswept/zero-dihedral case)
CyBetaFuselage → CY_beta_fus   (fuselage — needs x₀ cross-section and Fig 10.8 table)
```

---

## Physical background

Sideslip angle `beta` is the angle between the velocity vector and the aircraft plane of
symmetry.  A positive `beta` (nose left, wind from the right) produces a side force in the
negative y-direction.  The convention here follows DATCOM: `CY_beta` is negative when the
surface is stabilising (side force opposes the sideslip).

`CY_beta` has units of per radian.

---

## TailCantRotation — cant-angle decomposition of panel lift slope

**File:** `cy_beta_vtp.py`

### What it computes

`TailCantRotation` decomposes the physical-panel Polhamus lift slope
(`CL_alpha_panel`, output of `VTPSurface`'s `LiftCurveSlopePolhamus`)
into the two body-axis stability derivatives needed by the load-factor
constraints.

For **N** symmetric panels each canted at angle **φ** from horizontal:

```
CY_beta_tail  = −N × sin²(φ) × CL_alpha_panel × S_panel / S_ref
CL_alpha_tail = +N × cos²(φ) × CL_alpha_panel × S_panel / S_ref
```

**Physical interpretation:**

- The component of panel lift that acts in the body **y-direction** (side force)
  scales as sin(φ) per panel; the total side-force derivative is −N sin²(φ)
  times the normalised lift slope.  The sign is negative (stabilising: side
  force opposes sideslip) matching the DATCOM convention for `CY_beta`.
- The component that acts in the body **z-direction** (vertical lift) scales as
  cos(φ); the tail contribution to `CL_alpha_total` is +N cos²(φ) times the
  normalised lift slope.

**Limiting cases:**

| φ | Configuration | CY_beta_tail | CL_alpha_tail |
|---|---------------|-------------|---------------|
| 90° | pure VTP (vertical) | −N × CL_alpha_panel × S/S_ref | 0 |
| 0° | pure HTP (horizontal) | 0 | +N × CL_alpha_panel × S/S_ref |
| 45° | X-tail | −N/2 × CL_alpha_panel × S/S_ref | +N/2 × … |

At 45° the lateral and longitudinal authority are exactly equal in magnitude.

### Why physical-panel AR, not projected AR

`VTPSurface` feeds `tail_panel_ar` (physical panel AR) into Polhamus, not the
projected `tail_aero_vertical_ar`.  Using the physical AR gives the correct
Reynolds number scaling and Polhamus lift slope for the actual panel.  The
cant-angle decomposition in `TailCantRotation` then projects the panel lift
into body axes — the projection must happen *after* computing the lift slope,
not before.

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `CL_alpha_panel` | 2.5 | 1/rad | Physical-panel lift slope from Polhamus |
| `tail_physical_panel_area` | 0.08 | m² | Single-panel planform area |
| `tail_cant_angle` | 45.0 | deg | Panel cant angle from horizontal |
| `tail_panel_count` | 4.0 | — | Total number of panels |
| `wing_ref_area` | 0.45 | m² | Wing reference area S_ref |
| **`CY_beta_tail`** | — | 1/rad | Body-axis side-force derivative |
| **`CL_alpha_tail`** | — | 1/rad | Body-axis lift-slope contribution |

All partials are declared with `method='cs'`.

### UAV numerical example

SpaJeti H-wing X-tail baseline: N=4 panels, φ=45°, NACA 0012 panel at the
physical panel AR.  `VTPSurface` Polhamus gives `CL_alpha_panel ≈ 1.72 /rad`;
single panel area `S_panel = 0.08 m²`; wing reference area `S_ref = 0.45 m²`.

```
authority = N × CL_alpha_panel × S_panel / S_ref
          = 4 × 1.72 × 0.08 / 0.45
          = 1.2213 /rad

CY_beta_tail  = −1.2213 × sin²(45°) = −1.2213 × 0.5 = −0.6107 /rad
CL_alpha_tail = +1.2213 × cos²(45°) = +1.2213 × 0.5 = +0.6107 /rad
```

Both outputs have equal magnitude at 45° cant (X-tail equally shares lateral
and longitudinal authority).

---

## CyBetaVtp — vertical-equivalent tail panels (legacy H-tail path)

### What it computes

The side-force derivative produced by both vertical-equivalent tail panels:

```
CY_beta_vtp = -2 * k_v * CY_beta_v_eff * (S_v / S_ref)
```

The factor of **2** accounts for the two symmetric VTP panels.

### Terms

| Symbol | Name | How it is obtained |
|--------|------|--------------------|
| `k_v` | body interference factor | interpolated from Table 2 vs `2*r_i / b_v` |
| `CY_beta_v_eff` | isolated panel side-force slope | interpolated from Table 1 vs `AR_vtp` |
| `S_v` | reference area of one VTP panel | from tail geometry |
| `S_ref` | wing reference area | from tail geometry |

### Table 1 — isolated panel effectiveness CY_beta_v_eff vs AR

`CY_beta_v_eff` is the lift-curve slope of an isolated low-aspect-ratio surface treated as
an isolated wing.  It equals `CL_alpha` of the VTP panel.

The table is approximated from the Helmbold / DATCOM formula:

```
CY_beta_v_eff(AR) ≈ 2π·AR / (2 + √(4 + AR²))   [per radian]
```

Values in the code:

```
AR    :  0.25   0.50   0.75   1.00   1.50   2.00   3.00   4.00   5.00
CY_eff:  0.42   0.78   1.16   1.48   2.09   2.60   3.41   4.04   4.54
```

**Calibrate these numbers against your aerodynamic reference before using in
optimisation.**

### Table 2 — fuselage end-plate correction k_v vs 2*r_i/b_v

`k_v = CY_beta_v_wfh / CY_beta_v_eff` corrects for the fact that the fuselage (or HTP
body) acts as an end-plate and increases the effective aspect ratio of the VTP panel.
`k_v >= 1.0` always.

```
r_i  = fuselage depth radius at the VTP quarter-chord station
b_v  = full VTP panel span (tip to tip, symmetric about attachment point)
```

Values in the code (source: DATCOM Figure 5.3.1.1-24 / Roskam Part VI):

```
2ri/bv:  0.0   0.1   0.2   0.3   0.4   0.5   0.6   0.7   0.8   0.9   1.0
k_v   :  1.00  1.08  1.18  1.28  1.38  1.48  1.56  1.63  1.69  1.73  1.76
```

**Calibrate these numbers against your aerodynamic reference before using in
optimisation.**

### Inputs and output

| Variable | Source | Units |
|----------|--------|-------|
| `fuselage_vtp_span_ratio` (2*r_i/b_v) | tail geometry | — |
| `vtp_ar` | tail geometry | — |
| `vtp_area` | tail geometry | m² |
| `wing_ref_area` | tail geometry | m² |
| **`CY_beta_vtp`** | output | 1/rad |

---

## CyBetaWing — main wing (currently excluded from Ny)

> **Status:** Component is implemented but is not currently wired into
> `LateralLoadFactor`.  For a wing with zero sweep and zero dihedral
> `CY_beta_wing = 0` exactly, so the exclusion has no numerical effect for
> a typical unswept UAV wing.  Wire it in when sweep or significant dihedral
> is present.

### What it computes

The side-force derivative produced by the main wing due to its quarter-chord sweep and
dihedral, with a Prandtl-Glauert Mach correction.  Reference: DATCOM 5.1.1.1-c.

```
CY_beta_wing = (CY_beta_w_CL + CY_beta_w_dihedral) * Mach_correction
```

### Terms

**Sweep contribution** (depends on CL — larger at high lift):

```
CY_beta_w_CL = 6·tan(Λ)·sin(Λ) / [AR·π·(AR + 4·cos(Λ))] · CL²
```

For zero sweep this term is exactly zero.

**Dihedral contribution** (constant for a given design):

```
CY_beta_w_dihedral = -0.00573 · Γ     (Γ in radians)
```

Positive dihedral (wings angled upward) gives a negative, stabilising contribution.

**Mach correction** (Prandtl-Glauert, always > 1 for subsonic flight):

```
β_M  = √(1 − M²·cos²(Λ))
Mach_correction = (AR + 4·cos(Λ)) / (AR·β_M + 4·cos(Λ))
```

### Notes on sign and magnitude

For a typical UAV wing with no sweep and no dihedral `CY_beta_wing = 0` exactly.  Adding
aft sweep makes `CY_beta_w_CL > 0` (slightly destabilising when the aircraft is producing
lift).  Adding dihedral makes `CY_beta_w_dihedral < 0` (stabilising).

The wing contribution is usually small compared with the VTP contribution.

### Inputs and output

| Variable | Type | Units |
|----------|------|-------|
| `mach` (`Dynamic.Atmosphere.MACH`) | per node | — |
| `CL` | per node, plain input | — |
| `aircraft:wing:aspect_ratio` | scalar design var | — |
| `aircraft:wing:sweep` | scalar design var | deg (quarter-chord) |
| `aircraft:wing:dihedral` | scalar design var | deg |
| **`CY_beta_wing`** | output per node | 1/rad |

`CL` is a plain `add_input` so the component is self-contained.  When wiring into a
full mission model, connect it from the flight-dynamics lift-coefficient output.

`num_nodes` is supported; pass it as a constructor option:

```python
CyBetaWing(num_nodes=nn)
```

---

## Interpolation helper

`_piecewise_linear(x, xp, fp)` is a module-level function used by `CyBetaVtp`.

It performs a piecewise linear interpolation that is compatible with OpenMDAO complex-step
derivative computation.  The key property is that the index lookup uses only the real part
of `x`, while the slope computation is fully algebraic and therefore propagates complex
perturbations correctly.

```python
# slope of the active interval carries the complex perturbation
t = (x0 - xp[i]) / (xp[i + 1] - xp[i])
return fp[i] + t * (fp[i + 1] - fp[i])
```

The function clamps the result to the defined table range (no extrapolation outside the
first or last interval).

---

## LateralLoadFactor — Ny

**File:** `lateral_load_factor.py`

### What it computes

Converts the passive tail side-force derivative to a lateral load factor at a
fixed design-point flight condition (fixed tail — no rudder):

```
CY_beta_total = CY_beta_tail                         [/rad]  (tail only)
CY            = CY_beta_total × beta                 [−]
side_force    = CY × q × S_ref                       [N]
Ny            = −side_force / (mass × g)             [−]  (positive magnitude)
```

`CY_beta_tail` carries a negative sign by DATCOM convention (wind from left →
stabilising force toward right → `CY < 0`).  Negating in the final step gives
a positive `Ny`.

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `CY_beta_tail` | 0.0 | 1/rad | From `TailCantRotation` |
| `beta_deg` | 15.0 | deg | Max sideslip angle |
| `dynamic_pressure` | 1000.0 | Pa | q at test condition |
| `wing_ref_area` | 0.45 | m² | From tail geometry |
| `aircraft_mass` | 15.0 | kg | At test condition |
| **`CY_beta_total`** | — | 1/rad | Passive side-force slope (tail) |
| **`CY`** | — | − | Total side-force coefficient |
| **`side_force`** | — | N | Side force at test condition |
| **`Ny`** | — | − | Lateral load factor magnitude |

### Optimisation constraint

```python
prob.model.add_constraint('Ny', lower=5.0)
```

### Wiring example

```python
from aviary.subsystems.geometry.spajeti_based.tail_geometry import XTailGeometry
from aviary.subsystems.aerodynamics.SpaJeti_based.lifting_surface import VTPSurface
from aviary.subsystems.aerodynamics.SpaJeti_based.lateral_load_factor import LateralLoadFactor

model = om.Group()
model.add_subsystem('geom',     XTailGeometry(),   promotes=['*'])
# VTPSurface internally wires Polhamus → TailCantRotation → CY_beta_tail, CL_alpha_tail
model.add_subsystem('vtp_surf', VTPSurface(VTP_SURFACE_CFG), promotes=['*'])
model.add_subsystem('lat_load', LateralLoadFactor(), promotes=['*'])
```

---

## LongitudinalLoadFactor — Nz

**File:** `lateral_load_factor.py`

### What it computes

Computes the vertical (pull-up) load factor the wing can generate at maximum angle of
attack using the endplate-corrected 3-D lift slope:

```
CL    = CL_alpha × alpha_max                         [−]
lift  = CL × q × S_ref                               [N]
Nz    = lift / (mass × g)                            [−]  (positive upward)
```

`CL_alpha` must be the **total** lift slope including the tail contribution.  Wire
`CL_alpha_total = wing_CL_alpha + CL_alpha_tail` (from a `total_cl_alpha` ExecComp)
into `LongitudinalLoadFactor.CL_alpha`, not just the wing lift slope.
`wing_CL_alpha` comes from `LiftCurveSlopePolhamus` with `AR_eff` from
`EndplateARCorrection`; `CL_alpha_tail` comes from `TailCantRotation`.

`alpha_max_deg` defaults to **15°** (NACA 4415 at Re~2e6 from airfoiltools.com).  This is measured from the zero-lift line
(equivalent to the aerodynamic angle of attack); for a near-symmetric UAV wing
the zero-lift angle is small, so this is a reasonable approximation.

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `CL_alpha` | 4.0 | 1/rad | Wing 3-D lift slope (endplate-corrected Polhamus) |
| `alpha_max_deg` | **12.0** | deg | Max angle of attack for check |
| `dynamic_pressure` | 1000.0 | Pa | q at test condition |
| `wing_ref_area` | 0.45 | m² | Wing reference area |
| `aircraft_mass` | 15.0 | kg | At test condition |
| **`CL`** | — | − | Wing lift coefficient at alpha_max |
| **`lift`** | — | N | Wing lift force at alpha_max |
| **`Nz`** | — | − | Vertical load factor |

### Current design values and achievable Nz

Using the H-tail UAV parameters from `README_lift_curve_slope.md` (AR_eff = 11.09,
unswept wing, M ≈ 0, thin-airfoil section):

```
CL_alpha  = 2π × 11.09 / (2 + √(11.09² + 4))  =  5.25  /rad
CL_max    = 5.25 × (12° × π/180)               =  1.10
```

Achievable Nz at sea level (ρ = 1.225 kg/m³), mass = 15 kg, S_ref = 0.45 m²:

| q [Pa] | V [m/s] | Lift [N] | **Nz** |
|-------:|--------:|---------:|-------:|
|    400 |    25.5 |      198 |   1.35 |
|    600 |    31.3 |      297 |   2.02 |
|    800 |    36.1 |      396 |   2.69 |
| **1 000** | **40.4** | **495** | **3.36** |
|  1 200 |    44.2 |      594 |   4.04 |
|  1 500 |    49.5 |      743 |   5.05 |

**At the component default (q = 1 000 Pa): Nz ≈ 3.4.**

The table shows Nz is limited to roughly 3–4 at typical cruise–to–dive-speed
conditions with the current geometry.  Increasing AR_eff (larger endplates, longer
span) or raising alpha_max raises CL_alpha and CL_max proportionally.

> **Note on CL_alpha magnitude:** at AR_eff = 11.09 the wing is operating well into
> the high-AR regime where the Polhamus formula approaches the 2-D limit.  If the
> endplate correction is removed (raw AR = 7.2), CL_alpha drops to ≈ 4.78 /rad and
> Nz at q = 1 000 Pa becomes ≈ 3.07 — a ~9 % reduction.

### Wiring example

```python
from aviary.subsystems.aerodynamics.SpaJeti_based.lift_curve_slope import (
    EndplateARCorrection, LiftCurveSlopePolhamus,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.lateral_load_factor import (
    LongitudinalLoadFactor,
)

model.add_subsystem('endplate',  EndplateARCorrection(),   promotes=['*'])
model.add_subsystem('polhamus',  LiftCurveSlopePolhamus(), promotes=['*'])
model.connect('endplate.AR_eff', 'polhamus.aspect_ratio')
model.add_subsystem('long_load', LongitudinalLoadFactor(), promotes=['*'])
model.connect('polhamus.CL_alpha', 'long_load.CL_alpha')
```

### Optimisation constraint

```python
prob.model.add_constraint('Nz', lower=<required_load_factor>)
```

---

## Wiring both load factors together

```python
import openmdao.api as om
from aviary.subsystems.geometry.spajeti_based.tail_geometry import XTailGeometry
from aviary.subsystems.aerodynamics.SpaJeti_based.lifting_surface import (
    WingSurface, VTPSurface,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.lateral_load_factor import (
    LateralLoadFactor, LongitudinalLoadFactor,
)

model = om.Group()
# Tail geometry → tail_physical_panel_area, tail_panel_ar, tail_cant_angle, …
model.add_subsystem('geom',      XTailGeometry(),          promotes=['*'])
# WingSurface: Scholz AR correction + Polhamus → wing_CL_alpha
model.add_subsystem('wing_surf', WingSurface(WING_SURFACE_CFG), promotes=['*'])
# VTPSurface: Polhamus(tail_panel_ar) → TailCantRotation → CY_beta_tail, CL_alpha_tail
model.add_subsystem('vtp_surf',  VTPSurface(VTP_SURFACE_CFG),  promotes=['*'])
# Lateral Ny constraint
model.add_subsystem('lat_load',  LateralLoadFactor(),      promotes=['*'])
# Sum wing + tail lift slopes for Nz constraint
model.add_subsystem('cl_alpha_sum', om.ExecComp(
    'CL_alpha_total = wing_CL_alpha + CL_alpha_tail'), promotes=['*'])
model.add_subsystem('long_load', LongitudinalLoadFactor(), promotes_inputs=[
    ('CL_alpha', 'CL_alpha_total'), 'alpha_max_deg', 'dynamic_pressure',
    'wing_ref_area', 'aircraft_mass',
], promotes_outputs=['*'])
```
