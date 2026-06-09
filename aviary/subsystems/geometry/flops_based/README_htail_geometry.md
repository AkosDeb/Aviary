# H-Tail VTP Geometry

This document covers `htail_geometry.py`, an OpenMDAO `ExplicitComponent` that defines
the geometry of the vertical tail panels (VTPs) of an H-tail configuration and derives
the intermediate quantities needed for the lateral stability analysis in
`aerodynamics/flops_based/cy_beta_vtp.py`.

---

## Why this component exists

The standard Aviary GASP empennage model (`gasp_based/empennage.py`) sizes a tail by
volume coefficient and derives geometry from that.  For H-tail lateral stability work
the useful design parameters are the direct geometric dimensions — span, root chord,
taper, and sweep — so this component inverts the approach: geometry in, area and AR out.

```text
Standard GASP approach:
  volume_coefficient + AR + taper → area → span → chord

HTailGeometry approach:
  span (b_v) + root_chord + taper + sweep → area → AR → mean_chord
```

This means the optimizer can directly control VTP panel shape.

---

## H-tail configuration

In an H-tail the two VTP panels are mounted at the tips of the horizontal tail, one on
each side.  Each panel is symmetric about its attachment point — it extends the same
distance above and below the HTP surface.

```text
            [VTP tip]
               |
   ============|============  HTP
               |
            [VTP tip]  (lower half)

   ←  b_v/2  →|←  b_v/2  →   (half-span each side of HTP)
```

`b_v` is therefore the full panel span from lower tip to upper tip.

---

## Geometry definitions

| Symbol | Aviary variable | Definition |
|--------|----------------|------------|
| `b_v` | `Aircraft.VerticalTail.SPAN` | Full VTP panel span, tip to tip |
| `c_root` | `Aircraft.VerticalTail.ROOT_CHORD` | Chord at the attachment line (root) |
| `λ` | `Aircraft.VerticalTail.TAPER_RATIO` | Tip chord / root chord |
| `Λ` | `Aircraft.VerticalTail.SWEEP` | Quarter-chord sweep angle |
| `r_i` | `Aircraft.Fuselage.MAX_HEIGHT / 2` | Fuselage depth radius at VTP quarter-chord |

---

## What is computed

### VTP panel area

```
S_v = b_v * c_root * (1 + λ) / 2
```

This is the trapezoidal wing area formula: average chord times span.  For a rectangular
panel (λ = 1) it simplifies to `S_v = b_v * c_root`.

### VTP aspect ratio

```
AR_v = b_v² / S_v = 2 * b_v / (c_root * (1 + λ))
```

### VTP mean (average) chord

```
c_mean = S_v / b_v = c_root * (1 + λ) / 2
```

### Fuselage-to-VTP span ratio

```
fuselage_vtp_span_ratio = 2 * r_i / b_v
```

This ratio is the input to the fuselage interference lookup table in `CyBetaVtp`.  It
compares the fuselage diameter to the full VTP span: a large ratio means the fuselage is
wide relative to the panel and the end-plate correction will be large.

For the current UAV (fuselage height 0.25 m, VTP span 0.31 m):

```
r_i    = 0.25 / 2 = 0.125 m
2*r_i  = 0.25 m
b_v    = 0.31 m
ratio  = 0.25 / 0.31 ≈ 0.81
```

### Wing reference area and half-span

```
wing_ref_area  = Aircraft.Wing.AREA   (taper=1, sweep=0 → S_ref = c_root_w * span = area)
wing_half_span = Aircraft.Wing.SPAN / 2
```

These are passed straight through for use by the aerodynamic components downstream.

---

## Inputs

| Aviary variable | CSV key | Role |
|----------------|---------|------|
| `Aircraft.Fuselage.MAX_HEIGHT` | `aircraft:fuselage:max_height` | r_i source |
| `Aircraft.VerticalTail.SPAN` | `aircraft:vertical_tail:span` | design var b_v |
| `Aircraft.VerticalTail.ROOT_CHORD` | `aircraft:vertical_tail:root_chord` | design var |
| `Aircraft.VerticalTail.TAPER_RATIO` | `aircraft:vertical_tail:taper_ratio` | design var |
| `Aircraft.VerticalTail.SWEEP` | `aircraft:vertical_tail:sweep` | design var (future use) |
| `Aircraft.Wing.SPAN` | `aircraft:wing:span` | for wing half-span output |
| `Aircraft.Wing.AREA` | `aircraft:wing:area` | for wing reference area |

`Aircraft.VerticalTail.SWEEP` is accepted as a design input for future use (e.g. swept
AR corrections, wetted-area calculations).  It does not affect any current output and no
partial is declared for it.

---

## Outputs

| Name | Aviary variable | Units | Notes |
|------|----------------|-------|-------|
| `aircraft:vertical_tail:area` | `Aircraft.VerticalTail.AREA` | m² | fed to `CyBetaVtp` |
| `aircraft:vertical_tail:aspect_ratio` | `Aircraft.VerticalTail.ASPECT_RATIO` | — | fed to `CyBetaVtp` |
| `aircraft:vertical_tail:average_chord` | `Aircraft.VerticalTail.AVERAGE_CHORD` | m | for drag etc. |
| `fuselage_vtp_span_ratio` | plain string | — | fed to `CyBetaVtp` |
| `wing_ref_area` | plain string | m² | fed to `CyBetaVtp` |
| `wing_half_span` | plain string | m | available for future components |

The Aviary outputs (`AREA`, `ASPECT_RATIO`, `AVERAGE_CHORD`) use `add_aviary_output` so
they integrate with any downstream Aviary component that expects those standard variable
names.

---

## Partials

Complex-step derivatives (`method='cs'`) are declared only for non-zero partial pairs:

| Output | Depends on |
|--------|-----------|
| `AREA` | `SPAN`, `ROOT_CHORD`, `TAPER_RATIO` |
| `ASPECT_RATIO` | `SPAN`, `ROOT_CHORD`, `TAPER_RATIO` |
| `AVERAGE_CHORD` | `ROOT_CHORD`, `TAPER_RATIO` |
| `fuselage_vtp_span_ratio` | `MAX_HEIGHT`, `SPAN` (VTP) |
| `wing_ref_area` | `Wing.AREA` |
| `wing_half_span` | `Wing.SPAN` |

`SWEEP` has no declared partials (zero contribution to all current outputs).

---

## CSV changes needed when integrating into the full model

**Remove** these two entries (they become computed outputs):

```
aircraft:vertical_tail:area
aircraft:vertical_tail:aspect_ratio
```

**Add** these two entries (they become design inputs):

```
aircraft:vertical_tail:span,<b_v_in_meters>,m
aircraft:vertical_tail:root_chord,<c_root_in_meters>,m
```

For the current UAV design (area=0.08 m², AR=1.2, taper=0.4) the back-calculated values
are:

```
aircraft:vertical_tail:span,0.3098,m
aircraft:vertical_tail:root_chord,0.3690,m
```

Keep these in the CSV unchanged:

```
aircraft:vertical_tail:taper_ratio,0.4,unitless
aircraft:vertical_tail:sweep,20.0,deg
```

---

## Wiring into the problem

```python
from aviary.subsystems.geometry.flops_based.htail_geometry import HTailGeometry
from aviary.subsystems.aerodynamics.flops_based.cy_beta_vtp import CyBetaVtp

model.add_subsystem('htail_geom', HTailGeometry(), promotes=['*'])
model.add_subsystem('cy_vtp',     CyBetaVtp(),     promotes=['*'])
```

With `promotes=['*']` the Aviary variable names connect automatically between the two
components.  The plain-string outputs (`fuselage_vtp_span_ratio`, `wing_ref_area`) also
promote correctly because both components use the same string.
