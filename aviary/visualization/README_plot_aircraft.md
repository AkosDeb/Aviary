# Aircraft 3-D Visualizer (`plot_aircraft.py`)

Reads any Aviary CSV aircraft config and draws an interactive matplotlib 3-D
figure — no OpenMDAO run required. Fuselage shape uses the same
`SuperellipseFuselageGeometry` model as the drag/stability build-up.

## Usage

**Command line:**

```bash
python -m aviary.visualization.plot_aircraft aviary/models/aircraft/small_uav/small_uav.csv

# with non-default superellipse params (match your SuperellipseFuselageGeometry setup):
python -m aviary.visualization.plot_aircraft small_uav.csv \
    --nose-frac 0.20 --tail-frac 0.35 \
    --base-w-frac 0.20 --base-h-frac 0.20 --exponent 4.0

# horizontal-small-UAV style ellipsoid nose and engine-sized aft base:
python -m aviary.visualization.plot_aircraft \
    aviary/models/aircraft/horizontal_small_uav/horizontal_small_uav.csv \
    --nose-type ellipsoid --nose-aspect-ratio 2.0 \
    --tail-frac 0.20 --exponent-blend-frac 0.10 \
    --aft-base-diameter 0.1726
```

**From Python:**

```python
from aviary.visualization.plot_aircraft import visualize

visualize('aviary/models/aircraft/small_uav/small_uav.csv')

# with explicit superellipse shape (defaults match SuperellipseFuselageGeometry):
visualize('aviary/models/aircraft/small_uav/small_uav.csv',
          nose_frac=0.20, tail_frac=0.35,
          base_w_frac=0.20, base_h_frac=0.20,
          exponent=4.0)

visualize('aviary/models/aircraft/horizontal_small_uav/horizontal_small_uav.csv',
          nose_type='ellipsoid', nose_aspect_ratio=2.0,
          tail_frac=0.20, exponent_blend_frac=0.10,
          aft_base_diameter=0.1726)
```

## What Is Drawn

| Part | Variables read from CSV | Notes |
|------|------------------------|-------|
| Fuselage | `aircraft:fuselage:length`, `max_width`, `max_height` | Superellipse cross-section (see below) |
| Wing (both halves) | `aircraft:wing:span` (or `area`+`aspect_ratio`), `taper_ratio`, `sweep`, `dihedral` | Quarter-chord sweep |
| Horizontal tail | `aircraft:horizontal_tail:area`, `aspect_ratio`, `taper_ratio`, `sweep` | Flat trapezoid, zero dihedral |
| Vertical tail | `aircraft:vertical_tail:area`, `aspect_ratio`, `taper_ratio`, `sweep` | Span in Z-up direction |
| Engines | `aircraft:nacelle:avg_diameter`, `avg_length`, `aircraft:engine:wing_locations`, `num_wing_engines`, `num_fuselage_engines` | Cylinders hung below wing |

**Coordinate convention:** x = forward, y = right (starboard), z = up. All
values converted to metres internally (handles both ft and m CSVs).

## Fuselage Shape Model

The fuselage cross-section at each station is a superellipse:

```
|y/a|^n + |z/b|^n = 1
```

where `a = width/2`, `b = height/2`, and `n` is the exponent.

The longitudinal profile uses three regions. For `nose_type='power_law'`, the
nose uses the `nose_frac` length and `nose_power_exponent` profile. For
`nose_type='ellipsoid'`, the nose length is `nose_aspect_ratio * max_width / 2`;
sections are circular (`n=2`) through the nose, then blend to the body exponent
over `exponent_blend_frac` of fuselage length.

| Region | x/L range | Width / height |
|--------|-----------|----------------|
| Nose | `0 … nose_frac` | smoothstep from 0 → max |
| Mid-body | `nose_frac … 1-tail_frac` | constant max_width × max_height |
| Aft body | `1-tail_frac … 1` | smoothstep from max → `base_w_frac × max_width` (and `base_h_frac × max_height`) |

These parameters must be set to match the `SuperellipseFuselageGeometry` component inputs used in the actual model. The visualizer defaults to the SpaJeti baseline. For aft-mounted engines, `--aft-base-diameter` can be used directly, or `--aft-engine-clearance` can be added to the CSV nacelle diameter to derive the aft base fractions.

### Superellipse parameter table

| Parameter | `visualize()` kwarg | CLI flag | Default | Matches component input |
|-----------|--------------------|----|---------|------------------------|
| Nose growth fraction | `nose_frac` | `--nose-frac` | 0.20 | `nose_length_fraction` |
| Aft taper fraction | `tail_frac` | `--tail-frac` | 0.35 | `tail_length_fraction` |
| Aft width fraction | `base_w_frac` | `--base-w-frac` | 0.20 | `base_width_fraction` |
| Aft height fraction | `base_h_frac` | `--base-h-frac` | 0.20 | `base_height_fraction` |
| Cross-section exponent | `exponent` | `--exponent` | 4.0 | `superellipse_exponent` |
| Nose type | `nose_type` | `--nose-type` | `power_law` | component option `nose_type` |
| Nose power exponent | `nose_power_exponent` | `--nose-power-exponent` | 0.5 | `nose_power_exponent` |
| Nose aspect ratio | `nose_aspect_ratio` | `--nose-aspect-ratio` | 2.0 | `nose_aspect_ratio` |
| Exponent blend fraction | `exponent_blend_frac` | `--exponent-blend-frac` | 0.10 | exponent transition length |
| Aft base diameter | `aft_base_diameter` | `--aft-base-diameter` | None | derived base fractions |
| Aft engine clearance | `aft_engine_clearance` | `--aft-engine-clearance` | None | engine diameter + clearance |

## Wing Chord Calculation

Root chord is derived from area and span using the trapezoidal formula:

```
c_root = 2 × S / (b × (1 + λ))
```

consistent with `MACGeometryComp`. Tip chord: `c_tip = λ × c_root`.

## UAV Numerical Example (SpaJeti small_uav.csv)

```
fuselage:   length = 2.00 m   max_width = 0.30 m   max_height = 0.25 m
wing:       span   = 1.80 m   area      = 0.45 m²  c_root     = 0.312 m
            sweep  = 0.0 deg  dihedral  = 3.0 deg  taper      = 0.6
h-tail:     span   = 0.671 m  c_root    = 0.298 m  sweep      = 5.0 deg
v-tail:     height = 0.379 m  c_root    = 0.452 m  sweep      = 20.0 deg
nacelle:    d      = 0.15 m   length    = 0.40 m
```

Superellipse fuselage (n=4, defaults): nose ends at x=0.40 m, constant body
0.40–1.30 m, aft taper 1.30–2.00 m, base 0.06 m × 0.05 m.
