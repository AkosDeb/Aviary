# Superellipse Fuselage Geometry

`superellipse_fuselage.py` provides a parametric fuselage geometry component for
the SpaJeti drag/stability build-up. It is wired into the SpaJeti model as a
report-only geometry source; its outputs are not yet used to replace the mission
polar parasite drag inputs.

## Shape Model

Cross-sections are superellipses:

```text
|y/a|^n + |z/b|^n = 1
```

where:

| Parameter | Meaning |
|-----------|---------|
| `a` | half-width |
| `b` | half-height |
| `n` | superellipse exponent |

`n = 2` gives an ellipse. `n = 4` gives a rounded-rectangle-like section, which
matches the current SpaJeti rounded-square fuselage assumption better than a
pure circular body.

The longitudinal body uses:

| Region | Model |
|--------|-------|
| nose | smoothstep growth from zero width/height to max width/height |
| mid-body | constant max width/height |
| aft body | smoothstep taper to `base_width_fraction` and `base_height_fraction` |

## OpenMDAO Component

`SuperellipseFuselageGeometry` inputs:

| Input | Units | Description |
|-------|-------|-------------|
| `fuselage_length` | m | Total fuselage length |
| `max_width` | m | Maximum top-view width |
| `max_height` | m | Maximum body height |
| `nose_length_fraction` | unitless | Fraction of length assigned to nose growth |
| `tail_length_fraction` | unitless | Fraction of length assigned to aft taper |
| `base_width_fraction` | unitless | Aft-end width / max width |
| `base_height_fraction` | unitless | Aft-end height / max height |
| `superellipse_exponent` | unitless | Cross-section exponent |

Outputs:

| Output | Units | Intended use |
|--------|-------|--------------|
| `fuselage_planform_area` | m^2 | Roskam `S_plf_fus` candidate for fuselage drag due to lift |
| `fuselage_base_area` | m^2 | Roskam `S_b_fus` candidate |
| `fuselage_base_diameter` | m | `sqrt(4*S_b_fus/pi)` |
| `fuselage_wetted_area` | m^2 | Parasite drag wetted area, excluding base cap |
| `fuselage_equivalent_diameter` | m | Diameter of circle with same max cross-section area |
| `fuselage_fineness_ratio` | unitless | `length / equivalent_diameter` |
| `fuselage_max_cross_section_area` | m^2 | Maximum section area |
| `fuselage_volume` | m^3 | Integrated volume |
| `fuselage_centroid_x` | m | Geometric volume centroid from nose |

## Current SpaJeti Baseline

With:

```text
length = 2.0 m
max_width = 0.30 m
max_height = 0.30 m
nose_fraction = 0.20
tail_fraction = 0.35
base_width_fraction = 0.20
base_height_fraction = 0.20
superellipse_exponent = 4.0
```

the component gives approximately:

| Output | Value |
|--------|-------|
| `S_plf_fus` | `0.4560 m^2` |
| `S_b_fus` | `3.337e-3 m^2` |
| base diameter `d_b` | `0.0652 m` |
| `S_wet_fus` | `1.6271 m^2` |
| equivalent diameter | `0.3259 m` |
| fineness ratio | `6.14` |
| volume | `0.11305 m^3` |
| centroid x | `0.9367 m` |

This is a rounded-square cross-section: `max_width = max_height`, with rounded
edges controlled by `superellipse_exponent = 4`.

## Next Integration Steps

1. Decide whether the baseline `nose/tail/base` fractions match the actual
   SpaJeti fuselage.
2. Wire `fuselage_wetted_area`, `fuselage_equivalent_diameter`, and
   `fuselage_fineness_ratio` into parasite drag reporting.
3. DONE: wire `fuselage_planform_area` and `fuselage_base_area` into the
   report-only fuselage drag-due-to-lift component.
4. Keep base drag excluded for now, but implement it later as a separate optional
   term.
