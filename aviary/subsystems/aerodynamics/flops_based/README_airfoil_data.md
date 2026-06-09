# Airfoil Data Catalog

This document covers `airfoil_data.py` and the airfoil section properties used by
the local lifting-surface and drag build-up components.

The catalog is intentionally small and explicit. It is not a replacement for
XFOIL/MSES/CFD data; it is a conceptual-design data layer for keeping all
surface-level assumptions visible and reusable.

---

## Data Object

`AirfoilData` is a frozen dataclass with these fields:

| Field | Units | Meaning |
|-------|-------|---------|
| `name` | - | Airfoil name |
| `cl_alpha_per_rad` | 1/rad | Incompressible 2-D section lift slope |
| `cl_max` | unitless | 2-D maximum lift coefficient at `re_ref` |
| `cd_min` | unitless | Minimum profile drag coefficient near design lift |
| `cm_ac` | unitless | Pitching moment about aerodynamic centre / quarter chord |
| `tc_ratio` | unitless | Thickness-to-chord ratio `(t/c)` |
| `camber_ratio` | unitless | Maximum camber-to-chord ratio |
| `max_thickness_location` | unitless | Chordwise maximum-thickness location `(x/c)_m` |
| `re_ref` | unitless | Reference Reynolds number for the section data |

Important convention:

```text
cl_alpha_per_rad is incompressible 2-D data.
```

Do not Prandtl-Glauert correct it before passing it into
`LiftCurveSlopePolhamus`. The Polhamus/DATCOM 3-D formula handles compressibility
internally through `beta = sqrt(1 - M^2)`.

---

## Maximum Thickness Location `(x/c)_m`

The parasite drag lifting-surface form factor uses:

```text
FF = 1 + L'*(t/c) + 100*(t/c)^4
```

Therefore `(x/c)_m` must be stored with the airfoil data.

Per the current project convention requested for this implementation:

| Condition | `L'` |
|-----------|------|
| `(x/c)_m <= 0.30` | `1.2` |
| `(x/c)_m > 0.30` | `2.0` |

The separate Roskam/DATCOM lifting-surface correction factor `R_LS` is computed
from Mach and sweep in `aero_utils.lifting_surface_correction_factor`.

Typical values:

| Airfoil family | Typical `(x/c)_m` |
|----------------|-------------------|
| NACA 4-digit | `0.30` |
| NACA 6-series / NACA 64A | `0.40` |

Current NACA 4-digit entries use `max_thickness_location = 0.30`.

---

## Current Catalog

| Airfoil | `cl_alpha` [/rad] | `CLmax_2D` | `CDmin` | `Cm_ac` | `t/c` | camber | `(x/c)_m` | `Re_ref` |
|---------|-------------------|------------|---------|---------|-------|--------|-----------|----------|
| NACA 0009 | 5.95 | 1.10 | 0.0055 | 0.000 | 0.09 | 0.00 | 0.30 | 1.0e6 |
| NACA 0012 | 5.73 | 1.30 | 0.0070 | 0.000 | 0.12 | 0.00 | 0.30 | 2.0e6 |
| NACA 2412 | 5.93 | 1.45 | 0.0062 | -0.047 | 0.12 | 0.02 | 0.30 | 2.0e6 |
| NACA 4412 | 6.10 | 1.50 | 0.0060 | -0.099 | 0.12 | 0.04 | 0.30 | 2.0e6 |
| NACA 4415 | 6.00 | 1.60 | 0.0076 | -0.100 | 0.15 | 0.04 | 0.30 | 2.0e6 |

Representative Reynolds numbers for SpaJeti:

| Surface | Characteristic chord | Flight condition | Approx. Re |
|---------|----------------------|------------------|------------|
| Wing | 0.25-0.33 m | 550 km/h, 5000 m ISA | about 2e6 |
| VTP | 0.20-0.30 m | 550 km/h, 5000 m ISA | about 1.5e6 |

---

## How Airfoil Data Is Used

### Lift

`LiftingSurfaceGroup` contains `AirfoilConstantsComp`, which exposes airfoil data
as OpenMDAO outputs at group scope:

| Output | Source field |
|--------|--------------|
| `section_cl_alpha` | `cl_alpha_per_rad` |
| `section_cl_max` | `cl_max` |
| `section_cd_min` | `cd_min` |
| `section_cm_ac` | `cm_ac` |
| `section_tc` | `tc_ratio` |
| `section_camber` | `camber_ratio` |
| `section_max_thickness_location` | `max_thickness_location` |

`section_cl_alpha` feeds `LiftCurveSlopePolhamus`.

### M_crit

`section_tc` feeds `MachCriticalComp`. In the SpaJeti model, the wing value is
promoted as `wing_section_tc` and can be optimized independently of the CSV
`aircraft:wing:thickness_to_chord` until weight/geometry coupling is completed.

### Parasite Drag

Future parasite-drag wiring should use:

| Drag input | Airfoil source |
|------------|----------------|
| `thickness_to_chord` | `section_tc` |
| `max_thickness_location_over_chord` | `section_max_thickness_location` |

This keeps the lifting-surface form factor tied to the selected airfoil instead
of hardcoding `(x/c)_m = 0.4` or any other value.

---

## Adding A New Airfoil

1. Add a new `AirfoilData` constant in `airfoil_data.py`.
2. Use incompressible 2-D `cl_alpha_per_rad`.
3. Record the Reynolds number and data source.
4. Include `max_thickness_location`; use `0.40` for NACA 6-series style sections
   only when that is correct for the chosen airfoil.
5. Update this README table.
6. Update `SurfaceConfig` selection in `run_horizontal_small_uav.py` if the new
   airfoil should be used by the wing, VTP, HTP, canard, etc.
