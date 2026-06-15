# SpaJeti-Based Aerodynamics

This package contains the local SpaJeti/Roskam-style aerodynamic components.
It is intentionally separate from `aviary.subsystems.aerodynamics.flops_based`,
which should remain reserved for original Aviary/FLOPS implementations.

## Ownership Boundary

- `flops_based`: original Aviary/FLOPS components, such as `lift.py`,
  `skin_friction_drag.py`, and the stock `InducedDrag`.
- `SpaJeti_based`: local UAV/Roskam/DATCOM-style additions and replacements,
  including airfoil data, lifting-surface setup, lift-curve slope corrections,
  parasite drag build-up, Roskam induced drag, fuselage lift-induced drag, and
  lateral-stability helpers.

## Current Local Modules

- `airfoil_data.py`: local airfoil metadata, including thickness location and
  leading-edge-radius helpers used by drag and lift corrections.
- `lifting_surface.py`, `surface_config.py`, `surface_geometry.py`: local
  lifting-surface configuration and geometry wrappers.
- `lift_curve_slope.py`: Polhamus/Scholz-style lift-curve-slope utilities and
  effective aspect-ratio correction.
- `parasite_drag.py`: Roskam/DATCOM-style CD0 build-up.
- `induced_drag.py`: Roskam wing induced drag and fuselage lift-induced drag.
- `roskam_aero_group.py`: local mission aero group that combines the SpaJeti
  parasite-drag path with original FLOPS lift/induced-drag infrastructure where
  appropriate.

New custom aero components should be added here first. Only import from
`flops_based` when the dependency is intentionally an original Aviary/FLOPS
component.
