# Aeroelasticity Model Components

This folder contains the component-level implementations for the SpaJeti
aeroelasticity subsystem. The parent folder README explains the full signal
flow; this file is a quick map for the model files themselves.

## Spanwise Model

`schrenk_lift_distribution.py`

- Builds a Schrenk lift distribution from planform lift and equivalent
  elliptical loading.
- Integrates lift/span into shear force, bending moment, and aerodynamic torque.
- This is the current lift-distribution method used before VLM/AVL calibration.

`spanwise_wingbox.py`

- Builds semispan stations, local chord, wingbox width and height, `EI(y)`,
  `GJ(y)`, structural mass/span, and structural pitch inertia/span.
- Uses the wing geometry and structural thickness inputs. This is the main
  Step 3 spanwise structural model.

`spanwise_beam_response.py`

- Integrates curvature and torsion rate from Schrenk bending moment and torque.
- Outputs static slope, deflection, twist, tip deflection, and tip twist.

`spanwise_mass_distribution.py`

- Adds engine, fuel, servo, elevon, and VTP mass effects onto the spanwise
  structural mass and pitch-inertia arrays.
- Receives tail-geometry-owned tip mass and pitch inertia equivalents through
  the `VTPTipInertia` compatibility adapter.
- Concentrated masses are distributed numerically onto nearby span stations so
  the optimizer receives smooth enough quantities for screening.

`spanwise_equivalent_properties.py`

- Reduces spanwise stiffness and inertia arrays into scalar equivalent values.
- This is the bridge that lets the older scalar divergence, flutter, P-K, load,
  and strength components consume the Step 3 spanwise data.

## Flutter And Static Screens

`beam_modal_flutter.py`

- Computes one finite-element bending mode and one finite-element torsion mode
  from the spanwise beam arrays.
- Projects the structure into a two-mode aeroelastic system.
- Uses quasi-steady strip aerodynamics for the first Step 4 modal flutter
  screen.
- Runs a reduced-frequency Theodorsen P-K iteration on the same modal system.
- Outputs modal dry frequencies, design-point maximum real eigenvalue, flutter
  speed, flutter margin, modal P-K speed, modal P-K frequency, modal P-K
  margin, modal damping, modal frequency, and critical mode.
- Current role: active optimizer constraint when
  `AEROELASTIC_FLUTTER_MODEL = 'beam_modal_3dof_pk'`; skipped when the aircraft
  config selects `legacy_scalar`. Legacy mode therefore avoids the beam-modal
  spanwise structural matrices for flutter, but it still leaves the Step-3
  spanwise structure/mass/equivalent-property components active for mass and
  scalar checks.

`flutter.py`

- Lumped quasi-steady 3-DOF flutter screen using plunge, torsion, and control
  rotation.
- Its design-point maximum real eigenvalue is the active flutter stability
  constraint when the aircraft config selects `legacy_scalar`.

`pk_flutter.py`

- Lumped P-K style flutter sweep with Theodorsen strip aerodynamics.
- Useful as a higher-fidelity scalar comparison, but it still uses the lumped
  structural representation.

`static_aeroelastic.py`

- Scalar divergence, reversal, and control-effectiveness estimates.

## Structure, Mass, And Strength Helpers

`structural_box.py`

- Root wingbox geometry, elastic-axis location, aerodynamic-center to elastic-axis
  offset, control hinge, and control inertia bookkeeping.
- Kept as the geometry/control bookkeeping layer after the Step 3 spanwise
  model was added.

`vtp_inertia.py`

- Maps tail-geometry-owned tip mass, pitch inertia, center of gravity, and
  chordwise offset equivalents into the legacy VTP aeroelastic output names.
- Uses the existing half-wing convention expected by `SpanwiseMassDistribution`.
- The shape-specific mass-property calculation lives in `TailGeometryGroup`, so
  the beam model responds to the active tail backend instead of duplicating VTP
  geometry logic here.
- The VTP inertia contributes through the spanwise mass model.

`strength_margins.py`

- Computes root bending and torsion strength margins from the scalar equivalent
  wingbox properties.

`constant_lift_loads.py`

- Older constant-load helper retained for comparison and simple load checks.

## Current Limitations

- The model is a beam-level optimizer screen, not a NASTRAN replacement.
- Schrenk loading is not yet calibrated to H-wing VLM/AVL load distributions.
- `BeamModalFlutter` has only one bending and one torsion mode.
- Modal aerodynamics are quasi-steady and Theodorsen strip theory; no DLM or
  generalized aerodynamic force matrices are used yet.
- VTP aerodynamic coupling is not included in the beam-modal screen.
- Control-surface unsteady aerodynamics and servo dynamics are not modeled.
