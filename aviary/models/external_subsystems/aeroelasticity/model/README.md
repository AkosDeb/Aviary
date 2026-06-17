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
- Receives generated VTP tip mass and pitch inertia from `VTPTipInertia`.
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
- Current role: reporting and calibration only, not the primary optimizer
  constraint.

`flutter.py`

- Lumped quasi-steady 3-DOF flutter screen using plunge, torsion, and control
  rotation.
- Its design-point maximum real eigenvalue is currently the primary active
  flutter stability constraint.

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

- Computes VTP mass, pitch inertia, center of gravity, and chordwise offset
  bookkeeping for H-wing tip-mounted vertical surfaces.
- Uses the half-wing convention: two mirrored upper/lower panels at one wingtip.
  Their vertical static offsets cancel, while pitch inertia adds.
- Computes mass from VTP geometry and `aeroelasticity:vtp_areal_density`, so the
  beam model responds when VTP span changes in the optimizer.
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
