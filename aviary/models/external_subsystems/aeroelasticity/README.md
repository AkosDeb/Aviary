# SpaJeti Aeroelasticity External Subsystem

This folder contains the custom pre-NASTRAN aeroelasticity model used by the
horizontal small UAV case. The goal is to keep the model cheap enough for
gradient-based OpenMDAO/Aviary optimization while moving away from pure scalar
rules of thumb toward spanwise loads, spanwise structure, and modal flutter
screening.

The subsystem is implemented as OpenMDAO components and is wired by
`aeroelasticity_builder.py`.

## Current Status

The model can currently:

- estimate wingbox geometry, elastic-axis location, and control-surface
  bookkeeping from planform and spar fractions;
- build spanwise wingbox stiffness and mass arrays;
- compute a Schrenk spanwise lift distribution;
- integrate spanwise shear, bending moment, torque, deflection, and twist;
- add distributed and concentrated mass effects from structure, VTPs, engine,
  fuel, servo, and elevon assumptions;
- compute geometry-derived H-wing VTP tip mass and pitch inertia for the
  upper/lower endplate pair on each wingtip;
- reduce the spanwise beam back to equivalent scalar properties for the scalar
  fallback constraints and legacy screens;
- run scalar divergence, reversal, quasi-steady flutter, and P-K flutter
  screens;
- run a first beam-modal flutter screen using one finite-element bending mode
  and one finite-element torsion mode;
- run a reduced-frequency Theodorsen P-K iteration on those beam modes.

The model cannot yet:

- replace NASTRAN-level shell/beam finite-element validation;
- compute aerodynamic loads from VLM, DLM, CFD, or AVL directly;
- model unsteady generalized aerodynamic forces for the spanwise modal model;
- include VTP aerodynamic strips in the modal flutter screen;
- handle detailed control-surface freeplay, hinge compliance, or servo dynamics;
- certify flutter margins. Treat the results as optimization guidance and
  screening only.

## Step 3 Versus Step 4

Step 3 is the spanwise beam data model. It creates the arrays that describe the
wing along the semispan:

- station locations;
- local chord;
- local wingbox width and height;
- local `EI` and `GJ`;
- Schrenk lift, shear, bending moment, and torque;
- spanwise mass and pitch inertia;
- static deflection and twist.

Step 4 is the modal aeroelastic model. It consumes the Step 3 arrays, solves
finite-element bending and torsion mode shapes, projects the structure into
modal coordinates, and evaluates two two-mode aeroelastic stability screens:
a quasi-steady state-space screen and a reduced-frequency Theodorsen P-K screen.

The Step 4 screen is implemented and can be the active optimizer flutter
constraint. The aircraft config exposes a switch so long-running studies can
fall back to the older scalar quasi-steady flutter screen.

## Signal Flow

```text
MaterialSelector
  -> material density and allowables

WingboxStructuralEstimate
  -> elastic axis, AC-to-EA offset, control hinge data, root wingbox dimensions

SpanwiseWingboxProperties
  -> y, chord(y), wingbox width(y), wingbox height(y), EI(y), GJ(y),
     structural mass/span(y), structural pitch inertia/span(y)

SchrenkLiftDistribution
  -> lift/span(y), shear(y), bending moment(y), torque(y)

SpanwiseBeamResponse
  -> slope(y), deflection(y), twist(y), tip deflection, tip twist

SpanwiseMassDistribution
  -> added mass/span(y), total mass/span(y), total pitch inertia/span(y),
     total half-wing mass

SpanwiseEquivalentProperties
  -> equivalent EI, GJ, plunge stiffness, torsion stiffness, mass/span,
     pitch inertia/span

StaticAeroelastic, QuasiSteadyFlutterScreen, PKFlutterAnalysis
  -> active scalar divergence, reversal, flutter, and P-K screening outputs

BeamModalFlutter
  -> optional active/reporting modal frequencies, modal lambda, modal flutter
     speed, modal flutter margin, modal P-K flutter speed, modal P-K margin
```

## Active Optimization Constraints

The always-active aeroelastic constraint is:

- `aeroelasticity:divergence_speed_margin >= 0`

The active flutter constraint depends on `AEROELASTIC_FLUTTER_MODEL` in
`horizontal_small_uav_config.py`:

- `beam_modal_3dof_pk`: uses
  `aeroelasticity:beam_modal_3dof_pk_flutter_speed_margin >= 0` and builds
  `BeamModalFlutter`, including the spanwise modal structural/aerodynamic
  matrices used for the beam-modal flutter eigenanalysis.
- `legacy_scalar`: skips `BeamModalFlutter` and uses
  `aeroelasticity:max_real_eigenvalue_at_design <= 0`. In this mode the
  aeroelastic flutter analysis does not build the spanwise modal structural
  matrices.

The spanwise wingbox, mass distribution, and equivalent-property path still runs
in both modes because the SpaJeti mass model consumes those structural arrays.
So `legacy_scalar` is a cheaper flutter-analysis fallback, not a complete
rollback to a pre-spanwise structure/mass model.

## Component Summary

| Component | File | Purpose |
| --- | --- | --- |
| `MaterialSelector` | `materials/material_selector.py` | Selects material density, stiffness, and allowables. |
| `WingboxStructuralEstimate` | `model/structural_box.py` | Root wingbox geometry, elastic axis, control hinge, and control inertia bookkeeping. |
| `SpanwiseWingboxProperties` | `model/spanwise_wingbox.py` | Spanwise chord, wingbox dimensions, `EI(y)`, `GJ(y)`, mass/span, and pitch inertia/span. |
| `SchrenkLiftDistribution` | `model/schrenk_lift_distribution.py` | Schrenk lift distribution and integrated load resultants. |
| `SpanwiseBeamResponse` | `model/spanwise_beam_response.py` | Static Euler-Bernoulli deflection and torsion twist from spanwise loads. |
| `SpanwiseMassDistribution` | `model/spanwise_mass_distribution.py` | Adds engine, fuel, servo, elevon, and VTP mass effects to the structural mass arrays. |
| `SpanwiseEquivalentProperties` | `model/spanwise_equivalent_properties.py` | Reduces spanwise properties into scalar values consumed by legacy active checks. |
| `StaticAeroelastic` | `model/static_aeroelastic.py` | Divergence, reversal, and control-effectiveness screen. |
| `QuasiSteadyFlutterScreen` | `model/flutter.py` | Lumped 3-DOF quasi-steady flutter screen. |
| `PKFlutterAnalysis` | `model/pk_flutter.py` | Lumped 3-DOF P-K style flutter sweep with Theodorsen strip aerodynamics. |
| `BeamModalFlutter` | `model/beam_modal_flutter.py` | Spanwise finite-element bending/torsion modal flutter screen with quasi-steady and reduced-frequency Theodorsen P-K diagnostics. |
| `ConstantLiftLoads` | `model/constant_lift_loads.py` | Older constant lift load model kept for comparison. |
| `StrengthMargins` | `model/strength_margins.py` | Root bending and torsion strength margins. |
| `VTPTipInertia` | `model/vtp_inertia.py` | Geometry-derived H-wing VTP tip mass, pitch inertia, and chordwise offset bookkeeping. |

## Important Outputs

Spanwise load and structure outputs:

- `aeroelasticity:spanwise_stations`
- `aeroelasticity:spanwise_chord`
- `aeroelasticity:spanwise_EI`
- `aeroelasticity:spanwise_GJ`
- `aeroelasticity:schrenk_lift_per_unit_span`
- `aeroelasticity:schrenk_shear_force`
- `aeroelasticity:schrenk_bending_moment`
- `aeroelasticity:schrenk_torque`
- `aeroelasticity:spanwise_deflection`
- `aeroelasticity:spanwise_twist`
- `aeroelasticity:schrenk_tip_deflection`
- `aeroelasticity:schrenk_tip_twist_deg`

Spanwise mass outputs:

- `aeroelasticity:vtp_tip_mass`
- `aeroelasticity:vtp_tip_pitch_inertia`
- `aeroelasticity:spanwise_added_mass_per_unit_span`
- `aeroelasticity:spanwise_total_mass_per_unit_span`
- `aeroelasticity:spanwise_total_pitch_inertia_per_unit_span`
- `aeroelasticity:spanwise_total_half_wing_mass`

Beam-modal outputs:

- `aeroelasticity:beam_modal_bending_frequency`
- `aeroelasticity:beam_modal_torsion_frequency`
- `aeroelasticity:beam_modal_max_real_eigenvalue_at_design`
- `aeroelasticity:beam_modal_flutter_speed`
- `aeroelasticity:beam_modal_flutter_speed_margin`
- `aeroelasticity:beam_modal_critical_mode`
- `aeroelasticity:beam_modal_pk_flutter_speed`
- `aeroelasticity:beam_modal_pk_flutter_frequency`
- `aeroelasticity:beam_modal_pk_flutter_speed_margin`
- `aeroelasticity:beam_modal_pk_mode_damping`
- `aeroelasticity:beam_modal_pk_mode_frequency`
- `aeroelasticity:beam_modal_pk_converged`

## Known Assumptions

- Schrenk loading is used as the current lift distribution approximation.
- The spanwise structural model is a beam abstraction, not a shell model.
- `SpanwiseEquivalentProperties` deliberately bridges the spanwise model into
  the older scalar divergence and flutter checks.
- The modal flutter screen currently uses one bending mode and one torsion mode.
- Beam-modal aerodynamics are quasi-steady plus Theodorsen strip-theory P-K;
  no DLM/GAF model is attached yet.
- Concentrated masses are smeared to nearby spanwise stations for optimizer
  robustness.
- VTP tip mass uses one half-wing convention: two mirrored panels at one
  wingtip, generated from VTP geometry and `aeroelasticity:vtp_areal_density`.
- VTP aerodynamic loading is not yet included in the beam-modal flutter model.

## Validation Commands

Focused aeroelastic tests:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe -m unittest `
  aviary.models.external_subsystems.aeroelasticity.model.test_schrenk_lift_distribution `
  aviary.models.external_subsystems.aeroelasticity.model.test_spanwise_wingbox `
  aviary.models.external_subsystems.aeroelasticity.model.test_spanwise_beam_response `
  aviary.models.external_subsystems.aeroelasticity.model.test_spanwise_mass_distribution `
  aviary.models.external_subsystems.aeroelasticity.model.test_spanwise_equivalent_properties `
  aviary.models.external_subsystems.aeroelasticity.model.test_beam_modal_flutter
```

Smoke-test the aircraft case without reports:

```powershell
$env:OPENMDAO_REPORTS='0'
& C:/Software/Anaconda/envs/aviary/python.exe aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

## Upgrade Path

Near-term upgrades before moving into a NASTRAN workflow:

- calibrate Schrenk loads against VLM or AVL results for the H-wing planform;
- add more bending and torsion modes to `BeamModalFlutter`;
- replace Theodorsen strip-theory modal aerodynamics with modal generalized
  aerodynamic forces;
- add VTP aerodynamic participation to the modal flutter screen;
- connect the beam-modal stability metric as an active optimization constraint
  only after calibration;
- compare spanwise static deflection and twist against an independent beam or
  finite-element model.
