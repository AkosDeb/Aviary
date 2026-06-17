# SpaJeti v1.0.0 H-wing - Model Changelog

## v1.29.0 — Step 5: 3-DOF control surface in BeamModalFlutter + active constraint switch

- Extended `BeamModalFlutter` from 2-mode [bending, torsion] to 3-mode [bending, torsion,
  control rotation]. Control mode shape is a rigid binary rotation over the elevon span.
  3×3 modal mass matrix includes bending–control (via `CONTROL_STATIC_UNBALANCE`) and
  torsion–control (via `CONTROL_INERTIA_PER_UNIT_SPAN`) cross terms. GAF: Theodorsen C(k)
  for [h, α] block; quasi-steady for δ column (Theodorsen-Garrick deferred). Same C(k)
  scaling applied to ch_alpha terms, consistent with `PKFlutterAnalysis`.
- New outputs: `BEAM_MODAL_CONTROL_FREQUENCY`, `BEAM_MODAL_3DOF_PK_FLUTTER_SPEED`,
  `BEAM_MODAL_3DOF_PK_FLUTTER_FREQUENCY`, `BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN`,
  `BEAM_MODAL_3DOF_PK_CONVERGED`, `BEAM_MODAL_3DOF_PK_MODE_DAMPING`,
  `BEAM_MODAL_3DOF_PK_MODE_FREQUENCY`.
- Switched active optimizer flutter constraint from quasi-steady
  `MAX_REAL_EIGENVALUE_AT_DESIGN <= 0` to beam-modal 3-DOF P-K speed margin
  `BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN >= 0`.

---

## v1.28.0 - 2026-06-17

**Geometry-derived VTP tip inertia for Step 5**

- `VTPTipInertia` now computes the half-wing H-wing endplate mass from VTP
  geometry and `aeroelasticity:vtp_areal_density` instead of relying on the
  FLOPS vertical-tail mass, which is zeroed in the current H-wing setup.
- The half-wing convention is explicit: each wingtip carries two mirrored VTP
  panels, one up and one down. Their vertical static offsets cancel, while their
  pitch inertias add.
- Generated `aeroelasticity:vtp_tip_mass` and
  `aeroelasticity:vtp_tip_pitch_inertia` are now promoted into
  `SpanwiseMassDistribution`, so beam modes respond to VTP span and areal
  density.
- Added regression tests for VTP areal-density and span sensitivity, and added
  the generated VTP inertial values to the SpaJeti result report.

Versioning note:
- This is a `MINOR` bump because it changes the Step-5 aeroelastic mass/inertia
  physics feeding the spanwise beam and beam-modal flutter screens.

---

## v1.27.0 - 2026-06-17

**Beam-modal P-K flutter diagnostics**

- Extended `BeamModalFlutter` with a reduced-frequency Theodorsen P-K iteration
  on the finite-element bending and torsion modes.
- Added beam-modal P-K speed, frequency, margin, damping, modal frequency, and
  convergence outputs.
- Wired the new Step-4 modal P-K outputs into `AeroelasticityGroup`, the
  SpaJeti result report, tests, and aeroelasticity documentation.
- The beam-modal outputs remain reporting/calibration diagnostics; the existing
  scalar design-point eigenvalue constraint stays active until modal calibration
  is complete.

Versioning note:
- This is a `MINOR` bump because it adds new aeroelastic physics and report
  outputs to the Step-4 beam-modal path.

---

## v1.26.1 - 2026-06-17

**Aeroelasticity documentation refresh**

- Replaced the stale aeroelasticity README with the current spanwise Step-3 and
  beam-modal Step-4 architecture.
- Added a model-folder component catalog documenting the Schrenk, spanwise
  wingbox, spanwise mass, spanwise equivalent, scalar flutter, P-K, and
  beam-modal components next to their implementations.

Versioning note:
- This is a `PATCH` bump because it documents the existing v1.26.0
  implementation without changing model behavior.

---

## v1.26.0 - 2026-06-17

**Beam-modal flutter screen**

- Added `BeamModalFlutter`, the first Step-4 spanwise beam-modal aeroelastic
  screen.
- The component computes one finite-element bending mode and one finite-element
  torsion mode from the Step-3 spanwise beam arrays, evaluates design-point
  modal stability, and scans for a modal flutter speed.
- The aerodynamic model is quasi-steady strip theory for now; it is the bridge
  toward a later modal P-K/generalized-aerodynamic-force implementation.
- Wired the new modal outputs into `AeroelasticityGroup` and the SpaJeti results
  report. The existing scalar `lambda_max` constraint is retained until the
  modal screen is calibrated.

Versioning note:
- This is a `MINOR` bump because it adds the first beam-modal flutter physics
  path.

---

## v1.25.1 - 2026-06-17

**Step-3 optimization validation and console-safe report text**

- Ran the full v1.25.0 optimization with the spanwise-equivalent aeroelastic
  constraints active.
- IPOPT reached local infeasibility, but the final physical constraint table
  satisfied the model-level checks: Ny, Nz, M_crit margin, divergence margin,
  design-point flutter eigenvalue, SLS thrust, fuel budget, and engine mass.
- Replaced a non-ASCII approximate symbol in the parasite-drag report so the
  normal Windows console can complete post-processing without requiring
  `PYTHONIOENCODING=utf-8`.

Versioning note:
- This is a `PATCH` bump because it records validation and fixes report text
  only; the Step-3 physics implementation remains v1.25.0.

---

## v1.25.0 - 2026-06-17

**Spanwise-equivalent aeroelastic active constraints**

- Added `SpanwiseEquivalentProperties`, reducing spanwise `EI(y)`, `GJ(y)`,
  total mass/span, and total pitch inertia/span into scalar properties for the
  existing divergence, flutter, P-K, load, and strength checks.
- Switched promoted scalar `EI`, `GJ`, torsional stiffness, plunge stiffness,
  mass/span, and pitch inertia/span to the spanwise-equivalent reducer.
- Kept `WingboxStructuralEstimate` for root geometry, elastic-axis, and control
  bookkeeping only.
- Connected live engine mass and mission fuel mass into the spanwise mass model.

Versioning note:
- This is a `MINOR` bump because active aeroelastic constraints now consume the
  Step-3 spanwise beam reduction.

---

## v1.24.0 - 2026-06-17

**Spanwise aeroelastic mass distribution**

- Added `SpanwiseMassDistribution` to combine structural wingbox mass with
  explicit VTP tip, engine, fuel, servo, and elevon mass terms.
- The component reports total spanwise mass per span, total pitch inertia per
  span, added mass per span, and total half-wing mass for the future beam modal
  flutter model.
- Added explicit aeroelastic mass input names so the spanwise model does not
  depend on FLOPS mass outputs that may intentionally be zeroed for the H-wing
  endplates.
- Existing scalar divergence/flutter constraints remain unchanged.

Versioning note:
- This is a `MINOR` bump because it adds a new aeroelastic mass-property physics
  component.

---

## v1.23.0 - 2026-06-17

**Schrenk spanwise beam response**

- Added `SpanwiseBeamResponse`, which integrates Schrenk bending moment and
  torque against local `EI(y)` and `GJ(y)`.
- Wired Schrenk distributed loads and the spanwise beam response into
  `AeroelasticityGroup` as analysis/reporting outputs.
- Existing scalar divergence/flutter constraints still use the equivalent
  wingbox path until the spanwise beam is fully reviewed.
- Added tests for closed-form constant moment/torque response and clamped-root
  boundary conditions.

Versioning note:
- This is a `MINOR` bump because it adds a new aeroelastic physics component
  and live group wiring.

---

## v1.22.0 - 2026-06-17

**Spanwise wingbox properties**

- Added `SpanwiseWingboxProperties`, the first Step-3 aeroelastic beam-model
  component.
- The component computes spanwise stations, local chord, wingbox width/height,
  `EI(y)`, `GJ(y)`, mass per span, and pitch inertia per span.
- Wired the component into `AeroelasticityGroup` as analysis-only with unique
  output names, leaving the current scalar divergence/flutter constraints
  unchanged.
- Added tests for tapered-wing station trends and structural `t/c` stiffness
  response.

Versioning note:
- This is a `MINOR` bump because it adds new aeroelastic structural physics.

---

## v1.21.1 - 2026-06-17

**Aeroelastic t/c validation**

- Ran a full-model sensitivity check at `wing_section_tc = 0.05, 0.15, 0.18`.
- Confirmed structural `t/c` follows `wing_section_tc`, and `EI`, `GJ`, and
  divergence speed increase monotonically with thickness.
- Added a regression test for monotonic wingbox stiffness response to structural
  `t/c`.
- Marked the Step 2 validation complete in `TOOD.md`.

Versioning note:
- This is a `PATCH` bump because it validates and tests the v1.21.0 wiring.

---

## v1.21.0 - 2026-06-17

**Aeroelastic structural t/c coupling**

- Added `aeroelasticity:structural_thickness_to_chord` as the explicit wingbox
  thickness input for aeroelasticity.
- Connected live `wing_section_tc` to the aeroelastic structural wingbox so M_crit
  and structural stiffness now respond to the same airfoil thickness design
  variable.
- Updated `TOOD.md` to mark the Schrenk backend as done and the structural t/c
  coupling as implemented pending validation.

Versioning note:
- This is a `MINOR` bump because it changes aeroelastic structural physics wiring.

---

## v1.20.0 - 2026-06-17

**Schrenk aeroelastic load backend**

- Added `SchrenkLiftDistribution`, a spanwise half-wing load-distribution
  component for the custom pre-NASTRAN aeroelasticity path.
- The component outputs span stations, local chord, lift per span, shear,
  bending moment, torque, and compatible root/tip summary values.
- Added the aeroelasticity improvement roadmap to `TOOD.md`.

Versioning note:
- This is a `MINOR` bump because it adds a new aeroelastic analysis component,
  although it is not yet wired into the optimizer.

---

## v1.19.0 - 2026-06-15

**Optimization feasibility recovery**

- Added `aircraft:wing:area` as an optimization design variable with bounds
  `0.40 <= S_ref <= 0.75 m^2`.
- Restored the climb phase transcription to `num_segments = 4`, `order = 3`
  to get back to a more stable continuation point before re-refining the mesh.
- Kept the v1.18.5 SpaJeti 3D report camera/fallback fix.

Versioning note:
- This is a `MINOR` bump because it changes the optimization design space by
  adding wing area as a new design variable.

---

## v1.18.5 - 2026-06-15

**SpaJeti 3D camera robustness**

- Replaced the dashboard report's dependency on `aframe-orbit-controls` for the
  initial camera placement with a fixed camera view.
- Added a compact plan-view fallback overlay so the H-wing layout remains
  visible even if WebGL controls fail inside the dashboard iframe.

Versioning note:
- This is a `PATCH` bump because it only improves report rendering robustness.

---

## v1.18.4 - 2026-06-15

**SpaJeti 3D dashboard fallback link**

- Added a standard `reports/subsystems/spajeti_3d.md` report entry that links
  to the standalone SpaJeti 3D HTML file.
- Kept the custom dashboard Results tab, but made the geometry easier to find
  if the dashboard command is launched through an older or different Aviary
  entry point.

Versioning note:
- This is a `PATCH` bump because it only improves report discoverability.

---

## v1.18.3 - 2026-06-15

**Flat wing geometry**

- Set `aircraft:wing:dihedral` to `0.0 deg` in the horizontal small UAV input
  deck.
- Changed the SpaJeti 3D geometry fallback dihedral to `0.0 deg`.
- Updated the README geometry note to describe the flat-wing convention.

Versioning note:
- This is a `PATCH` bump because it changes a single geometry input and its
  visualization fallback.

---

## v1.18.2 - 2026-06-15

**SpaJeti 3D engine visual cleanup**

- Moved the dark turbojet visual from an intersecting aft-fuselage cylinder to
  a small aft nozzle disk.
- Added an overlay note identifying the dark aft disk as the turbojet nozzle.

Versioning note:
- This is a `PATCH` bump because it only clarifies the dashboard geometry
  visualization.

---

## v1.18.1 - 2026-06-15

**SpaJeti 3D geometry shape correction**

- Changed the dashboard SpaJeti fuselage visual from a simple cylinder to a
  faceted superellipse mesh using the same nose/tail/base fractions and
  rounded-square exponent as `SuperellipseFuselageGeometry`.
- Changed each wingtip VTP visual into two mirrored panels: one grows upward
  and one grows downward from the wingtip centerline, matching the split
  endplate interpretation used by the effective aspect-ratio correction.

Versioning note:
- This is a `PATCH` bump because it corrects visualization geometry only.

---

## v1.18.0 - 2026-06-15

**SpaJeti 3D dashboard geometry**

- Added a SpaJeti-specific 3D HTML report written to
  `reports/spajeti_aircraft_3d.html` after each run.
- The report uses live optimized geometry values for fuselage length/width,
  wing span/area/taper/sweep/dihedral, twin wingtip VTP endplates, and engine
  diameter.
- Updated the Aviary dashboard to add a `SpaJeti 3D Geometry` Results tab when
  that report file exists.

Versioning note:
- This is a `MINOR` bump because it adds a new user-facing dashboard report.

---

## v1.17.1 - 2026-06-15

**Longitudinal load-factor report fix**

- Fixed the printed Nz reproduction to use `ALPHA_MAX_DEG` instead of a
  hardcoded `12.0 deg`.
- This makes the reproduced `CL`, lift, and `Nz` agree with the OpenMDAO
  `LongitudinalLoadFactor` outputs when `ALPHA_MAX_DEG = 15.0`.

Versioning note:
- This is a `PATCH` bump because it fixes report post-processing only.

---

## v1.17.0 - 2026-06-15

**Full SpaJeti/Roskam mission drag integration**

- Replaced the mission-phase FLOPS lift-balance and induced-drag components
  inside `SpaJeti_based/roskam_aero_group.py` with local SpaJeti components.
- Mission drag now includes:

  `CD = CD0 + CDI_wing + CDI_fus`

  before conversion to force with `D = CD*q*S_ref`.
- Added mission-node fuselage lift-induced drag using Roskam Eq. 4.33 with
  `alpha = CL / CL_alpha_w`, so the fuselage lift-drag term varies along the
  trajectory instead of remaining report-only.
- Added the required mission static parameters for `wing_CL_alpha`,
  `fuselage_base_area`, `fuselage_planform_area`, and
  `fuselage_fineness_ratio`.
- Confirmed the wing section `t/c` lower optimization bound is `0.05`.

Versioning note:
- This is a `MINOR` bump because the mission drag physics and aero ownership
  wiring changed.

---

## v1.16.0 - 2026-06-15

**SpaJeti aero package split**

- Added `aviary/subsystems/aerodynamics/SpaJeti_based/` for local
  SpaJeti/Roskam/DATCOM-style aero modules.
- Moved the local airfoil, lifting-surface, lift-curve-slope, parasite-drag,
  Roskam induced-drag, lateral-stability, and support modules out of the
  `flops_based` namespace.
- Restored `flops_based/induced_drag.py` to the original FLOPS `InducedDrag`
  component only; local `RoskamInducedDragComp` and
  `FuselageLiftInducedDragComp` now live in `SpaJeti_based/induced_drag.py`.
- Updated model and builder imports to use `SpaJeti_based` for local
  implementations while retaining original FLOPS imports where intentionally
  used.

Versioning note:
- This is a `MINOR` bump because it is an aero-module ownership/package
  reorganization with no intended physics change.

---

## v1.15.2 - 2026-06-15

**Fuselage base diameter output**

- Added `fuselage_base_diameter` to `SuperellipseFuselageGeometry`:

  `d_b = sqrt(4*S_b_fus/pi)`

- Promoted and printed the value in the SpaJeti model alongside `S_b_fus`.

Versioning note:
- This is a `PATCH` bump because it adds a derived geometry output and diagnostics.

---

## v1.15.1 - 2026-06-15

**Rounded-square fuselage geometry**

- Changed the local `SuperellipseFuselageGeometry` path to use a rounded-square
  cross-section: `max_width = max_height = 0.30 m`, with rounded edges from the
  superellipse exponent `n = 4`.
- The superellipse fuselage geometry is now driven by local fixed outputs
  `fus_max_width` and `fus_max_height`, rather than the legacy CSV
  `aircraft:fuselage:max_width/max_height` values.
- Updated docs/tests to describe the local rounded-square convention instead of
  saying the geometry must match the CSV dimensions.

Versioning note:
- This is a `PATCH` bump because it changes the local fuselage geometry
  parameterization, not the architecture.

---

## v1.15.0 - 2026-06-15

**Fuselage equivalent diameter bug fix + wing wetted area safeguard + CDi_fus table interpolation**

### Bug fix — `FUSELAGE_EQUIV_DIAMETER_M`

The constant was hardcoded as `0.169 m`, derived assuming a 15 cm square
cross-section (`d = 0.15 × sqrt(4/π)`). The actual fuselage is a 30 cm × 25 cm
superellipse (n = 4), giving a much larger max-section area.

Corrected value:

```
A_max = superellipse_area(0.30, 0.25, 4.0) = 0.06953 m²
FUSELAGE_EQUIV_DIAMETER_M = sqrt(4 × A_max / π) ≈ 0.2975 m
```

Same formula as `SuperellipseFuselageGeometry.compute()` so the K_wf fuselage
correction factor and the post-run geometry report are now consistent.

Effect on wing exposed wetted area at baseline:

| | d_fus | s_buried | swet_wing |
|-|-------|---------|----------|
| Before fix | 0.169 m | 0.0264 m² | 0.8472 m² |
| After fix  | 0.298 m | 0.0465 m² | 0.8070 m² |

Impact: ~4.7% smaller wing Swet → slightly lower K_wf (fuselage enlargement effect
on root chord buried fraction is larger, so slightly more of the planform is
exposed), and correspondingly lower CD0_wing estimate.

New module-level constants added:
- `FUSELAGE_MAX_WIDTH_M = 0.30` (must match `aircraft:fuselage:max_width` in CSV)
- `FUSELAGE_MAX_HEIGHT_M = 0.25` (must match `aircraft:fuselage:max_height` in CSV)

### Runtime safeguard

Added two assertions in `_print_parasite_drag()` immediately after computing
`s_buried` and `swet_wing`. These raise `ValueError` with descriptive messages if:
- `s_buried` is not in `(0, wing_area)` — fuselage buries whole wing or is zero
- `swet_wing` is not in `(0, 2 × wing_area]` — physically impossible bounds

### Unit tests

New test module `aviary/models/aircraft/horizontal_small_uav/tests/test_wing_exposed_swet.py`
(9 tests, no OpenMDAO required):

- `TestFuselageEquivDiameter` — verifies `FUSELAGE_EQUIV_DIAMETER_M` matches
  `superellipse_area` formula and is positive and within the bounding-rectangle
  equivalent diameter
- `TestWingExposedWettedArea` — verifies `s_buried > 0`, `s_buried < S_wing`,
  `swet_wing > 0`, `swet_wing ≤ 2S_wing`, the `(S_planform − S_buried) × 2`
  convention, and that the fuselage buries less than 50 % of the planform

### `FuselageLiftInducedDragComp` — Roskam table interpolation

`eta_finite_cylinder` and `crossflow_drag_coefficient` are no longer user-supplied
scalar constants. Both are now interpolated from Roskam Part VI digitised tables
inside `compute()`:

**eta** (finite-cylinder interference factor) — Fig. 4.32

| l/d  |  2   |  6   | 12   | 18   | 28   |
|------|------|------|------|------|------|
| eta  | 0.52 | 0.64 | 0.71 | 0.75 | 0.79 |

**c_d_c** (crossflow drag coefficient) — Fig. 4.31

| Mc = M·sin(α) | 0.00 | 0.25 | 0.40 | 0.50 | 0.70 |
|----------------|------|------|------|------|------|
| c_d_c          | 1.20 | 1.20 | 1.27 | 1.37 | 1.68 |

Interpolation is linear (`numpy.interp`) and clamped to the table endpoints.
Both computed values are now promoted as outputs (`eta_finite_cylinder`,
`crossflow_drag_coefficient`) for post-run diagnostics.

**New inputs:** `fuselage_fineness_ratio` (from `SuperellipseFuselageGeometry`),
`Mach` (promoted from `design_mach` fixed output).
**Removed inputs:** `eta_finite_cylinder`, `crossflow_drag_coefficient`.

**SpaJeti design point** (l/d = 2.0/0.2975 ≈ 6.7, Mc = 0.477·sin(15°) ≈ 0.123):

```
eta   = interp(6.72,  [2,6,12,18,28],           [0.52,0.64,0.71,0.75,0.79]) = 0.648
c_d_c = interp(0.123, [0,0.25,0.40,0.50,0.70],  [1.20,1.20,1.27,1.37,1.68]) = 1.20
```

(Old hardcoded values: eta=0.85, c_d_c=1.20. The planform term coefficient drops
from 1.02 → 0.778, a 24 % reduction.)

Versioning note:
- `MINOR` bump folded into 1.15.0 because it replaces hardcoded placeholders with
  real Roskam chart physics — this is a new formula implementation, not just a
  parameter change.

---

## v1.14.0 - 2026-06-15

**Aircraft 3-D visualizer from CSV config**

- New file `aviary/visualization/plot_aircraft.py`: standalone matplotlib 3-D
  visualizer that reads any Aviary CSV config directly (no OpenMDAO run needed).
- Draws fuselage, wing, h-tail, v-tail, and engine nacelles.
- **Fuselage uses `SuperellipseFuselageGeometry` shape model** — same smoothstep
  nose/aft profile, superellipse cross-section exponent, and `base_width/height_fraction`
  as the drag/CDi_fus build-up, so the visualizer matches the physical model.
- Superellipse parameters (`nose_frac`, `tail_frac`, `base_w_frac`, `base_h_frac`,
  `exponent`) are kwargs to `visualize()` with defaults matching the SpaJeti baseline;
  also exposed as CLI flags `--nose-frac`, `--tail-frac`, `--base-w-frac`,
  `--base-h-frac`, `--exponent`.
- All CSV units converted to metres internally (handles ft/m configs).
- Wing root chord derived via `c_root = 2S / (b(1+λ))`, consistent with
  `MACGeometryComp`.
- New `aviary/visualization/README_plot_aircraft.md`: usage, variables table,
  fuselage shape model, parameter table, UAV numerical example.

Usage:

```bash
python -m aviary.visualization.plot_aircraft aviary/models/aircraft/small_uav/small_uav.csv
```

```python
from aviary.visualization.plot_aircraft import visualize
visualize('aviary/models/aircraft/small_uav/small_uav.csv',
          nose_frac=0.20, tail_frac=0.35, base_w_frac=0.20, base_h_frac=0.20, exponent=4.0)
```

Verified on all three UAV configs:
- `small_uav.csv` — 2 m fuselage, 1.8 m span
- `pareto_front_uav.csv`
- `horizontal_small_uav.csv`

Versioning note:
- This is a `MINOR` bump because it adds a new tool that will be used alongside
  every design iteration.

---

## v1.13.0 - 2026-06-10

**CDi wired into FLOPS mission polar via span efficiency correction**

- Added `span_eff_correction` ExecComp in `add_load_factor_subsystems`:

      e_span_eff = e_oswald * AR_eff / AR_geo
      → promotes output as `aircraft:wing:span_efficiency_factor`

  This makes the FLOPS `InducedDrag` compute:
      CDi_flops = CL^2 / (pi * AR_geo * e_span_eff)
                = CL^2 / (pi * AR_eff * e_oswald)  =  Roskam Eq. 4.8 ✓

  The FLOPS polar CDi now uses the Scholz AR_eff endplate correction and the
  Roskam Eq. 4.12 span efficiency, instead of the CSV fixed value (e = 0.90,
  AR = geometric AR). The optimizer sees gradients of range w.r.t. VTP span
  (via AR_eff) and wing geometry (via CL_alpha_w → e_oswald) through the CDi path.

- Added `e_span_eff (wired to FLOPS)` diagnostic to the optimization results print.
- Confirmed that `aircraft:wing:span_efficiency_factor` is NOT a pre-mission
  computed output in Aviary (verified by listing all model outputs after
  `prob.add_pre_mission_systems(); prob.setup()`), so the ExecComp promotion is
  conflict-free.
- CD0 mission-polar wiring is BLOCKED by Aviary pre-mission ownership of
  `aircraft:fuselage:wetted_area` and `aircraft:vertical_tail:wetted_area`.
  Three paths are documented in TOOD.md (scaler, factor ratio, TabularAeroGroup).

Versioning note:
- This is a `MINOR` bump because it wires a new physics path into the mission
  polar (CDi responds to optimizer design variables for the first time).

---

## v1.12.0 - 2026-06-10

**Report-only fuselage drag due to lift**

- Added `FuselageLiftInducedDragComp` implementing Roskam Part VI Eq. 4.33:
  `CDi_fus = 2*alpha^2*S_b_fus/S + eta*c_d_c*alpha^3*S_plf_fus/S`.
- Wired `SuperellipseFuselageGeometry` into the SpaJeti model as a report-only
  geometry source using `Aircraft.Fuselage.LENGTH`, `MAX_WIDTH`, and `MAX_HEIGHT`.
- Wired `fuselage_base_area` and `fuselage_planform_area` into `CDi_fus`.
- Uses the same design-point aircraft alpha path as the wing load-factor/Mach
  checks: `alpha_max_deg`.
- `eta` and `c_d_c` are scalar placeholders until the Roskam charts are digitized
  and verified.
- Base drag remains excluded and is tracked as a separate TODO.

Versioning note:
- This is a `MINOR` bump because it adds a new aerodynamic component and
  report-only model wiring.

---

## v1.11.0 - 2026-06-10

**Parametric superellipse fuselage geometry**

- Added `SuperellipseFuselageGeometry` as a standalone OpenMDAO component.
- The component models the fuselage as smooth superellipse cross-sections with
  configurable nose, mid-body, and aft taper regions.
- Outputs include `fuselage_planform_area` (`S_plf_fus` candidate),
  `fuselage_base_area` (`S_b_fus` candidate), wetted area, equivalent diameter,
  fineness ratio, maximum cross-section area, volume, and geometric centroid.
- Added tests and documentation. The component is not yet wired into parasite
  drag, fuselage drag-due-to-lift, CG, or stability.

Versioning note:
- This is a `MINOR` bump because it adds a new geometry component that will feed
  multiple aerodynamic build-up terms.

---

## v1.10.0 - 2026-06-10

**Live leading-edge suction parameter wiring**

- `MACGeometryComp` now outputs the wing leading-edge sweep as `wing_le_sweep`.
- `RoskamInducedDragComp(compute_leading_edge_suction=True)` computes:
  `r_LE/c = 1.1019*(t/c)^2`, `r_LE = (r_LE/c)*MAC`, `Re_LER`, and the Roskam
  Figure 4.7 leading-edge suction parameter used in Eq. 4.12.
- The SpaJeti model now wires `wing_section_tc`, `wing_c_mac`, `wing_le_sweep`,
  `AR_eff`, taper, Mach, static pressure, and temperature into the local CDi
  component.
- Added diagnostics: `wing_le_radius`, `wing_Re_LER`, and
  `wing_le_suction_parameter`.
- Made the Polhamus AR/Mach guard messages array-safe so low-AR warnings do not
  crash model execution.

Versioning note:
- This is a `MINOR` bump because it adds a live physics path and new model
  diagnostics for the induced-drag calculation.

---

## v1.9.1 - 2026-06-10

**Conservative Figure 4.7 guardrails**

- `leading_edge_suction_parameter_roskam()` now raises `NotImplementedError`
  for Figure 4.7 main-chart cases with `x < 1.3e5`; those placeholder values are
  not allowed until a clean chart digitization is checked.
- The high-`x` inset branch still returns a value, but emits a `RuntimeWarning`
  stating that the inset digitization is first-pass placeholder data from the
  supplied image.
- Clarified that the H-tail/endplate wing should pass `AR_eff`, not geometric AR,
  into Roskam induced drag and the Figure 4.7 helper.

Versioning note:
- This is a `PATCH` bump because it tightens validation/documentation for the
  new v1.9.0 helper without adding a new subsystem.

---

## v1.9.0 - 2026-06-10

**Roskam Figure 4.7 leading-edge suction parameter**

- Renamed the local Roskam induced-drag `r_fourier` concept to
  `leading_edge_suction_parameter` in code, tests, and docs.
- Added first-pass Roskam Figure 4.7 interpolation helper:
  `leading_edge_suction_parameter_roskam(Re_LER, M, Lambda_LE, AR, taper)`.
  The table is a linearized digitization from the supplied chart image and uses
  the inset curve for the high-`x` region.
- Added leading-edge Reynolds helper:
  `leading_edge_reynolds_number_from_mach(...)`, using
  `Re_LER = rho * U * r_LE / mu`.
- Added `leading_edge_radius_ratio_naca_4_digit(t/c)` for the standard
  `r_LE/c = 1.1019*(t/c)^2` NACA 4-digit estimate.
- Extended `AirfoilData` with `leading_edge_radius_ratio` and exposed it as
  `section_leading_edge_radius_ratio` through `AirfoilConstantsComp`.
- Updated induced-drag and airfoil documentation, plus TODO notes for the
  remaining wiring step from live airfoil/chord/sweep data into `CDi`.

Versioning note:
- This is a `MINOR` bump because it adds new reusable aerodynamic physics
  helpers and airfoil data needed by the induced-drag model.

---

## v1.8.2 - 2026-06-09

**Drag TODO refinements**

- Added TODO items to double-check wing exposed wetted area before polar wiring:
  `S_wet_wing = 2 * (S_ref - S_buried_in_fuselage)`.
- Added TODO items to better define fuselage geometry for drag: length,
  equivalent diameter, cross-section/wetted-area model, wing carry-through area,
  and base/exhaust treatment.
- Added TODO item for fuselage/body induced drag.
- Added TODO item to digitize and implement the Roskam Figure 4.7 leading-edge
  suction parameter as a reusable interpolation helper before expanding the
  induced-drag model beyond the current no-twist first term.
- Corrected the TODO note for `CDi` to match v1.8.1: no hidden `1.05 * CL`
  trim multiplier.

Versioning note:
- This is a `PATCH` bump because it updates planning/documentation only.

---

## v1.8.1 - 2026-06-09

**Exposed wetted-area helper, guardrail docs, and no-trim CDi**

- Added `exposed_wetted_area_lifting_surface()` in `aero_utils.py`:

      S_wet_exposed = (S_planform - S_inside_fuselage) * sides

  with hard validation for nonpositive planform area, negative buried area,
  buried area larger than planform area, and nonpositive side count.
- Documented that lifting-surface `wetted_area` must be exposed wetted area:
  subtract the area buried inside the fuselage/body, then multiply by two for
  upper and lower surfaces.
- Removed the `1.05 * CL` trim multiplier from the local `RoskamInducedDragComp`;
  `CDi` now uses the actual input `CL` directly. Trim increments should be added
  later from an explicit trim/tail-load analysis, not as a hidden default.
- Updated `README_induced_drag.md` to document the no-trim convention.
- Expanded drag safety-guard documentation for exposed wetted area, Reynolds
  number, form-factor limits, `R_LS`, `R_wf`, and component-level invalid inputs.

Verification:
- `python -m pytest aviary\subsystems\aerodynamics\flops_based\test\test_parasite_drag.py aviary\subsystems\aerodynamics\flops_based\test\test_induced_drag.py -q`
  passes.
- `python -m py_compile aviary\subsystems\aerodynamics\aero_utils.py aviary\subsystems\aerodynamics\flops_based\parasite_drag.py aviary\subsystems\aerodynamics\flops_based\induced_drag.py aviary\models\aircraft\horizontal_small_uav\run_horizontal_small_uav.py`
  passes.

Versioning note:
- This is a `PATCH` bump because it corrects/refines formulas, validation, and
  documentation in the existing v1.8 drag implementation.

---

## v1.8.0 - 2026-06-09

**Roskam wing induced drag component (Eq. 4.8, no twist)**

- New helpers in `aero_utils.py`:
  - `oswald_efficiency_roskam(cl_alpha_w, ar_eff, r_fourier)` — Roskam Part VI Eq. 4.12
  - `wing_induced_drag_roskam(cl, cl_alpha_w, ar_eff, ...)` — Eq. 4.8 first term
- New `RoskamInducedDragComp` in `induced_drag.py`:

      CDi = C_Lw^2 / (pi * AR_eff * e)   with  C_Lw = 1.05 * CL (Eq. 4.11)
      e   from Roskam Eq. 4.12 using AR_eff and wing_CL_alpha

  Options: `cl_trim_factor = 1.05` (Eq. 4.11), `r_fourier = 0.98` (SpaJeti default).
- AR convention enforced: **AR_eff must be used for ALL aerodynamic calculations**
  (lift, drag, pitch, stability derivatives).  Geometric AR must not appear in
  any new aero formula.  `RoskamInducedDragComp` uses AR_eff throughout.
- Twist terms from Eq. 4.8 excluded (untwisted wing, ε_t = 0); documented in
  `README_induced_drag.md` for future implementation when twist is introduced.
- `RoskamInducedDragComp` wired into `add_load_factor_subsystems` after `long_load`;
  promotes `CDi` and `e_oswald` to model scope.
- `CDi` and `e_oswald` added to the optimization results print block.
- New `README_induced_drag.md`: formula derivation, I/O table, excluded twist terms,
  AR convention note, and UAV numerical example (~26 % CDi reduction from endplate).

Not yet wired into the FLOPS mission drag polar (fuel burn unchanged).

Verification:
- `python -m pytest aviary\subsystems\aerodynamics\flops_based\test\test_parasite_drag.py -q`
  passes with 5 tests (unchanged).
- `final_setup()` runs cleanly; `CDi` and `e_oswald` appear at model scope.

Versioning note:
- This is a `MINOR` bump (new physics component).

---

## v1.7.3 - 2026-06-09

**DATCOM/Roskam lifting-surface CD0 correction factors**

- Updated the local lifting-surface parasite-drag form factor to the requested
  DATCOM/Roskam thickness term:
  `FF = 1 + L'*(t/c) + 100*(t/c)^4`.
- Added `airfoil_thickness_location_parameter()` with the current project
  convention: `L'=1.2` for `(x/c)_m <= 0.30`, otherwise `L'=2.0`.
- Added `lifting_surface_correction_factor()` as a linearized digitization of
  Roskam Part VI Figure 4.2 for `R_LS`.
- Added `wing_fuselage_interference_factor()` as a linearized digitization of
  Roskam Part VI Figure 4.1 for `R_wf`, interpolated in Mach and `log10(Re_fus)`.
- `RoskamParasiteDragBuildUp` now outputs `lifting_surface_correction_factor`
  and multiplies lifting-surface `CD0_component` by `R_LS`; `R_wf` is intended
  to be supplied through the existing `interference_factor` input.
- Updated drag, airfoil, and TODO documentation to match the new formula path.

Verification:
- `python -m pytest aviary\subsystems\aerodynamics\flops_based\test\test_parasite_drag.py -q`
  passes with 5 tests.

Versioning note:
- This is a `PATCH` bump because it corrects/refines formulas and digitized
  empirical data in the existing v1.7 drag implementation.

---

## v1.7.2 - 2026-06-09

**Aero documentation for drag, lift, and airfoil data**

- New `README_drag_build_up.md`: documents the Roskam/DATCOM parasite drag
  build-up, coefficient-only output convention, Reynolds number helpers,
  Raymer roughness cutoff, roughness presets, flat-plate `Cf`, lifting-surface
  form factor, DATCOM/Raymer body form factors, OpenMDAO inputs/outputs, and
  current wiring status.
- New `README_airfoil_data.md`: documents `AirfoilData`, the current airfoil
  catalog, `(x/c)_m` maximum-thickness-location convention, and how airfoil data
  feeds lift, M_crit, and future parasite-drag wiring.
- Updated `README_lift_curve_slope.md` with links to the airfoil and drag docs
  and a note that `AirfoilConstantsComp` is the source of section data for lift,
  M_crit, and parasite drag.

Versioning note:
- This is a `PATCH` bump because it is documentation only.

---

## v1.7.1 - 2026-06-09

**Raymer roughness cutoff and max-thickness-location data**

- `aero_utils.py` now implements Raymer roughness-limited cutoff Reynolds number:
  - subsonic: `Re_co = 38 * (l/k)**1.053`
  - transonic/supersonic: `Re_co = 44 * (l/k)**1.053 * M**1.16`
- Added roughness-height presets from Raymer/Hornung notes:
  aluminum, flat coating, unpolished sheet, polished sheet, and flat CFK.
- Clarified lifting-surface form factor input as `(x/c)_m`, the airfoil maximum
  thickness location. NACA 4-digit catalog entries now carry
  `max_thickness_location=0.30`; NACA 6-series style sections should use ~0.40.
- `AirfoilConstantsComp` now exposes `section_max_thickness_location` for future
  parasite-drag wiring.

Verification:
- `python -m pytest aviary\subsystems\aerodynamics\flops_based\test\test_parasite_drag.py -q`
  passes with 4 tests.

Versioning note:
- This is a `PATCH` bump because it corrects/formalizes formulas and adds airfoil
  data to the existing v1.7.0 drag scaffold.

---

## v1.7.0 - 2026-06-09

**Roskam/DATCOM parasite drag build-up scaffold**

- New file `aviary/subsystems/aerodynamics/aero_utils.py`:
  reusable aero utilities for Sutherland viscosity, ideal-gas density, speed of
  sound, Reynolds number, roughness cutoff, flat-plate `Cf`, DATCOM body/fuselage
  form factor, Raymer fuselage form factor, and lifting-surface form factor.
- New file `aviary/subsystems/aerodynamics/flops_based/parasite_drag.py`:
  `RoskamParasiteDragBuildUp`, an OpenMDAO component for component-wise parasite
  drag coefficient build-up:

      CD0 = sum(Cf * FF * Q * (1 + leakage) * Swet) / Sref

- Output convention locked in: drag modules return dimensionless coefficients
  (`CD0`, later `CDi`, and total `CD`). Force in Newtons belongs in a separate
  `D = q*Sref*CD` force component.
- `component_kind='fuselage'` uses the DATCOM body/fuselage form factor by
  default. `component_kind='raymer_fuselage'` is available only for comparison.
- New pytest-style tests in
  `aviary/subsystems/aerodynamics/flops_based/test/test_parasite_drag.py`.

Verification:
- `python -m pytest aviary\subsystems\aerodynamics\flops_based\test\test_parasite_drag.py -q`
  passes with 3 tests.

Versioning note:
- This is a `MINOR` bump because it adds a new physics component / reusable aero
  utility layer. Use `PATCH` only for fixes/docs/parameter tweaks, and `MAJOR`
  only for architecture-level changes such as a new DV set, EOM, or mission structure.

---

## v1.6.0 - 2026-06-09

**Wing t/c design variable for the M_crit constraint**

- `run_horizontal_small_uav.py` promotes the wing airfoil-section `section_tc` from
  `WingSurface` as model-scope `wing_section_tc`.
- New design variable: `wing_section_tc`, bounded `0.08 <= t/c <= 0.18`, with the
  NACA 4415 value `0.15` as the initial/reference value.
- The existing constraint remains `mach_crit_margin = M_crit - mach_upper_bound >= 0`,
  where `mach_upper_bound = DASH_MACH + M_CRIT_SAFETY_MARGIN = 0.57`.
- Reporting now prints both the fixed FLOPS CSV `aircraft:wing:thickness_to_chord`
  and the optimized section value used by `MachCriticalComp`.
- Added optimizer selection: default to IPOPT when available, otherwise fall back to
  SciPy SLSQP; set `SPAJETI_OPTIMIZER=IPOPT` or `SPAJETI_OPTIMIZER=SLSQP` to override.

Verification note:
- `final_setup` passes with `wing_section_tc` as a design variable.
- A `run_model` check at `wing_section_tc=0.08` gives `M_crit=0.5885` and
  `mach_crit_margin=+0.0185`, confirming the M_crit path responds correctly.
- Local full optimization fell back to SLSQP because `cyipopt` is not installed.
  SLSQP exited at iteration 1 with `Singular matrix C in LSQ subproblem`; IPOPT is
  still recommended for this infeasible-start problem.

---
## v1.5.0 — 2026-06-09

**Component CG estimator with bypass mode**

- New file `cg_estimator.py` (`aviary/subsystems/geometry/flops_based/`):
  - `CGComputeComp`: ExplicitComponent with one set of (mass, x, y, z) inputs per
    registered component; computes x_cg, y_cg, z_cg, total_mass via weighted sum.
    Complex-step partials; fully differentiable w.r.t. variable masses.
  - `CGEstimatorGroup`: builder-pattern Group.  Call `add_component()` before
    `add_subsystem()` to register components.  Two mass types:
    - `mass=float`  -- fixed constant (input default, overridable via prob.set_val)
    - `mass='var'`  -- string naming a model-scope variable; promoted/connected at
                       call site.
    All positions (x, y, z) are always fixed floats (optionally promoted to DVs later).
  - `bypass=True` swaps the estimator for a plain IndepVarComp with four manual
    constants.  Output names are identical; downstream consumers are unaffected.
    Switching is a one-line Python change: `CG_BYPASS = True/False`.

- New constants in `run_horizontal_small_uav.py`:
  `CG_BYPASS`, `CG_X_MANUAL_M`, `CG_Y_MANUAL_M`, `CG_Z_MANUAL_M`, `CG_MASS_MANUAL_KG`.

- SpaJeti component list (in `build_cg_estimator()`):
  Fixed: wing_struct (1.5 kg), fuselage (1.0 kg), empennage (0.25 kg),
         vtp_pair (0.15 kg), landing_gear (0.20 kg), avionics (0.25 kg).
  Variable: engine -> SmallTurbojetVariables.MASS (connected via prob.model.connect),
            payload -> aircraft:crew_and_payload:total_payload_mass (promoted),
            fuel    -> mission:total_fuel (promoted).
  All masses are placeholder estimates; replace with FLOPS outputs when weight
  breakdown is added (see TOOD.md weight estimation).

- New model-scope outputs: `x_cg`, `y_cg`, `z_cg`, `total_mass`.
- Main results print: CG block with approximate static margin (wing AC only).
- `print_cg_detail()`: full component contribution table with mass * x moments.

---

## v1.4.0 — 2026-06-09

**Aircraft coordinate frame + MAC geometry component**

- New file `surface_geometry.py`: `MACGeometryComp` computes wing MAC chord and
  body-frame position using the standard trapezoidal formulas:

      c_r    = 2 * S / (b * (1 + lambda))
      c_mac  = (2/3) * c_r * (1 + lambda + lambda^2) / (1 + lambda)
      y_mac  = (b / 6) * (1 + 2*lambda) / (1 + lambda)

      tan(Lambda_LE) = tan(Lambda_c4) + c_r * (1 - lambda) / b
      x_mac_le = x_apex + y_mac * tan(Lambda_LE)
      z_mac_le = z_apex - y_mac * tan(dihedral)
      x_mac_c4 = x_mac_le + c_mac / 4

- Aircraft reference frame established (origin at nose tip):
      x  --  positive AFT (fuselage station)   [m]
      y  --  positive STARBOARD                [m]
      z  --  positive DOWN                     [m]
  z-down per user specification; differs from FLOPS (z-up).

- `LiftingSurfaceGroup` gains a sixth hook `_setup_mac_geometry()` (no-op by default).
  `WingSurface` overrides it to wire `MACGeometryComp` using Group-scope surface
  geometry and two new apex inputs (`wing_x_apex`, `wing_z_apex`) from `load_cond`.

- Two new `load_cond` outputs: `wing_x_apex = 0.80 m`, `wing_z_apex = 0.00 m`.
  These are geometric inputs (fixed); promote to design variables when the CG /
  static-margin loop is added.

- Two new `wing_surface` promotes_inputs: `surface_area` (aircraft:wing:area),
  `dihedral_deg` (aircraft:wing:dihedral).

- New model-scope outputs: `wing_root_chord`, `wing_c_mac`, `wing_y_mac`,
  `wing_x_mac_le`, `wing_z_mac_le`, `wing_x_mac_c4`.

- `print_aero_detail` Section 1b: full MAC breakdown with formula and numerical
  check. Main results print: new Wing MAC Geometry section.

- Baseline (SpaJeti: b=1.8m, S=0.45m^2, lambda=0.6, sweep=0, dih=3deg, x_apex=0.80m):
  c_r=0.313m, c_mac=0.255m, y_mac=0.413m, x_mac_c4=0.893m, z_mac_le=-0.022m.

---

Version scheme: `MAJOR.MINOR.PATCH`
- `PATCH` — bug fix, doc update, parameter value change
- `MINOR` — new physics component, new constraint, new output
- `MAJOR` — architectural redesign (new DV set, new EOM, new mission structure)

---

## v1.3.1 — 2026-06-09

**Wing airfoil changed to NACA 4415; M_crit constraint (not M_DD)**

- `airfoil_data.py`: added `NACA_4415` (t/c=0.15, cl_alpha=6.00/rad, cl_max=1.60).
- Wing airfoil changed from `NACA_4412` (t/c=0.12) to `NACA_4415` (t/c=0.15).
  CSV `aircraft:wing:thickness_to_chord` updated from 0.12 to 0.15.
- Constraint switched from `mach_dd_margin = M_DD - check >= 0` to
  `mach_crit_margin = M_crit - check >= 0`.  M_DD retained as informational output.
- Constant renamed `M_DD_SAFETY_MARGIN` → `M_CRIT_SAFETY_MARGIN` = 0.05.
- Baseline (NACA 4415, t/c=0.15, sweep=0, alpha_max=12 deg):
  CL~1.02, M_DD~0.634, M_crit~0.526, check=0.57 → **margin=-0.044 (active constraint)**.
  Constraint becomes feasible when t/c is added as a DV or alpha_max is reduced
  via `StallAlphaComp` (TOOD.md: alpha_max/CL_max from stall).

---

## v1.3.0 — 2026-06-09

**Weisshaar M_DD / M_crit component + optimizer constraint**

- New file `mach_critical.py`: `MachCriticalComp` implements Weisshaar Eq. 36:

      M_DD = K_A / cos(phi_25) - (t/c) / cos^2(phi_25) - CL / (10 * cos^3(phi_25))
      M_crit = M_DD - (0.1/80)^(1/3)   K_A = 0.887 (SEE = 3.95 %)

  CL is derived from the **same alpha_max** shared with the Nz constraint:
  `CL = wing_CL_alpha * alpha_max_rad` (currently ALPHA_MAX_DEG = 12 deg).
  This couples M_DD and Nz to the same alpha, ready for future `StallAlphaComp`.

- `LiftingSurfaceGroup` gains a fifth hook `_setup_mach_critical()` (no-op by default).
  `WingSurface` overrides it to wire `MachCriticalComp` using Group-scope `section_tc`,
  `surface_sweep_c4`, `surface_CL_alpha`, and model-scope `alpha_max_deg`.

- Constraint is on **M_DD** (not M_crit) against `DASH_MACH + safety buffer`:
  `mach_dd_margin = M_DD - mach_upper_bound >= 0`
  `mach_upper_bound = DASH_MACH + M_DD_SAFETY_MARGIN = 0.52 + 0.05 = 0.57`
  `M_crit` is computed and printed as an informational output only.

- New `load_cond` outputs: `alpha_max_deg = 12.0`, `mach_upper_bound = 0.57`.
  `alpha_max_deg` also drives `LongitudinalLoadFactor` (replacing `set_val`).

- New model-scope outputs: `M_DD`, `M_crit`, `mach_dd_margin`.
- `print_aero_detail` Section 2b: full M_DD breakdown with alpha-based CL and constraint status.
- Baseline (NACA 4412, t/c=0.12, sweep=0, alpha=12 deg):
  CL=1.02, M_DD=0.665, M_crit=0.557, margin=0.095 (constraint not active).

---

## v1.2.0 — 2026-06-09

**Modular lifting surface architecture**

- New `airfoil_data.py`: `AirfoilData` frozen dataclass + NACA 0009/0012/2412/4412 catalog.
- New `surface_config.py`: `SurfaceConfig` frozen dataclass (airfoil + role flags).
- New `lifting_surface.py`: `LiftingSurfaceGroup` base class (4-hook setup pattern),
  `WingSurface` (Scholz AR correction + K_wf Polhamus), `VTPSurface` (CyBeta + CyDelta).
- `run_horizontal_small_uav.py`: replaced 5 flat subsystems with `WingSurface` and
  `VTPSurface` Groups; removed 6 redundant `set_val` calls.
- Airfoil selection: Wing = NACA 4412, VTP = NACA 0012.
- `print_aero_detail` updated: shows actual airfoil name and section cl_alpha.

---

## v1.1.0 — 2026-06-09

**Scholz (INCAS 2018) winglet AR correction**

- Replaced `EndplateARCorrection` with `ScholzWingletARCorrection`.
- k_WL changed to 2.45 (experimental average; 2.0 is theoretical optimum).
- Split-winglet penalty = 0.90 for H-tail symmetric VTP pair.
- AR_eff formula: `b_eff = b_h + 2h/k_WL`, `AR_eff = AR_geo * (b_eff/b_h)^2 * split_penalty`.
- Full derivation documented in `README_lift_curve_slope.md`.

---

## v1.0.0 — 2026-06-08

**Initial model**

- SpaJeti H-wing UAV (15 kg MTOW, ~0.45 m^2 wing, small turbojet).
- Aviary Level-2 fallout problem with FLOPS height-energy EOM.
- Design variables: wing span, VTP span, SLS thrust, phase Mach schedule.
- Constraints: Ny >= 7, Nz >= 7 (550 km/h / 5 km ISA), T/W >= 1.5, fuel budget.
- Objective: maximize range.
- External subsystems: `SmallTurbojetModel`, `HTailGeometry`,
  `LateralLoadFactor`, `LongitudinalLoadFactor`.
