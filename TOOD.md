# TODO

## CyBetaFuselage <span style="color: #ef4444; font-weight: bold">[TODO]</span> — proper implementation required

`CyBetaFuselage` exists in `cy_beta_vtp.py` but is **not wired into `LateralLoadFactor`**.

The following steps must be completed before it can be used:

1. **Variable cross-section at x_0** — `S_0` is currently approximated as the maximum
   circular cross-section `π*(d_f/2)²`. The correct value is the fuselage cross-sectional
   area at station `x_0`, where potential flow ends:

       x_0 = (0.378 + 0.527 * x1/l_f) * l_f

   `x1` is the fuselage station where `dS/dx` first reaches its most negative value
   (start of aft-fuselage taper). Requires either a fuselage area distribution input
   or a parameterised taper model.

2. **Ki multi-step graph lookup** — the wing-fuselage interference factor `Ki` is
   currently a simple linear fit (`Ki = 1.0 + 0.5 * z_w`). The full Roskam Part VI
   Figure 10.8 is a family of curves over wing vertical position and fuselage fineness
   ratio. Replace with a proper 2-D bilinear table (similar to `_KV_TABLE_2D`) once
   the chart is digitised.

3. **Wire into `LateralLoadFactor`** — add `CY_beta_fuselage` input and include it in
   `CY_beta_total = CY_beta_vtp + CY_beta_wing + CY_beta_fuselage`.

4. **High-wing case** — the current implementation only handles `z_w ∈ [0, 1]`
   (mid-to-low wing). High-wing aircraft (`z_w < 0`) are not modelled.

## EndplateARCorrection <span style="color: #ef4444; font-weight: bold">[TODO]</span> — endplate effect on VTP CL_alpha not yet investigated

`EndplateARCorrection` corrects the **wing** (horizontal panel) AR for the endplate
effect of the VTP panels.  The VTP's own CL_alpha (used in `CyDeltaRudder`) currently
uses `LiftCurveSlopePolhamus` with the raw VTP geometry — no endplate correction is
applied to the VTP itself.

Whether the wing acts as a meaningful endplate for the VTP (and how large the effect
is relative to the VTP's low AR) has not been assessed.  Investigate before adding
a correction to `CyDeltaRudder.CL_alpha_v`.

---

## Airfoil data per surface <span style="color: #22c55e; font-weight: bold">[DONE — v1.2.0, 2026-06-09]</span>

`airfoil_data.py` -- `AirfoilData` frozen dataclass + NACA 0009/0012/2412/4412 catalog.
`surface_config.py` -- `SurfaceConfig` frozen dataclass (airfoil + role flags).
`lifting_surface.py` -- `LiftingSurfaceGroup` base + `WingSurface` + `VTPSurface`.

Current selection:
  Wing: NACA 4412  (cl_alpha = 6.10 /rad, Re~2e6, incompressible)
  VTP:  NACA 0012  (cl_alpha = 5.73 /rad, Re~2e6, incompressible)

To add HTP or a canard: create `HTPSurface(LiftingSurfaceGroup)` following the
VTPSurface pattern, add a `SurfaceConfig` constant, and wire it in
`add_load_factor_subsystems`.

Remaining gap:
  `section_tc` (from `AirfoilConstantsComp`) and
  `Aircraft.VerticalTail.THICKNESS_TO_CHORD` (CSV -> `CyDeltaRudder`) are separate
  variables.  Align them (or add a constraint) when parasite drag is implemented.

---

## M_crit / MDD checker <span style="color: #f59e0b; font-weight: bold">[PARTIAL — M_DD done, CL wiring pending]</span> — enforce cruise Mach stays below critical Mach

Thicker airfoils increase CL_alpha and structural efficiency but reduce M_crit.
Higher cruise CL (heavy aircraft, low speed) also lowers M_crit.
As the optimizer pushes t/c up and CL up, cruise Mach may exceed M_crit, causing
wave drag not captured by the current subsonic FLOPS polar.

### <span style="color: #22c55e; font-weight: bold">[DONE — v1.3.0]</span> Weisshaar M_DD formula

`mach_critical.py` — `MachCriticalComp(om.ExplicitComponent)` implements:

    M_DD = K_A / cos(phi_25) - (t/c) / cos^2(phi_25) - CL / (10 * cos^3(phi_25))
    M_crit = M_DD - (0.1/80)^(1/3)   ~   M_DD - 0.1077
    CL = wing_CL_alpha * alpha_max_rad   (same alpha as Nz constraint)

K_A = 0.887 (Weisshaar optimised, SEE = 3.95 %, best directly-solvable form).

Wired into `WingSurface._setup_mach_critical()` via `LiftingSurfaceGroup` hook.
CL is coupled to `alpha_max_deg` (shared with `LongitudinalLoadFactor`) and
`surface_CL_alpha` (from Polhamus at Group scope) -- no separate `cruise_cl` input.

Constraint: `mach_dd_margin = M_DD - mach_upper_bound >= 0`
where `mach_upper_bound = DASH_MACH + M_DD_SAFETY_MARGIN = 0.52 + 0.05 = 0.57`.
M_crit is computed as informational output only.

Note: **CL appears explicitly** -- a higher alpha (or higher CL_alpha from thicker wing)
directly reduces M_DD, coupling the Nz and M_DD constraints through alpha_max.

### Remaining steps

1. **CL connection** — wire `cruise_cl_ref` to the mission phase CL output instead of
   the fixed `0.1` constant:
   `CL_cruise = gross_mass * g / (q_cruise * wing_area)`.
   This closes the aero-structures-performance loop.

2. **XFOIL automation for section M_crit** — implement a script that sweeps Mach
   and finds the section M_crit from first principles (more accurate than Weisshaar):
   - For the selected airfoil, sweep Mach from 0.3 to 0.9.
   - At each Mach, run XFOIL at the design CL / alpha.
   - Compute the critical pressure coefficient (sonic condition):
         CP_crit = (2 / (gamma * M^2)) * [(2/(gamma+1) + (gamma-1)/(gamma+1) * M^2)^(gamma/(gamma-1)) - 1]
   - Record CP_min (minimum surface pressure) from XFOIL.
   - Section M_crit = Mach where CP_min first equals CP_crit.
   This gives the actual airfoil M_crit, bypassing the empirical K_A assumption.

3. **Wing t/c as design variable** -- DONE in v1.6.0.
   `wing_section_tc` is promoted from `WingSurface.section_tc` and bounded
   `0.08 <= t/c <= 0.18`. This is the value used by `MachCriticalComp`.
   The CSV `aircraft:wing:thickness_to_chord` remains fixed at 0.15 for FLOPS
   mass/geometry until the weight model is made live.

   Verification: at `wing_section_tc = 0.08`, `M_crit = 0.5885` and
   `mach_crit_margin = +0.0185` against the 0.57 check.

4. **Document** — add M_crit constraint to the Constraints table in `README.md`
   and update `README_lift_curve_slope.md` with the formula.

---

## alpha_max / CL_max from stall <span style="color: #ef4444; font-weight: bold">[TODO]</span> — replace hardcoded alpha in Nz and M_crit

Currently `LongitudinalLoadFactor` uses a hardcoded `alpha_max_deg = 12.0` and the
M_crit constraint uses a fixed `cruise_cl_ref = 0.1`.  Both should be derived from
the actual stall characteristics of the selected airfoil and 3-D finite-wing effects.

### Goals

1. **Compute alpha_stall** — the angle of attack at which the wing first stalls.
   This is the physical upper limit for the Nz calculation; using a higher alpha
   overestimates Nz and gives a non-conservative constraint.

2. **Replace hardcoded alpha=12 deg in Nz** — the constraint
   `Nz = CL_alpha * alpha_max * q * S / (m*g) >= NZ_MIN` becomes physically meaningful
   when `alpha_max` is the actual stall angle rather than an arbitrary constant.

3. **Feed CL_max into M_crit** — the highest wing CL the aircraft will actually
   experience (at minimum cruise speed or in pull-up) should replace the fixed
   `cruise_cl_ref = 0.1` in `MachCriticalComp`.  A higher operating CL reduces M_crit.

### Steps

1. **3-D CL_max from airfoil data** — estimate wing CL_max using the Diederich / DATCOM
   taper-sweep correction:

       CL_max_3D = k_CL * CL_max_2D

   where `k_CL ~ 0.9` for moderate taper (DATCOM chart, Section 4.1.1.4) and
   `CL_max_2D` comes from `AirfoilData.cl_max` (already in the catalog).

2. **alpha_stall from 3-D lift curve** — invert the Polhamus lift curve:

       alpha_stall = CL_max_3D / CL_alpha_3D

   where `CL_alpha_3D` is `wing_CL_alpha` from `WingSurface`.

3. **OpenMDAO component** — implement `StallAlphaComp(om.ExplicitComponent)`:
   - Inputs: `section_cl_max` (Group scope from AirfoilConstantsComp),
             `surface_CL_alpha` (from Polhamus), `k_cl_max` (default 0.9)
   - Outputs: `CL_max_3D`, `alpha_stall_deg`, `alpha_stall_rad`
   - Add as step 4 in `WingSurface.setup()` via a `_setup_stall()` hook, similar
     to `_setup_mach_critical()`.

4. **Wire into `LongitudinalLoadFactor`** — replace the fixed `alpha_max_deg` input
   with the `alpha_stall_deg` output from `StallAlphaComp`.
   Add a guard: `alpha_max = min(alpha_stall_deg, ALPHA_ABS_MAX)` to prevent
   unrealistic values if the solver wanders.

5. **Wire CL_max into M_crit** — connect `CL_max_3D` (or `CL_cruise` from the
   mission phase) to `MachCriticalComp.cruise_cl` instead of the fixed
   `cruise_cl_ref`.  Use the cruise CL (level flight) rather than CL_max for the
   M_crit check, since M_crit is most relevant at high speed (low CL), not at stall.

6. **Sensitivity** — increasing t/c:
   - raises CL_alpha (higher Nz if CL_alpha*alpha_stall allows)
   - lowers M_crit (wave drag constraint gets tighter)
   The M_crit and Nz constraints together will bound the feasible t/c range once
   both are wired to physics-based values.

---

## Parasite drag <span style="color: #ef4444; font-weight: bold">[TODO]</span> — add fuselage and wing/VTP wetted-area drag

The current FLOPS drag polar zeroes out VTP wetted area (`wetted_area = 0`) and uses
a fixed fuselage wetted area in the CSV.  No component-level parasite drag breakdowns
are tracked.

Steps:

1. **Wing/VTP skin-friction drag** — implement a flat-plate `Cf` estimate
   (Schlichting turbulent BL) per component:

       Cf = 0.455 / (log10(Re_mac))^2.58 / (1 + 0.144*M^2)^0.65

   Multiply by form factor `FF` (Shevell or DATCOM) and wetted area to get `CD0_component`.

2. **Fuselage drag** — use `wetted_area_fuselage * Cf_fus * FF_fus`; the fuselage
   `FF` accounts for fineness ratio (l_f / d_f).

3. **Sum and inject** — add component drag contributions to the FLOPS `zero_lift_drag_coeff`
   via an `ExecComp` or dedicated `ParasiteDragBuildUp` component.

4. Enable VTP wetted area in `horizontal_small_uav.csv` (currently `wetted_area = 0`).

5. Update `README_lateral_stability.md` and `README.md` when wired in.

---

## Cl_beta <span style="color: #ef4444; font-weight: bold">[TODO]</span> — lateral-directional stability derivative (dihedral effect)

`Cl_beta` (rolling moment due to sideslip) is the primary dihedral-effect stability
derivative.  It is not currently computed; only `CY_beta` (side force) and `Cn_beta`
(implicit from the Ny constraint) are included.

Steps:

1. **Wing dihedral contribution** — implement the DATCOM / Roskam Part VI formula:

       Cl_beta_wing = -(Gamma / 57.3) * (CL / 4) * ... (taper/AR correction)

   where `Gamma` is the geometric dihedral angle (currently 3 deg in the CSV).

2. **VTP contribution** — the H-tail VTPs mounted at the wingtips contribute a large
   destabilising `Cl_beta` in sideslip (dihedral-effect reversal for high-mounted
   endplates).  Quantify this term; it may dominate.

3. **Constraint** — for static lateral stability require `Cl_beta < 0` (stable sign
   convention: negative = rolling away from sideslip).

4. **Connect geometry** — wire `aircraft:wing:dihedral`, `aircraft:wing:sweep`,
   `aircraft:wing:taper_ratio`, and `AR_eff` into the new component.

5. Add as a new subsystem in `add_load_factor_subsystems` and document in
   `README_lateral_stability.md`.

---

## Coordinate frame + MAC geometry <span style="color: #22c55e; font-weight: bold">[DONE — v1.4.0, 2026-06-09]</span>

Body-fixed reference frame established.  Wing MAC geometry computed as a live
OpenMDAO variable (updates automatically when span / area change as DVs).

**Frame (origin at nose tip):**
    x  --  positive AFT (fuselage station)   [m]
    y  --  positive STARBOARD                [m]
    z  --  positive DOWN                     [m]

**New file:** `surface_geometry.py` -- `MACGeometryComp` with formulas:
    c_r    = 2 * S / (b * (1 + lambda))
    c_mac  = (2/3) * c_r * (1 + lambda + lambda^2) / (1 + lambda)
    y_mac  = (b / 6) * (1 + 2*lambda) / (1 + lambda)
    x_mac_le = x_apex + y_mac * tan(Lambda_LE)
    z_mac_le = z_apex - y_mac * tan(dihedral)   [z-down: positive dihedral -> tip UP -> z_tip < z_root]
    x_mac_c4 = x_mac_le + c_mac / 4

**Available at model scope:** `wing_root_chord`, `wing_c_mac`, `wing_y_mac`,
`wing_x_mac_le`, `wing_z_mac_le`, `wing_x_mac_c4`.

**Geometric inputs (load_cond IndepVarComp):** `wing_x_apex = 0.80 m`, `wing_z_apex = 0.00 m`.

### Remaining steps

1. **HTP MAC** -- add `HTPSurface._setup_mac_geometry()` following the WingSurface
   pattern once an HTP lifting-surface Group is created.

2. **VTP MAC** -- VTP spans in the z direction (upward from wing tip); a `VTPMACGeometryComp`
   or a `span_axis` flag in `MACGeometryComp` is needed.  VTP apex is referenced to
   the wing tip LE: `x_vtp_apex_abs = wing_x_mac_le_tip + x_vtp_offset` where
   `wing_x_mac_le_tip = x_apex_wing + (b/2)*tan(Lambda_LE_wing)`.

3. **Apex as DV** -- promote `wing_x_apex` / `wing_z_apex` to design variables
   when the CG / static-margin loop is added.  Gradient path already exists since
   `MACGeometryComp` is in the differentiable OpenMDAO graph.

4. **CG moment arm** -- once CG x-station is known:
   - `x_LEMAC = wing_x_mac_le`
   - `SM_frac  = (x_NP - x_cg) / wing_c_mac`
   - Wire into `Cm_alpha` and tail volume coefficients.

5. **Prerequisite for** CG estimation, Cm_alpha, Cn_beta, and tail sizing.

---

## CG estimation <span style="color: #22c55e; font-weight: bold">[DONE — v1.5.0, 2026-06-09]</span>

`CGEstimatorGroup` in `aviary/subsystems/geometry/flops_based/cg_estimator.py`.
Builder API: `add_component(name, mass, x, y, z)` where mass is float (fixed) or
str (model-scope variable).  Outputs: `x_cg`, `y_cg`, `z_cg`, `total_mass`.
Bypass mode (`CG_BYPASS=True`) replaces estimator with an IndepVarComp — one-line
switch, same output names.

SpaJeti component list in `build_cg_estimator()`:
  Fixed structural estimates (PLACEHOLDER -- replace when FLOPS mass breakdown added).
  Variable: engine mass (from SmallTurbojet), payload and fuel (from Aviary).

### Remaining steps

1. **Replace placeholder masses** -- wire FLOPS component mass outputs
   (aircraft:wing:mass, aircraft:fuselage:mass, etc.) once the weight breakdown
   subsystem is added (see TOOD.md weight estimation).

2. **Fuel balance unification** -- merge fuel mass / fuel CG into the CG estimator
   so that CG travel during fuel burn is tracked in the optimisation loop.
   Currently fuel is added at a fixed CG station; the correct approach is to
   partition fuel into front/rear tank contributions with separate x-stations.
   (See TOOD.md fuel balance item.)

3. **CG travel envelope** -- check CG at zero-fuel (ZFW) and max-fuel (MTOW)
   conditions; both must satisfy the static margin constraint.  Requires
   expressing fuel_mass in the CG sum as a scalar that varies from 0 to MTOW.

4. **Static margin** -- implement `StaticMarginComp` (separate module):
       SM = (x_NP - x_cg) / wing_c_mac
   where x_NP is the full-aircraft neutral point.  Add as an optimizer constraint
   (SM_min <= SM <= SM_max).  Prerequisite: tail geometry moment arms.

5. **Prerequisite for** Cm_alpha, tail sizing, Cn_beta.

---

## Fuel balance <span style="color: #ef4444; font-weight: bold">[TODO]</span> — unify fuel CG into the weight/CG estimator

Fuel burns during the mission, shifting the CG aft.  Currently the fuel mass is
added to the CG estimator at a single fixed x-station (0.90 m), which does not
capture CG travel during flight.

Steps:

1. **Partition fuel into tanks** -- define front tank (x ~ 0.85 m) and rear tank
   (x ~ 0.95 m) with a fuel distribution parameter.  Register both as separate
   CG components.  At zero fuel both masses are 0; at MTOW they sum to FUEL_CAPACITY_KG.

2. **Connect to mission burn** -- wire av.Mission.TOTAL_FUEL to the combined
   fuel component so the optimizer sees the CG shift as fuel is consumed.

3. **CG travel constraint** -- add constraint: CG at ZFW (min fuel) must also
   satisfy the static margin bounds.  May require a second CG evaluator or a
   parametric fuel-fraction variable.

4. **Cross-link with FuelBudgetEstimate** -- the existing FuelBudgetEstimate
   component already tracks total fuel; reuse those outputs rather than
   duplicating the fuel variable.

---

## Parasite drag audit <span style="color: #ef4444; font-weight: bold">[TODO]</span> -- assess Aviary built-in drag before CD_i work

Before implementing the AR_eff-consistent induced-drag correction, audit the current
Aviary drag path and decide whether to extend it or bypass it with a small custom
build-up for the SpaJeti UAV.

Initial repo read:
- `ComputedAeroGroup` already assembles FLOPS `MuxComponent`, `SkinFriction`,
  `SkinFrictionDrag`, `InducedDrag`, `CompressibilityDrag`, and `TotalDrag`.
- `MuxComponent` can pass wing, HTP, VTP, fuselage, and nacelle wetted areas,
  fineness ratios, characteristic lengths, and laminar fractions into the skin
  friction build-up.
- `SkinFrictionDrag` already forms `CD0` from component `Cf`, wetted area, FLOPS
  form factors, and a global excrescence factor.
- `InducedDrag` uses geometric AR and `aircraft:wing:span_efficiency_factor`, so
  it does not currently know about `AR_eff` from the Scholz endplate correction.

Started clean reimplementation:
- `aviary/subsystems/aerodynamics/aero_utils.py` contains universal helpers for
  Sutherland viscosity, ideal-gas density, speed of sound, Reynolds number,
  roughness-limited Reynolds number, flat-plate `Cf`, DATCOM body/fuselage form
  factor, Raymer fuselage form factor, DATCOM/Roskam lifting-surface `R_LS`,
  wing/fuselage `R_wf`, and the `L'` airfoil thickness-location parameter. Use
  these from drag, aeroelasticity, and stability code rather than duplicating
  Reynolds/Cf/interference formulas.
- `aviary/subsystems/aerodynamics/flops_based/parasite_drag.py` contains the first
  `RoskamParasiteDragBuildUp` OpenMDAO component:
      `CD0 = sum(Cf * FF * R_LS * Q * (1 + leakage) * Swet) / Sref`
  It is array-based by component and currently supports `lifting_surface`, `body`,
  `fuselage` (DATCOM, default for fuselage), and `raymer_fuselage` (implemented
  for comparison but not default) form-factor kinds. Not wired into the mission drag polar yet.
  For wing/fuselage interference, compute `R_wf` from `wing_fuselage_interference_factor`
  and pass it through the component `interference_factor` input.
  Output convention: drag modules return dimensionless coefficients only
  (`CD0`, later `CDi`, and total `CD`). Drag force in Newtons must be computed in
  a separate force component as `D = q * Sref * CD`.

Questions to answer next:
1. **Can Aviary's built-in FLOPS parasite drag be used directly?** Check whether the
   small-UAV CSV supplies realistic `WETTED_AREA`, `FINENESS`,
   `CHARACTERISTIC_LENGTH`, and laminar-flow inputs for wing, HTP, VTP, fuselage,
   and nacelle. VTP wetted area is currently zero, so H-tail parasite drag is being
   suppressed.

2. **What is missing for this configuration?** Identify gaps for twin VTP panels,
   H-tail endplate geometry, exposed elevon/control-surface drag increments,
   low-Reynolds-number UAV assumptions, and component-level reporting.

3. **Implementation choice** -- if the FLOPS chain is adequate, wire/repair its
   inputs and add reporting for `skin_friction_drag_coeff` / `CD0`. If it is too
   opaque or too transport-aircraft-specific, implement a local
   `ParasiteDragBuildUp` component in the Aviary drag module that computes wing,
   HTP, VTP, fuselage, nacelle, and elevon/control-surface increments explicitly.

4. **Documentation** -- record what Aviary can already compute, what it cannot,
   and why any custom component is needed before changing the optimizer drag polar.

---

## CD_i <span style="color: #ef4444; font-weight: bold">[TODO]</span> — induced drag using effective AR from Scholz correction

The FLOPS drag polar uses `aircraft:wing:span_efficiency_factor` (fixed at 0.90
in the CSV) and the geometric AR.  The Scholz AR correction (AR_eff) improves the
actual induced drag, but FLOPS does not see it — the polar is inconsistent with
the load-factor subsystem.

Steps:

1. **Explicit CD_i component** — implement:

       e_eff    = span_efficiency * (AR_eff / AR_geo)   (approximate scaling)
       CD_i     = CL^2 / (pi * AR_eff * e_eff)

   Connect `AR_eff` from `ScholzWingletARCorrection` and `CL` from the mission phase.

2. **Replace or supplement FLOPS polar** — either override FLOPS `span_efficiency_factor`
   with the AR_eff-consistent value, or subtract the FLOPS CD_i term and add the
   corrected one.

3. **Add to detail print** — show CD_i alongside CL_alpha in `print_aero_detail`.

4. **Sensitivity study** — compare range with geometric AR vs AR_eff in the polar
   to quantify the benefit of the H-tail endplate design.

---

## Cn_beta <span style="color: #ef4444; font-weight: bold">[TODO]</span> — directional stability derivative (weathercock stability)

`Cn_beta` (yawing moment due to sideslip) is not computed.  The Ny constraint
currently only checks side-force magnitude, not whether the aircraft is
directionally stable.

Steps:

1. **VTP contribution (stabilising)** — dominant term (Roskam Part VI):

       Cn_beta_vtp = CL_alpha_v * (1 - d_sigma/d_beta) * (S_v / S) * (l_v / b)

   where `l_v` is the VTP moment arm (x_v_ac - x_cg), `d_sigma/d_beta` is the
   sidewash gradient (~0.1 for conventional layouts).

2. **Fuselage contribution (destabilising)** — implement:

       Cn_beta_fus = -2 * K_n * K_Rl * Vol_fus / (S * b)

   where `K_n` and `K_Rl` are DATCOM interference factors.

3. **Total and stability constraint** — `Cn_beta = Cn_beta_vtp + Cn_beta_fus > 0`
   for weathercock stability.

4. **Prerequisite** — requires CG x-station (moment arm `l_v`) from the CG
   estimation task above.

5. Add to `README_lateral_stability.md`.

---

## Tail geometry <span style="color: #ef4444; font-weight: bold">[TODO]</span> — complete empennage geometry for HTP and VTP

`HTailGeometry` computes VTP area and AR from VTP span.  A complete tail geometry
component covering the full empennage is missing.

Steps:

1. **HTP geometry** — compute HTP MAC, moment arm (`l_h = x_HTP_ac - x_cg`),
   and volume coefficient `V_h = S_h * l_h / (S * MAC_wing)`.

2. **VTP geometry** — extend to include VTP moment arm `l_v = x_VTP_ac - x_cg`
   and volume coefficient `V_v = S_v * l_v / (S * b)`.

3. **H-tail specifics** — for the twin-VTP layout, `S_v` is the total area of
   both VTPs; `l_v` is measured to their combined aerodynamic centre.

4. **Wire CG** — both moment arms depend on CG x-position; connect to the CG
   estimation component.

---

## Tail sizing <span style="color: #ef4444; font-weight: bold">[TODO]</span> — volume coefficient method for HTP and VTP

Neither the HTP nor the VTP is sized by a volume coefficient requirement.
Only a lower bound on VTP span is imposed (0.15 m).

Steps:

1. **Target volume coefficients** — typical values for UAVs:
   - `V_h_target ~ 0.35` (longitudinal, HTP)
   - `V_v_target ~ 0.04` (directional, VTP)

2. **Sizing constraints** — add:
   `V_h >= V_h_target` and `V_v >= V_v_target`
   as optimizer constraints, replacing or complementing the raw span bounds.

3. **Trade study** — show how volume coefficient requirements change optimal VTP
   span and compare against the current Ny-only constraint.

4. **Prerequisite** — requires tail geometry (moment arms) and CG estimation.

---

## Cm_alpha <span style="color: #ef4444; font-weight: bold">[TODO]</span> — longitudinal pitch stability derivative

`Cm_alpha` (pitching moment slope w.r.t. angle of attack) is not computed.
Static margin is hardcoded at 0.10 in the CSV with no gradient to the optimizer.

Steps:

1. **Neutral point estimate** — use the DATCOM / Roskam approach:

       x_NP / MAC = (CL_alpha_wing * x_ac_wing + CL_alpha_ht * eta_h * (S_h/S) * x_ac_ht)
                    / (CL_alpha_wing + CL_alpha_ht * eta_h * (S_h/S))

   where `eta_h` is the HTP dynamic-pressure ratio (~0.9 for aft-mounted HTP),
   and `x_ac_wing`, `x_ac_ht` are the wing and HTP aerodynamic centres.

2. **Static margin as constraint** — `SM = (x_NP - x_cg) / MAC`;
   replace the fixed CSV value with a live optimizer constraint.

3. **Cm_alpha** — `Cm_alpha = -CL_alpha_total * SM`; must be negative for
   pitch stability.

4. **Connect CL_alpha** — wire `CL_alpha` from `LiftCurveSlopePolhamus` (wing)
   and a second instance for the HTP.

5. **Prerequisite** — requires CG estimation and tail geometry.

---

## Aeroelasticity <span style="color: #ef4444; font-weight: bold">[TODO]</span> — wing divergence and flutter speed check

As wing span and AR grow (optimizer pushes toward long, slender wing), divergence
and flutter become the binding structural constraints before stress does.
Neither is currently computed.

Important prerequisite: any meaningful wing aeroelastic or elevon-surface load
analysis needs a spanwise lift/load distribution, not only scalar CL/CL_alpha.
Source this distribution from somewhere explicit before sizing spars, skins,
hinges, or elevons: e.g. a vortex-lattice/lifting-line model, OpenAeroStruct,
AVL/XFLR5-exported section loads, or a conservative analytic Schrenk/elliptic
loading approximation. The distribution must include wing panels and elevon
regions so hinge moments, local torsion, and control-surface aeroelastic reversal
can be checked.

Steps:

1. **Divergence speed** — for a straight wing, torsional divergence speed is:

       V_D = sqrt(2 * GJ * e / (rho * c * a * e_distance))

   where `GJ` is torsional stiffness, `c` is chord, `a = dCL/dalpha`, and
   `e_distance` is the distance between the shear centre and aerodynamic centre.
   Implement a simplified estimate using structural scalars from the mass model.

2. **Flutter** — simplified Theodorsen-strip approach or a mass-ratio / frequency-
   ratio criterion (Collar triangle).  Even a conservative margin check
   (`V_flutter > 1.2 * V_dive`) would bound the design space.

3. **Constraint** — add `V_divergence > V_dive` and `V_flutter > V_dive` as
   optimizer constraints, where `V_dive = 1.25 * V_cruise`.

4. **Aeroelastic AR penalty** — on a very flexible wing, the effective span is
   reduced by wash-out; quantify the correction to AR_eff from `ScholzWingletARCorrection`.

5. **Connection** — wire `aircraft:wing:span`, `aircraft:wing:taper_ratio`,
   `aircraft:wing:thickness_to_chord` (proxy for `GJ`), and cruise dynamic pressure.

---

## Weight estimation <span style="color: #ef4444; font-weight: bold">[TODO]</span> — component-level mass breakdown

Empty mass is fixed at 7 kg with no breakdown.  The optimizer cannot trade
structural mass against aerodynamic performance because there are no gradients
from geometry to mass.

Steps:

1. **Wing mass** — implement a simplified wing mass model:

       m_wing ~ K_w * S^1.5 * (1 + 2*lambda) / ((1 + lambda) * AR^0.5) * (n_ult * m_gross)^0.5 / (t/c)^0.5

   or use the FLOPS wing mass formula with the current `bending_material_mass_scaler`.
   Ensure it responds to AR, span, and t/c design variables.

2. **Fuselage mass** — parameterise by fuselage length and diameter rather than
   fixing at a fraction of `empty_mass`.

3. **HTP and VTP mass** — currently `mass_scaler = 0` for VTP (H-tail endplates
   are unweighted).  Assign a structural mass per unit area based on UAV composite
   construction (~1.0–1.5 kg/m^2).

4. **Engine and fuel system** — already sized by `SmallTurbojetModel`; verify
   fuel system mass scales with total fuel capacity.

5. **Total empty mass as output** — replace the fixed `EMPTY_MASS_KG = 7` constant
   with the sum of component masses; make it a live optimizer variable so mass
   trades are captured.

6. **Prerequisite for** CG estimation (mass stations must be known).
