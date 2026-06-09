# TODO

## CyBetaFuselage — proper implementation required

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

## EndplateARCorrection — endplate effect on VTP CL_alpha not yet investigated

`EndplateARCorrection` corrects the **wing** (horizontal panel) AR for the endplate
effect of the VTP panels.  The VTP's own CL_alpha (used in `CyDeltaRudder`) currently
uses `LiftCurveSlopePolhamus` with the raw VTP geometry — no endplate correction is
applied to the VTP itself.

Whether the wing acts as a meaningful endplate for the VTP (and how large the effect
is relative to the VTP's low AR) has not been assessed.  Investigate before adding
a correction to `CyDeltaRudder.CL_alpha_v`.

---

## Airfoil data per surface — replace thin-airfoil cl_alpha defaults

All `LiftCurveSlopePolhamus` instances currently use the thin-airfoil default
`section_lift_slope = 2*pi ~= 6.283 /rad`.  Real sections are 5–10 % lower.

For each lifting surface (wing, VTP, HTP):

1. Select an airfoil (e.g. NACA 0012 for VTP/HTP, NACA 4412 or similar for wing).
2. Extract `cl_alpha` at the design Mach and Reynolds number (XFoil / XFLR5 / MSES).
3. Wire the result into the corresponding `LiftCurveSlopePolhamus` instance via the
   `section_lift_slope` input.
4. Document the chosen airfoil and data source in the component README.

Priority: wing first (largest impact on CL_alpha and therefore Nz), then VTP (affects Ny).

---

## M_crit / MDD checker — enforce cruise Mach stays below critical Mach

Thicker airfoils increase CL_alpha and structural efficiency but reduce M_crit.
Higher cruise CL (heavy aircraft, low speed) also lowers M_crit.
As the optimizer pushes t/c up and CL up, cruise Mach may exceed M_crit, causing
wave drag not captured by the current subsonic FLOPS polar.

Steps:

1. **M_crit correlation** — implement the Korn equation (Shevell form):

       M_crit ~ kappa_A / (cos(sweep))^(1/3) - t/c / (cos(sweep))^(3/2) - CL / (10*(cos(sweep))^3)

   where `kappa_A` is the airfoil technology factor (0.87 conventional, 0.95 supercritical).
   Note: **CL appears explicitly** — a higher cruise lift coefficient directly reduces
   M_crit, so the constraint tightens as the aircraft becomes heavier or slower.

2. **CL connection** — wire the cruise CL from the mission phase output:
   `CL_cruise = gross_mass * g / (q_cruise * wing_area)`.
   This makes M_crit a function of both the geometry DVs (t/c, sweep) and the
   mission state (Mach, altitude, mass), closing the aero–structures–performance loop.

3. **MDD margin constraint** — add an optimizer constraint:
   `cruise_mach <= M_crit - delta_M_margin`
   where `delta_M_margin ~ 0.02`, giving `cruise_mach <= MDD`.

4. **Connect design variables** — M_crit depends on:
   - `aircraft:wing:thickness_to_chord` (primary driver for thick-wing penalty)
   - `aircraft:wing:sweep` (delays M_crit via cosine correction)
   - `CL_cruise` from the mission (heavier/slower aircraft lowers M_crit)
   - `kappa_A` (airfoil technology constant — set per chosen section)

5. Document in `README_lift_curve_slope.md` and add to the Constraints table in
   `README.md` for the model.

---

## Parasite drag — add fuselage and wing/VTP wetted-area drag

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

## Cl_beta — lateral-directional stability derivative (dihedral effect)

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

## Coordinate frame — define body-fixed reference for moment and CG calculations

No aircraft body-fixed coordinate system is currently defined.  All stability
derivatives (`CY_beta`, `Cn_beta`, `Cm_alpha`, `Cl_beta`) and CG estimates
require an agreed reference origin and axis convention.

Steps:

1. **Choose convention** — standard stability axes: x forward (fuselage nose),
   y right, z down.  Origin at fuselage nose station x = 0.

2. **Define reference stations** — record:
   - Wing MAC leading-edge x-station (`x_LEMAC`)
   - CG nominal location as fraction of MAC (`x_cg / MAC`)
   - Neutral point (aerodynamic centre) x-station (`x_NP`)
   - HTP and VTP aerodynamic centre stations

3. **Add to CSV or constants file** — `x_LEMAC`, fuselage reference length, MAC,
   and reference area should be accessible to all stability-derivative components.

4. **Prerequisite for** CG estimation, Cm_alpha, Cn_beta, and tail sizing.

---

## CG estimation — compute CG position from component masses and locations

Currently gross_mass and empty_mass are fixed scalar inputs.  No CG x-position
is estimated; the static margin is a hardcoded CSV constant (0.10) with no
gradient back to the optimizer.

Steps:

1. **Component mass stations** — assign an x-station (fraction of fuselage length)
   to each mass group: wing, fuselage, engine, fuel, payload, HTP, VTP.

2. **CG sum** — implement `x_cg = sum(m_i * x_i) / m_total` as an OpenMDAO
   `ExecComp` or `ExplicitComponent`.

3. **Static margin as output** — compute `SM = (x_NP - x_cg) / MAC` and expose
   as an optimizer constraint (`SM >= 0.05`, `SM <= 0.25`), replacing the
   fixed 0.10 in the CSV.

4. **Fuel burn shift** — CG moves as fuel burns; check CG travel between
   full-fuel and zero-fuel conditions stays within limits.

5. **Prerequisite for** Cm_alpha and tail sizing.

---

## CD_i — induced drag using effective AR from Scholz correction

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

## Cn_beta — directional stability derivative (weathercock stability)

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

## Tail geometry — complete empennage geometry for HTP and VTP

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

## Tail sizing — volume coefficient method for HTP and VTP

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

## Cm_alpha — longitudinal pitch stability derivative

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

## Aeroelasticity — wing divergence and flutter speed check

As wing span and AR grow (optimizer pushes toward long, slender wing), divergence
and flutter become the binding structural constraints before stress does.
Neither is currently computed.

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

## Weight estimation — component-level mass breakdown

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
