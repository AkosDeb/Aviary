# SpaJeti v1.0.0 H-wing - Model Changelog

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
