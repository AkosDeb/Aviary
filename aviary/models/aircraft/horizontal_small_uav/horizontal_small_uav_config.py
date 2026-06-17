"""
Configuration constants for horizontal_small_uav aircraft model.

This module centralizes all aircraft-specific parameters, constraints, design
limits, and airfoil selections used in the run script. Keeping these separate
makes it easy to adjust parameters without modifying the orchestration logic.
"""

import math
from pathlib import Path

from aviary.subsystems.aerodynamics.SpaJeti_based.airfoil_data import NACA_0012, NACA_4415
from aviary.subsystems.aerodynamics.SpaJeti_based.surface_config import SurfaceConfig
from aviary.subsystems.geometry.flops_based.superellipse_fuselage import superellipse_area

# ── File paths ────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parents[4]
AIRCRAFT_DATA = Path(__file__).with_name('horizontal_small_uav.csv')
OUTPUT_ROOT = REPO_ROOT / 'outputs'

# ── Model version ─────────────────────────────────────────────────────────────
# Bump manually in line with CHANGELOG.md:
#   patch (x.y.Z) -- bug fix, doc tweak, parameter change
#   minor (x.Y.0) -- new physics component or constraint
#   major (X.0.0) -- architectural redesign (new DV set, new EOM, new mission)
MODEL_VERSION = '1.33.0'
PROBLEM_NAME = 'run_horizontal_small_uav_try_v1'
VERSIONED_RUN_NAME = f'{PROBLEM_NAME}_v{MODEL_VERSION}'
OUTPUT_DIR = OUTPUT_ROOT / f'{VERSIONED_RUN_NAME}_out'

# ── Custom variable names ─────────────────────────────────────────────────────
AVAILABLE_FUEL = 'horizontal_small_uav:available_fuel'
FUEL_BUDGET_MARGIN = 'horizontal_small_uav:fuel_budget_margin'

# ── Mass parameters (design envelope) ─────────────────────────────────────────
EMPTY_MASS_KG = 7
FUEL_CAPACITY_KG = 8.0
ENGINE_MASS_LIMIT_KG = 5.0

# Note: MAX_TAKEOFF_MASS_KG is imported from phase_info.py (mission definition)

# ── Design-point flight condition for Ny / Nz constraints ─────────────────────
# ISA 5 000 m: rho = 0.7357 kg/m^3,  a = 320.5 m/s
# V = 550 km/h = 152.78 m/s  ->  M = 0.477,  q = 8 581 Pa
CONSTRAINT_MACH = 0.477
CONSTRAINT_Q_PA = 8_581.0

# ── Parametric fuselage geometry defaults ────────────────────────────────────
# Rounded-square cross-section: width = height, with rounded edges from the
# superellipse exponent. This local geometry feeds drag-reporting geometry and
# fuselage-intersection estimates; legacy CSV max_width/max_height entries are
# not used by this local geometry path.
FUSELAGE_ROUNDED_SQUARE_SIDE_M = 0.30
FUSELAGE_MAX_WIDTH_M = FUSELAGE_ROUNDED_SQUARE_SIDE_M
FUSELAGE_MAX_HEIGHT_M = FUSELAGE_ROUNDED_SQUARE_SIDE_M
FUSELAGE_NOSE_LENGTH_FRACTION = 0.20
FUSELAGE_TAIL_LENGTH_FRACTION = 0.35
FUSELAGE_BASE_WIDTH_FRACTION = 0.20
FUSELAGE_BASE_HEIGHT_FRACTION = 0.20
FUSELAGE_SUPERELLIPSE_EXPONENT = 4.0

# ── Fuselage equivalent diameter for K_wf ─────────────────────────────────────
# d_eq = sqrt(4 * A_max / pi) where A_max is the max-section superellipse area.
# Same formula as SuperellipseFuselageGeometry so K_wf and the post-run geometry
# report use a consistent diameter.
_fus_A_max = superellipse_area(
    FUSELAGE_MAX_WIDTH_M, FUSELAGE_MAX_HEIGHT_M, FUSELAGE_SUPERELLIPSE_EXPONENT
)
FUSELAGE_EQUIV_DIAMETER_M = math.sqrt(4.0 * _fus_A_max / math.pi)

# ── Fixed rudder parameters ───────────────────────────────────────────────────
RUDDER_CF_C = 0.25  # rudder chord / VTP chord
RUDDER_ETA_ROOT = 0.0  # inboard edge (fraction of VTP span)
RUDDER_ETA_TIP = 0.75  # outboard edge (fraction of VTP span)
RUDDER_DELTA_DEG = 20.0  # max rudder deflection [deg]  (also used as beta)

# ── VTP design-variable bounds ────────────────────────────────────────────────
VTP_SPAN_INITIAL_M = 0.310  # sqrt(0.08 * 1.2) from original CSV baseline
VTP_SPAN_LOWER_M = 0.15
VTP_SPAN_UPPER_M = 0.60

# ── Scholz winglet AR correction parameters ───────────────────────────────────
# k_WL reference values (Scholz 2018):
#   1.0  geometric ideal — winglet folded flat (unrealistic)
#   2.0  McLean / Howe   — theoretical optimum (best-case preliminary design)
#   2.45 Dubs / Zimmer   — experimental average; more realistic for real designs
#   2.8  real aircraft average — conservative lower bound from A/C performance data
# Using 2.45 for conceptual design: theoretical optimum is 2.0 but experimental
# data (Dubs, Zimmer) consistently shows 2.45 as the achievable average.
K_WL = 2.45  # winglet effectiveness penalty factor (Scholz 2018)
SPLIT_PENALTY = 0.90  # H-tail symmetric VTPs extend up+down: 90% of standard winglet

# ── Airfoil selection per surface ─────────────────────────────────────────────
# cl_alpha_per_rad is the INCOMPRESSIBLE (M~0) 2-D value.
# Polhamus handles 3-D compressibility via beta=sqrt(1-M^2) internally.
# Source: Abbott & von Doenhoff (1959); XFOIL at Re~2e6.
WING_AIRFOIL = NACA_4415  # cambered 15 % chord -- structural depth, CL_max at Re~2e6
VTP_AIRFOIL = NACA_0012  # symmetric 12 % chord -- lateral stability and control

# Wing section t/c is the OpenMDAO airfoil-section value used by MachCriticalComp.
# Keep aircraft:wing:thickness_to_chord fixed in the CSV for now so FLOPS weights
# remain unchanged while we test the M_crit constraint.
WING_TC_INITIAL = WING_AIRFOIL.tc_ratio
WING_TC_LOWER = 0.05
WING_TC_UPPER = 0.18

# ── Surface aerodynamic configurations ────────────────────────────────────────
WING_SURFACE_CFG = SurfaceConfig(
    name='wing',
    airfoil=WING_AIRFOIL,
    endplate_correction=True,
    k_wl=K_WL,
    split_penalty=SPLIT_PENALTY,
    fuselage_diameter=FUSELAGE_EQUIV_DIAMETER_M,
)
VTP_SURFACE_CFG = SurfaceConfig(
    name='vtp',
    airfoil=VTP_AIRFOIL,
    has_control_surface=True,
)

# ── Constraint lower bounds ───────────────────────────────────────────────────
NY_MIN = 7.0
NZ_MIN = 7.0
TW_MIN = 1.5  # T/W at SLS

# ── Alpha max for Nz and M_DD constraints ────────────────────────────────────
# Both LongitudinalLoadFactor (Nz) and MachCriticalComp (M_DD check) use the same
# alpha so their constraints stay consistent.
# Source: NACA 4415 polar at Re ~ 2e6 from airfoiltools.com
# (http://airfoiltools.com/airfoil/details?airfoil=naca4415-il).
# Stall occurs at approximately alpha = 15 deg -- accurate enough for current
# design phase.  Replace with StallAlphaComp output when implemented (see TOOD.md).
ALPHA_MAX_DEG = 15.0

# ── M_crit safety check ───────────────────────────────────────────────────────
# Constraint: M_crit >= DASH_MACH + M_CRIT_SAFETY_MARGIN
#   M_CRIT_SAFETY_MARGIN = 0.05 keeps the design 50 Mach-counts below sonic onset.
#   NOTE: CL is computed as CL_alpha_3D * alpha_max_rad (same as Nz).  This is a
#   conservative estimate -- at DASH_MACH the actual CL~0.03, not CL_max~1.0.
#   With NACA 4415 (t/c=0.15) and CL~1.0, baseline M_crit ~0.53 which is below
#   the 0.57 check -- the constraint will show as ACTIVE/VIOLATED until t/c becomes
#   a DV or alpha_max is reduced.  See TOOD.md 'alpha_max / CL_max from stall'.
# Note: DASH_MACH is imported from phase_info.py (mission definition)
M_CRIT_SAFETY_MARGIN = 0.05
# MACH_UPPER_BOUND is computed at runtime: DASH_MACH + M_CRIT_SAFETY_MARGIN

# ── Wing apex position (aircraft reference frame) ─────────────────────────────
# Origin: nose tip.  x: positive AFT (fuselage station).  y: positive starboard.
# z: positive DOWN.  Fuselage length = 2.0 m; wing root LE ~40 % aft of nose.
# These are geometric inputs; promote to design variables for CG / SM optimisation.
WING_X_APEX_M = 0.80  # x-station of wing root LE from nose [m]
WING_Z_APEX_M = 0.00  # z-station of wing root LE from nose datum [m]

# ── H-wing junction interference factor ──────────────────────────────────────
# Applied to both wing (on top of R_wf_fus) and VTP (with R_wf_vtp = 1.0).
# Represents the parasite drag penalty at the wingtip wing-VTP junction.
# Roskam/Raymer wing-winglet junctions: 1.03-1.08 for a clean, well-faired joint.
H_WING_INTERFERENCE_FACTOR = 1.04

# ── ISA 5 000 m standard atmosphere ──────────────────────────────────────────
# Used by the parasite drag report (dash flight condition, same as Ny/Nz constraint).
CRUISE_STATIC_PRESSURE_PA = 54048.0  # Pa
CRUISE_TEMPERATURE_K = 255.65  # K

# ── ISA 5 000 m air density for aeroelastic analysis ─────────────────────────
ISA_5KM_DENSITY_KGM3 = 0.7357  # kg/m³

# ── Aeroelastic speed constraints (TOOD.md Aeroelasticity step 3) ─────────────
# V_dive = 1.25 * V_design per CS-23/CS-VLA convention; divergence and flutter
# speeds must exceed V_dive.  The quasi-steady flutter screen uses the smooth
# MAX_REAL_EIGENVALUE_AT_DESIGN output as the optimizer constraint (exact CS
# derivatives); FLUTTER_SPEED_MARGIN is reported for information only.
DIVE_SPEED_FACTOR = 1.25
# AERO_REQUIRED_SPEED_MS is computed at runtime: DIVE_SPEED_FACTOR * (550.0 / 3.6)

# ── Output detail flag ────────────────────────────────────────────────────────
PRINT_AERO_DETAIL = True  # set False to suppress the wing/VTP/rudder aero breakdown
