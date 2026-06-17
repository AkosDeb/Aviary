import csv
import math
import os
import shutil
import sys
import warnings
from pathlib import Path

import numpy as np
import openmdao.api as om

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault('OPENMDAO_USE_MPI', '0')

import aviary.api as av

try:
    from .phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH, phase_info
except ImportError:
    from phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH, phase_info

from aviary.subsystems.propulsion.small_turbojet import (
    SmallTurbojetModel,
    SmallTurbojetVariables,
)
from aviary.subsystems.geometry.flops_based.htail_geometry import HTailGeometry
from aviary.subsystems.geometry.flops_based.cg_estimator import CGEstimatorGroup
from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    SuperellipseFuselageGeometry,
    superellipse_area,
    superellipse_width_height_distribution,
    _superellipse_surface_points,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.airfoil_data import NACA_0012, NACA_4415
from aviary.subsystems.aerodynamics.SpaJeti_based.surface_config import SurfaceConfig
from aviary.subsystems.aerodynamics.SpaJeti_based.lifting_surface import WingSurface, VTPSurface
from aviary.subsystems.aerodynamics.SpaJeti_based.lateral_load_factor import (
    LateralLoadFactor,
    LongitudinalLoadFactor,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.induced_drag import (
    FuselageLiftInducedDragComp,
    RoskamInducedDragComp,
)
from aviary.subsystems.aerodynamics.roskam_aero_builder import RoskamAeroBuilder
from aviary.models.external_subsystems.aeroelasticity.aeroelasticity_builder import (
    AeroelasticityGroup,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


AIRCRAFT_DATA = Path(__file__).with_name('horizontal_small_uav.csv')
EMPTY_MASS_KG = 7
FUEL_CAPACITY_KG = 8.0
ENGINE_MASS_LIMIT_KG = 5.0

# ── Model version ─────────────────────────────────────────────────────────────
# Bump manually in line with CHANGELOG.md:
#   patch (x.y.Z) -- bug fix, doc tweak, parameter change
#   minor (x.Y.0) -- new physics component or constraint
#   major (X.0.0) -- architectural redesign (new DV set, new EOM, new mission)
MODEL_VERSION = '1.28.0'

OUTPUT_ROOT = REPO_ROOT / 'outputs'
PROBLEM_NAME = 'run_horizontal_small_uav_try_v1'
VERSIONED_RUN_NAME = f'{PROBLEM_NAME}_v{MODEL_VERSION}'
OUTPUT_DIR = OUTPUT_ROOT / f'{VERSIONED_RUN_NAME}_out'
AVAILABLE_FUEL = 'horizontal_small_uav:available_fuel'
FUEL_BUDGET_MARGIN = 'horizontal_small_uav:fuel_budget_margin'

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
FUSELAGE_MAX_WIDTH_M           = FUSELAGE_ROUNDED_SQUARE_SIDE_M
FUSELAGE_MAX_HEIGHT_M          = FUSELAGE_ROUNDED_SQUARE_SIDE_M
FUSELAGE_NOSE_LENGTH_FRACTION  = 0.20
FUSELAGE_TAIL_LENGTH_FRACTION  = 0.35
FUSELAGE_BASE_WIDTH_FRACTION   = 0.20
FUSELAGE_BASE_HEIGHT_FRACTION  = 0.20
FUSELAGE_SUPERELLIPSE_EXPONENT = 4.0

# ── Fuselage equivalent diameter for K_wf ─────────────────────────────────────
# d_eq = sqrt(4 * A_max / pi) where A_max is the max-section superellipse area.
# Same formula as SuperellipseFuselageGeometry so K_wf and the post-run geometry
# report use a consistent diameter.
_fus_A_max = superellipse_area(
    FUSELAGE_MAX_WIDTH_M, FUSELAGE_MAX_HEIGHT_M, FUSELAGE_SUPERELLIPSE_EXPONENT
)
FUSELAGE_EQUIV_DIAMETER_M = math.sqrt(4.0 * _fus_A_max / math.pi)

# (Fuselage eta and c_d_c are now interpolated inside FuselageLiftInducedDragComp
#  from the Roskam Part VI lookup tables — no module-level constants needed.)

# ── Fixed rudder parameters ───────────────────────────────────────────────────
RUDDER_CF_C       = 0.25   # rudder chord / VTP chord
RUDDER_ETA_ROOT   = 0.0    # inboard edge (fraction of VTP span)
RUDDER_ETA_TIP    = 0.75   # outboard edge (fraction of VTP span)
RUDDER_DELTA_DEG  = 20.0   # max rudder deflection [deg]  (also used as beta)

# ── VTP design-variable bounds ────────────────────────────────────────────────
VTP_SPAN_INITIAL_M = 0.310   # sqrt(0.08 * 1.2) from original CSV baseline
VTP_SPAN_LOWER_M   = 0.15
VTP_SPAN_UPPER_M   = 0.60

# ── Scholz winglet AR correction parameters ───────────────────────────────────
# k_WL reference values (Scholz 2018):
#   1.0  geometric ideal — winglet folded flat (unrealistic)
#   2.0  McLean / Howe   — theoretical optimum (best-case preliminary design)
#   2.45 Dubs / Zimmer   — experimental average; more realistic for real designs
#   2.8  real aircraft average — conservative lower bound from A/C performance data
# Using 2.45 for conceptual design: theoretical optimum is 2.0 but experimental
# data (Dubs, Zimmer) consistently shows 2.45 as the achievable average.
K_WL          = 2.45  # winglet effectiveness penalty factor (Scholz 2018)
SPLIT_PENALTY = 0.90  # H-tail symmetric VTPs extend up+down: 90% of standard winglet

# ── Airfoil selection per surface ─────────────────────────────────────────────
# cl_alpha_per_rad is the INCOMPRESSIBLE (M~0) 2-D value.
# Polhamus handles 3-D compressibility via beta=sqrt(1-M^2) internally.
# Source: Abbott & von Doenhoff (1959); XFOIL at Re~2e6.
WING_AIRFOIL = NACA_4415   # cambered 15 % chord -- structural depth, CL_max at Re~2e6
VTP_AIRFOIL  = NACA_0012   # symmetric 12 % chord -- lateral stability and control

# Wing section t/c is the OpenMDAO airfoil-section value used by MachCriticalComp.
# Keep aircraft:wing:thickness_to_chord fixed in the CSV for now so FLOPS weights
# remain unchanged while we test the M_crit constraint.
WING_TC_INITIAL = WING_AIRFOIL.tc_ratio
WING_TC_LOWER   = 0.05
WING_TC_UPPER   = 0.18

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
TW_MIN = 1.5   # T/W at SLS

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
M_CRIT_SAFETY_MARGIN = 0.05
MACH_UPPER_BOUND     = DASH_MACH + M_CRIT_SAFETY_MARGIN  # = 0.57

# ── Wing apex position (aircraft reference frame) ─────────────────────────────
# Origin: nose tip.  x: positive AFT (fuselage station).  y: positive starboard.
# z: positive DOWN.  Fuselage length = 2.0 m; wing root LE ~40 % aft of nose.
# These are geometric inputs; promote to design variables for CG / SM optimisation.
WING_X_APEX_M = 0.80   # x-station of wing root LE from nose [m]
WING_Z_APEX_M = 0.00   # z-station of wing root LE from nose datum [m]

# ── CG estimation ─────────────────────────────────────────────────────────────
# Bypass: set CG_BYPASS=True and edit the three manual values to override.
# Estimate mode: component list is defined in add_cg_estimation() below.
CG_BYPASS         = False
CG_X_MANUAL_M     = 0.91   # [m] x_cg override -- only active when CG_BYPASS=True
CG_Y_MANUAL_M     = 0.00   # [m] y_cg override
CG_Z_MANUAL_M     = 0.05   # [m] z_cg override
CG_MASS_MANUAL_KG = 15.0   # [kg] total mass override

# ── H-wing junction interference factor ──────────────────────────────────────
# Applied to both wing (on top of R_wf_fus) and VTP (with R_wf_vtp = 1.0).
# Represents the parasite drag penalty at the wingtip wing-VTP junction.
# Roskam/Raymer wing-winglet junctions: 1.03-1.08 for a clean, well-faired joint.
H_WING_INTERFERENCE_FACTOR = 1.04

# ── ISA 5 000 m standard atmosphere ──────────────────────────────────────────
# Used by the parasite drag report (dash flight condition, same as Ny/Nz constraint).
CRUISE_STATIC_PRESSURE_PA = 54048.0   # Pa
CRUISE_TEMPERATURE_K       = 255.65   # K

# ── ISA 5 000 m air density for aeroelastic analysis ─────────────────────────
ISA_5KM_DENSITY_KGM3 = 0.7357   # kg/m³

# ── Aeroelastic speed constraints (TOOD.md Aeroelasticity step 3) ─────────────
# V_dive = 1.25 * V_design per CS-23/CS-VLA convention; divergence and flutter
# speeds must exceed V_dive.  The quasi-steady flutter screen uses the smooth
# MAX_REAL_EIGENVALUE_AT_DESIGN output as the optimizer constraint (exact CS
# derivatives); FLUTTER_SPEED_MARGIN is reported for information only.
DIVE_SPEED_FACTOR = 1.25
AERO_REQUIRED_SPEED_MS = DIVE_SPEED_FACTOR * 550.0 / 3.6   # 190.97 m/s

# ── Output detail flag ────────────────────────────────────────────────────────
PRINT_AERO_DETAIL = True  # set False to suppress the wing/VTP/rudder aero breakdown


def ipopt_available():
    try:
        from pyoptsparse import OPT
        OPT('IPOPT')
        return True
    except Exception:
        return False


def select_optimizer():
    requested = os.environ.get('SPAJETI_OPTIMIZER', '').strip().upper()
    if requested:
        return requested
    return 'IPOPT' if ipopt_available() else 'SLSQP'


OPTIMIZER = select_optimizer()


class FuelBudgetEstimate(om.ExplicitComponent):
    """Fuel available after fixed masses and optimized engine mass are reserved."""

    def setup(self):
        self.add_input(av.Aircraft.Design.GROSS_MASS, val=15.0, units='kg')
        self.add_input(av.Aircraft.Design.EMPTY_MASS, val=7.0, units='kg')
        self.add_input(av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, val=2.5, units='kg')
        self.add_input(av.Mission.TOTAL_FUEL, val=1.0, units='kg')
        self.add_input('engine_mass', val=3.0, units='kg')
        self.add_output(AVAILABLE_FUEL, val=2.5, units='kg')
        self.add_output(FUEL_BUDGET_MARGIN, val=1.5, units='kg')
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        available_fuel = (
            inputs[av.Aircraft.Design.GROSS_MASS]
            - inputs[av.Aircraft.Design.EMPTY_MASS]
            - inputs[av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS]
            - inputs['engine_mass']
        )
        outputs[AVAILABLE_FUEL] = available_fuel
        outputs[FUEL_BUDGET_MARGIN] = available_fuel - inputs[av.Mission.TOTAL_FUEL]


def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'


def safe_get(prob, var, units=None):
    try:
        value = prob.get_val(var, units=units) if units else prob.get_val(var)
        return value[0] if hasattr(value, '__len__') else value
    except Exception as err:
        return f'not available: {err}'


def print_result(label, value, unit=''):
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {value:>12.4f} {unit}'.rstrip())


def print_scientific_result(label, value, unit=''):
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {value:>12.6e} {unit}'.rstrip())


def print_percent_result(label, value):
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {100.0 * value:>12.2f} %')


def print_aero_detail(prob):
    """Detailed wing, VTP, and rudder breakdown - all inputs and derived values."""

    # Safe attribute fallback for variables that may not exist in all Aviary versions
    _VTP_AVG_CHORD  = getattr(av.Aircraft.VerticalTail, 'AVERAGE_CHORD',
                               'aircraft:vertical_tail:average_chord')
    _VTP_ROOT_CHORD = getattr(av.Aircraft.VerticalTail, 'ROOT_CHORD',
                               'aircraft:vertical_tail:root_chord')
    _WING_DIHEDRAL  = getattr(av.Aircraft.Wing, 'DIHEDRAL', 'aircraft:wing:dihedral')

    # ── Retrieve all values ───────────────────────────────────────────────────
    wing_span     = safe_get(prob, av.Aircraft.Wing.SPAN,             'm')
    wing_area     = safe_get(prob, av.Aircraft.Wing.AREA,             'm**2')
    wing_ar       = safe_get(prob, av.Aircraft.Wing.ASPECT_RATIO)
    wing_sweep    = safe_get(prob, av.Aircraft.Wing.SWEEP,            'deg')
    wing_taper    = safe_get(prob, av.Aircraft.Wing.TAPER_RATIO)
    wing_tc       = safe_get(prob, av.Aircraft.Wing.THICKNESS_TO_CHORD)
    wing_section_tc = safe_get(prob, 'wing_section_tc')
    wing_dihedral = safe_get(prob, _WING_DIHEDRAL,                   'deg')

    ar_eff        = safe_get(prob, 'AR_eff')
    k_h           = safe_get(prob, 'k_h')
    wing_cl_alpha = safe_get(prob, 'wing_CL_alpha')

    wing_root_chord = safe_get(prob, 'wing_root_chord',  'm')
    wing_c_mac      = safe_get(prob, 'wing_c_mac',       'm')
    wing_y_mac      = safe_get(prob, 'wing_y_mac',       'm')
    wing_x_mac_le   = safe_get(prob, 'wing_x_mac_le',    'm')
    wing_z_mac_le   = safe_get(prob, 'wing_z_mac_le',    'm')
    wing_x_mac_c4   = safe_get(prob, 'wing_x_mac_c4',   'm')
    wing_le_sweep   = safe_get(prob, 'wing_le_sweep',   'deg')

    vtp_span      = safe_get(prob, av.Aircraft.VerticalTail.SPAN,             'm')
    vtp_area      = safe_get(prob, av.Aircraft.VerticalTail.AREA,             'm**2')
    vtp_ar        = safe_get(prob, av.Aircraft.VerticalTail.ASPECT_RATIO)
    vtp_avg_chord = safe_get(prob, _VTP_AVG_CHORD,                           'm')
    vtp_root_c    = safe_get(prob, _VTP_ROOT_CHORD,                          'm')
    vtp_sweep     = safe_get(prob, av.Aircraft.VerticalTail.SWEEP,           'deg')
    vtp_taper     = safe_get(prob, av.Aircraft.VerticalTail.TAPER_RATIO)
    vtp_tc        = safe_get(prob, av.Aircraft.VerticalTail.THICKNESS_TO_CHORD)
    fus_vtp_ratio = safe_get(prob, 'fuselage_vtp_span_ratio')
    cl_alpha_v    = safe_get(prob, 'CL_alpha_v')

    cy_beta_vtp   = safe_get(prob, 'CY_beta_vtp')
    cy_delta_r    = safe_get(prob, 'CY_delta_r')
    ny            = safe_get(prob, 'Ny')
    nz            = safe_get(prob, 'Nz')
    nz_cl_om      = safe_get(prob, 'CL')
    nz_lift_om    = safe_get(prob, 'lift', 'N')

    m_dd          = safe_get(prob, 'M_DD')
    m_crit        = safe_get(prob, 'M_crit')
    m_crit_margin = safe_get(prob, 'mach_crit_margin')

    beta_pg = (1.0 - CONSTRAINT_MACH ** 2) ** 0.5

    print('\n' + '=' * 70)
    print('AERODYNAMIC DETAIL BREAKDOWN')
    print('  (toggle with PRINT_AERO_DETAIL in run_horizontal_small_uav.py)')
    print('=' * 70)

    # ── 1. Wing geometry inputs ───────────────────────────────────────────────
    print('\nWing Geometry - Inputs')
    print('-' * 70)
    print_result('Span b  [design var]',          wing_span,    'm')
    print_result('Reference area S_ref  [design var]', wing_area,    'm^2')
    print_result('AR_geo = b^2/S',                wing_ar,      '')
    print_result('Quarter-chord sweep Lc/4',      wing_sweep,   'deg')
    print_result('Taper ratio lambda',            wing_taper,   '')
    print_result('Thickness/chord t/c [FLOPS fixed]', wing_tc,      '')
    print_result('Section t/c [M_crit design var]',   wing_section_tc, '')
    print_result('Dihedral',                      wing_dihedral,'deg')

    # ── 1b. Wing MAC geometry ─────────────────────────────────────────────────
    print('\nWing Mean Aerodynamic Chord - MACGeometryComp')
    print('-' * 70)
    print('  Frame: origin at nose tip; x positive AFT; y positive starboard; z positive DOWN')
    print()
    print(f'  {"Formula: c_r = 2*S / (b*(1+lambda))":<69}')
    print(f'  {"         c_mac = (2/3)*c_r*(1+lambda+lambda^2)/(1+lambda)":<69}')
    print(f'  {"         y_mac = (b/6)*(1+2*lambda)/(1+lambda)":<69}')
    print(f'  {"         tan_LE = tan(c4_sweep) + c_r*(1-lambda)/b":<69}')
    print(f'  {"         x_mac_le = x_apex + y_mac*tan_LE":<69}')
    print(f'  {"         z_mac_le = z_apex - y_mac*tan(dihedral)":<69}')
    print()
    print(f'  {"Wing root LE (apex) x-station [fixed]":<35} = {WING_X_APEX_M:>10.4f} m')
    print(f'  {"Wing root LE (apex) z-station [fixed]":<35} = {WING_Z_APEX_M:>10.4f} m')
    print_result('Root chord c_r (derived from S,b,lambda)', wing_root_chord, 'm')
    print_result('MAC chord c_mac',                           wing_c_mac,      'm')
    print_result('MAC BL station y_mac (from CL)',            wing_y_mac,      'm')
    print_result('MAC LE x-station from nose',                wing_x_mac_le,   'm')
    print_result('MAC LE z-station from nose datum',          wing_z_mac_le,   'm')
    print_result('MAC c/4 x-station (aero centre x)',         wing_x_mac_c4,   'm')
    if not any(isinstance(v, str) for v in (wing_x_mac_c4, wing_c_mac, wing_span)):
        fus_len = safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm')
        if not isinstance(fus_len, str):
            print_result('  x_mac_c4 / fuselage length',      wing_x_mac_c4 / fus_len, '')

    # ── 2. Scholz winglet AR correction ──────────────────────────────────────
    print('\nEndplate AR Correction - Scholz (2018) Winglet Method')
    print('-' * 70)
    print(f'  {"Method":<33}   Scholz INCAS 2018, Eq. 6')
    print(f'  {"k_WL  [McLean/Howe default]":<33} = {K_WL:>10.4f}')
    print(f'  {"Split-winglet penalty (up+down)":<33} = {SPLIT_PENALTY:>10.4f}  (90% of std winglet)')
    print()
    if not any(isinstance(v, str) for v in (vtp_span, wing_span, wing_ar)):
        h_over_b = vtp_span / wing_span
        b_eff    = wing_span + 2.0 * vtp_span / K_WL
        ar_raw   = wing_ar * (b_eff / wing_span) ** 2
        ar_final = ar_raw * SPLIT_PENALTY
        print_result('VTP span h  [design var]',            vtp_span, 'm')
        print_result('Wing span b_h  [design var]',         wing_span, 'm')
        print_result('h / b_h',                             h_over_b, '')
        print()
        print(f'  Step 1  b_eff = b_h + 2h/k_WL')
        print_result('  b_eff',                             b_eff,  'm')
        print()
        print(f'  Step 2  AR_eff_raw = AR_geo * (b_eff/b_h)^2')
        print_result('  AR_geo  (no endplate)',              wing_ar, '')
        print_result('  AR_eff_raw',                        ar_raw,  '')
        print()
        print(f'  Step 3  AR_eff = AR_eff_raw * split_penalty')
        print_result('  AR_eff (reproduced)',                ar_final, '')
        print_result('  AR_eff (from OpenMDAO)',             ar_eff,   '')
        print_result('  k_h = AR_eff / AR_geo',             k_h,      '')
        print()
        ar_gain     = ar_eff - wing_ar if not isinstance(ar_eff, str) else 0.0
        ar_gain_pct = (ar_eff / wing_ar - 1.0) * 100.0 if not isinstance(ar_eff, str) else 0.0
        print_result('AR gain = AR_eff - AR_geo',           ar_gain,     '')
        print_result('AR gain (%)',                          ar_gain_pct, '%')

    # ── 2b. Wing M_crit / M_DD (Weisshaar Eq. 36) ─────────────────────────────
    _m_delta = (0.1 / 80.0) ** (1.0 / 3.0)
    print('\nWing M_crit / M_DD - Weisshaar Eq. 36 (K_A = 0.887)')
    print('-' * 70)
    print(f'  {"Formula: M_DD = K_A/cos(phi) - (t/c)/cos^2 - CL/(10*cos^3)":<69}')
    print(f'  {"         M_crit = M_DD - (0.1/80)^(1/3) ~ M_DD - 0.108":<69}')
    print()
    print(f'  {"Input: K_A (Weisshaar optimised)":<35} = {0.887:>10.4f}')
    print(f'  {"Input: t/c  [design var]":<35} = ', end='')
    print(f'{wing_section_tc:>10.4f}  ({WING_AIRFOIL.name})'
          if not isinstance(wing_section_tc, str) else wing_section_tc)
    print(f'  {"Input: Nz_min":<35} = {NZ_MIN:>10.4f}')
    print(f'  {"Input: aircraft_mass":<35} = {MAX_TAKEOFF_MASS_KG:>10.4f} kg')
    print(f'  {"Input: dynamic_pressure":<35} = {CONSTRAINT_Q_PA:>10.1f} Pa')
    print(f'  {"Input: sweep c/4  phi_25":<35} = ', end='')
    print(f'{wing_sweep:>10.4f} deg' if not isinstance(wing_sweep, str) else wing_sweep)
    if not any(isinstance(v, str) for v in (wing_sweep, wing_section_tc)):
        wing_area_val = safe_get(prob, av.Aircraft.Wing.AREA, 'm**2')
        if not isinstance(wing_area_val, str):
            cl_design = NZ_MIN * MAX_TAKEOFF_MASS_KG * 9.80665 / (CONSTRAINT_Q_PA * wing_area_val)
            phi_rad = wing_sweep * np.pi / 180.0
            cos1    = np.cos(phi_rad)
            mdd_r   = 0.887 / cos1 - wing_section_tc / cos1**2 - cl_design / (10.0 * cos1**3)
            mcrit_r = mdd_r - _m_delta
            print()
            print_result('CL = Nz_min*m*g / (q*S)',                cl_design, '')
            print_result('M_DD (reproduced)',                       mdd_r,    '')
            print_result('M_DD (from OpenMDAO)',                    m_dd,     '')
            print_result('(0.1/80)^(1/3) = M_DD - M_crit',        _m_delta, '')
            print_result('M_crit (reproduced)',                     mcrit_r,  '[informational]')
            print_result('M_crit (from OpenMDAO)',                  m_crit,   '[informational]')
            print()
            print(f'  {"M_crit check: DASH_MACH + safety":<35} = '
                  f'{DASH_MACH:.3f} + {M_CRIT_SAFETY_MARGIN:.3f} = {MACH_UPPER_BOUND:.4f}')
            print_result('mach_crit_margin = M_crit-check [min 0]', m_crit_margin, '')
            if not isinstance(m_crit_margin, str):
                status = 'OK' if m_crit_margin >= 0.0 else 'VIOLATED'
                print(f'  {"Constraint status":<35}   {status}')

    # ── 3. Wing Polhamus ──────────────────────────────────────────────────────
    print(f'\nWing CL_alpha - LiftCurveSlopePolhamus (Roskam 8.22)  [{WING_AIRFOIL.name}]')
    print('-' * 70)
    print(f'  {"Input: AR_eff (from endplate corr.)":<35} = ', end='')
    print(f'{ar_eff:>10.4f}' if not isinstance(ar_eff, str) else ar_eff)
    print(f'  {"Input: Mach at design point":<35} = {CONSTRAINT_MACH:>10.4f}')
    print(f'  {"Input: beta_PG = sqrt(1 - M^2)":<35} = {beta_pg:>10.4f}')
    print(f'  {"Input: t/c (Abbott & von Doenhoff)":<35} = ', end='')
    wing_tc_val = safe_get(prob, 'wing_section_tc')
    print(f'{wing_tc_val:>10.4f}' if not isinstance(wing_tc_val, str) else wing_tc_val)
    print(f'  {"Input: quarter-chord sweep Lambda_c/4":<35} = ', end='')
    print(f'{wing_sweep:>10.4f} deg' if not isinstance(wing_sweep, str) else wing_sweep)
    print(f'  {"Input: taper ratio Lambda":<35} = ', end='')
    print(f'{wing_taper:>10.4f}' if not isinstance(wing_taper, str) else wing_taper)
    print_result('Output: CL_alpha  (wing-alone)',        wing_cl_alpha, '/rad')

    # ── 4. VTP geometry inputs ────────────────────────────────────────────────
    print('\nVTP Geometry - HTailGeometry (each panel; x2 for full H-tail)')
    print('-' * 70)
    print_result('Input:  b_v  [design var]',            vtp_span,   'm')
    print_result('Input:  c_root  [CSV]',                vtp_root_c, 'm')
    print_result('Input:  taper ratio Lambda  [CSV]',         vtp_taper,  '')
    print_result('Input:  sweep c/4  [CSV]',             vtp_sweep,  'deg')
    print_result('Input:  t/c  [CSV]',                   vtp_tc,     '')
    print_result('Output: S_v = b_v*c_root*(1+Lambda)/2',    vtp_area,   'm^2')
    print_result('Output: AR_v = b_v^2/S_v',             vtp_ar,     '')
    print_result('Output: c_mean = S_v / b_v',          vtp_avg_chord, 'm')
    print_result('Output: 2r_i/b_v  (fus. endplate)',   fus_vtp_ratio, '')
    if not isinstance(fus_vtp_ratio, str):
        note = ('(fuselage wide -> strong k_v correction)' if fus_vtp_ratio > 0.5
                else '(moderate fuselage endplate effect)')
        print(f'  {"   note":<33}   {note}')

    # ── 5. VTP aerodynamics ───────────────────────────────────────────────────
    print(f'\nVTP CL_alpha_v - LiftCurveSlopePolhamus (raw AR_v)  [{VTP_AIRFOIL.name}]')
    print('-' * 70)
    print(f'  {"Input: AR_v (raw, no k_h)":<35} = ', end='')
    print(f'{vtp_ar:>10.4f}' if not isinstance(vtp_ar, str) else vtp_ar)
    print(f'  {"Input: Mach":<35} = {CONSTRAINT_MACH:>10.4f}')
    print(f'  {"Input: beta_PG":<35} = {beta_pg:>10.4f}')
    print(f'  {"Input: section lift slope":<35} = {VTP_AIRFOIL.cl_alpha_per_rad:>10.4f} /rad'
          f'  ({VTP_AIRFOIL.name}, Re={VTP_AIRFOIL.re_ref:.0e}, incompressible)')
    print(f'  {"Input: t/c (Abbott & von Doenhoff)":<35} = {VTP_AIRFOIL.tc_ratio:>10.4f}')
    print_result('Input: sweep c/4',                    vtp_sweep,  'deg')
    print_result('Input: taper ratio Lambda',                vtp_taper,  '')
    print_result('Output: CL_alpha_v',                  cl_alpha_v, '/rad')

    # ── 6. Rudder / CY_delta_r ────────────────────────────────────────────────
    print('\nRudder Authority - CyDeltaRudder')
    print('-' * 70)
    print_result('Input: S_v (per panel)',               vtp_area,   'm^2')
    print_result('Input: VTP taper ratio Lambda',             vtp_taper,  '')
    print_result('Input: VTP t/c',                       vtp_tc,     '')
    print_result('Input: S_ref (wing)',                  wing_area,  'm^2')
    print_result('Input: CL_alpha_v',                   cl_alpha_v, '/rad')
    print(f'  {"Input: rudder chord fraction cf/c":<35} = {RUDDER_CF_C:>10.4f}')
    print(f'  {"Input: inboard edge eta_root":<35} = {RUDDER_ETA_ROOT:>10.4f}  (fraction of VTP span)')
    print(f'  {"Input: outboard edge eta_tip":<35} = {RUDDER_ETA_TIP:>10.4f}  (fraction of VTP span)')
    print(f'  {"Input: max deflection delta_r":<35} = {RUDDER_DELTA_DEG:>10.4f} deg')
    print_result('Output: CY_delta_r',                  cy_delta_r, '/rad')

    # ── 7. Passive side-force ─────────────────────────────────────────────────
    print('\nPassive Side-Force - CyBetaVtp (x2 for both VTP panels)')
    print('-' * 70)
    print_result('Input: AR_v (from HTailGeometry)',     vtp_ar,         '')
    print_result('Input: S_v per panel',                 vtp_area,       'm^2')
    print_result('Input: S_ref (wing)',                  wing_area,      'm^2')
    print_result('Input: 2r_i/b_v (fus. ratio)',         fus_vtp_ratio,  '')
    print(f'  {"x2 factor for both panels applied internally"}')
    print_result('Output: CY_beta_vtp',                 cy_beta_vtp,    '/rad')

    # ── 8. Lateral load factor Ny ─────────────────────────────────────────────
    print('\nLateral Load Factor - LateralLoadFactor')
    print('-' * 70)
    print(f'  Formula:  CY = CY_beta_vtpxbeta + CY_deltarxdelta_r')
    print(f'            side_force = CY x q x S_ref')
    print(f'            Ny = -side_force / (m x g)')
    print()
    print(f'  {"Design point":<33}   550 km/h, 5 km ISA')
    print(f'  {"q":<33} = {CONSTRAINT_Q_PA:>10.1f} Pa')
    print_result('S_ref',                               wing_area, 'm^2')
    print(f'  {"m":<33} = {MAX_TAKEOFF_MASS_KG:>10.4f} kg')
    print(f'  {"beta (sideslip, conservative = delta_r)":<33} = {RUDDER_DELTA_DEG:>10.4f} deg')
    print(f'  {"delta_r (rudder deflection)":<33} = {RUDDER_DELTA_DEG:>10.4f} deg')

    if not any(isinstance(v, str) for v in (cy_beta_vtp, cy_delta_r, wing_area)):
        beta_rad    = RUDDER_DELTA_DEG * np.pi / 180.0
        delta_r_rad = RUDDER_DELTA_DEG * np.pi / 180.0
        cy_passive  = cy_beta_vtp * beta_rad
        cy_active   = cy_delta_r  * delta_r_rad
        cy_total    = cy_passive + cy_active
        side_force  = cy_total * CONSTRAINT_Q_PA * wing_area
        ny_calc     = -side_force / (MAX_TAKEOFF_MASS_KG * 9.80665)

        print()
        print_result('CY_beta_vtp x beta   (passive VTP)', cy_passive,  '')
        print_result('CY_delta_r  x delta_r (active rudder)', cy_active, '')
        print_result('CY_total',                          cy_total,   '')
        print_result('side_force = CY x q x S',          side_force, 'N')
        print_result('Ny (reproduced)',                   ny_calc,   f'[min {NY_MIN}]')
        print_result('Ny (OpenMDAO)',                     ny,        f'[min {NY_MIN}]')
        if cy_total != 0.0:
            print()
            print(f'  Contribution breakdown:')
            print(f'  {"  Passive VTP (sideslip)":<33} = {100.0 * cy_passive / cy_total:>10.1f} %')
            print(f'  {"  Active rudder":<33} = {100.0 * cy_active  / cy_total:>10.1f} %')

    # ── 9. Longitudinal load factor Nz ────────────────────────────────────────
    print('\nLongitudinal Load Factor - LongitudinalLoadFactor')
    print('-' * 70)
    print(f'  Formula:  CL   = CL_alpha x alpha_max')
    print(f'            lift = CL x q x S_ref')
    print(f'            Nz   = lift / (m x g)')
    print()
    print(f'  {"Design point":<33}   550 km/h, 5 km ISA')
    print(f'  {"q":<33} = {CONSTRAINT_Q_PA:>10.1f} Pa')
    print_result('S_ref',                               wing_area,    'm^2')
    print(f'  {"m":<33} = {MAX_TAKEOFF_MASS_KG:>10.4f} kg')
    print(f'  {"alpha_max  [fixed input]":<33} = {ALPHA_MAX_DEG:>10.4f} deg')
    print_result('Input: CL_alpha (endplate, wing-alone)', wing_cl_alpha, '/rad')

    if not any(isinstance(v, str) for v in (wing_cl_alpha, wing_area)):
        alpha_max_rad = ALPHA_MAX_DEG * np.pi / 180.0
        cl_calc   = wing_cl_alpha * alpha_max_rad
        lift_calc = cl_calc * CONSTRAINT_Q_PA * wing_area
        nz_calc   = lift_calc / (MAX_TAKEOFF_MASS_KG * 9.80665)
        print()
        print_result('CL = CL_alpha x alpha_max',          cl_calc,   '')
        print_result('CL (OpenMDAO)',                   nz_cl_om,  '')
        print_result('Lift = CL x q x S (reproduced)', lift_calc, 'N')
        print_result('Lift (OpenMDAO)',                 nz_lift_om,'N')
        print_result('Nz (reproduced)',                 nz_calc,   f'[min {NZ_MIN}]')
        print_result('Nz (OpenMDAO)',                   nz,        f'[min {NZ_MIN}]')


def apply_aircraft_mass_and_fuel_limits(prob):
    prob.aviary_inputs.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    # Disable Aviary's built-in excess_fuel_capacity constraint - it has zero
    # gradient w.r.t. our DVs (fuel tank size is fixed), which makes SLSQP's
    # LSQ subproblem singular.  FUEL_BUDGET_MARGIN covers the same check.
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.IGNORE_FUEL_CAPACITY_CONSTRAINT, True)
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.TOTAL_CAPACITY, FUEL_CAPACITY_KG, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.WING_FUEL_CAPACITY, 0.0, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.FUSELAGE_FUEL_CAPACITY, FUEL_CAPACITY_KG, 'kg')


class FuselageExposedWettedAreaComp(om.ExplicitComponent):
    """Fuselage exposed wetted area after wing (and optional tail) airfoil cutouts.

    The wing exits the fuselage skin on each side (left and right), punching a hole
    whose shape is the airfoil cross-section at the local chord.  The area of that
    hole is A = K * (t/c) * c_fus², where K is the airfoil cross-section area
    coefficient (= 0.6843 for NACA 4-digit series, analytically integrated from the
    standard thickness distribution).

    The chord at the fuselage wall accounts for taper:
        c_fus = c_root * (1 - (1 - taper) * d_fus / b)
    where d_fus = fuselage max width and b = wing span.

    A symmetrical hook for a fuselage-mounted tail is included (htail_c_root_at_fus
    and htail_tc_at_fus default to 0 -- set when a fuselage tail is implemented).

    Formula
    -------
    c_fus  = wing_root_chord * (1 - (1 - taper_ratio) * fus_max_width / wing_span)
    Swet_fus_exposed = fuselage_wetted_area
                     - 2 * K_wing  * wing_tc  * c_fus**2
                     - 2 * K_htail * htail_tc * htail_c_root_at_fus**2

    Inputs
    ------
    fuselage_wetted_area  : m²   Gross outer surface from SuperellipseFuselageGeometry
    wing_root_chord       : m    Wing root chord (from WingSurface)
    wing_section_tc       : –    Wing t/c ratio (design variable)
    fus_max_width         : m    Fuselage max width at wing station
    wing_span             : m    Full wing span (Aircraft.Wing.SPAN, design variable)
    wing_taper_ratio      : –    Wing taper ratio (Aircraft.Wing.TAPER_RATIO)
    htail_c_root_at_fus   : m    H-tail root chord at fuselage wall (default 0; future use)
    htail_tc_at_fus       : –    H-tail t/c at fuselage wall (default 0; future use)

    Outputs
    -------
    fuselage_exposed_wetted_area : m²   Net fuselage wetted area after cutouts

    Options
    -------
    K_wing  : float  Airfoil cross-section area coeff for wing  (default 0.6843)
    K_htail : float  Airfoil cross-section area coeff for h-tail (default 0.6843)

    Numerical example — UAV baseline (b=1.8 m, AR=7.2, taper=0.8, t/c=0.15, d_fus=0.155 m)
    -----------------------------------------------------------------------------------------
    c_root = 2 * S_wing / (b * (1 + taper)) = 2 * 0.45 / (1.8 * 1.8) = 0.2778 m
    c_fus  = 0.2778 * (1 - 0.2 * 0.155 / 1.8)  = 0.2778 * 0.9828 = 0.2730 m
    A_hole = 0.6843 * 0.15 * 0.2730**2          = 0.00765 m²
    wing cutout (×2)                             = 0.01530 m²
    Swet_fus_exposed = Swet_fus_gross - 0.0153 m²
    """

    def initialize(self):
        self.options.declare('K_wing',  default=0.6843, types=float)
        self.options.declare('K_htail', default=0.6843, types=float)

    def setup(self):
        self.add_input('fuselage_wetted_area',  val=0.60,  units='m**2')
        self.add_input('wing_root_chord',        val=0.278, units='m')
        self.add_input('wing_section_tc',        val=0.15,  units='unitless')
        self.add_input('fus_max_width',          val=0.155, units='m')
        self.add_input('wing_span',              val=1.8,   units='m')
        self.add_input('wing_taper_ratio',       val=0.8,   units='unitless')
        self.add_input('htail_c_root_at_fus',    val=0.0,   units='m')
        self.add_input('htail_tc_at_fus',        val=0.0,   units='unitless')

        self.add_output('fuselage_exposed_wetted_area', val=0.585, units='m**2')

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        K_w = self.options['K_wing']
        K_h = self.options['K_htail']

        c_root  = inputs['wing_root_chord']
        tc      = inputs['wing_section_tc']
        d_fus   = inputs['fus_max_width']
        b       = inputs['wing_span']
        taper   = inputs['wing_taper_ratio']
        c_htail = inputs['htail_c_root_at_fus']
        tc_h    = inputs['htail_tc_at_fus']

        c_fus   = c_root * (1.0 - (1.0 - taper) * d_fus / b)

        wing_cutout  = 2.0 * K_w * tc   * c_fus**2
        htail_cutout = 2.0 * K_h * tc_h * c_htail**2

        outputs['fuselage_exposed_wetted_area'] = (
            inputs['fuselage_wetted_area'] - wing_cutout - htail_cutout
        )


def add_load_factor_subsystems(prob):
    """Wire H-tail geometry, lift-slope, and Ny/Nz constraint components."""
    model = prob.model

    # ── Fixed design-point inputs (flight condition + rudder geometry) ────────
    fixed = om.IndepVarComp()
    fixed.add_output('delta_r_deg',      val=RUDDER_DELTA_DEG,    units='deg')
    fixed.add_output('beta_deg',         val=RUDDER_DELTA_DEG,    units='deg')
    fixed.add_output('dynamic_pressure', val=CONSTRAINT_Q_PA,     units='Pa')
    fixed.add_output('aircraft_mass',    val=MAX_TAKEOFF_MASS_KG,  units='kg')
    fixed.add_output('rudder_cf_c',      val=RUDDER_CF_C,         units='unitless')
    fixed.add_output('rudder_eta_root',  val=RUDDER_ETA_ROOT,     units='unitless')
    fixed.add_output('rudder_eta_tip',   val=RUDDER_ETA_TIP,      units='unitless')
    fixed.add_output('fus_nose_frac',    val=FUSELAGE_NOSE_LENGTH_FRACTION, units='unitless')
    fixed.add_output('fus_tail_frac',    val=FUSELAGE_TAIL_LENGTH_FRACTION, units='unitless')
    fixed.add_output('fus_max_width',    val=FUSELAGE_MAX_WIDTH_M,  units='m')
    fixed.add_output('fus_max_height',   val=FUSELAGE_MAX_HEIGHT_M, units='m')
    fixed.add_output('fus_base_width_frac',  val=FUSELAGE_BASE_WIDTH_FRACTION,  units='unitless')
    fixed.add_output('fus_base_height_frac', val=FUSELAGE_BASE_HEIGHT_FRACTION, units='unitless')
    fixed.add_output('fus_superellipse_exp', val=FUSELAGE_SUPERELLIPSE_EXPONENT, units='unitless')
    fixed.add_output('design_mach',      val=CONSTRAINT_MACH,     units='unitless',
                     desc='Mach at the constraint flight condition -- shared by all surface Polhamus instances')
    fixed.add_output('constraint_static_pressure', val=CRUISE_STATIC_PRESSURE_PA, units='Pa',
                     desc='Static pressure at the constraint flight condition')
    fixed.add_output('constraint_temperature',     val=CRUISE_TEMPERATURE_K,      units='K',
                     desc='Static temperature at the constraint flight condition')
    fixed.add_output('alpha_max_deg',    val=ALPHA_MAX_DEG,       units='deg',
                     desc='Max operating alpha [deg] -- used by Nz (LongitudinalLoadFactor)')
    fixed.add_output('mach_upper_bound', val=MACH_UPPER_BOUND,    units='unitless',
                     desc='M_crit check Mach: DASH_MACH + M_CRIT_SAFETY_MARGIN; constraint M_crit >= this value')
    fixed.add_output('nz_min',           val=NZ_MIN,              units='unitless',
                     desc='Min longitudinal load factor -- used by MachCriticalComp to derive design CL')
    # ── Wing apex position in the aircraft reference frame ────────────────────
    # Frame: origin at nose tip; x positive AFT (fuselage station); y positive
    # starboard; z positive DOWN.  These are geometric inputs -- promote to DVs
    # when the CG / static-margin loop is added.
    fixed.add_output('wing_x_apex', val=WING_X_APEX_M, units='m',
                     desc='Wing root LE x-station from nose (positive aft) [m]')
    fixed.add_output('wing_z_apex', val=WING_Z_APEX_M, units='m',
                     desc='Wing root LE z-station from nose datum (positive down) [m]')
    model.add_subsystem('load_cond', fixed, promotes_outputs=['*'])

    # ── H-tail VTP geometry: span/chord -> area, AR, fuselage_vtp_span_ratio ──
    model.add_subsystem('htail_geom', HTailGeometry(), promotes=['*'])

    # VTP wetted area: 2 panels × 2 sides, wingtip-mounted (no fuselage cutout).
    # Formula matches project Swet convention: (S_planform - S_inside_fuselage) × 2
    # per panel, × 2 panels. Feeds trajectory parameter 'vtp_wetted_area' so that
    # VTP span DV changes propagate to mission parasite drag.
    model.add_subsystem(
        'vtp_wetted_area_comp',
        om.ExecComp(
            'vtp_wetted_area = 4.0 * vtp_area',
            vtp_area={'val': 0.08, 'units': 'm**2'},
            vtp_wetted_area={'val': 0.32, 'units': 'm**2'},
        ),
        promotes=['*'],
    )

    # ── Parametric fuselage geometry: report-only until drag/CG integration ──
    model.add_subsystem(
        'superellipse_fuselage',
        SuperellipseFuselageGeometry(),
        promotes_inputs=[
            ('fuselage_length', av.Aircraft.Fuselage.LENGTH),
            ('max_width', 'fus_max_width'),
            ('max_height', 'fus_max_height'),
            ('nose_length_fraction', 'fus_nose_frac'),
            ('tail_length_fraction', 'fus_tail_frac'),
            ('base_width_fraction', 'fus_base_width_frac'),
            ('base_height_fraction', 'fus_base_height_frac'),
            ('superellipse_exponent', 'fus_superellipse_exp'),
        ],
        promotes_outputs=[
            'fuselage_planform_area',
            'fuselage_base_area',
            'fuselage_base_diameter',
            'fuselage_wetted_area',
            'fuselage_equivalent_diameter',
            'fuselage_fineness_ratio',
            'fuselage_max_cross_section_area',
            'fuselage_volume',
            'fuselage_centroid_x',
        ],
    )

    # ── Wing surface: Scholz AR correction + Polhamus + M_crit ──────────────
    model.add_subsystem(
        'wing_surface', WingSurface(cfg=WING_SURFACE_CFG),
        promotes_inputs=[
            ('surface_ar',       av.Aircraft.Wing.ASPECT_RATIO),
            ('surface_span',     av.Aircraft.Wing.SPAN),
            ('surface_sweep_c4', av.Aircraft.Wing.SWEEP),
            ('surface_taper',    av.Aircraft.Wing.TAPER_RATIO),
            ('surface_area',     av.Aircraft.Wing.AREA),
            ('dihedral_deg',     'aircraft:wing:dihedral'),
            ('endplate_span',    av.Aircraft.VerticalTail.SPAN),
            'design_mach',
            'mach_upper_bound',
            'nz_min',
            'aircraft_mass',
            'dynamic_pressure',
            'wing_x_apex',
            'wing_z_apex',
        ],
        promotes_outputs=[
            ('surface_CL_alpha', 'wing_CL_alpha'),
            ('k_eff',            'k_h'),
            'AR_eff',
            'M_DD',
            'M_crit',
            'mach_crit_margin',
            ('section_tc', 'wing_section_tc'),
            'wing_root_chord',
            'wing_c_mac',
            'wing_y_mac',
            'wing_x_mac_le',
            'wing_z_mac_le',
            'wing_x_mac_c4',
            'wing_le_sweep',
        ],
    )

    # ── Fuselage exposed wetted area: gross Swet minus wing airfoil cutouts ───
    # wing_root_chord and wing_section_tc are now available (promoted by wing_surface).
    model.add_subsystem(
        'fus_exposed_swet',
        FuselageExposedWettedAreaComp(
            K_wing=WING_AIRFOIL.cross_section_area_coeff,
        ),
        promotes_inputs=[
            'fuselage_wetted_area',
            'wing_root_chord',
            'wing_section_tc',
            ('fus_max_width',    'fus_max_width'),
            ('wing_span',        av.Aircraft.Wing.SPAN),
            ('wing_taper_ratio', av.Aircraft.Wing.TAPER_RATIO),
        ],
        promotes_outputs=['fuselage_exposed_wetted_area'],
    )

    # ── VTP surface: Polhamus + CyBetaVtp + CyDeltaRudder ────────────────────
    model.add_subsystem(
        'vtp_surface', VTPSurface(cfg=VTP_SURFACE_CFG),
        promotes_inputs=[
            ('surface_ar',       'vtp_ar'),                        # from htail_geom
            ('surface_area',     'vtp_area'),                      # from htail_geom
            ('surface_sweep_c4', av.Aircraft.VerticalTail.SWEEP),
            ('surface_taper',    av.Aircraft.VerticalTail.TAPER_RATIO),
            'design_mach',
            'fuselage_vtp_span_ratio',
            'wing_ref_area',
            av.Aircraft.HorizontalTail.AREA,
            av.Aircraft.HorizontalTail.ASPECT_RATIO,
            av.Aircraft.Fuselage.LENGTH,
            av.Aircraft.VerticalTail.TAPER_RATIO,
            av.Aircraft.VerticalTail.THICKNESS_TO_CHORD,
            'rudder_cf_c', 'rudder_eta_root', 'rudder_eta_tip', 'delta_r_deg',
        ],
        promotes_outputs=[
            ('surface_CL_alpha', 'CL_alpha_v'),
            'CY_beta_vtp',
            'CY_delta_r',
        ],
    )

    # ── Lateral load factor -> Ny ──────────────────────────────────────────────
    model.add_subsystem('lat_load', LateralLoadFactor(), promotes=['*'])

    # ── Longitudinal load factor -> Nz ────────────────────────────────────────
    model.add_subsystem(
        'long_load', LongitudinalLoadFactor(),
        promotes_inputs=[
            ('CL_alpha',      'wing_CL_alpha'),
            'dynamic_pressure',
            'wing_ref_area',
            'aircraft_mass',
            'alpha_max_deg',
        ],
        promotes_outputs=['CL', 'lift', 'Nz'],
    )

    # ── Wing induced drag -- Roskam Part VI Eq. 4.8 (no twist) ───────────────
    # CDi = CL^2 / (pi * AR_eff * e), with no hidden 1.05 trim multiplier.
    # e comes from Roskam Eq. 4.12 using the leading-edge suction parameter.
    # AR_eff must be used here -- NOT the geometric AR.
    model.add_subsystem(
        'roskam_cdi',
        RoskamInducedDragComp(compute_leading_edge_suction=True),
        promotes_inputs=[
            'CL',
            ('AR_eff',     'AR_eff'),
            ('CL_alpha_w', 'wing_CL_alpha'),
            ('mach', 'design_mach'),
            ('static_pressure', 'constraint_static_pressure'),
            ('temperature', 'constraint_temperature'),
            ('mean_aerodynamic_chord', 'wing_c_mac'),
            ('thickness_to_chord', 'wing_section_tc'),
            ('leading_edge_sweep', 'wing_le_sweep'),
            ('taper_ratio', av.Aircraft.Wing.TAPER_RATIO),
        ],
        promotes_outputs=[
            'CDi',
            'e_oswald',
            ('leading_edge_radius', 'wing_le_radius'),
            ('leading_edge_reynolds_number', 'wing_Re_LER'),
            ('leading_edge_suction_parameter', 'wing_le_suction_parameter'),
        ],
    )

    # ── Fuselage drag due to lift -- Roskam Part VI Eq. 4.33, report-only ────
    model.add_subsystem(
        'fuselage_cdi',
        FuselageLiftInducedDragComp(),
        promotes_inputs=[
            ('aircraft_alpha', 'alpha_max_deg'),
            'fuselage_base_area',
            'fuselage_planform_area',
            ('reference_area', av.Aircraft.Wing.AREA),
            'fuselage_fineness_ratio',
            ('Mach', 'design_mach'),
        ],
        promotes_outputs=[
            'CDi_fus',
            'CDi_fus_base_area_term',
            'CDi_fus_planform_term',
            'eta_finite_cylinder',
            'crossflow_drag_coefficient',
        ],
    )

    # ── Wire Roskam CDi into FLOPS mission polar via span efficiency factor ────
    # FLOPS InducedDrag formula: CDi = CL^2 / (pi * AR_geo * e_span_eff)
    # With e_span_eff = e_oswald * (AR_eff / AR_geo):
    #   CDi_flops = CL^2 / (pi * AR_eff * e_oswald)  =  Roskam Eq. 4.8  ✓
    # e_oswald comes from RoskamInducedDragComp (Eq. 4.12); AR_eff from Scholz correction.
    # aircraft:wing:span_efficiency_factor is NOT a pre-mission computed output --
    # it is safe to promote our output to this name without a connection conflict.
    model.add_subsystem(
        'span_eff_correction',
        om.ExecComp(
            'e_span_eff = e_oswald * AR_eff / AR_geo',
            e_oswald={'val': 0.85, 'units': 'unitless'},
            AR_eff={'val': 8.0, 'units': 'unitless'},
            AR_geo={'val': 7.0, 'units': 'unitless'},
            e_span_eff={'val': 0.97, 'units': 'unitless'},
        ),
        promotes_inputs=[
            'e_oswald',
            'AR_eff',
            ('AR_geo', av.Aircraft.Wing.ASPECT_RATIO),
        ],
        promotes_outputs=[
            ('e_span_eff', av.Aircraft.Wing.SPAN_EFFICIENCY_FACTOR),
        ],
    )


def add_aeroelasticity_subsystems(prob):
    """Add the aeroelasticity module and the aeroelastic AR correction to the model.

    The AeroelasticityGroup is added directly at model scope (not inside pre_mission)
    so it can receive wing_CL_alpha from wing_surface without creating an execution-
    order cycle through the pre_mission group.

    Implements TOOD.md Aeroelasticity steps 3 (constraints wired in build_problem),
    4 (AR_eff wash-in correction diagnostic), and 5 (input connections).
    """
    model = prob.model

    # ── Step 5: connect wing geometry and CL_alpha ────────────────────────────
    # Aircraft.Wing.* and Aircraft.VerticalTail.* auto-connect via promotes=['*']
    # from Aviary pre_mission. Only wing_CL_alpha needs an explicit connection
    # because it is computed outside pre_mission by add_load_factor_subsystems.
    model.add_subsystem(
        'aeroelasticity',
        AeroelasticityGroup(material_name='aluminum_6061_t6'),
        promotes_inputs=['*'],
        promotes_outputs=['*'],
    )
    model.connect('wing_CL_alpha', AE.LIFT_CURVE_SLOPE)
    model.connect('wing_section_tc', AE.STRUCTURAL_THICKNESS_TO_CHORD)
    model.connect(premission_propulsion_var(SmallTurbojetVariables.MASS), AE.ENGINE_MASS)
    model.connect(av.Mission.TOTAL_FUEL, AE.FUEL_MASS)

    # ── Step 4: aeroelastic AR wash-in correction (diagnostic, not optimizer DV)
    # Below divergence the wing twists nose-up (wash-in since EA is aft of AC),
    # amplifying loads by 1/(1-q/q_div).  For induced drag the effective AR
    # degrades by the same factor:
    #   AR_eff_ae = AR_eff * (1 - q_design / q_div)
    # Reported as a measure of aeroelastic flexibility.  If AR_eff_ae deviates
    # significantly from AR_eff, the CDi and range calculation should account for it.
    model.add_subsystem(
        'ae_ar_correction',
        om.ExecComp(
            'AR_eff_ae = AR_eff * fmax(1.0 - q_design / q_div, 0.0)',
            AR_eff={'val': 8.0, 'units': 'unitless'},
            q_design={'val': CONSTRAINT_Q_PA, 'units': 'Pa'},
            q_div={'val': 1.0e6, 'units': 'Pa'},
            AR_eff_ae={'val': 8.0, 'units': 'unitless'},
        ),
        promotes_inputs=[
            'AR_eff',
            ('q_design', 'dynamic_pressure'),
            ('q_div', AE.DIVERGENCE_DYNAMIC_PRESSURE),
        ],
        promotes_outputs=['AR_eff_ae'],
    )


def add_fuel_budget_constraint(prob):
    prob.model.add_subsystem(
        'fuel_budget_estimate',
        FuelBudgetEstimate(),
        promotes_inputs=[
            av.Aircraft.Design.GROSS_MASS,
            av.Aircraft.Design.EMPTY_MASS,
            av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
            av.Mission.TOTAL_FUEL,
        ],
        promotes_outputs=[AVAILABLE_FUEL, FUEL_BUDGET_MARGIN],
    )
    prob.model.connect(
        premission_propulsion_var(SmallTurbojetVariables.MASS),
        'fuel_budget_estimate.engine_mass',
    )


def write_payload_range_report(prob):
    reports_dir = Path(prob.get_reports_dir(force=True))
    csv_path = reports_dir / 'payload_range_data.csv'

    payload_kg = safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 'kg')
    fuel_kg = safe_get(prob, av.Mission.TOTAL_FUEL, 'kg')
    range_km = safe_get(prob, av.Mission.RANGE, 'km')

    if any(isinstance(value, str) for value in (payload_kg, fuel_kg, range_km)):
        return None

    rows = [
        {
            'Mission Name': 'Zero Range',
            'Payload (kg)': payload_kg,
            'Fuel (kg)': 0.0,
            'Range (km)': 0.0,
        },
        {
            'Mission Name': 'Fallout Mission',
            'Payload (kg)': payload_kg,
            'Fuel (kg)': fuel_kg,
            'Range (km)': range_km,
        },
    ]

    with csv_path.open('w', newline='') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                'Mission Name',
                'Payload (kg)',
                'Fuel (kg)',
                'Range (km)',
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    return csv_path


def write_spajeti_aircraft_3d_report(prob):
    """Write a lightweight SpaJeti-specific 3D geometry report for the dashboard."""
    reports_dir = Path(prob.get_reports_dir(force=True))
    html_path = reports_dir / 'spajeti_aircraft_3d.html'
    subsystems_dir = reports_dir / 'subsystems'
    subsystem_link_path = subsystems_dir / 'spajeti_3d.md'

    def _num(value, default):
        return default if isinstance(value, str) else float(np.atleast_1d(value)[0])

    fus_len = _num(safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm'), 2.0)
    fus_w = _num(safe_get(prob, 'fus_max_width', 'm'), FUSELAGE_MAX_WIDTH_M)
    fus_h = _num(safe_get(prob, 'fus_max_height', 'm'), FUSELAGE_MAX_HEIGHT_M)

    wing_span = _num(safe_get(prob, av.Aircraft.Wing.SPAN, 'm'), 1.8)
    wing_area = _num(safe_get(prob, av.Aircraft.Wing.AREA, 'm**2'), 0.45)
    wing_taper = _num(safe_get(prob, av.Aircraft.Wing.TAPER_RATIO), 0.6)
    wing_sweep = math.radians(_num(safe_get(prob, av.Aircraft.Wing.SWEEP, 'deg'), 0.0))
    wing_dihedral = math.radians(_num(safe_get(prob, av.Aircraft.Wing.DIHEDRAL, 'deg'), 0.0))
    wing_cr = 2.0 * wing_area / (wing_span * (1.0 + wing_taper))
    wing_ct = wing_cr * wing_taper
    wing_semispan = 0.5 * wing_span

    vtp_span = _num(safe_get(prob, av.Aircraft.VerticalTail.SPAN, 'm'), VTP_SPAN_INITIAL_M)
    vtp_area = _num(safe_get(prob, av.Aircraft.VerticalTail.AREA, 'm**2'), 0.08)
    vtp_taper = _num(safe_get(prob, av.Aircraft.VerticalTail.TAPER_RATIO), 0.4)
    vtp_sweep = math.radians(_num(safe_get(prob, av.Aircraft.VerticalTail.SWEEP, 'deg'), 20.0))
    vtp_cr = 2.0 * vtp_area / (vtp_span * (1.0 + vtp_taper))
    vtp_ct = vtp_cr * vtp_taper

    engine_diameter = _num(
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.DIAMETER), 'm'),
        0.12,
    )
    engine_length = 0.35 * engine_diameter

    def p(x_aft, y_right, z_down):
        # A-Frame: x = aircraft aft/forward, y = up, z = right.
        return f'{x_aft - 0.5 * fus_len:.5f} {-z_down:.5f} {y_right:.5f}'

    def tri(a, b, c, color, opacity='0.82'):
        return (
            f'<a-triangle vertex-a="{a}" vertex-b="{b}" vertex-c="{c}" '
            f'color="{color}" opacity="{opacity}" side="double"></a-triangle>'
        )

    def quad(a, b, c, d, color, opacity='0.82'):
        return tri(a, b, c, color, opacity) + '\n' + tri(a, c, d, color, opacity)

    def fuselage_entities(n_x=18, n_t=24):
        x_norm = np.linspace(0.0, 1.0, n_x)
        widths, heights = superellipse_width_height_distribution(
            x_norm,
            fus_w,
            fus_h,
            FUSELAGE_NOSE_LENGTH_FRACTION,
            FUSELAGE_TAIL_LENGTH_FRACTION,
            FUSELAGE_BASE_WIDTH_FRACTION,
            FUSELAGE_BASE_HEIGHT_FRACTION,
        )
        theta = np.linspace(0.0, 2.0 * np.pi, n_t, endpoint=False)
        rings = []
        for x_frac, width, height in zip(x_norm, widths, heights):
            y_vals, z_vals = _superellipse_surface_points(
                width,
                height,
                FUSELAGE_SUPERELLIPSE_EXPONENT,
                theta,
            )
            rings.append([p(x_frac * fus_len, y, z_down) for y, z_down in zip(y_vals, z_vals)])

        faces = []
        for i in range(n_x - 1):
            for j in range(n_t):
                jp = (j + 1) % n_t
                faces.append(quad(
                    rings[i][j],
                    rings[i + 1][j],
                    rings[i + 1][jp],
                    rings[i][jp],
                    '#c9ced6',
                    '0.92',
                ))
        return '\n'.join(faces)

    def wing_panel(sign):
        y0 = 0.0
        yt = sign * wing_semispan
        x_le_root = WING_X_APEX_M
        z_root = WING_Z_APEX_M
        x_le_tip = x_le_root + 0.25 * wing_cr + abs(yt) * math.tan(wing_sweep) - 0.25 * wing_ct
        z_tip = z_root - abs(yt) * math.tan(wing_dihedral)
        return quad(
            p(x_le_root, y0, z_root),
            p(x_le_root + wing_cr, y0, z_root),
            p(x_le_tip + wing_ct, yt, z_tip),
            p(x_le_tip, yt, z_tip),
            '#3b82f6',
        )

    def vtp_panel(y_sign, z_sign):
        y = y_sign * wing_semispan
        x_le_root = WING_X_APEX_M + 0.25 * wing_cr + wing_semispan * math.tan(wing_sweep) - 0.25 * wing_ct
        z_mid = WING_Z_APEX_M - wing_semispan * math.tan(wing_dihedral)
        z_tip = z_mid + z_sign * 0.5 * vtp_span
        x_le_tip = x_le_root + 0.25 * vtp_cr + 0.5 * vtp_span * math.tan(vtp_sweep) - 0.25 * vtp_ct
        return quad(
            p(x_le_root, y, z_mid),
            p(x_le_root + vtp_cr, y, z_mid),
            p(x_le_tip + vtp_ct, y, z_tip),
            p(x_le_tip, y, z_tip),
            '#0f766e',
            '0.88',
        )

    entities = '\n'.join([
        fuselage_entities(),
        wing_panel(1.0),
        wing_panel(-1.0),
        vtp_panel(1.0, 1.0),
        vtp_panel(1.0, -1.0),
        vtp_panel(-1.0, 1.0),
        vtp_panel(-1.0, -1.0),
        '<a-cylinder radius="{:.5f}" height="{:.5f}" rotation="0 0 90" '
        'position="{}" color="#111827" opacity="0.90" segments-radial="32"></a-cylinder>'.format(
            0.42 * engine_diameter,
            engine_length,
            p(fus_len + 0.5 * engine_length, 0.0, 0.0),
        ),
    ])

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>SpaJeti 3D Geometry</title>
  <script src="https://aframe.io/releases/1.5.0/aframe.min.js"></script>
  <style>
    body {{ margin: 0; background: #eef3f8; font-family: Arial, sans-serif; }}
    .note {{
      position: fixed; left: 16px; top: 12px; z-index: 10;
      background: rgba(255,255,255,0.88); padding: 10px 12px; border-radius: 6px;
      color: #1f2937; font-size: 13px; line-height: 1.35;
    }}
    .fallback {{
      position: fixed; right: 16px; bottom: 16px; z-index: 10;
      width: 300px; max-width: 34vw;
      background: rgba(255,255,255,0.88); padding: 8px; border-radius: 6px;
      color: #1f2937; font-size: 12px;
    }}
    .fallback svg {{ display: block; width: 100%; height: auto; }}
  </style>
</head>
<body>
  <div class="note">
    <b>SpaJeti optimized geometry</b><br>
    b = {wing_span:.3f} m, VTP span = {vtp_span:.3f} m, fuselage = {fus_len:.3f} m<br>
    Fuselage uses the rounded-square superellipse mesh. VTPs mirror up/down from each wingtip.
    Dark aft disk is the small turbojet nozzle.
  </div>
  <div class="fallback">
    <b>Plan view fallback</b>
    <svg viewBox="0 0 300 150" aria-label="SpaJeti H-wing planform fallback">
      <rect x="0" y="0" width="300" height="150" fill="#dbeafe"/>
      <ellipse cx="150" cy="75" rx="58" ry="20" fill="#c9ced6" stroke="#94a3b8"/>
      <polygon points="90,70 210,70 238,62 238,68 210,78 90,78 62,68 62,62" fill="#3b82f6" opacity="0.82"/>
      <polygon points="56,51 68,51 74,75 68,99 56,99 50,75" fill="#0f766e" opacity="0.88"/>
      <polygon points="232,51 244,51 250,75 244,99 232,99 226,75" fill="#0f766e" opacity="0.88"/>
      <circle cx="211" cy="75" r="5" fill="#111827" opacity="0.90"/>
    </svg>
  </div>
  <a-scene background="color: #dbeafe">
    <a-camera position="0 1.8 7.5" rotation="-12 0 0"></a-camera>
    <a-entity light="type: ambient; intensity: 0.62"></a-entity>
    <a-entity light="type: directional; intensity: 0.72" position="-2 4 3"></a-entity>
    <a-entity id="aircraft" rotation="0 -35 0">
{entities}
    </a-entity>
    <a-grid color="#94a3b8" opacity="0.35" position="0 -0.45 0"></a-grid>
  </a-scene>
</body>
</html>
"""
    html_path.write_text(html, encoding='utf-8')
    subsystems_dir.mkdir(parents=True, exist_ok=True)
    subsystem_link_path.write_text(
        '# SpaJeti 3D Geometry\n\n'
        'The SpaJeti H-wing geometry report is also available as a standalone '
        'interactive HTML file:\n\n'
        '[Open SpaJeti 3D Geometry](../spajeti_aircraft_3d.html)\n\n'
        'If the dashboard was launched with an older Aviary entry point and the '
        'Results tab is missing, use this subsystem report link or open '
        '`reports/spajeti_aircraft_3d.html` directly.\n',
        encoding='utf-8',
    )
    return html_path


def add_cg_estimation(prob, cg_est):
    """Add the CG estimator (or bypass IVC) to the model and wire live masses."""
    model = prob.model

    # Variable masses that live at Aviary model scope: promote so they auto-connect
    model.add_subsystem(
        'cg_est', cg_est,
        promotes_inputs=[
            av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
            av.Mission.TOTAL_FUEL,
        ],
        promotes_outputs=['x_cg', 'y_cg', 'z_cg', 'total_mass'],
    )

    if not cg_est.bypass:
        # Engine mass lives inside pre_mission.propulsion -- needs explicit connect
        prob.model.connect(
            premission_propulsion_var(SmallTurbojetVariables.MASS),
            'cg_est.cg_engine_mass',
        )


def print_cg_detail(prob, cg_est):
    """Print component CG breakdown table."""
    x_cg       = safe_get(prob, 'x_cg',       'm')
    y_cg       = safe_get(prob, 'y_cg',       'm')
    z_cg       = safe_get(prob, 'z_cg',       'm')
    total_mass = safe_get(prob, 'total_mass', 'kg')

    print('\n' + '=' * 70)
    print('CG ESTIMATION')
    if cg_est.bypass:
        print('  (BYPASS MODE -- manual values)')
    print('=' * 70)

    if cg_est.bypass:
        print_result('x_cg  [manual]',   x_cg,       'm')
        print_result('y_cg  [manual]',   y_cg,       'm')
        print_result('z_cg  [manual]',   z_cg,       'm')
        print_result('total_mass [manual]', total_mass, 'kg')
        return

    print(f'\n  {"Component":<20} {"Mass [kg]":>10} {"x [m]":>8} {"y [m]":>8} {"z [m]":>8} {"x*m":>10}')
    print('  ' + '-' * 66)
    sum_mass = 0.0
    sum_xm   = 0.0
    sum_ym   = 0.0
    sum_zm   = 0.0
    for name, mass_spec, x_def, y_def, z_def in cg_est.components:
        m_val = safe_get(prob, f'cg_est.cg_compute.{name}_mass', 'kg')
        x_val = safe_get(prob, f'cg_est.cg_compute.{name}_x',    'm')
        y_val = safe_get(prob, f'cg_est.cg_compute.{name}_y',    'm')
        z_val = safe_get(prob, f'cg_est.cg_compute.{name}_z',    'm')
        if any(isinstance(v, str) for v in (m_val, x_val, y_val, z_val)):
            print(f'  {name:<20}  (not available)')
            continue
        xm = m_val * x_val
        sum_mass += m_val
        sum_xm   += xm
        sum_ym   += m_val * y_val
        sum_zm   += m_val * z_val
        var_tag = '*' if isinstance(mass_spec, str) else ' '
        print(f'  {name:<20}{var_tag}{m_val:>10.3f} {x_val:>8.3f} {y_val:>8.3f} {z_val:>8.3f} {xm:>10.4f}')
    print('  ' + '-' * 66)
    print(f'  {"TOTAL":<21}{sum_mass:>10.3f}', end='')
    if sum_mass > 0:
        xcg = sum_xm / sum_mass
        ycg = sum_ym / sum_mass
        zcg = sum_zm / sum_mass
        print(f'                              {sum_xm:>10.4f}')
        print()
        print(f'  * mass from OpenMDAO variable (live -- updates with optimizer)')
    else:
        print()
    print()
    print_result('x_cg  (from OpenMDAO)', x_cg,       'm')
    print_result('y_cg  (from OpenMDAO)', y_cg,       'm')
    print_result('z_cg  (from OpenMDAO)', z_cg,       'm')
    print_result('total_mass (from OpenMDAO)', total_mass, 'kg')
    # Cross-check against gross mass
    gross = safe_get(prob, av.Aircraft.Design.GROSS_MASS, 'kg')
    if not any(isinstance(v, str) for v in (total_mass, gross)):
        delta = total_mass - gross
        print_result('  delta vs gross mass', delta, 'kg')
        note = ('OK' if abs(delta) < 1.0 else
                'WARNING: >1 kg delta -- update component mass estimates')
        print(f'  {"  note":<33}   {note}')


def remove_dashboard_incompatible_recorder(prob):
    """Avoid a dashboard crash on custom promoted design-variable metadata."""
    opt_history_path = Path(prob.get_outputs_dir()) / 'optimization_history.db'
    if opt_history_path.exists():
        opt_history_path.unlink()


def build_cg_estimator():
    """Construct the CGEstimatorGroup for the SpaJeti H-wing UAV.

    Mass estimates labelled [PLACEHOLDER] will be replaced when the FLOPS
    component mass breakdown is exposed (see TOOD.md weight estimation).
    Positions are x-stations in the aircraft frame (origin nose, positive aft).
    """
    cg_est = CGEstimatorGroup(
        bypass=CG_BYPASS,
        x_cg_manual=CG_X_MANUAL_M,
        y_cg_manual=CG_Y_MANUAL_M,
        z_cg_manual=CG_Z_MANUAL_M,
        total_mass_manual=CG_MASS_MANUAL_KG,
    )

    # ── Fixed-mass structural components [PLACEHOLDER masses] ─────────────────
    # Replace with FLOPS component mass outputs when weight breakdown is added.
    cg_est.add_component('wing_struct',  mass=1.50, x=WING_X_APEX_M, y=0.0, z=WING_Z_APEX_M)
    cg_est.add_component('fuselage',     mass=1.00, x=1.00,          y=0.0, z=0.00)
    cg_est.add_component('empennage',    mass=0.25, x=1.85,          y=0.0, z=0.00)
    cg_est.add_component('vtp_pair',     mass=0.15, x=1.75,          y=0.0, z=-0.15)
    cg_est.add_component('landing_gear', mass=0.20, x=0.95,          y=0.0, z=0.15)
    cg_est.add_component('avionics',     mass=0.25, x=0.45,          y=0.0, z=0.00)

    # ── Variable-mass components (wired to optimizer / Aviary outputs) ─────────
    # 'cg_engine_mass'  -> connected in add_cg_estimation via prob.model.connect
    # av.Mission.TOTAL_FUEL and TOTAL_PAYLOAD_MASS -> promoted at call site
    cg_est.add_component('engine',  mass='cg_engine_mass',
                         x=1.55, y=0.0, z=0.00)
    cg_est.add_component('payload', mass=av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
                         x=0.85, y=0.0, z=0.05)
    cg_est.add_component('fuel',    mass=av.Mission.TOTAL_FUEL,
                         x=0.90, y=0.0, z=0.05)

    return cg_est


def build_problem():
    OUTPUT_ROOT.mkdir(exist_ok=True)
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    prob = av.AviaryProblem(
        problem_type=av.ProblemType.FALLOUT,
        verbosity=av.Verbosity.VERBOSE,
        name=VERSIONED_RUN_NAME,
        work_dir=OUTPUT_ROOT,
    )
    prob.load_inputs(aircraft_data=AIRCRAFT_DATA, phase_info=phase_info)
    prob.load_external_subsystems([
        SmallTurbojetModel(),
        RoskamAeroBuilder(h_wing_interference_factor=H_WING_INTERFERENCE_FACTOR),
    ])

    prob.check_and_preprocess_inputs()
    apply_aircraft_mass_and_fuel_limits(prob)

    gross_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.GROSS_MASS, units='kg')
    empty_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.EMPTY_MASS, units='kg')
    engine_mass_upper_kg = min(ENGINE_MASS_LIMIT_KG, gross_mass_kg - empty_mass_kg)

    cg_est = build_cg_estimator()

    prob.add_pre_mission_systems()
    add_load_factor_subsystems(prob)
    add_aeroelasticity_subsystems(prob)
    prob.add_phases()
    prob.add_post_mission_systems()
    add_fuel_budget_constraint(prob)
    add_cg_estimation(prob, cg_est)
    prob.link_phases()

    prob.add_driver(OPTIMIZER, max_iter=150, verbosity=av.Verbosity.VERBOSE)
    if OPTIMIZER == 'IPOPT':
        prob.driver.opt_settings['mu_strategy'] = 'adaptive'
        prob.driver.opt_settings['mu_init'] = 0.1
        prob.driver.opt_settings['nlp_scaling_method'] = 'none'
        prob.driver.opt_settings['acceptable_tol'] = 5e-3
        prob.driver.opt_settings['acceptable_iter'] = 5
        prob.driver.opt_settings['acceptable_constr_viol_tol'] = 1e-3
        prob.driver.opt_settings['acceptable_dual_inf_tol'] = 1.0

    # ── Design variables ──────────────────────────────────────────────────────
    prob.add_design_variables()   # engine DVs, phase Mach schedules, etc.
    prob.model.add_design_var(
        av.Aircraft.Wing.SPAN,
        lower=1.34, upper=2.0, units='m', ref=1.8,
    )
    prob.model.add_design_var(
        av.Aircraft.Wing.AREA,
        lower=0.40, upper=0.75, units='m**2', ref=0.45,
    )
    prob.model.add_design_var(
        av.Aircraft.VerticalTail.SPAN,
        lower=VTP_SPAN_LOWER_M, upper=VTP_SPAN_UPPER_M,
        units='m', ref=VTP_SPAN_INITIAL_M,
    )
    prob.model.add_design_var(
        'wing_section_tc',
        lower=WING_TC_LOWER, upper=WING_TC_UPPER, ref=WING_TC_INITIAL,
    )

    # ── Constraints ───────────────────────────────────────────────────────────
    prob.model.add_constraint(
        premission_propulsion_var(SmallTurbojetVariables.MASS),
        upper=engine_mass_upper_kg, units='kg', ref=engine_mass_upper_kg,
    )
    prob.model.add_constraint(
        FUEL_BUDGET_MARGIN,
        lower=0.0, units='kg', ref=FUEL_CAPACITY_KG,
    )
    prob.model.add_constraint('Ny', lower=NY_MIN, ref=NY_MIN)
    prob.model.add_constraint('Nz', lower=NZ_MIN, ref=NZ_MIN)
    prob.model.add_constraint(
        'mach_crit_margin',
        lower=0.0, ref=0.1,
    )
    prob.model.add_constraint(
        av.Aircraft.Engine.SCALED_SLS_THRUST,
        lower=TW_MIN * MAX_TAKEOFF_MASS_KG * 9.80665,
        units='N', ref=300.0,
    )
    # ── Aeroelastic constraints (TOOD.md Aeroelasticity step 3) ──────────────
    # Divergence: V_div >= V_dive = 1.25 * V_design (CS gradient via StaticAeroelastic)
    prob.model.add_constraint(
        AE.DIVERGENCE_SPEED_MARGIN, lower=0.0, units='m/s', ref=50.0,
    )
    # Flutter: smooth eigenvalue metric from QuasiSteadyFlutterScreen (exact CS
    # derivatives).  MAX_REAL_EIGENVALUE_AT_DESIGN <= 0 means the design-point
    # state matrix has no growing mode.  Flutter speed margin is reported only.
    prob.model.add_constraint(
        AE.MAX_REAL_EIGENVALUE_AT_DESIGN, upper=0.0, units='1/s', ref=1.0,
    )

    prob.add_objective()

    prob.setup()
    prob.set_initial_guesses()
    prob.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.VerticalTail.SPAN, VTP_SPAN_INITIAL_M, 'm')
    prob.set_val('wing_section_tc', WING_TC_INITIAL)

    # ── Aeroelasticity initial conditions ────────────────────────────────────
    # Override default 1.225 kg/m³ (sea level) with ISA 5 000 m density.
    prob.set_val(AE.AIR_DENSITY, ISA_5KM_DENSITY_KGM3, units='kg/m**3')
    # Override the group's default REQUIRED_SPEED (1.15 * V_design) with the
    # V_dive = 1.25 * V_design constraint per TOOD.md step 3.
    prob.set_val(AE.REQUIRED_SPEED, AERO_REQUIRED_SPEED_MS, units='m/s')

    return prob, engine_mass_upper_kg, cg_est


def print_parasite_drag_detail(prob):
    """Roskam/DATCOM parasite drag breakdown at the dash flight condition.

    Computed as a post-processing step using helper functions directly (no
    OpenMDAO solve overhead).  Not connected to the optimizer drag polar.
    """
    from aviary.subsystems.aerodynamics.aero_utils import (
        flat_plate_skin_friction_coeff,
        form_factor_datcom_body,
        form_factor_lifting_surface,
        lifting_surface_correction_factor,
        reynolds_number_from_mach,
        wing_fuselage_interference_factor,
    )

    print('\n' + '=' * 70)
    print('PARASITE DRAG BREAKDOWN (Roskam/DATCOM)')
    print(f'  Flight point  : M = {DASH_MACH:.3f},  '
          f'P = {CRUISE_STATIC_PRESSURE_PA:.0f} Pa,  '
          f'T = {CRUISE_TEMPERATURE_K:.2f} K  (ISA 5 000 m)')
    print(f'  R_h (H-wing)  : {H_WING_INTERFERENCE_FACTOR:.4f}  '
          f'(wing-VTP wingtip junction; applied to wing and VTP)')
    print(f'  R_wf_vtp      : 1.0  (VTP wingtip-mounted, no fuselage interference)')
    print(f'  Base drag (fus aft): EXCLUDED  (engine exhaust fills base)')
    print('=' * 70)

    wing_area       = safe_get(prob, av.Aircraft.Wing.AREA,                    'm**2')
    wing_c_mac      = safe_get(prob, 'wing_c_mac',                              'm')
    wing_root_chord = safe_get(prob, 'wing_root_chord',                         'm')
    wing_sweep      = safe_get(prob, av.Aircraft.Wing.SWEEP,                   'deg')
    wing_tc         = safe_get(prob, 'wing_section_tc')

    vtp_area        = safe_get(prob, 'vtp_area',                               'm**2')
    vtp_avg_chord   = safe_get(prob, 'vtp_avg_chord',                           'm')
    vtp_sweep       = safe_get(prob, av.Aircraft.VerticalTail.SWEEP,           'deg')
    vtp_tc          = safe_get(prob, av.Aircraft.VerticalTail.THICKNESS_TO_CHORD)

    fus_length      = safe_get(prob, av.Aircraft.Fuselage.LENGTH,              'm')
    fus_wetted      = safe_get(prob, 'fuselage_exposed_wetted_area',           'm**2')
    fus_equiv_diam  = safe_get(prob, 'fuselage_equivalent_diameter',           'm')

    _missing = {
        'wing_area':       wing_area,       'wing_c_mac':    wing_c_mac,
        'wing_root_chord': wing_root_chord, 'wing_sweep':    wing_sweep,
        'wing_tc':         wing_tc,         'vtp_area':      vtp_area,
        'vtp_avg_chord':   vtp_avg_chord,   'vtp_sweep':     vtp_sweep,
        'vtp_tc':          vtp_tc,          'fus_length':    fus_length,
        'fus_wetted':      fus_wetted,      'fus_equiv_diam': fus_equiv_diam,
    }
    _failed = [k for k, v in _missing.items() if isinstance(v, str)]
    if _failed:
        print(f'  geometry values not available: {", ".join(_failed)} -- skipping')
        return

    # ── Wetted areas ───────────────────────────────────────────────────────────
    # Wing: (planform - buried panel at fuselage root) × 2 sides
    s_buried  = (FUSELAGE_EQUIV_DIAMETER_M / 2.0) * wing_root_chord  # one-sided
    swet_wing = (wing_area - s_buried) * 2.0

    # ── Wetted area sanity checks ──────────────────────────────────────────────
    if not (0.0 < s_buried < wing_area):
        raise ValueError(
            f'Wing buried area sanity check FAILED: s_buried = {s_buried:.5f} m² '
            f'must satisfy 0 < s_buried < {wing_area:.5f} m². '
            f'Check FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m '
            f'and wing_root_chord = {wing_root_chord:.4f} m.'
        )
    if not (0.0 < swet_wing <= 2.0 * wing_area):
        raise ValueError(
            f'Wing exposed wetted area sanity check FAILED: swet_wing = {swet_wing:.5f} m² '
            f'must satisfy 0 < swet_wing <= {2.0 * wing_area:.5f} m². '
            f'Check fuselage diameter and wing planform area.'
        )

    # VTP: S_wet = S_ref_vtp_total × 2  where S_ref_vtp_total = 2 panels × vtp_area
    # = 4 × vtp_area_per_panel.  Conservative: no junction cutout at wing-VTP root.
    swet_vtp = vtp_area * 4.0

    swet_fus = fus_wetted

    # ── Reynolds numbers ───────────────────────────────────────────────────────
    mach = DASH_MACH
    p_s  = CRUISE_STATIC_PRESSURE_PA
    t_s  = CRUISE_TEMPERATURE_K

    re_wing = reynolds_number_from_mach(mach, p_s, t_s, wing_c_mac)
    re_vtp  = reynolds_number_from_mach(mach, p_s, t_s, vtp_avg_chord)
    re_fus  = reynolds_number_from_mach(mach, p_s, t_s, fus_length)

    # ── Skin-friction coefficients (fully turbulent) ───────────────────────────
    cf_wing = flat_plate_skin_friction_coeff(re_wing, mach)
    cf_vtp  = flat_plate_skin_friction_coeff(re_vtp,  mach)
    cf_fus  = flat_plate_skin_friction_coeff(re_fus,  mach)

    # ── Form factors ───────────────────────────────────────────────────────────
    ff_wing = form_factor_lifting_surface(
        wing_tc, max_thickness_location_over_chord=WING_AIRFOIL.max_thickness_location,
    )
    ff_vtp  = form_factor_lifting_surface(
        vtp_tc,  max_thickness_location_over_chord=VTP_AIRFOIL.max_thickness_location,
    )
    fus_fineness = fus_length / fus_equiv_diam
    ff_fus  = form_factor_datcom_body(fus_fineness)

    # ── R_LS: use quarter-chord sweep as proxy for max-thickness-line sweep ───
    r_ls_wing = lifting_surface_correction_factor(mach, np.cos(np.deg2rad(wing_sweep)))
    r_ls_vtp  = lifting_surface_correction_factor(mach, np.cos(np.deg2rad(vtp_sweep)))

    # ── Interference factors ───────────────────────────────────────────────────
    r_wf = wing_fuselage_interference_factor(re_fus, mach)   # fuselage-wing
    r_h  = H_WING_INTERFERENCE_FACTOR                         # H-wing junction

    # Wing: R_wf (fuselage junction) × R_h (wingtip junction)
    # VTP:  1.0  (no fuselage junction)  × R_h
    # Fus:  R_wf only
    q_wing = r_wf * r_h
    q_vtp  = r_h
    q_fus  = r_wf

    # ── CD0 per component (K_LP = 0, clean) ───────────────────────────────────
    s_ref    = wing_area
    cd0_wing = cf_wing * ff_wing * r_ls_wing * q_wing * swet_wing / s_ref
    cd0_vtp  = cf_vtp  * ff_vtp  * r_ls_vtp  * q_vtp  * swet_vtp  / s_ref
    cd0_fus  = cf_fus  * ff_fus              * q_fus  * swet_fus  / s_ref
    cd0_tot  = cd0_wing + cd0_vtp + cd0_fus

    # ── Print table ────────────────────────────────────────────────────────────
    print()
    hdr = (f'  {"Component":<12} {"Swet[m²]":>9} {"L[m]":>7} {"Re":>10}'
           f' {"Cf":>9} {"FF":>7} {"R_LS":>6} {"Q_eff":>7} {"CD0":>9} {"share":>7}')
    sep = '  ' + '-' * (len(hdr) - 2)
    print(hdr)
    print(sep)

    rows = [
        ('Wing',     swet_wing, wing_c_mac,    re_wing, cf_wing, ff_wing, r_ls_wing, q_wing, cd0_wing),
        ('VTP×2',    swet_vtp,  vtp_avg_chord, re_vtp,  cf_vtp,  ff_vtp,  r_ls_vtp,  q_vtp,  cd0_vtp),
        ('Fuselage', swet_fus,  fus_length,    re_fus,  cf_fus,  ff_fus,  1.0,       q_fus,  cd0_fus),
    ]
    for name, swet, length, re, cf, ff, r_ls, q, cd0 in rows:
        pct = 100.0 * cd0 / cd0_tot if cd0_tot > 0.0 else 0.0
        print(f'  {name:<12} {swet:>9.4f} {length:>7.4f} {re:>10.3e}'
              f' {cf:>9.5f} {ff:>7.4f} {r_ls:>6.4f} {q:>7.4f} {cd0:>9.5f} {pct:>6.1f}%')

    print(sep)
    print(f'  {"TOTAL":<12} {swet_wing+swet_vtp+swet_fus:>9.4f} {"":>7} {"":>10}'
          f' {"":>9} {"":>7} {"":>6} {"":>7} {cd0_tot:>9.5f} {"100.0%":>7}')

    print()
    print(f'  Notes:')
    print(f'    R_wf = {r_wf:.4f}  (DATCOM Fig 4.1, Re_fus = {re_fus:.3e}, M = {mach:.3f})')
    print(f'    VTP×2 Swet = 4 × Aircraft.VerticalTail.AREA  '
          f'(2 panels × 2 sides, no cutout -- conservative)')
    print(f'    Wing buried panel at root approx {s_buried:.5f} m²  '
          f'(d_fus/2 × c_root)  [d_fus = FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m]')
    print(f'    Fuselage Swet = fuselage_exposed_wetted_area (superellipse gross minus 2x wing airfoil holes)')
    print(f'    Fuselage d_eq (model) = {fus_equiv_diam:.4f} m,  FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m'
          f'  (should match),  fineness l/d = {fus_fineness:.1f}')


def build_xdsm_problem():
    prob, _, _cg = build_problem()
    prob.final_setup()
    return prob


def main():
    print('\n' + '=' * 70)
    print('HORIZONTAL-TAIL SMALL UAV RANGE OPTIMIZATION')
    print(f'  Version         : v{MODEL_VERSION}  (SpaJeti v1.0.0 H-wing)')
    print('  Layout          : H-tail (wing + twin-VTP endplates) + small turbojet')
    print('  Design variables: wing span, wing area, VTP span, wing section t/c, scaled SLS thrust, phase Mach')
    print('  Constraints     : Ny >= 7, Nz >= 7 (550 km/h, 5 km), T/W >= 1.5, fuel,')
    print(f'                    M_crit >= {MACH_UPPER_BOUND:.2f} (DASH + {M_CRIT_SAFETY_MARGIN:.2f}),')
    print(f'                    V_div >= {AERO_REQUIRED_SPEED_MS:.1f} m/s, lambda_max(design) <= 0')
    print('  Objective       : maximize range')
    print(f'  Method          : {OPTIMIZER} gradient-based')
    print('=' * 70 + '\n')

    prob, engine_mass_upper_kg, cg_est = build_problem()

    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore', RuntimeWarning)
        prob.run_aviary_problem()

    remove_dashboard_incompatible_recorder(prob)
    payload_range_csv = write_payload_range_report(prob)
    spajeti_3d_html = write_spajeti_aircraft_3d_report(prob)

    print('\n' + '=' * 70)
    print('OPTIMIZATION RESULTS')
    print(f'  Model version : v{MODEL_VERSION}  (SpaJeti v1.0.0 H-wing)')
    print(f'  Output dir    : {OUTPUT_DIR}')
    print('=' * 70 + '\n')

    print('Aircraft Geometry:')
    print('-' * 70)
    print_result('Wing Span',         safe_get(prob, av.Aircraft.Wing.SPAN, 'm'), 'm')
    print_result('Wing Area',         safe_get(prob, av.Aircraft.Wing.AREA, 'm**2'), 'm^2')
    print_result('Wing Aspect Ratio', safe_get(prob, av.Aircraft.Wing.ASPECT_RATIO))
    print_result('Wing Section t/c (M_crit DV)', safe_get(prob, 'wing_section_tc'))
    print_result('Wing AR_eff (endplate)', safe_get(prob, 'AR_eff'))
    print_result('Wing k_h (endplate factor)', safe_get(prob, 'k_h'))
    print_result('VTP Span',          safe_get(prob, av.Aircraft.VerticalTail.SPAN, 'm'), 'm')
    print_result('VTP Area',          safe_get(prob, av.Aircraft.VerticalTail.AREA, 'm**2'), 'm^2')
    print_result('VTP Aspect Ratio',  safe_get(prob, av.Aircraft.VerticalTail.ASPECT_RATIO))
    print_result('VTP tip mass (2 panels/wingtip)', safe_get(prob, AE.VTP_TIP_MASS, 'kg'), 'kg')
    print_result(
        'VTP tip pitch inertia',
        safe_get(prob, AE.VTP_TIP_PITCH_INERTIA, 'kg*m**2'),
        'kg*m^2',
    )
    print_result('H-Tail Area',       safe_get(prob, av.Aircraft.HorizontalTail.AREA, 'm**2'), 'm^2')
    print_result('Fuselage Length',   safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm'), 'm')
    print_result('Fuselage S_plf',    safe_get(prob, 'fuselage_planform_area', 'm**2'), 'm^2')
    print_result('Fuselage S_b',      safe_get(prob, 'fuselage_base_area', 'm**2'), 'm^2')
    print_result('Fuselage d_b',      safe_get(prob, 'fuselage_base_diameter', 'm'), 'm')
    print_result('Fuselage Swet geom', safe_get(prob, 'fuselage_wetted_area', 'm**2'), 'm^2')
    print_result('Fuselage d_eq geom', safe_get(prob, 'fuselage_equivalent_diameter', 'm'), 'm')
    print_result('Fuselage fineness geom', safe_get(prob, 'fuselage_fineness_ratio'), '')

    print('\nLoad Factors (550 km/h, 5 km ISA):')
    print('-' * 70)
    print('\nWing MAC Geometry:')
    print('-' * 70)
    print_result('Wing root chord c_r',           safe_get(prob, 'wing_root_chord',  'm'), 'm')
    print_result('Wing MAC c_mac',                safe_get(prob, 'wing_c_mac',       'm'), 'm')
    print_result('Wing MAC BL y_mac',             safe_get(prob, 'wing_y_mac',       'm'), 'm')
    print_result('Wing MAC LE x-station',         safe_get(prob, 'wing_x_mac_le',    'm'), 'm')
    print_result('Wing MAC LE z-station',         safe_get(prob, 'wing_z_mac_le',    'm'), 'm')
    print_result('Wing MAC c/4 x-station',        safe_get(prob, 'wing_x_mac_c4',   'm'), 'm')
    print_result('Wing LE sweep',                 safe_get(prob, 'wing_le_sweep',  'deg'), 'deg')

    print('\nLoad Factors (550 km/h, 5 km ISA):')
    print('-' * 70)
    print_result('Wing CL_alpha (endplate, wing-alone)', safe_get(prob, 'wing_CL_alpha'), '/rad')
    print_result('VTP CL_alpha_v',  safe_get(prob, 'CL_alpha_v'), '/rad')
    print_result('CY_beta_vtp',     safe_get(prob, 'CY_beta_vtp'), '/rad')
    print_result('CY_delta_r',      safe_get(prob, 'CY_delta_r'), '/rad')
    print_result('Ny (lateral)',     safe_get(prob, 'Ny'), f'[min {NY_MIN}]')
    print_result('Nz (vertical)',    safe_get(prob, 'Nz'), f'[min {NZ_MIN}]')
    print_result('CL (at Nz point)', safe_get(prob, 'CL'), '')
    print_result('CDi (Roskam Eq 4.8, no twist)', safe_get(prob, 'CDi'), '')
    print_result('  e_span_eff (mission CDI)',
                 safe_get(prob, av.Aircraft.Wing.SPAN_EFFICIENCY_FACTOR), '[AR_eff-corrected]')
    print_result('CDi_fus (Roskam Eq 4.33)',       safe_get(prob, 'CDi_fus'), '[mission included]')
    print_result('  CDi_fus base-area term',       safe_get(prob, 'CDi_fus_base_area_term'), '')
    print_result('  CDi_fus planform term',        safe_get(prob, 'CDi_fus_planform_term'), '')
    print_result('  eta (l/d interp, Fig 4.32)',   safe_get(prob, 'eta_finite_cylinder'), '')
    print_result('  c_d_c (Mc interp, Fig 4.31)',  safe_get(prob, 'crossflow_drag_coefficient'), '')
    print_result('e_oswald (Roskam Eq 4.12)',      safe_get(prob, 'e_oswald'), '')
    print_result('Wing r_LE',                      safe_get(prob, 'wing_le_radius', 'm'), 'm')
    print_scientific_result('Wing Re_LER',         safe_get(prob, 'wing_Re_LER'), '')
    print_result('LE suction parameter R',         safe_get(prob, 'wing_le_suction_parameter'), '')
    print_result('Wing M_DD (Weisshaar)', safe_get(prob, 'M_DD'), '[informational]')
    print_result('Wing M_crit',           safe_get(prob, 'M_crit'), '')
    print_result('M_crit check Mach',     MACH_UPPER_BOUND,
                 f'(DASH={DASH_MACH} + {M_CRIT_SAFETY_MARGIN})')
    print_result('mach_crit_margin (M_crit - check)',
                 safe_get(prob, 'mach_crit_margin'), '[min 0.00]')

    print('\nAeroelasticity (TOOD.md step 3-5):')
    print('-' * 70)
    div_speed  = safe_get(prob, AE.DIVERGENCE_SPEED, 'm/s')
    div_margin = safe_get(prob, AE.DIVERGENCE_SPEED_MARGIN, 'm/s')
    rev_speed  = safe_get(prob, AE.REVERSAL_SPEED, 'm/s')
    rev_margin = safe_get(prob, AE.REVERSAL_SPEED_MARGIN, 'm/s')
    flutter_v  = safe_get(prob, AE.FLUTTER_SPEED, 'm/s')
    flutter_m  = safe_get(prob, AE.FLUTTER_SPEED_MARGIN, 'm/s')
    lambda_max = safe_get(prob, AE.MAX_REAL_EIGENVALUE_AT_DESIGN, '1/s')
    pk_v       = safe_get(prob, AE.PK_FLUTTER_SPEED, 'm/s')
    pk_m       = safe_get(prob, AE.PK_FLUTTER_SPEED_MARGIN, 'm/s')
    ar_eff_ae  = safe_get(prob, 'AR_eff_ae')
    ar_eff_rig = safe_get(prob, 'AR_eff')
    div_q      = safe_get(prob, AE.DIVERGENCE_DYNAMIC_PRESSURE, 'Pa')
    print_result('V_required (1.25 * V_design)',  AERO_REQUIRED_SPEED_MS,   'm/s')
    print_result('Divergence speed V_div',         div_speed,  'm/s')
    print_result('Divergence speed margin [min 0]', div_margin, 'm/s')
    print_result('Reversal speed V_rev',           rev_speed,  'm/s')
    print_result('Reversal speed margin',           rev_margin, 'm/s')
    print_result('Flutter speed V_f (QS bisection)', flutter_v, 'm/s [informational]')
    print_result('Flutter speed margin',           flutter_m,  'm/s [informational]')
    print_result('lambda_max at design [max 0]',   lambda_max, '1/s')
    print_result(
        'Beam-modal f_bend',
        safe_get(prob, AE.BEAM_MODAL_BENDING_FREQUENCY, 'Hz'),
        'Hz [Step 4]',
    )
    print_result(
        'Beam-modal f_torsion',
        safe_get(prob, AE.BEAM_MODAL_TORSION_FREQUENCY, 'Hz'),
        'Hz [Step 4]',
    )
    print_result(
        'Beam-modal lambda_max',
        safe_get(prob, AE.BEAM_MODAL_MAX_REAL_EIGENVALUE_AT_DESIGN, '1/s'),
        '1/s [Step 4]',
    )
    print_result(
        'Beam-modal flutter speed',
        safe_get(prob, AE.BEAM_MODAL_FLUTTER_SPEED, 'm/s'),
        'm/s [Step 4]',
    )
    print_result(
        'Beam-modal flutter margin',
        safe_get(prob, AE.BEAM_MODAL_FLUTTER_SPEED_MARGIN, 'm/s'),
        'm/s [Step 4]',
    )
    print_result(
        'Beam-modal PK speed',
        safe_get(prob, AE.BEAM_MODAL_PK_FLUTTER_SPEED, 'm/s'),
        'm/s [Step 4]',
    )
    print_result(
        'Beam-modal PK margin',
        safe_get(prob, AE.BEAM_MODAL_PK_FLUTTER_SPEED_MARGIN, 'm/s'),
        'm/s [Step 4]',
    )
    print_result(
        'Beam-modal PK freq',
        safe_get(prob, AE.BEAM_MODAL_PK_FLUTTER_FREQUENCY, 'Hz'),
        'Hz [Step 4]',
    )
    print_result(
        'Beam-modal PK converged',
        safe_get(prob, AE.BEAM_MODAL_PK_CONVERGED),
        '[1=yes]',
    )
    print_result('PK flutter speed (Theodorsen)',   pk_v,       'm/s [informational]')
    print_result('PK flutter speed margin',         pk_m,       'm/s [informational]')
    print()
    print_result('AR_eff (rigid Scholz)',            ar_eff_rig, '')
    print_result('AR_eff_ae (aeroelastic corrected)', ar_eff_ae, '')
    if not any(isinstance(v, str) for v in (ar_eff_ae, ar_eff_rig, div_q)):
        flex_ratio = CONSTRAINT_Q_PA / div_q
        ae_penalty_pct = (ar_eff_rig - ar_eff_ae) / ar_eff_rig * 100.0
        print_result('  q_design / q_div (flexibility)', flex_ratio,      '')
        print_result('  AR_eff aeroelastic penalty',     ae_penalty_pct, '%')
    sls_thrust = safe_get(prob, av.Aircraft.Engine.SCALED_SLS_THRUST, 'N')
    print_result('SLS Thrust', sls_thrust, 'N')
    if not isinstance(sls_thrust, str):
        tw = sls_thrust / (MAX_TAKEOFF_MASS_KG * 9.80665)
        print_result(f'T/W at SLS', tw, f'[min {TW_MIN}]')

    print('\nSmall Turbojet:')
    print('-' * 70)
    print_result('Mass Constraint Upper', engine_mass_upper_kg, 'kg')
    print_result(
        'Diameter',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.DIAMETER), 'm'), 'm',
    )
    print_result(
        'Max RPM',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MAX_RPM), 'rpm'), 'rpm',
    )
    engine_mass = safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MASS), 'kg')
    print_result('Mass', engine_mass, 'kg')
    print_scientific_result(
        'SFC',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.SFC), 'kg/(N*s)'),
        'kg/(N*s)',
    )

    print('\nMission Performance:')
    print('-' * 70)
    print_result('Range',             safe_get(prob, av.Mission.RANGE, 'km'), 'km')
    print_result('Total Fuel',        safe_get(prob, av.Mission.TOTAL_FUEL, 'kg'), 'kg')
    print_result('Fuel Capacity',     FUEL_CAPACITY_KG, 'kg')
    print_result('Mass-Budget Limit', safe_get(prob, AVAILABLE_FUEL, 'kg'), 'kg')
    print_result('Fuel Margin',       safe_get(prob, FUEL_BUDGET_MARGIN, 'kg'), 'kg')
    print_result('Payload',           safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 'kg'), 'kg')
    if payload_range_csv:
        print_result('Payload/Range CSV', str(payload_range_csv))
    print_result('SpaJeti 3D dashboard HTML', str(spajeti_3d_html))

    print('\nMass Breakdown:')
    print('-' * 70)
    gross_mass = safe_get(prob, av.Aircraft.Design.GROSS_MASS, 'kg')
    empty_mass = safe_get(prob, av.Aircraft.Design.EMPTY_MASS, 'kg')
    print_result('Gross Mass', gross_mass, 'kg')
    print_result('Empty Mass', empty_mass, 'kg')
    if not any(isinstance(v, str) for v in (gross_mass, empty_mass, engine_mass)):
        print_result('Empty + Engine', empty_mass + engine_mass, 'kg')

    print('\nCentre of Gravity:')
    print('-' * 70)
    print_result('x_cg', safe_get(prob, 'x_cg', 'm'), 'm')
    print_result('y_cg', safe_get(prob, 'y_cg', 'm'), 'm')
    print_result('z_cg', safe_get(prob, 'z_cg', 'm'), 'm')
    print_result('total_mass (CG estimator)', safe_get(prob, 'total_mass', 'kg'), 'kg')
    x_mac_c4 = safe_get(prob, 'wing_x_mac_c4', 'm')
    c_mac_val = safe_get(prob, 'wing_c_mac',    'm')
    x_cg_val  = safe_get(prob, 'x_cg',         'm')
    if not any(isinstance(v, str) for v in (x_mac_c4, c_mac_val, x_cg_val)):
        sm_approx = (x_mac_c4 - x_cg_val) / c_mac_val
        print_result('  SM_approx (wing AC only)', sm_approx, '')
        note = ('stable (CG fwd of wing AC)' if sm_approx > 0
                else 'UNSTABLE -- CG aft of wing AC')
        print(f'  {"  note":<33}   {note}')

    print_cg_detail(prob, cg_est)

    if PRINT_AERO_DETAIL:
        print_aero_detail(prob)
        print_parasite_drag_detail(prob)

    # ── Constraint satisfaction check (actual physical values) ─────────────
    # IMPORTANT: The Aviary opt_report.html and OpenMDAO driver table show
    # SCALED values (val/ref) against UNSCALED physical bounds.  Any DV or
    # constraint with ref != 1 will appear violated there even when satisfied.
    # Example proof: wing_section_tc shows ~0.62 with upper=0.18 in the table,
    # but actual t/c = 0.62 * ref(0.15) = 0.093 -- within [0.05, 0.18].
    # The values below come from prob.get_val() (actual unscaled model values).
    print('\n' + '=' * 70)
    print('CONSTRAINT SATISFACTION (actual physical values from prob.get_val)')
    print('  opt_report.html shows val/ref -- trust THIS table, not that one.')
    print('=' * 70)

    def _ccheck(label, val, limit, sense='>=', unit=''):
        if isinstance(val, str):
            print(f'  {label:<46}  {"not available"}')
            return
        ok = (val >= limit) if sense == '>=' else (val <= limit)
        status = 'OK' if ok else '*** VIOLATION ***'
        print(f'  {label:<46} = {val:>10.4f} {unit}  [{sense}{limit}]  {status}')

    _ccheck(f'Ny  (lateral load factor)',       safe_get(prob, 'Ny'),                       NY_MIN)
    _ccheck(f'Nz  (longitudinal load factor)',   safe_get(prob, 'Nz'),                       NZ_MIN)
    _ccheck(f'mach_crit_margin',                 safe_get(prob, 'mach_crit_margin'),          0.0)
    _ccheck(f'divergence speed margin',
            safe_get(prob, AE.DIVERGENCE_SPEED_MARGIN, 'm/s'),                               0.0, unit='m/s')
    _ccheck(f'lambda_max at design point',
            safe_get(prob, AE.MAX_REAL_EIGENVALUE_AT_DESIGN, '1/s'),                         0.0, sense='<=', unit='1/s')
    _sls = safe_get(prob, av.Aircraft.Engine.SCALED_SLS_THRUST, 'N')
    _sls_min = TW_MIN * MAX_TAKEOFF_MASS_KG * 9.80665
    _ccheck(f'SLS thrust',                       _sls,                                        _sls_min, unit='N')
    _ccheck(f'fuel budget margin',               safe_get(prob, FUEL_BUDGET_MARGIN, 'kg'),   0.0, unit='kg')
    _eng = safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MASS), 'kg')
    _ccheck(f'engine mass',                      _eng,                                        engine_mass_upper_kg, sense='<=', unit='kg')
    print()
    print('  DVs (opt_report shows val/ref; actual physical values below):')
    _ccheck(f'  wing span',   safe_get(prob, av.Aircraft.Wing.SPAN,            'm'),   1.34, unit='m')
    _ccheck(f'  wing area',   safe_get(prob, av.Aircraft.Wing.AREA,            'm**2'), 0.40, unit='m^2')
    _ccheck(f'  VTP span',    safe_get(prob, av.Aircraft.VerticalTail.SPAN,    'm'),   0.15, unit='m')
    _ccheck(f'  wing_section_tc', safe_get(prob, 'wing_section_tc'),                   0.05, unit='')

    print('\n' + '=' * 70)
    print('OPTIMIZATION COMPLETE')
    print('=' * 70)

    prob.cleanup()
    return prob


if __name__ == '__main__':
    prob = main()
