"""Terminal reporting utilities for the horizontal small UAV example."""

import numpy as np

import aviary.api as av
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetVariables
from aviary.utils.print_utils import safe_get, print_result, print_scientific_result

try:
    from .horizontal_small_uav_config import *
    from .phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH
except ImportError:
    from horizontal_small_uav_config import *
    from phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH

MACH_UPPER_BOUND = DASH_MACH + M_CRIT_SAFETY_MARGIN
AERO_REQUIRED_SPEED_MS = DIVE_SPEED_FACTOR * 550.0 / 3.6


def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'


def print_aero_detail(prob):
    """Detailed wing, VTP, and rudder breakdown - all inputs and derived values."""

    # Safe attribute fallback for variables that may not exist in all Aviary versions
    _VTP_AVG_CHORD  = getattr(av.Aircraft.VerticalTail, 'AVERAGE_CHORD',
                               'aircraft:vertical_tail:average_chord')
    _VTP_ROOT_CHORD = getattr(av.Aircraft.VerticalTail, 'ROOT_CHORD',
                               'aircraft:vertical_tail:root_chord')
    _WING_DIHEDRAL  = getattr(av.Aircraft.Wing, 'DIHEDRAL', 'aircraft:wing:dihedral')

    # â”€â”€ Retrieve all values â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 1. Wing geometry inputs â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 1b. Wing MAC geometry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 2. Scholz winglet AR correction â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 2b. Wing M_crit / M_DD (Weisshaar Eq. 36) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 3. Wing Polhamus â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 4. VTP geometry inputs â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 5. VTP aerodynamics â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 6. Rudder / CY_delta_r â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 7. Passive side-force â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    print('\nPassive Side-Force - CyBetaVtp (x2 for both VTP panels)')
    print('-' * 70)
    print_result('Input: AR_v (from HTailGeometry)',     vtp_ar,         '')
    print_result('Input: S_v per panel',                 vtp_area,       'm^2')
    print_result('Input: S_ref (wing)',                  wing_area,      'm^2')
    print_result('Input: 2r_i/b_v (fus. ratio)',         fus_vtp_ratio,  '')
    print(f'  {"x2 factor for both panels applied internally"}')
    print_result('Output: CY_beta_vtp',                 cy_beta_vtp,    '/rad')

    # â”€â”€ 8. Lateral load factor Ny â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ 9. Longitudinal load factor Nz â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ Wetted areas â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Wing: (planform - buried panel at fuselage root) Ã— 2 sides
    s_buried  = (FUSELAGE_EQUIV_DIAMETER_M / 2.0) * wing_root_chord  # one-sided
    swet_wing = (wing_area - s_buried) * 2.0

    # â”€â”€ Wetted area sanity checks â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if not (0.0 < s_buried < wing_area):
        raise ValueError(
            f'Wing buried area sanity check FAILED: s_buried = {s_buried:.5f} mÂ² '
            f'must satisfy 0 < s_buried < {wing_area:.5f} mÂ². '
            f'Check FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m '
            f'and wing_root_chord = {wing_root_chord:.4f} m.'
        )
    if not (0.0 < swet_wing <= 2.0 * wing_area):
        raise ValueError(
            f'Wing exposed wetted area sanity check FAILED: swet_wing = {swet_wing:.5f} mÂ² '
            f'must satisfy 0 < swet_wing <= {2.0 * wing_area:.5f} mÂ². '
            f'Check fuselage diameter and wing planform area.'
        )

    # VTP: S_wet = S_ref_vtp_total Ã— 2  where S_ref_vtp_total = 2 panels Ã— vtp_area
    # = 4 Ã— vtp_area_per_panel.  Conservative: no junction cutout at wing-VTP root.
    swet_vtp = vtp_area * 4.0

    swet_fus = fus_wetted

    # â”€â”€ Reynolds numbers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    mach = DASH_MACH
    p_s  = CRUISE_STATIC_PRESSURE_PA
    t_s  = CRUISE_TEMPERATURE_K

    re_wing = reynolds_number_from_mach(mach, p_s, t_s, wing_c_mac)
    re_vtp  = reynolds_number_from_mach(mach, p_s, t_s, vtp_avg_chord)
    re_fus  = reynolds_number_from_mach(mach, p_s, t_s, fus_length)

    # â”€â”€ Skin-friction coefficients (fully turbulent) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    cf_wing = flat_plate_skin_friction_coeff(re_wing, mach)
    cf_vtp  = flat_plate_skin_friction_coeff(re_vtp,  mach)
    cf_fus  = flat_plate_skin_friction_coeff(re_fus,  mach)

    # â”€â”€ Form factors â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    ff_wing = form_factor_lifting_surface(
        wing_tc, max_thickness_location_over_chord=WING_AIRFOIL.max_thickness_location,
    )
    ff_vtp  = form_factor_lifting_surface(
        vtp_tc,  max_thickness_location_over_chord=VTP_AIRFOIL.max_thickness_location,
    )
    fus_fineness = fus_length / fus_equiv_diam
    ff_fus  = form_factor_datcom_body(fus_fineness)

    # â”€â”€ R_LS: use quarter-chord sweep as proxy for max-thickness-line sweep â”€â”€â”€
    r_ls_wing = lifting_surface_correction_factor(mach, np.cos(np.deg2rad(wing_sweep)))
    r_ls_vtp  = lifting_surface_correction_factor(mach, np.cos(np.deg2rad(vtp_sweep)))

    # â”€â”€ Interference factors â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    r_wf = wing_fuselage_interference_factor(re_fus, mach)   # fuselage-wing
    r_h  = H_WING_INTERFERENCE_FACTOR                         # H-wing junction

    # Wing: R_wf (fuselage junction) Ã— R_h (wingtip junction)
    # VTP:  1.0  (no fuselage junction)  Ã— R_h
    # Fus:  R_wf only
    q_wing = r_wf * r_h
    q_vtp  = r_h
    q_fus  = r_wf

    # â”€â”€ CD0 per component (K_LP = 0, clean) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    s_ref    = wing_area
    cd0_wing = cf_wing * ff_wing * r_ls_wing * q_wing * swet_wing / s_ref
    cd0_vtp  = cf_vtp  * ff_vtp  * r_ls_vtp  * q_vtp  * swet_vtp  / s_ref
    cd0_fus  = cf_fus  * ff_fus              * q_fus  * swet_fus  / s_ref
    cd0_tot  = cd0_wing + cd0_vtp + cd0_fus

    # â”€â”€ Print table â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    print()
    hdr = (f'  {"Component":<12} {"Swet[mÂ²]":>9} {"L[m]":>7} {"Re":>10}'
           f' {"Cf":>9} {"FF":>7} {"R_LS":>6} {"Q_eff":>7} {"CD0":>9} {"share":>7}')
    sep = '  ' + '-' * (len(hdr) - 2)
    print(hdr)
    print(sep)

    rows = [
        ('Wing',     swet_wing, wing_c_mac,    re_wing, cf_wing, ff_wing, r_ls_wing, q_wing, cd0_wing),
        ('VTPÃ—2',    swet_vtp,  vtp_avg_chord, re_vtp,  cf_vtp,  ff_vtp,  r_ls_vtp,  q_vtp,  cd0_vtp),
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
    print(f'    VTPÃ—2 Swet = 4 Ã— Aircraft.VerticalTail.AREA  '
          f'(2 panels Ã— 2 sides, no cutout -- conservative)')
    print(f'    Wing buried panel at root approx {s_buried:.5f} mÂ²  '
          f'(d_fus/2 Ã— c_root)  [d_fus = FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m]')
    print(f'    Fuselage Swet = fuselage_exposed_wetted_area (superellipse gross minus 2x wing airfoil holes)')
    print(f'    Fuselage d_eq (model) = {fus_equiv_diam:.4f} m,  FUSELAGE_EQUIV_DIAMETER_M = {FUSELAGE_EQUIV_DIAMETER_M:.4f} m'
          f'  (should match),  fineness l/d = {fus_fineness:.1f}')




def print_run_header(MODEL_VERSION, OPTIMIZER):
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


def print_optimization_summary(
    prob,
    engine_mass_upper_kg,
    payload_range_csv,
    spajeti_3d_html,
    MODEL_VERSION,
    OUTPUT_DIR,
):
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
        'Beam-modal 3DOF PK flutter speed',
        safe_get(prob, AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, 'm/s'),
        'm/s',
    )
    print_result(
        'Beam-modal 3DOF PK flutter margin [ACTIVE CONSTRAINT]',
        safe_get(prob, AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, 'm/s'),
        'm/s',
    )
    print_result(
        'Beam-modal control frequency',
        safe_get(prob, AE.BEAM_MODAL_CONTROL_FREQUENCY, 'Hz'),
        'Hz',
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

    print('\nCentre of Gravity (SpaJetiMassGroup):')
    print('-' * 70)
    print_result('Aircraft x CG', safe_get(prob, 'aircraft_x_cg', 'm'), 'm')
    print_result('Aircraft z CG', safe_get(prob, 'aircraft_z_cg', 'm'), 'm')
    print_result('Aircraft empty mass', safe_get(prob, 'aircraft_empty_mass', 'kg'), 'kg')
    print_result('Aircraft total mass', safe_get(prob, 'aircraft_total_mass', 'kg'), 'kg')
    print('  Static-margin printout is disabled until CG and MAC x sign conventions are unified.')

    if PRINT_AERO_DETAIL:
        print_aero_detail(prob)
        print_parasite_drag_detail(prob)

    # â”€â”€ Constraint satisfaction check (actual physical values) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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


