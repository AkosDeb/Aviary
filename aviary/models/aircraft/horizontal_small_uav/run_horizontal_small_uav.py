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
    from . import horizontal_small_uav_config as report_config
    from . import phase_info as report_phase_info
    from .phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH, phase_info
except ImportError:
    import horizontal_small_uav_config as report_config
    import phase_info as report_phase_info
    from phase_info import MAX_TAKEOFF_MASS_KG, DASH_MACH, phase_info

from aviary.models.aircraft.reporting.dashboard_reports import (
    configure_dashboard_context,
    write_payload_range_report,
    write_spajeti_aircraft_3d_report,
    write_optimization_summary_html,
)
from aviary.models.aircraft.reporting.printing_utils import (
    check_nan_inf_outputs,
    configure_print_context,
    print_optimization_summary,
    print_run_header,
)

try:
    from .optimization_setup import OPTIMIZER, configure_optimization, save_coloring_cache
except ImportError:
    from optimization_setup import OPTIMIZER, configure_optimization, save_coloring_cache

from aviary.subsystems.propulsion.small_turbojet import (
    SmallTurbojetModel,
    SmallTurbojetVariables,
)
from aviary.subsystems.geometry.spajeti_based.tail_geometry import TailGeometryGroup
from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    SuperellipseFuselageGeometry,
)
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
from aviary.subsystems.mass.spajeti_based.mass_group import SpaJetiMassGroup

try:
    from .horizontal_small_uav_config import *
except ImportError:
    from horizontal_small_uav_config import *

configure_print_context(report_config, report_phase_info)
configure_dashboard_context(report_config)

# (Fuselage eta and c_d_c are now interpolated inside FuselageLiftInducedDragComp
#  from the Roskam Part VI lookup tables â€” no module-level constants needed.)

# Compute derived constants at runtime
MACH_UPPER_BOUND = DASH_MACH + M_CRIT_SAFETY_MARGIN  # Used by MachCriticalComp
AERO_REQUIRED_SPEED_MS = DIVE_SPEED_FACTOR * 550.0 / 3.6  # 190.97 m/s



def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'


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
    hole is A = K * (t/c) * c_fusÂ², where K is the airfoil cross-section area
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
    fuselage_wetted_area  : mÂ²   Gross outer surface from SuperellipseFuselageGeometry
    wing_root_chord       : m    Wing root chord (from WingSurface)
    wing_section_tc       : â€“    Wing t/c ratio (design variable)
    fus_max_width         : m    Fuselage max width at wing station
    wing_span             : m    Full wing span (Aircraft.Wing.SPAN, design variable)
    wing_taper_ratio      : â€“    Wing taper ratio (Aircraft.Wing.TAPER_RATIO)
    htail_c_root_at_fus   : m    H-tail root chord at fuselage wall (default 0; future use)
    htail_tc_at_fus       : â€“    H-tail t/c at fuselage wall (default 0; future use)

    Outputs
    -------
    fuselage_exposed_wetted_area : mÂ²   Net fuselage wetted area after cutouts

    Options
    -------
    K_wing  : float  Airfoil cross-section area coeff for wing  (default 0.6843)
    K_htail : float  Airfoil cross-section area coeff for h-tail (default 0.6843)

    Numerical example â€” UAV baseline (b=1.8 m, AR=7.2, taper=0.8, t/c=0.15, d_fus=0.155 m)
    -----------------------------------------------------------------------------------------
    c_root = 2 * S_wing / (b * (1 + taper)) = 2 * 0.45 / (1.8 * 1.8) = 0.2778 m
    c_fus  = 0.2778 * (1 - 0.2 * 0.155 / 1.8)  = 0.2778 * 0.9828 = 0.2730 m
    A_hole = 0.6843 * 0.15 * 0.2730**2          = 0.00765 mÂ²
    wing cutout (Ã—2)                             = 0.01530 mÂ²
    Swet_fus_exposed = Swet_fus_gross - 0.0153 mÂ²
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
        self.add_input('tail_fuselage_cutout_area', val=0.0, units='m**2')

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
        tail_cutout = inputs['tail_fuselage_cutout_area']

        c_fus   = c_root * (1.0 - (1.0 - taper) * d_fus / b)

        wing_cutout  = 2.0 * K_w * tc   * c_fus**2
        htail_cutout = 2.0 * K_h * tc_h * c_htail**2

        outputs['fuselage_exposed_wetted_area'] = (
            inputs['fuselage_wetted_area'] - wing_cutout - htail_cutout - tail_cutout
        )


def add_load_factor_subsystems(prob):
    """Wire H-tail geometry, lift-slope, and Ny/Nz constraint components."""
    model = prob.model

    # â”€â”€ Fixed design-point inputs (flight condition + rudder geometry) â”€â”€â”€â”€â”€â”€â”€â”€
    fixed = om.IndepVarComp()
    fixed.add_output('delta_r_deg',      val=RUDDER_DELTA_DEG,    units='deg')
    fixed.add_output('beta_deg',         val=RUDDER_DELTA_DEG,    units='deg')
    fixed.add_output('dynamic_pressure', val=CONSTRAINT_Q_PA,     units='Pa')
    fixed.add_output('aircraft_mass',    val=MAX_TAKEOFF_MASS_KG,  units='kg')
    fixed.add_output('rudder_cf_c',      val=RUDDER_CF_C,         units='unitless')
    fixed.add_output('rudder_eta_root',  val=RUDDER_ETA_ROOT,     units='unitless')
    fixed.add_output('rudder_eta_tip',   val=RUDDER_ETA_TIP,      units='unitless')
    fixed.add_output('tail_cant_angle',  val=TAIL_CANT_ANGLE_DEG, units='deg')
    fixed.add_output('fus_nose_frac',    val=FUSELAGE_NOSE_LENGTH_FRACTION, units='unitless')
    fixed.add_output('fus_tail_frac',    val=FUSELAGE_TAIL_LENGTH_FRACTION, units='unitless')
    fixed.add_output('fus_max_width',    val=FUSELAGE_MAX_WIDTH_M,  units='m')
    fixed.add_output('fus_max_height',   val=FUSELAGE_MAX_HEIGHT_M, units='m')
    fixed.add_output('fus_aft_engine_clearance', val=FUSELAGE_AFT_ENGINE_CLEARANCE_M, units='m',
                     desc='Total aft fuselage base clearance added to turbojet diameter')
    fixed.add_output('fus_superellipse_exp', val=FUSELAGE_SUPERELLIPSE_EXPONENT, units='unitless')
    fixed.add_output('fus_nose_power_exp',  val=FUSELAGE_NOSE_POWER_EXPONENT,   units='unitless',
                     desc='Nose contour power-law exponent: s=(x/L_nose)^p. '
                          '0.5=parabolic ogive, 1.0=cone')
    fixed.add_output('fus_nose_aspect_ratio', val=FUSELAGE_NOSE_ASPECT_RATIO, units='unitless',
                     desc='Ellipsoid nose length divided by radius. '
                          '1.0=hemisphere, 2.0=prolate ellipsoid, 3.0=long radome')
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
    # â”€â”€ Wing apex position in the aircraft reference frame â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Frame: origin at nose tip; x positive AFT (fuselage station); y positive
    # starboard; z positive DOWN.  These are geometric inputs -- promote to DVs
    # when the CG / static-margin loop is added.
    fixed.add_output('wing_x_apex', val=WING_X_APEX_M, units='m',
                     desc='Wing root LE x-station from nose (positive aft) [m]')
    fixed.add_output('wing_z_apex', val=WING_Z_APEX_M, units='m',
                     desc='Wing root LE z-station from nose datum (positive down) [m]')
    model.add_subsystem('load_cond', fixed, promotes_outputs=['*'])

    # Tail geometry: layout-specific internals -> stable physical/aero outputs.
    model.add_subsystem('tail_geom', TailGeometryGroup(tail_type=TAIL_TYPE), promotes=['*'])

    if FUSELAGE_AFT_BASE_MODE == 'engine':
        model.add_subsystem(
            'aft_base_geometry',
            om.ExecComp(
                [
                    'aft_base_diameter = engine_diameter + clearance',
                    'fus_base_width_frac = (engine_diameter + clearance) / fus_max_width',
                    'fus_base_height_frac = (engine_diameter + clearance) / fus_max_height',
                ],
                engine_diameter={'val': 0.14, 'units': 'm'},
                clearance={'val': FUSELAGE_AFT_ENGINE_CLEARANCE_M, 'units': 'm'},
                fus_max_width={'val': FUSELAGE_MAX_WIDTH_M, 'units': 'm'},
                fus_max_height={'val': FUSELAGE_MAX_HEIGHT_M, 'units': 'm'},
                aft_base_diameter={'val': 0.165, 'units': 'm'},
                fus_base_width_frac={'val': 0.55, 'units': 'unitless'},
                fus_base_height_frac={'val': 0.55, 'units': 'unitless'},
            ),
            promotes_inputs=[
                ('engine_diameter', premission_propulsion_var(SmallTurbojetVariables.DIAMETER)),
                ('clearance', 'fus_aft_engine_clearance'),
                'fus_max_width',
                'fus_max_height',
            ],
            promotes_outputs=['aft_base_diameter', 'fus_base_width_frac', 'fus_base_height_frac'],
        )
    elif FUSELAGE_AFT_BASE_MODE == 'fixed':
        aft_base = om.IndepVarComp()
        aft_base.add_output(
            'fus_base_width_frac',
            val=FUSELAGE_BASE_WIDTH_FRACTION,
            units='unitless',
        )
        aft_base.add_output(
            'fus_base_height_frac',
            val=FUSELAGE_BASE_HEIGHT_FRACTION,
            units='unitless',
        )
        aft_base.add_output(
            'aft_base_diameter',
            val=0.5 * FUSELAGE_MAX_WIDTH_M * FUSELAGE_BASE_WIDTH_FRACTION
                + 0.5 * FUSELAGE_MAX_HEIGHT_M * FUSELAGE_BASE_HEIGHT_FRACTION,
            units='m',
        )
        model.add_subsystem('aft_base_geometry', aft_base, promotes_outputs=['*'])
    else:
        raise ValueError(
            f'Unsupported FUSELAGE_AFT_BASE_MODE={FUSELAGE_AFT_BASE_MODE!r}; '
            'use "engine" or "fixed".'
        )

    # TailGeometryGroup publishes tail_drag_wetted_area and
    # tail_drag_characteristic_length for mission parasite drag.  For the X-tail,
    # wetted area is all panels x 2 sides with no fuselage cutout.
    # â”€â”€ Parametric fuselage geometry: report-only until drag/CG integration â”€â”€
    model.add_subsystem(
        'superellipse_fuselage',
        SuperellipseFuselageGeometry(nose_type=FUSELAGE_NOSE_TYPE),
        promotes_inputs=[
            ('fuselage_length', av.Aircraft.Fuselage.LENGTH),
            ('max_width', 'fus_max_width'),
            ('max_height', 'fus_max_height'),
            ('nose_length_fraction', 'fus_nose_frac'),
            ('tail_length_fraction', 'fus_tail_frac'),
            ('base_width_fraction', 'fus_base_width_frac'),
            ('base_height_fraction', 'fus_base_height_frac'),
            ('superellipse_exponent', 'fus_superellipse_exp'),
            ('nose_power_exponent',   'fus_nose_power_exp'),
            ('nose_aspect_ratio',     'fus_nose_aspect_ratio'),
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

    # â”€â”€ Wing surface: Scholz AR correction + Polhamus + M_crit â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    model.add_subsystem(
        'wing_surface', WingSurface(cfg=WING_SURFACE_CFG),
        promotes_inputs=[
            ('surface_ar',       av.Aircraft.Wing.ASPECT_RATIO),
            ('surface_span',     av.Aircraft.Wing.SPAN),
            ('surface_sweep_c4', av.Aircraft.Wing.SWEEP),
            ('surface_taper',    av.Aircraft.Wing.TAPER_RATIO),
            ('surface_area',     av.Aircraft.Wing.AREA),
            ('dihedral_deg',     'aircraft:wing:dihedral'),
            ('endplate_span',    'tail_aero_vertical_span'),
            'design_mach',
            'mach_upper_bound',
            'nz_min',
            'aircraft_mass',
            'dynamic_pressure',
            'wing_x_apex',
            'wing_z_apex',
            ('fuselage_diameter', 'fuselage_equivalent_diameter'),
        ],
        promotes_outputs=[
            ('surface_CL_alpha', 'wing_CL_alpha'),
            ('k_eff',            'k_h'),
            'AR_eff',
            'K_wf',
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

    # â”€â”€ Fuselage exposed wetted area: gross Swet minus wing airfoil cutouts â”€â”€â”€
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
            'tail_fuselage_cutout_area',
        ],
        promotes_outputs=['fuselage_exposed_wetted_area'],
    )

    # â”€â”€ VTP surface: Polhamus + CyBetaVtp + CyDeltaRudder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    model.add_subsystem(
        'vtp_surface', VTPSurface(cfg=VTP_SURFACE_CFG),
        promotes_inputs=[
            ('surface_ar',       'tail_panel_ar'),
            ('surface_sweep_c4', av.Aircraft.VerticalTail.SWEEP),
            ('surface_taper',    av.Aircraft.VerticalTail.TAPER_RATIO),
            'design_mach',
            'wing_ref_area',
            'tail_physical_panel_area',
            'tail_cant_angle',
            'tail_panel_count',
        ],
        promotes_outputs=[
            ('surface_CL_alpha', 'CL_alpha_v'),
            'CY_beta_tail',
            'CL_alpha_tail',
        ],
    )

    # â”€â”€ Lateral load factor -> Ny â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    model.add_subsystem('lat_load', LateralLoadFactor(), promotes=['*'])

    # â”€â”€ Longitudinal load factor -> Nz â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Sum wing + tail body-axis lift slopes before feeding Nz constraint.
    model.add_subsystem(
        'total_cl_alpha',
        om.ExecComp(
            'CL_alpha_total = wing_CL_alpha + CL_alpha_tail',
            wing_CL_alpha={'val': 4.0, 'units': 'unitless'},
            CL_alpha_tail={'val': 0.5, 'units': 'unitless'},
            CL_alpha_total={'val': 4.5, 'units': 'unitless'},
        ),
        promotes_inputs=['wing_CL_alpha', 'CL_alpha_tail'],
        promotes_outputs=['CL_alpha_total'],
    )

    model.add_subsystem(
        'long_load', LongitudinalLoadFactor(),
        promotes_inputs=[
            ('CL_alpha',      'CL_alpha_total'),
            'dynamic_pressure',
            'wing_ref_area',
            'aircraft_mass',
            'alpha_max_deg',
        ],
        promotes_outputs=['CL', 'lift', 'Nz'],
    )

    # â”€â”€ Wing induced drag -- Roskam Part VI Eq. 4.8 (no twist) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ Fuselage drag due to lift -- Roskam Part VI Eq. 4.33, report-only â”€â”€â”€â”€
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

    # -- Wire endplate AR gain into mission polar via span efficiency factor ----
    # Mission InducedDrag formula: CDi = CL^2 / (pi * AR_geo * e_span_eff)
    # With e_span_eff = AR_eff / AR_geo this becomes:
    #   CDi = CL^2 / (pi * AR_eff)   [perfect leading-edge suction, e_oswald = 1]
    # This is the correct upper-bound (minimum CDi) at any cruise CL.
    # The previous formula multiplied by e_oswald from RoskamInducedDragComp, which
    # was evaluated at the Nz-constraint stall CL (~1.41) rather than cruise CL
    # (~0.03), making e_span_eff physically inconsistent with the mission polar.
    # Dropping e_oswald fixes the inconsistency; perfect LE suction is an acceptable
    # upper-bound assumption at conceptual design stage (TOOD item 8 fix option 2).
    model.add_subsystem(
        'span_eff_correction',
        om.ExecComp(
            'e_span_eff = AR_eff / AR_geo',
            AR_eff={'val': 8.0, 'units': 'unitless'},
            AR_geo={'val': 7.0, 'units': 'unitless'},
            e_span_eff={'val': 1.14, 'units': 'unitless'},
        ),
        promotes_inputs=[
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

    # â”€â”€ Step 5: connect wing geometry and CL_alpha â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Aircraft.Wing.* and Aircraft.VerticalTail.* auto-connect via promotes=['*']
    # from Aviary pre_mission. Only wing_CL_alpha needs an explicit connection
    # because it is computed outside pre_mission by add_load_factor_subsystems.
    model.add_subsystem(
        'aeroelasticity',
        AeroelasticityGroup(
            material_name='aluminum_6061_t6',
            flutter_model=(
                'legacy_scalar'
                if AEROELASTIC_FLUTTER_MODEL == 'none'
                else AEROELASTIC_FLUTTER_MODEL
            ),
            num_speed_samples=AEROELASTIC_SPEED_SAMPLES,
            bisection_iterations=AEROELASTIC_BISECTION_ITER,
            pk_iterations=AEROELASTIC_PK_ITERATIONS,
        ),
        promotes_inputs=['*'],
        promotes_outputs=['*'],
    )
    model.connect('wing_CL_alpha', AE.LIFT_CURVE_SLOPE)
    model.connect('wing_section_tc', AE.STRUCTURAL_THICKNESS_TO_CHORD)
    model.connect(premission_propulsion_var(SmallTurbojetVariables.MASS), AE.ENGINE_MASS)
    model.connect(av.Mission.TOTAL_FUEL, AE.FUEL_MASS)

    # â”€â”€ Step 4: aeroelastic AR wash-in correction (diagnostic, not optimizer DV)
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


def add_spajeti_mass_subsystems(prob):
    """Add SpaJeti physics-based mass, CG, and fuel budget to the model.

    Handles two impedance mismatches:
    1. Coordinate frame: run script 'wing_x_apex' is x-positive-AFT (0.80 m);
       WingStructuralMass expects x-positive-FORWARD (−0.80 m).
    2. Fuselage dimensions: promote from local IVC ('fus_max_width'/'fus_max_height')
       rather than Aircraft.Fuselage.MAX_WIDTH/MAX_HEIGHT from the CSV.

    SpaJetiMassGroup now owns structural_mass_sum and FuelBudgetComp internally;
    AVAILABLE_FUEL and FUEL_BUDGET_MARGIN are promoted to model scope.
    """
    model = prob.model

    # Sign flip: 'wing_x_apex' = 0.80 m (AFT) → 'wing_x_apex_fwd' = −0.80 m (FORWARD)
    model.add_subsystem(
        'wing_apex_to_fwd',
        om.ExecComp(
            'wing_x_apex_fwd = -wing_x_apex',
            wing_x_apex={'val': WING_X_APEX_M, 'units': 'm'},
            wing_x_apex_fwd={'val': -WING_X_APEX_M, 'units': 'm'},
        ),
        promotes_inputs=['wing_x_apex'],
        promotes_outputs=['wing_x_apex_fwd'],
    )

    model.add_subsystem(
        'spajeti_mass',
        SpaJetiMassGroup(num_wing_stations=21),
        promotes_inputs=[
            AE.SPANWISE_STATIONS,
            AE.SPANWISE_CHORD,
            AE.SPANWISE_MASS_PER_UNIT_SPAN,
            AE.FRONT_SPAR_FRACTION,
            AE.REAR_SPAR_FRACTION,
            av.Aircraft.Wing.TAPER_RATIO,
            av.Aircraft.Wing.SWEEP,
            ('wing_x_apex', 'wing_x_apex_fwd'),
            'wing_z_apex',
            av.Aircraft.Fuselage.LENGTH,
            (av.Aircraft.Fuselage.MAX_WIDTH, 'fus_max_width'),
            (av.Aircraft.Fuselage.MAX_HEIGHT, 'fus_max_height'),
            'fuselage_wetted_area',
            'fuselage_centroid_x',
            'tail_physical_structural_mass',
            'tail_physical_x_cg',
            'tail_physical_z_cg',
            av.Aircraft.Design.GROSS_MASS,
            av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
        ],
        promotes_outputs=[
            'wing_structural_mass',
            'fuselage_structural_mass',
            'htp_structural_mass',
            'vtp_structural_mass',
            'structural_empty_mass',
            'aircraft_zfw_x_cg',
            ('available_fuel', AVAILABLE_FUEL),
            ('fuel_budget_margin', FUEL_BUDGET_MARGIN),
        ],
    )

    # Connect live engine mass (from optimizer) and mission fuel into mass group
    model.connect(
        premission_propulsion_var(SmallTurbojetVariables.MASS),
        'spajeti_mass.engine_mass',
    )
    model.connect(av.Mission.TOTAL_FUEL, 'spajeti_mass.fuel_mass')



def remove_dashboard_incompatible_recorder(prob):
    """Avoid a dashboard crash on custom promoted design-variable metadata."""
    opt_history_path = Path(prob.get_outputs_dir()) / 'optimization_history.db'
    if opt_history_path.exists():
        opt_history_path.unlink()


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
        RoskamAeroBuilder(),
    ])

    prob.check_and_preprocess_inputs()
    apply_aircraft_mass_and_fuel_limits(prob)

    gross_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.GROSS_MASS, units='kg')
    empty_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.EMPTY_MASS, units='kg')
    engine_mass_upper_kg = min(ENGINE_MASS_LIMIT_KG, gross_mass_kg - empty_mass_kg)


    prob.add_pre_mission_systems()
    add_load_factor_subsystems(prob)
    add_aeroelasticity_subsystems(prob)
    add_spajeti_mass_subsystems(prob)
    prob.add_phases()
    prob.add_post_mission_systems()
    prob.link_phases()

    # Shared structural-box geometry used by both aeroelasticity and SpaJeti mass.
    prob.model.set_input_defaults(AE.FRONT_SPAR_FRACTION, val=0.15, units='unitless')
    prob.model.set_input_defaults(AE.REAR_SPAR_FRACTION, val=0.60, units='unitless')

    configure_optimization(prob, OPTIMIZER, engine_mass_upper_kg)

    prob.setup()
    prob.set_initial_guesses()
    prob.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.VerticalTail.SPAN, VTP_SPAN_INITIAL_M, 'm')
    prob.set_val('wing_section_tc', WING_TC_INITIAL)

    # â”€â”€ Aeroelasticity initial conditions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Override default 1.225 kg/mÂ³ (sea level) with ISA 5 000 m density.
    prob.set_val(AE.AIR_DENSITY, ISA_5KM_DENSITY_KGM3, units='kg/m**3')
    # Override the group's default REQUIRED_SPEED (1.15 * V_design) with the
    # V_dive = 1.25 * V_design constraint per TOOD.md step 3.
    prob.set_val(AE.REQUIRED_SPEED, AERO_REQUIRED_SPEED_MS, units='m/s')

    return prob, engine_mass_upper_kg


def build_xdsm_problem():
    prob, _ = build_problem()
    prob.final_setup()
    return prob


def main():
    print_run_header(MODEL_VERSION, OPTIMIZER)

    prob, engine_mass_upper_kg = build_problem()

    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore', RuntimeWarning)
        prob.run_aviary_problem()

    save_coloring_cache(prob)

    if report_config.PRINT_NAN_INF_GUARD:
        check_nan_inf_outputs(prob)

    # ── Gradient-robustness checks (TOOD.md "Gradient robustness") ──────────
    # Lazy import breaks the circular dependency: gradient_checks imports
    # build_problem from this module, so we cannot import it at module level.
    if report_config.RUN_BOUNDS_SENSITIVITY or report_config.RUN_CHECK_TOTALS:
        try:
            from .gradient_checks import main as _run_gradient_checks
        except ImportError:
            from gradient_checks import main as _run_gradient_checks
        pts = ('interior', 'lower', 'upper') if report_config.RUN_BOUNDS_SENSITIVITY else ('interior',)
        _run_gradient_checks(points=pts)

    remove_dashboard_incompatible_recorder(prob)
    payload_range_csv = write_payload_range_report(prob)
    spajeti_3d_html = write_spajeti_aircraft_3d_report(prob)
    write_optimization_summary_html(prob, engine_mass_upper_kg, MODEL_VERSION)

    print_optimization_summary(
        prob,
        engine_mass_upper_kg,
        payload_range_csv,
        spajeti_3d_html,
        MODEL_VERSION,
        OUTPUT_DIR,
    )

    prob.cleanup()
    return prob


if __name__ == '__main__':
    prob = main()






