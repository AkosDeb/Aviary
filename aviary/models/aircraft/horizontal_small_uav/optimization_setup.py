"""Optimization driver, design-variable, and constraint setup for SpaJeti."""

import os

import aviary.api as av

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetVariables

try:
    from .phase_info import MAX_TAKEOFF_MASS_KG
    from .horizontal_small_uav_config import *
except ImportError:
    from phase_info import MAX_TAKEOFF_MASS_KG
    from horizontal_small_uav_config import *


def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'

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


def configure_optimization(prob, optimizer, engine_mass_upper_kg):
    prob.add_driver(optimizer, max_iter=150, verbosity=av.Verbosity.VERBOSE)
    if optimizer == 'IPOPT':
        prob.driver.opt_settings['mu_strategy'] = 'adaptive'
        prob.driver.opt_settings['mu_init'] = 0.1
        prob.driver.opt_settings['nlp_scaling_method'] = 'none'
        prob.driver.opt_settings['acceptable_tol'] = 5e-3
        prob.driver.opt_settings['acceptable_iter'] = 5
        prob.driver.opt_settings['acceptable_constr_viol_tol'] = 1e-3
        prob.driver.opt_settings['acceptable_dual_inf_tol'] = 1.0

    # â”€â”€ Design variables â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    # â”€â”€ Constraints â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
    # â”€â”€ Aeroelastic constraints (TOOD.md Aeroelasticity step 3) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # Divergence: V_div >= V_dive = 1.25 * V_design (CS gradient via StaticAeroelastic)
    prob.model.add_constraint(
        AE.DIVERGENCE_SPEED_MARGIN, lower=0.0, units='m/s', ref=50.0,
    )
    # Flutter: beam-modal 3-DOF P-K speed margin as the active constraint.
    # V_flutter_3DOF >= V_dive (i.e. margin >= 0).  The old quasi-steady eigenvalue
    # is retained as an informational output; its constraint is disabled.
    prob.model.add_constraint(
        AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, lower=0.0, units='m/s', ref=50.0,
    )

    prob.add_objective()


