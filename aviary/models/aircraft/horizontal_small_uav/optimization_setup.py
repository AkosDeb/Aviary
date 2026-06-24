"""Optimization driver, design-variable, and constraint setup for SpaJeti."""

import os
from pathlib import Path

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
    if AEROELASTIC_FLUTTER_MODEL == 'legacy_scalar':
        # Scalar quasi-steady flutter screen: stable when the design-point eigenvalue
        # is non-positive.  Skips BeamModalFlutter.
        prob.model.add_constraint(
            AE.MAX_REAL_EIGENVALUE_AT_DESIGN, upper=0.0, units='1/s', ref=1.0,
        )
    elif AEROELASTIC_FLUTTER_MODEL == 'beam_modal_3dof_pk':
        # Beam-modal 3-DOF P-K speed margin as the active constraint.
        # V_flutter_3DOF >= V_dive (i.e. margin >= 0).
        prob.model.add_constraint(
            AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, lower=0.0, units='m/s', ref=50.0,
        )
    elif AEROELASTIC_FLUTTER_MODEL == 'none':
        pass  # flutter unconstrained; AeroelasticityGroup still runs in legacy_scalar mode
    else:
        raise ValueError(
            f'Unsupported AEROELASTIC_FLUTTER_MODEL={AEROELASTIC_FLUTTER_MODEL!r}. '
            'Use "beam_modal_3dof_pk", "legacy_scalar", or "none".'
        )

    prob.add_objective()
    _setup_coloring(prob, optimizer)


def _coloring_path():
    """Return the coloring cache path — sibling to OUTPUT_DIR so it survives directory wipes."""
    return OUTPUT_ROOT / f'{VERSIONED_RUN_NAME}_coloring.pkl'


def _setup_coloring(prob, optimizer):
    """Load a cached total Jacobian coloring or declare dynamic coloring for first run.

    The coloring file lives at OUTPUT_ROOT/<run_name>_coloring.pkl, outside OUTPUT_DIR,
    so it is not deleted when the output directory is wiped at the start of each run.
    Without caching, OpenMDAO recomputes the sparsity structure on every run — this
    was the source of the 600 s startup overhead observed in v1.33 with BeamModalFlutter.
    """
    if not USE_COLORING_CACHE or optimizer != 'IPOPT':
        return
    coloring_file = _coloring_path()
    if coloring_file.exists():
        try:
            prob.driver.use_fixed_coloring(str(coloring_file))
            print(f'[Coloring] Loaded cached total Jacobian coloring:\n  {coloring_file}')
            return
        except Exception as exc:
            print(f'[Coloring] Could not load cached coloring ({exc}); will recompute.')
    try:
        prob.driver.declare_coloring(show_summary=True, show_sparsity=False)
        print(f'[Coloring] Dynamic coloring declared — will be saved after first run to:\n'
              f'  {coloring_file}')
    except Exception as exc:
        print(f'[Coloring] declare_coloring() failed ({exc}); running without coloring cache.')


def save_coloring_cache(prob):
    """Save the computed total Jacobian coloring to disk for reuse on the next run.

    Call this after run_aviary_problem() returns.  If the cache file already exists
    (fixed coloring was used) or saving fails, this is a no-op.
    """
    if not USE_COLORING_CACHE or OPTIMIZER != 'IPOPT':
        return
    coloring_file = _coloring_path()
    if coloring_file.exists():
        return  # already saved from a previous run
    try:
        col = prob.driver.coloring_info.coloring
        if col is not None:
            OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
            col.save(str(coloring_file))
            print(f'[Coloring] Saved total Jacobian coloring to:\n  {coloring_file}')
        else:
            print('[Coloring] No coloring computed (coloring_info.coloring is None); '
                  'nothing to save.')
    except Exception as exc:
        print(f'[Coloring] Could not save coloring: {exc}')


