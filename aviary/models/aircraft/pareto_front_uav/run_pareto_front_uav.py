"""
Pareto front study: aspect ratio vs range trade-off.

Fixed Sref = 0.45 m^2.  For each AR in AR_SWEEP, span is set to
sqrt(AR * Sref) and held constant (not a design variable).  IPOPT then
optimises the remaining DVs (engine thrust, wing centre-of-pressure
location) to maximise range.

The resulting (wing_mass, range) pairs trace the Pareto front between
structural mass and mission performance: higher AR costs wing mass but
buys induced-drag reduction and therefore range.
"""

import csv
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
    from .phase_info import MAX_TAKEOFF_MASS_KG, phase_info
except ImportError:
    from phase_info import MAX_TAKEOFF_MASS_KG, phase_info

from aviary.subsystems.propulsion.small_turbojet import (
    SmallTurbojetModel,
    SmallTurbojetVariables,
)

# --------------------------------------------------------------------------- #
# Aircraft constants (same as horizontal_small_uav baseline)
# --------------------------------------------------------------------------- #
AIRCRAFT_DATA = Path(__file__).with_name('pareto_front_uav.csv')
EMPTY_MASS_KG = 7
FUEL_CAPACITY_KG = 8.0
ENGINE_MASS_LIMIT_KG = 5.0
WING_CENTER_DISTANCE_INITIAL = 0.45
CG_DISTANCE_FROM_NOSE = 0.42
HORIZONTAL_TAIL_AC_DISTANCE_FROM_NOSE = 0.88
HORIZONTAL_TAIL_EFFECTIVENESS = 0.18
STATIC_MARGIN_BOUNDS = (0.03, 0.10)
OUTPUT_ROOT = REPO_ROOT / 'outputs'
PROBLEM_NAME = 'pareto_front_uav'

AVAILABLE_FUEL = 'pareto_front_uav:available_fuel'
FUEL_BUDGET_MARGIN = 'pareto_front_uav:fuel_budget_margin'

# --------------------------------------------------------------------------- #
# Pareto sweep parameters
# --------------------------------------------------------------------------- #
WING_AREA_M2 = 0.45          # fixed reference area throughout the sweep
AR_SWEEP = np.array([0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0])


# --------------------------------------------------------------------------- #
# OpenMDAO components (identical to horizontal_small_uav)
# --------------------------------------------------------------------------- #
class StaticMarginEstimate(om.ExplicitComponent):
    """First-order longitudinal static-margin estimate for layout studies."""

    def setup(self):
        self.add_input(av.Aircraft.Wing.CENTER_DISTANCE, val=0.45)
        self.add_input(av.Aircraft.Fuselage.LENGTH, val=2.0, units='m')
        self.add_input(av.Aircraft.Wing.AREA, val=0.45, units='m**2')
        self.add_input(av.Aircraft.Wing.SPAN, val=1.8, units='m')
        self.add_input(av.Aircraft.HorizontalTail.AREA, val=0.15, units='m**2')
        self.add_output(av.Aircraft.Design.STATIC_MARGIN, val=0.10)
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        fuselage_length = inputs[av.Aircraft.Fuselage.LENGTH]
        wing_area = inputs[av.Aircraft.Wing.AREA]
        wing_span = inputs[av.Aircraft.Wing.SPAN]
        horizontal_tail_area = inputs[av.Aircraft.HorizontalTail.AREA]

        wing_ac_x = inputs[av.Aircraft.Wing.CENTER_DISTANCE] * fuselage_length
        cg_x = CG_DISTANCE_FROM_NOSE * fuselage_length
        tail_ac_x = HORIZONTAL_TAIL_AC_DISTANCE_FROM_NOSE * fuselage_length
        mean_aero_chord = wing_area / wing_span

        tail_shift = (
            HORIZONTAL_TAIL_EFFECTIVENESS
            * horizontal_tail_area
            / wing_area
            * (tail_ac_x - wing_ac_x)
        )
        neutral_point_x = wing_ac_x + tail_shift
        outputs[av.Aircraft.Design.STATIC_MARGIN] = (neutral_point_x - cg_x) / mean_aero_chord


class FuelBudgetEstimate(om.ExplicitComponent):
    """Fuel available after fixed masses and optimised engine mass are reserved."""

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


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'


def safe_get(prob, var, units=None):
    try:
        value = prob.get_val(var, units=units) if units else prob.get_val(var)
        return value[0] if hasattr(value, '__len__') else value
    except Exception as err:
        return f'not available: {err}'


def apply_aircraft_mass_and_fuel_limits(prob):
    prob.aviary_inputs.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.IGNORE_FUEL_CAPACITY_CONSTRAINT, False)
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.TOTAL_CAPACITY, FUEL_CAPACITY_KG, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.WING_FUEL_CAPACITY, 0.0, 'kg')
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.FUSELAGE_FUEL_CAPACITY, FUEL_CAPACITY_KG, 'kg')


def add_layout_design_variables(prob):
    layout_vars = om.IndepVarComp()
    layout_vars.add_output(
        av.Aircraft.Wing.CENTER_DISTANCE,
        val=WING_CENTER_DISTANCE_INITIAL,
        units='unitless',
    )
    prob.model.add_subsystem('layout_design_vars', layout_vars, promotes_outputs=['*'])
    prob.model.add_subsystem('static_margin_estimate', StaticMarginEstimate(), promotes=['*'])


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


# --------------------------------------------------------------------------- #
# Problem builder — span is a fixed parameter, not a design variable
# --------------------------------------------------------------------------- #
def build_problem(fixed_span_m: float):
    """
    Build an Aviary FALLOUT problem with wing span locked to fixed_span_m.

    Wing.SPAN is NOT added as a design variable.  IPOPT optimises engine
    thrust and wing centre-of-pressure location to maximise range.
    """
    ar = fixed_span_m ** 2 / WING_AREA_M2
    problem_name = f'{PROBLEM_NAME}_ar{ar:.1f}'
    output_dir = OUTPUT_ROOT / f'{problem_name}_out'
    if output_dir.exists():
        shutil.rmtree(output_dir)
    OUTPUT_ROOT.mkdir(exist_ok=True)

    prob = av.AviaryProblem(
        problem_type=av.ProblemType.FALLOUT,
        verbosity=av.Verbosity.BRIEF,
        name=problem_name,
        work_dir=OUTPUT_ROOT,
    )
    prob.load_inputs(aircraft_data=AIRCRAFT_DATA, phase_info=phase_info)
    prob.load_external_subsystems([SmallTurbojetModel()])
    prob.check_and_preprocess_inputs()
    apply_aircraft_mass_and_fuel_limits(prob)

    # Fix span for this Pareto point before the model is assembled
    prob.aviary_inputs.set_val(av.Aircraft.Wing.SPAN, fixed_span_m, 'm')

    gross_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.GROSS_MASS, units='kg')
    empty_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.EMPTY_MASS, units='kg')
    engine_mass_upper_kg = min(ENGINE_MASS_LIMIT_KG, gross_mass_kg - empty_mass_kg)

    prob.add_pre_mission_systems()
    add_layout_design_variables(prob)
    prob.add_phases()
    prob.add_post_mission_systems()
    add_fuel_budget_constraint(prob)
    prob.link_phases()

    prob.add_driver('IPOPT', max_iter=400, verbosity=av.Verbosity.BRIEF)
    prob.driver.opt_settings['mu_strategy'] = 'adaptive'
    prob.driver.opt_settings['mu_init'] = 0.1
    prob.driver.opt_settings['nlp_scaling_method'] = 'none'
    prob.driver.opt_settings['acceptable_tol'] = 5e-3
    prob.driver.opt_settings['acceptable_iter'] = 5
    prob.driver.opt_settings['acceptable_constr_viol_tol'] = 1e-3
    prob.driver.opt_settings['acceptable_dual_inf_tol'] = 1.0

    # Standard Aviary DVs (scaled SLS thrust, phase Mach schedules, etc.)
    prob.add_design_variables()

    # Wing.SPAN intentionally omitted — it is fixed for this Pareto point.
    prob.model.add_design_var(
        av.Aircraft.Wing.CENTER_DISTANCE,
        lower=0.25,
        upper=0.65,
        ref=0.45,
    )

    prob.model.add_constraint(
        premission_propulsion_var(SmallTurbojetVariables.MASS),
        upper=engine_mass_upper_kg,
        units='kg',
        ref=engine_mass_upper_kg,
    )
    prob.model.add_constraint(
        av.Aircraft.Design.STATIC_MARGIN,
        lower=STATIC_MARGIN_BOUNDS[0],
        upper=STATIC_MARGIN_BOUNDS[1],
        ref=0.10,
    )
    prob.model.add_constraint(
        FUEL_BUDGET_MARGIN,
        lower=0.0,
        units='kg',
        ref=FUEL_CAPACITY_KG,
    )

    prob.add_objective()
    prob.setup()
    prob.set_initial_guesses()

    prob.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
    # Reinforce the fixed span after set_initial_guesses (which may reset it)
    prob.set_val(av.Aircraft.Wing.SPAN, fixed_span_m, 'm')

    return prob, engine_mass_upper_kg


# --------------------------------------------------------------------------- #
# Pareto sweep
# --------------------------------------------------------------------------- #
def run_pareto_point(ar: float):
    """Run IPOPT at a single AR value.  Returns a result dict or None on failure."""
    span = np.sqrt(ar * WING_AREA_M2)
    chord = WING_AREA_M2 / span
    print(f'  span={span:.3f} m  chord={chord:.3f} m', flush=True)

    try:
        prob, _ = build_problem(fixed_span_m=span)
        with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
            warnings.simplefilter('ignore', RuntimeWarning)
            prob.run_aviary_problem()

        range_km = safe_get(prob, av.Mission.RANGE, 'km')
        wing_mass_kg = safe_get(prob, av.Aircraft.Wing.MASS, 'kg')
        total_fuel_kg = safe_get(prob, av.Mission.TOTAL_FUEL, 'kg')
        cdi_cruise = get_cruise_cdi(prob)
        prob.cleanup()

        if isinstance(range_km, str) or isinstance(wing_mass_kg, str):
            print(f'    output missing: range={range_km}  wing_mass={wing_mass_kg}')
            return None

        return {
            'ar': float(ar),
            'span_m': float(span),
            'chord_m': float(chord),
            'range_km': float(range_km),
            'wing_mass_kg': float(wing_mass_kg),
            'total_fuel_kg': float(total_fuel_kg) if not isinstance(total_fuel_kg, str) else None,
            'cdi_cruise': cdi_cruise,
        }

    except Exception as exc:
        print(f'    FAILED: {exc}')
        return None


def get_cruise_cdi(prob) -> float | None:
    """Return mean induced-drag coefficient over the cruise phase, or None on failure.

    Tries the two most likely Dymos paths (with and without solver_sub wrapper).
    """
    paths = [
        'traj.cruise.rhs_all.aerodynamics.InducedDrag.induced_drag_coeff',
        'traj.cruise.rhs_all.solver_sub.aerodynamics.InducedDrag.induced_drag_coeff',
    ]
    for p in paths:
        try:
            vals = prob.get_val(p)
            if vals is not None and len(vals) > 0:
                return float(np.mean(vals))
        except Exception:
            continue
    return None


def save_results(results: list, path: Path):
    if not results:
        return
    fieldnames = list(results[0].keys())
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(f'Results saved → {path}')


def plot_pareto_front(results: list):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available — skipping plot')
        return

    if not results:
        return

    wing_masses = [r['wing_mass_kg'] for r in results]
    ranges = [r['range_km'] for r in results]
    ars = [r['ar'] for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        f'Pareto Front — fixed Sref={WING_AREA_M2} m², AR sweep {AR_SWEEP[0]:.0f}–{AR_SWEEP[-1]:.0f}',
        fontsize=13,
    )

    # Left panel: Pareto front in objective space
    ax = axes[0]
    ax.plot(wing_masses, ranges, 'b-o', linewidth=2, markersize=7)
    for x, y, ar in zip(wing_masses, ranges, ars):
        ax.annotate(f'AR={ar:.0f}', (x, y), textcoords='offset points',
                    xytext=(5, 4), fontsize=8)
    ax.set_xlabel('Wing Structural Mass (kg)')
    ax.set_ylabel('Range (km)')
    ax.set_title('Pareto Front\n(each point: IPOPT-optimal at fixed AR)')
    ax.grid(True, alpha=0.3)

    # Right panel: AR vs range + wing mass + CDi (triple y-axis)
    cdis = [r.get('cdi_cruise') for r in results]
    has_cdi = any(v is not None for v in cdis)

    ax2 = axes[1]
    c_range = 'tab:blue'
    c_mass = 'tab:orange'
    c_cdi = 'tab:green'

    ln1, = ax2.plot(ars, ranges, 'o-', color=c_range, linewidth=2, label='Range (km)')
    ax2.set_xlabel('Aspect Ratio')
    ax2.set_ylabel('Range (km)', color=c_range)
    ax2.tick_params(axis='y', labelcolor=c_range)

    ax3 = ax2.twinx()
    ln2, = ax3.plot(ars, wing_masses, 's--', color=c_mass, linewidth=2, label='Wing Mass (kg)')
    ax3.set_ylabel('Wing Structural Mass (kg)', color=c_mass)
    ax3.tick_params(axis='y', labelcolor=c_mass)

    legend_handles = [ln1, ln2]

    if has_cdi:
        valid_cdi = [(ar, v) for ar, v in zip(ars, cdis) if v is not None]
        cdi_ars, cdi_vals = zip(*valid_cdi)
        ax4 = ax2.twinx()
        ax4.spines['right'].set_position(('outward', 65))
        ln3, = ax4.plot(cdi_ars, cdi_vals, '^:', color=c_cdi, linewidth=2, label='CDi (cruise mean)')
        ax4.set_ylabel('Induced Drag Coeff CDi (–)', color=c_cdi)
        ax4.tick_params(axis='y', labelcolor=c_cdi)
        legend_handles.append(ln3)

    ax2.legend(handles=legend_handles, loc='center left')
    ax2.set_title('AR vs Range, Wing Mass, and CDi')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = OUTPUT_ROOT / f'{PROBLEM_NAME}_pareto.png'
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f'Plot saved → {plot_path}')
    plt.show()


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def main():
    print('\n' + '=' * 70)
    print('PARETO FRONT STUDY: ASPECT RATIO vs RANGE')
    print(f'  Fixed wing area : {WING_AREA_M2} m²')
    print(f'  AR sweep        : {AR_SWEEP[0]:.0f} → {AR_SWEEP[-1]:.0f}  ({len(AR_SWEEP)} points)')
    print(f'  span range      : {np.sqrt(AR_SWEEP[0]*WING_AREA_M2):.3f} → {np.sqrt(AR_SWEEP[-1]*WING_AREA_M2):.3f} m')
    print(f'  Inner solver    : IPOPT (maximise range at each AR)')
    print(f'  Objectives      : range [km]  vs  wing structural mass [kg]')
    print('=' * 70 + '\n')

    results = []
    for i, ar in enumerate(AR_SWEEP):
        print(f'[{i + 1}/{len(AR_SWEEP)}] AR = {ar:.1f}')
        result = run_pareto_point(ar)
        if result is not None:
            results.append(result)
            print(f'    → range={result["range_km"]:.1f} km   wing_mass={result["wing_mass_kg"]:.4f} kg')
        else:
            print(f'    → skipped')

    converged = len(results)
    print(f'\n{converged}/{len(AR_SWEEP)} points converged\n')

    if results:
        csv_path = OUTPUT_ROOT / f'{PROBLEM_NAME}_pareto.csv'
        save_results(results, csv_path)

        print('\nPareto front summary:')
        print(f'  {"AR":>4}  {"span(m)":>8}  {"chord(m)":>9}  {"range(km)":>10}  {"wing_mass(kg)":>14}  {"CDi":>8}')
        print('  ' + '-' * 65)
        for r in results:
            cdi_str = f'{r["cdi_cruise"]:.5f}' if r.get('cdi_cruise') is not None else '     n/a'
            print(
                f'  {r["ar"]:>4.1f}  {r["span_m"]:>8.3f}  {r["chord_m"]:>9.3f}'
                f'  {r["range_km"]:>10.1f}  {r["wing_mass_kg"]:>14.4f}  {cdi_str:>8}'
            )

        plot_pareto_front(results)

    print('\n' + '=' * 70)
    print('SWEEP COMPLETE')
    print('=' * 70)
    return results


if __name__ == '__main__':
    results = main()
