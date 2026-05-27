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

# Keep this example serial unless the caller explicitly requests MPI.
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


AIRCRAFT_DATA = Path(__file__).with_name('horizontal_small_uav.csv')
EMPTY_MASS_KG = 7
FUEL_CAPACITY_KG = 8.0
ENGINE_MASS_LIMIT_KG = 5.0
WING_CENTER_DISTANCE_INITIAL = 0.45
CG_DISTANCE_FROM_NOSE = 0.42
HORIZONTAL_TAIL_AC_DISTANCE_FROM_NOSE = 0.88
HORIZONTAL_TAIL_EFFECTIVENESS = 0.18
STATIC_MARGIN_TARGET = 0.05
STATIC_MARGIN_BOUNDS = (0.03, 0.10)
OUTPUT_ROOT = REPO_ROOT / 'outputs'
PROBLEM_NAME = 'run_horizontal_small_uav'
OUTPUT_DIR = OUTPUT_ROOT / f'{PROBLEM_NAME}_out'
AVAILABLE_FUEL = 'horizontal_small_uav:available_fuel'
FUEL_BUDGET_MARGIN = 'horizontal_small_uav:fuel_budget_margin'


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
        name=PROBLEM_NAME,
        work_dir=OUTPUT_ROOT,
    )
    prob.load_inputs(aircraft_data=AIRCRAFT_DATA, phase_info=phase_info)
    prob.load_external_subsystems([SmallTurbojetModel()])

    prob.check_and_preprocess_inputs()
    apply_aircraft_mass_and_fuel_limits(prob)

    gross_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.GROSS_MASS, units='kg')
    empty_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.EMPTY_MASS, units='kg')
    engine_mass_upper_kg = min(ENGINE_MASS_LIMIT_KG, gross_mass_kg - empty_mass_kg)

    prob.add_pre_mission_systems()
    add_layout_design_variables(prob)
    prob.add_phases()
    prob.add_post_mission_systems()
    add_fuel_budget_constraint(prob)
    prob.link_phases()

    prob.add_driver('IPOPT', max_iter=500, verbosity=av.Verbosity.VERBOSE)
    prob.driver.opt_settings['mu_strategy'] = 'adaptive'
    prob.driver.opt_settings['mu_init'] = 0.1
    prob.driver.opt_settings['nlp_scaling_method'] = 'none'
    prob.driver.opt_settings['acceptable_tol'] = 5e-3
    prob.driver.opt_settings['acceptable_iter'] = 5
    prob.driver.opt_settings['acceptable_constr_viol_tol'] = 1e-3
    prob.driver.opt_settings['acceptable_dual_inf_tol'] = 1.0

    prob.add_design_variables()
    prob.model.add_design_var(
        av.Aircraft.Wing.SPAN,
        lower=1.34,
        upper=2.0,
        units='m',
        ref=1.8,
    )
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

    return prob, engine_mass_upper_kg


def build_xdsm_problem():
    prob, _ = build_problem()
    prob.final_setup()
    return prob


def main():
    print('\n' + '=' * 70)
    print('HORIZONTAL-TAIL SMALL UAV RANGE OPTIMIZATION')
    print('  Layout          : wing + horizontal tail + small turbojet')
    print('  Design variables: wing span, scaled SLS thrust, wing AC location, phase Mach schedules')
    print('  Objective       : maximize range')
    print('  Method          : IPOPT gradient-based')
    print('=' * 70 + '\n')

    prob, engine_mass_upper_kg = build_problem()

    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore', RuntimeWarning)
        prob.run_aviary_problem()

    remove_dashboard_incompatible_recorder(prob)
    payload_range_csv = write_payload_range_report(prob)

    print('\n' + '=' * 70)
    print('OPTIMIZATION RESULTS')
    print('=' * 70 + '\n')

    print('Aircraft Geometry:')
    print('-' * 70)
    print_result('Wing Span', safe_get(prob, av.Aircraft.Wing.SPAN, 'm'), 'm')
    print_result('Wing Area', safe_get(prob, av.Aircraft.Wing.AREA, 'm**2'), 'm^2')
    print_result('Wing Aspect Ratio', safe_get(prob, av.Aircraft.Wing.ASPECT_RATIO))
    print_result('Horizontal Tail Area', safe_get(prob, av.Aircraft.HorizontalTail.AREA, 'm**2'), 'm^2')
    print_result('Fuselage Length', safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm'), 'm')
    wing_center_distance = safe_get(prob, av.Aircraft.Wing.CENTER_DISTANCE)
    fuselage_length = safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm')
    print_result('Wing AC From Nose', wing_center_distance, 'fuselage length')
    if not any(isinstance(value, str) for value in (wing_center_distance, fuselage_length)):
        print_result('Wing AC From Nose', wing_center_distance * fuselage_length, 'm')
    static_margin = safe_get(prob, av.Aircraft.Design.STATIC_MARGIN)
    print_percent_result('Static Margin', static_margin)
    print_percent_result('Static Margin Target', STATIC_MARGIN_TARGET)
    if not isinstance(static_margin, str):
        print_percent_result('Static Margin Error', static_margin - STATIC_MARGIN_TARGET)

    print('\nSmall Turbojet:')
    print('-' * 70)
    print_result('Mass Constraint Upper', engine_mass_upper_kg, 'kg')
    print_result(
        'Diameter',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.DIAMETER), 'm'),
        'm',
    )
    print_result(
        'Max RPM',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MAX_RPM), 'rpm'),
        'rpm',
    )
    engine_mass = safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MASS), 'kg')
    print_result('Mass', engine_mass, 'kg')
    print_result('Scaled SLS Thrust', safe_get(prob, av.Aircraft.Engine.SCALED_SLS_THRUST, 'N'), 'N')
    print_scientific_result(
        'SFC',
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.SFC), 'kg/(N*s)'),
        'kg/(N*s)',
    )

    print('\nMission Performance:')
    print('-' * 70)
    print_result('Range', safe_get(prob, av.Mission.RANGE, 'km'), 'km')
    print_result('Total Fuel', safe_get(prob, av.Mission.TOTAL_FUEL, 'kg'), 'kg')
    print_result('Fuel Capacity', FUEL_CAPACITY_KG, 'kg')
    print_result('Mass-Budget Fuel Limit', safe_get(prob, AVAILABLE_FUEL, 'kg'), 'kg')
    print_result('Fuel Budget Margin', safe_get(prob, FUEL_BUDGET_MARGIN, 'kg'), 'kg')
    print_result(
        'Payload',
        safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 'kg'),
        'kg',
    )
    if payload_range_csv:
        print_result('Payload/Range CSV', str(payload_range_csv))

    print('\nMass Breakdown:')
    print('-' * 70)
    gross_mass = safe_get(prob, av.Aircraft.Design.GROSS_MASS, 'kg')
    empty_mass = safe_get(prob, av.Aircraft.Design.EMPTY_MASS, 'kg')
    print_result('Gross Mass', gross_mass, 'kg')
    print_result('Empty Mass', empty_mass, 'kg')
    if not any(isinstance(value, str) for value in (gross_mass, empty_mass, engine_mass)):
        print_result('Empty + Engine Mass', empty_mass + engine_mass, 'kg')
        print_result('Gross - Engine Mass', gross_mass - engine_mass, 'kg')

    print('\n' + '=' * 70)
    print('OPTIMIZATION COMPLETE')
    print('=' * 70)

    prob.cleanup()
    return prob


if __name__ == '__main__':
    prob = main()
