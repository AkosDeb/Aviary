import csv
import shutil
import sys
import warnings
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import aviary.api as av
from aviary.subsystems.propulsion.small_turbojet import (
    SmallTurbojetModel,
    SmallTurbojetVariables,
)
from aviary.variable_info.enums import Transcription


AIRCRAFT_DATA = Path(__file__).with_name("small_uav.csv")
MAX_TAKEOFF_MASS_KG = 15.0
EMPTY_MASS_KG = 3.5
FUEL_CAPACITY_KG = 8
ENGINE_MASS_LIMIT_KG = 5
OUTPUT_DIR = REPO_ROOT / "run_small_uav_mission_out"


def premission_propulsion_var(name):
    return f"pre_mission.propulsion.{name}"


phase_info = {
    "pre_mission": {
        "include_takeoff": False,
        "optimize_mass": False,
    },
    "climb": {
        "subsystem_options": {
            "aerodynamics": {"method": "computed"},
        },
        "user_options": {
            "num_segments": 4,
            "order": 3,

            "mach_optimize": False,
            "mach_initial": (0.12, "unitless"),
            "mach_final":   (0.25, "unitless"),

            "altitude_optimize": False,
            "altitude_initial": (50.0, "m"),
            "altitude_final": (5000.0, "m"),

            "mass_ref": (MAX_TAKEOFF_MASS_KG, "kg"),
            "mass_bounds": ((6.0, MAX_TAKEOFF_MASS_KG), "kg"),

            "throttle_enforcement": "path_constraint",

            "time_initial": (0.0, "min"),
            "time_duration_bounds": ((1.0, 30.0), "min"),

            "no_descent": True,

            "transcription": Transcription.COLLOCATION,
        },
        "initial_guesses": {
            "time": ([0.0, 20.0], "min"),
            "altitude": ([50.0, 5000.0], "m"),
            "mach": ([0.12, 0.25], "unitless"),
            "mass": ([14.5, 13.5], "kg"),
            "distance": ([0.0, 60.0], "km"),
        },
    },
    "cruise": {
        "subsystem_options": {
            "aerodynamics": {"method": "computed"},
        },
        "user_options": {
            "num_segments": 3,
            "order": 3,
            "mach_optimize": False,

            "mach_initial": (0.25, "unitless"),
            "mach_final": (0.25, "unitless"),
            "altitude_optimize": False,
            "altitude_initial": (5000.0, "m"),
            "altitude_final": (5000.0, "m"),
            "time_initial": (0.0, "min"),
            "time_duration_bounds": ((30.0, 400.0), "min"),
            "distance_initial": (0.0, "km"),
            "mass_bounds": ((6.0, MAX_TAKEOFF_MASS_KG), "kg"),
            "mass_ref": (MAX_TAKEOFF_MASS_KG, "kg"),
            "distance_bounds": ((0.0, 700.0), "km"),
            "throttle_enforcement": "path_constraint",
            "transcription": Transcription.COLLOCATION,
        },
        "initial_guesses": {
            "time": ([0.0, 60.0], "min"),
            "altitude": ([5000.0, 5000.0], "m"),
            "mach": ([0.25, 0.25], "unitless"),
            "mass": ([13.5, 9.0], "kg"),
            "distance": ([0.0, 200.0], "km"),
        },
    },
    "post_mission": {
        "include_landing": False,
        "constrain_range": False,
    },
}


def safe_get(prob, var, units=None):
    try:
        value = prob.get_val(var, units=units) if units else prob.get_val(var)
        return value[0] if hasattr(value, "__len__") else value
    except Exception as err:
        return f"not available: {err}"


def print_result(label, value, unit=""):
    if isinstance(value, str):
        print(f"{label:<35} = {value}")
    else:
        print(f"{label:<35} = {value:>12.4f} {unit}".rstrip())


def print_scientific_result(label, value, unit=""):
    if isinstance(value, str):
        print(f"{label:<35} = {value}")
    else:
        print(f"{label:<35} = {value:>12.6e} {unit}".rstrip())


def apply_aircraft_mass_and_fuel_limits(prob):
    prob.aviary_inputs.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, "kg")
    prob.aviary_inputs.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, "kg")
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.IGNORE_FUEL_CAPACITY_CONSTRAINT, False)
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.TOTAL_CAPACITY, FUEL_CAPACITY_KG, "kg")
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.WING_FUEL_CAPACITY, 0.0, "kg")
    prob.aviary_inputs.set_val(av.Aircraft.Fuel.FUSELAGE_FUEL_CAPACITY, FUEL_CAPACITY_KG, "kg")


def write_payload_range_report(prob):
    reports_dir = Path(prob.get_reports_dir(force=True))
    csv_path = reports_dir / "payload_range_data.csv"

    payload_kg = safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, "kg")
    fuel_kg    = safe_get(prob, av.Mission.TOTAL_FUEL, "kg")
    range_km   = safe_get(prob, av.Mission.RANGE, "km")

    if any(isinstance(value, str) for value in (payload_kg, fuel_kg, range_km)):
        return None

    rows = [
        {
            "Mission Name": "Zero Range",
            "Payload (kg)": payload_kg,
            "Fuel (kg)": 0.0,
            "Range (km)": 0.0,
        },
        {
            "Mission Name": "Fallout Mission",
            "Payload (kg)": payload_kg,
            "Fuel (kg)": fuel_kg,
            "Range (km)": range_km,
        },
    ]

    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["Mission Name", "Payload (kg)", "Fuel (kg)", "Range (km)"],
        )
        writer.writeheader()
        writer.writerows(rows)

    return csv_path


def build_problem():
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    prob = av.AviaryProblem(problem_type=av.ProblemType.FALLOUT, verbosity=av.Verbosity.VERBOSE)

    prob.load_inputs(
        aircraft_data=AIRCRAFT_DATA,
        phase_info=phase_info,
    )
    prob.load_external_subsystems([SmallTurbojetModel()])

    # check_and_preprocess_inputs() re-reads aviary_inputs from the raw CSV cache,
    # wiping any set_val calls made before it.  Apply mass overrides AFTER it so
    # they are the last write into aviary_inputs before the model is built.
    prob.check_and_preprocess_inputs()
    apply_aircraft_mass_and_fuel_limits(prob)

    gross_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.GROSS_MASS, units="kg")
    empty_mass_kg = prob.aviary_inputs.get_val(av.Aircraft.Design.EMPTY_MASS, units="kg")
    engine_mass_upper_kg = min(ENGINE_MASS_LIMIT_KG, gross_mass_kg - empty_mass_kg)

    prob.add_pre_mission_systems()
    prob.add_phases()
    prob.add_post_mission_systems()
    prob.link_phases()

    prob.add_driver("IPOPT", max_iter=500, verbosity=av.Verbosity.VERBOSE)
    # mu_strategy=monotone + mu_init=1e-5 on an infeasible starting point causes
    # Lagrange multipliers to diverge exponentially (inf_du: 1 -> 1e13 in 250 iters).
    # adaptive lets IPOPT loosen the barrier when primal progress stalls.
    # equilibration-based scaling is more robust than gradient-based when gradients
    # contain NaN at trial points.  acceptable_* lets IPOPT accept a near-optimal
    # solution rather than failing with "Invalid number" when tight tolerance
    # (1e-6) cannot be achieved due to FLOPS evaluation errors.
    prob.driver.opt_settings['mu_strategy'] = 'adaptive'
    prob.driver.opt_settings['mu_init'] = 0.1
    # 'equilibration-based' requires HSL (libhsl.dll) which is not installed.
    # 'none' avoids that dependency; ref= values on design variables already
    # give IPOPT reasonable relative scaling without HSL.
    prob.driver.opt_settings['nlp_scaling_method'] = 'none'
    prob.driver.opt_settings['acceptable_tol'] = 1e-3
    prob.driver.opt_settings['acceptable_iter'] = 15
    prob.add_design_variables()
    prob.model.add_design_var(
        av.Aircraft.Wing.SPAN,
        lower=1.34,  # sqrt(4 * 0.45 m^2) — enforces AR >= 4; below this FLOPS
                     # skin-friction form-factor evaluates outside its valid range
        upper=2.0,
        units="m",
        ref=1.8,
    )
    prob.model.add_constraint(
        premission_propulsion_var(SmallTurbojetVariables.MASS),
        upper=engine_mass_upper_kg,
        units="kg",
        ref=engine_mass_upper_kg,
    )
    prob.add_objective()

    prob.setup()
    prob.set_initial_guesses()
    # set_initial_guesses() replays raw CSV values into the model, overriding the
    # aviary_inputs overrides made in apply_aircraft_mass_and_fuel_limits.
    # Setting these directly here is the last write before the optimizer runs.
    prob.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
    prob.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')

    return prob, engine_mass_upper_kg


def build_xdsm_problem():
    prob, _ = build_problem()
    prob.final_setup()
    return prob


def main():
    print("\n" + "=" * 70)
    print("SMALL UAV RANGE OPTIMIZATION")
    print("  Design variables: wing span, turbojet diameter, turbojet length")
    print("  Objective       : maximize range")
    print("  Method          : IPOPT gradient-based")
    print("=" * 70 + "\n")

    prob, engine_mass_upper_kg = build_problem()

    with warnings.catch_warnings(), np.errstate(invalid="ignore", over="ignore"):
        warnings.simplefilter("ignore", RuntimeWarning)

        prob.run_aviary_problem()
    payload_range_csv = write_payload_range_report(prob)

    print("\n" + "=" * 70)
    print("OPTIMIZATION RESULTS")
    print("=" * 70 + "\n")

    print("Aircraft Geometry:")
    print("-" * 70)
    print_result("Wing Span", safe_get(prob, av.Aircraft.Wing.SPAN, "m"), "m")
    print_result("Wing Area", safe_get(prob, av.Aircraft.Wing.AREA, "m**2"), "m^2")
    print_result("Wing Aspect Ratio", safe_get(prob, av.Aircraft.Wing.ASPECT_RATIO))

    print("\nSmall Turbojet Geometry:")
    print("-" * 70)
    print_result("Mass Constraint Upper", engine_mass_upper_kg, "kg")
    print_result(
        "Diameter",
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.DIAMETER), "m"),
        "m",
    )
    print_result(
        "Length",
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.LENGTH), "m"),
        "m",
    )
    print_result(
        "Max RPM",
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MAX_RPM), "rpm"),
        "rpm",
    )
    engine_mass = safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.MASS), "kg")
    print_result(
        "Mass",
        engine_mass,
        "kg",
    )
    print_result(
        "Scaled SLS Thrust",
        safe_get(prob, av.Aircraft.Engine.SCALED_SLS_THRUST, "N"),
        "N",
    )
    print_scientific_result(
        "SFC",
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.SFC), "kg/(N*s)"),
        "kg/(N*s)",
    )

    print("\nMission Performance:")
    print("-" * 70)
    print_result("Range", safe_get(prob, av.Mission.RANGE, "km"), "km")
    print_result("Total Fuel", safe_get(prob, av.Mission.TOTAL_FUEL, "kg"), "kg")
    print_result("Fuel Capacity", FUEL_CAPACITY_KG, "kg")
    if payload_range_csv:
        print_result("Payload/Range CSV", str(payload_range_csv))

    print("\nMass Breakdown:")
    print("-" * 70)
    gross_mass = safe_get(prob, av.Aircraft.Design.GROSS_MASS, "kg")
    empty_mass = safe_get(prob, av.Aircraft.Design.EMPTY_MASS, "kg")
    print_result("Gross Mass", gross_mass, "kg")
    print_result("Empty Mass", empty_mass, "kg")
    if not any(isinstance(value, str) for value in (gross_mass, empty_mass, engine_mass)):
        print_result("Empty + Engine Mass", empty_mass + engine_mass, "kg")
        print_result("Gross - Engine Mass", gross_mass - engine_mass, "kg")

    print("\nAerodynamic Performance:")
    print("-" * 70)
    print_result(
        "Wing Loading",
        safe_get(prob, av.Aircraft.Design.WING_LOADING, "N/m**2"),
        "N/m^2",
    )
    print_result("T/W Ratio", safe_get(prob, av.Aircraft.Design.THRUST_TO_WEIGHT_RATIO))

    print("\n" + "=" * 70)
    print("OPTIMIZATION COMPLETE")
    print("=" * 70)

    prob.cleanup()
    return prob


if __name__ == "__main__":
    prob = main()
