import aviary.api as av
import csv
import warnings
from pathlib import Path

import numpy as np
from aviary.variable_info.enums import Transcription


AIRCRAFT_DATA = Path(__file__).with_name("small_uav.csv")


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
            "mach_final": (0.25, "unitless"),

            "altitude_optimize": False,
            "altitude_initial": (50.0, "m"),
            "altitude_final": (5000.0, "m"),

            "mass_ref": (15.0, "kg"),
            "mass_bounds": ((6.0, 15.0), "kg"),

            "throttle_enforcement": "path_constraint",

            "time_initial": (0.0, "min"),
            "time_duration_bounds": ((5.0, 30.0), "min"),

            "no_descent": True,

            "transcription": Transcription.COLLOCATION,
        },
        "initial_guesses": {
            "time": ([0.0, 20.0], "min"),
            "altitude": ([50.0, 5000.0], "m"),
            "mach": ([0.12, 0.25], "unitless"),
            "mass": ([12.0, 11.5], "kg"),
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
            "time_duration_bounds": ((30.0, 120.0), "min"),
            "distance_initial": (0.0, "km"),
            "mass_bounds": ((6.0, 12.0), "kg"),
            "distance_bounds": ((0.0, 700.0), "km"),
            "transcription": Transcription.COLLOCATION,
        },
        "initial_guesses": {
            "time": ([0.0, 60.0], "min"),
            "altitude": ([5000.0, 5000.0], "m"),
            "mach": ([0.25, 0.25], "unitless"),
            "mass": ([12.0, 10.5], "kg"),
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


def write_payload_range_report(prob):
    reports_dir = Path(prob.get_reports_dir(force=True))
    csv_path = reports_dir / "payload_range_data.csv"

    payload_lbm = safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, "lbm")
    fuel_lbm = safe_get(prob, av.Mission.TOTAL_FUEL, "lbm")
    range_nm = safe_get(prob, av.Mission.RANGE, "NM")

    if any(isinstance(value, str) for value in (payload_lbm, fuel_lbm, range_nm)):
        return None

    rows = [
        {
            "Mission Name": "Zero Range",
            "Payload (lbm)": payload_lbm,
            "Fuel (lbm)": 0.0,
            "Range (NM)": 0.0,
        },
        {
            "Mission Name": "Fallout Mission",
            "Payload (lbm)": payload_lbm,
            "Fuel (lbm)": fuel_lbm,
            "Range (NM)": range_nm,
        },
    ]

    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["Mission Name", "Payload (lbm)", "Fuel (lbm)", "Range (NM)"],
        )
        writer.writeheader()
        writer.writerows(rows)

    return csv_path


def main():
    print("\n" + "=" * 70)
    print("SMALL UAV RANGE OPTIMIZATION")
    print("  Design variable : wing span  (1.0 - 2.0 m, fixed reference area)")
    print("  Objective       : maximize range")
    print("  Method          : SLSQP gradient-based")
    print("=" * 70 + "\n")

    prob = av.AviaryProblem(problem_type=av.ProblemType.FALLOUT)

    prob.load_inputs(
        aircraft_data=AIRCRAFT_DATA,
        phase_info=phase_info,
    )

    prob.check_and_preprocess_inputs()

    prob.add_pre_mission_systems()
    prob.add_phases()
    prob.add_post_mission_systems()
    prob.link_phases()

    prob.add_driver("SLSQP", max_iter=200)
    prob.driver.options["tol"] = 1.0e-6
    prob.add_design_variables()
    prob.model.add_design_var(
        av.Aircraft.Wing.SPAN,
        lower=1.0,
        upper=2.0,
        units="m",
        ref=1.8,
    )
    prob.model.add_design_var(
        av.Aircraft.Engine.SCALE_FACTOR,
        lower=0.0002,
        upper=0.005,
        units="unitless",
        ref=0.0008,
    )
    prob.add_objective()

    prob.setup()
    prob.set_initial_guesses()
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

    print("\nMission Performance:")
    print("-" * 70)
    print_result("Range", safe_get(prob, av.Mission.RANGE, "km"), "km")
    print_result("Total Fuel", safe_get(prob, av.Mission.TOTAL_FUEL, "kg"), "kg")
    if payload_range_csv:
        print_result("Payload/Range CSV", str(payload_range_csv))

    print("\nMass Breakdown:")
    print("-" * 70)
    print_result("Gross Mass", safe_get(prob, av.Aircraft.Design.GROSS_MASS, "kg"), "kg")
    print_result("Empty Mass", safe_get(prob, av.Aircraft.Design.EMPTY_MASS, "kg"), "kg")

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
