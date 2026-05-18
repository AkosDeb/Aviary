from pathlib import Path
import argparse
import os
import runpy
import subprocess


"""
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm `
  --no-pdf
"""
try:
    import pyxdsm
    from pyxdsm.XDSM import XDSM, FUNC, LEFT, OPT, RIGHT, SOLVER
except ModuleNotFoundError:
    pyxdsm = None
    XDSM = None
    FUNC = "FUNC"
    LEFT = "LEFT"
    OPT = "OPT"
    RIGHT = "RIGHT"
    SOLVER = "SOLVER"


SYSTEM_TYPE_MAP = {
    "OPT": OPT,
    "FUNC": FUNC,
    "SOLVER": SOLVER,
}


SIDE_MAP = {
    "LEFT": LEFT,
    "RIGHT": RIGHT,
}


AVIARY_SYSTEMS = {
    "opt": ("OPT", r"\text{Optimizer}"),
    "geom": ("FUNC", r"\text{Geometry}"),
    "mass": ("FUNC", r"\text{Mass}"),
    "aero": ("FUNC", r"\text{Aerodynamics}"),
    "prop": ("FUNC", r"\text{Propulsion}"),
    "mission": ("FUNC", r"\text{Mission / Trajectory}"),
    "post": ("FUNC", r"\text{Post Mission}"),
}


VARIABLE_LABELS = {
    "aircraft:wing:span": r"b",
    "aircraft:wing:area": r"S_w",
    "aircraft:wing:aspect_ratio": r"AR",
    "aircraft:wing:wetted_area": r"S_{wet,w}",
    "aircraft:engine:scale_factor": r"f_{eng}",
    "aircraft:engine:scaled_sls_thrust": r"T_{SLS}",
    "aircraft:design:gross_mass": r"W_0",
    "aircraft:design:empty_mass": r"W_e",
    "aircraft:design:wing_loading": r"W/S",
    "aircraft:design:thrust_to_weight_ratio": r"T/W",
    "aircraft:fuel:total_capacity": r"W_{fuel,cap}",
    "mission:gross_mass": r"W_{0,mission}",
    "mission:range": r"R",
    "mission:total_fuel": r"W_{fuel}",
    "mission:objectives:fuel": r"J_{fuel}",
    "mission:objectives:range": r"J_R",
}


def latex_label(name):
    if name in VARIABLE_LABELS:
        return VARIABLE_LABELS[name]

    short_name = name.split(":")[-1].replace("_", r"\_")
    return rf"\text{{{short_name}}}"


def system_from_path(abs_name):
    lowered = abs_name.lower()

    if ".geometry." in lowered or "core_subsystems.geometry" in lowered:
        return "geom"
    if ".mass." in lowered or "core_subsystems.mass" in lowered:
        return "mass"
    if ".aerodynamics." in lowered or "core_subsystems.aerodynamics" in lowered:
        return "aero"
    if ".propulsion." in lowered or "engine" in lowered:
        return "prop"
    if lowered.startswith("traj.") or ".traj." in lowered:
        return "mission"
    if "post_mission" in lowered or "range_obj" in lowered or "fuel_obj" in lowered:
        return "post"

    return None


def promoted_io(model):
    inputs = {}
    outputs = {}

    for abs_name, meta in model.list_inputs(prom_name=True, units=True, out_stream=None):
        inputs[abs_name] = {
            "prom_name": meta.get("prom_name", abs_name),
            "system": system_from_path(abs_name),
        }

    for abs_name, meta in model.list_outputs(prom_name=True, units=True, out_stream=None):
        outputs[abs_name] = {
            "prom_name": meta.get("prom_name", abs_name),
            "system": system_from_path(abs_name),
        }

    return inputs, outputs


def merge_labels(existing, label, max_items=5):
    if not existing:
        labels = []
    else:
        labels = [item.strip() for item in existing.split(",")]

    if label not in labels and len(labels) < max_items:
        labels.append(label)
    elif label not in labels and r"\ldots" not in labels:
        labels.append(r"\ldots")

    return ", ".join(labels)


def create_xdsm_from_problem(
    prob,
    filename=None,
    output_dir="xdsm_outputs",
    build_pdf=True,
    quiet=False,
):
    """
    Build a high-level XDSM from an already configured OpenMDAO/Aviary problem.

    The problem must have completed setup, because this function reads OpenMDAO's
    design variables, objectives, constraints, promoted inputs/outputs, and
    absolute input-to-output connections.
    """

    model = prob.model
    problem_name = getattr(prob, "_name", None) or "aviary_problem"
    filename = filename or f"{problem_name}_auto_xdsm"

    systems = dict(AVIARY_SYSTEMS)
    connections = {}

    inputs_meta, outputs_meta = promoted_io(model)
    input_prom_to_systems = {}

    for meta in inputs_meta.values():
        system = meta["system"]
        prom_name = meta["prom_name"]
        if system:
            input_prom_to_systems.setdefault(prom_name, set()).add(system)

    design_vars = model.get_design_vars()
    objectives = model.get_objectives()
    constraints = model.get_constraints()

    for var_name in design_vars:
        for target_system in input_prom_to_systems.get(var_name, set()):
            label = latex_label(var_name)
            key = ("opt", target_system)
            connections[key] = merge_labels(connections.get(key), label)

    source_to_system = {
        abs_name: meta["system"]
        for abs_name, meta in outputs_meta.items()
        if meta["system"]
    }

    for abs_input, abs_output in model._conn_global_abs_in2out.items():
        source_system = source_to_system.get(abs_output)
        target_system = inputs_meta.get(abs_input, {}).get("system")

        if not source_system or not target_system or source_system == target_system:
            continue

        prom_name = inputs_meta[abs_input]["prom_name"]
        key = (source_system, target_system)
        connections[key] = merge_labels(connections.get(key), latex_label(prom_name))

    for var_name in objectives:
        for source_system in input_prom_to_systems.get(var_name, {"post", "mission"}):
            key = (source_system, "opt")
            connections[key] = merge_labels(connections.get(key), latex_label(var_name))

    for var_name in constraints:
        for source_system in input_prom_to_systems.get(var_name, {"post", "mission"}):
            key = (source_system, "opt")
            connections[key] = merge_labels(connections.get(key), latex_label(var_name))

    used_systems = {"opt"}
    for source, target in connections:
        used_systems.add(source)
        used_systems.add(target)

    systems = {
        system_id: system_data
        for system_id, system_data in systems.items()
        if system_id in used_systems
    }

    inputs = [
        ("opt", merge_labels("", latex_label(var_name)), "LEFT")
        for var_name in design_vars
    ]

    outputs = [
        ("opt", merge_labels("", latex_label(var_name)), "RIGHT")
        for var_name in objectives
    ]

    return create_xdsm(
        filename=filename,
        systems=systems,
        connections=connections,
        inputs=inputs,
        outputs=outputs,
        output_dir=output_dir,
        build_pdf=build_pdf,
        quiet=quiet,
    )


def create_xdsm_from_script(
    script_path,
    prob_name="prob",
    filename=None,
    output_dir="xdsm_outputs",
    build_pdf=True,
    quiet=False,
):
    """
    Execute an optimization script and create an XDSM from its global problem object.

    Prefer a lightweight script hook named build_xdsm_problem() so the model can be
    assembled for XDSM without running the full optimization. It also works with
    Aviary example scripts that end with:
        if __name__ == "__main__":
            prob = main()
    """

    if XDSM is None:
        raise RuntimeError(
            "pyXDSM is not installed in this Python environment. "
            "Install it or run this script from an environment that provides pyxdsm."
        )

    script_path = Path(script_path)
    namespace = runpy.run_path(str(script_path), run_name="__xdsm__")

    build_xdsm_problem = namespace.get("build_xdsm_problem")
    if build_xdsm_problem is not None:
        prob = build_xdsm_problem()
    else:
        namespace = runpy.run_path(str(script_path), run_name="__main__")
        prob = namespace.get(prob_name)

    if prob is None:
        raise RuntimeError(
            f"Could not find a global '{prob_name}' in {script_path}. "
            "Return the AviaryProblem from main() and assign it to that name."
        )

    filename = filename or f"{script_path.stem}_auto_xdsm"

    return create_xdsm_from_problem(
        prob=prob,
        filename=filename,
        output_dir=output_dir,
        build_pdf=build_pdf,
        quiet=quiet,
    )


def create_xdsm(
    filename,
    systems,
    connections,
    inputs=None,
    outputs=None,
    output_dir="xdsm_outputs",
    use_sfmath=True,
    build_pdf=True,
    quiet=False,
):
    if XDSM is None:
        raise RuntimeError(
            "pyXDSM is not installed in this Python environment. "
            "Install it or run this script from an environment that provides pyxdsm."
        )

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    x = XDSM(use_sfmath=use_sfmath)

    # --- systems ---
    for system_id, (system_type, system_label) in systems.items():
        if system_type not in SYSTEM_TYPE_MAP:
            raise ValueError(
                f"Invalid system type '{system_type}' for '{system_id}'"
            )

        x.add_system(system_id, SYSTEM_TYPE_MAP[system_type], system_label)

    # --- connections ---
    connection_list = normalize_connections(connections)

    for source, target, label in connection_list:
        validate_system_exists(source, systems, "connection source")
        validate_system_exists(target, systems, "connection target")
        x.connect(source, target, label)

    # --- inputs ---
    if inputs:
        for item in inputs:
            system_id, label, side = normalize_io_item(item, default_side="LEFT")
            x.add_input(system_id, label, SIDE_MAP[side])

    # --- outputs ---
    if outputs:
        for item in outputs:
            system_id, label, side = normalize_io_item(item, default_side="RIGHT")
            x.add_output(system_id, label, SIDE_MAP[side])

    # --- write locally first ---
    x.write(filename, build=False, quiet=quiet)

    # --- move files into xdsm_outputs ---
    for suffix in [".tex", ".tikz"]:
        src = Path(f"{filename}{suffix}")
        dst = output_path / src.name

        if dst.exists():
            dst.unlink()

        src.rename(dst)

    tikz_file = output_path / f"{filename}.tikz"

    # --- FIX 1: copy diagram_styles.tex locally ---
    pyxdsm_dir = Path(pyxdsm.__file__).resolve().parent
    style_src = pyxdsm_dir / "diagram_styles.tex"
    style_dst = output_path / "diagram_styles.tex"

    style_dst.write_text(style_src.read_text(encoding="utf-8"), encoding="utf-8")

    # --- FIX 2: patch tikz file (remove absolute Windows path) ---
    tikz_text = tikz_file.read_text(encoding="utf-8")

    tikz_text = tikz_text.replace(
        str(style_src.with_suffix("")).replace("\\", "/"),
        "diagram_styles",
    )

    tikz_text = tikz_text.replace(
        str(style_src).replace("\\", "/"),
        "diagram_styles.tex",
    )

    tikz_file.write_text(tikz_text, encoding="utf-8")

    tex_file = output_path / f"{filename}.tex"

    # --- build PDF via Docker ---
    if build_pdf:
        docker_config_dir = output_path / ".docker"
        docker_config_dir.mkdir(exist_ok=True)

        command = [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{output_path.resolve()}:/workdir",
            "-w",
            "/workdir",
            "texlive/texlive",
            "pdflatex",
            tex_file.name,
        ]

        print("\nBuilding PDF with Docker:")
        print(" ".join(command))

        env = os.environ.copy()
        env["DOCKER_CONFIG"] = str(docker_config_dir.resolve())

        try:
            subprocess.run(command, check=True, env=env)
        except (FileNotFoundError, subprocess.CalledProcessError) as err:
            print("\nPDF build skipped/failed.")
            print("Generated .tex and .tikz files are still available.")
            print("Start Docker Desktop and rerun without --no-pdf, or rerun with --no-pdf.")
            print(f"Docker error: {err}")
            build_pdf = False

    if not quiet:
        print(f"\nCreated:")
        print(f"  {tex_file}")
        print(f"  {tikz_file}")
        if build_pdf:
            print(f"  {output_path / (filename + '.pdf')}")

    return x

def normalize_connections(connections):
    if isinstance(connections, dict):
        return [
            (source, target, label)
            for (source, target), label in connections.items()
        ]

    normalized = []

    for connection in connections:
        if len(connection) != 3:
            raise ValueError(
                f"Invalid connection format: {connection}. "
                "Expected: (source, target, label)"
            )

        normalized.append(connection)

    return normalized


def normalize_io_item(item, default_side):
    if len(item) == 2:
        system_id, label = item
        side = default_side
    elif len(item) == 3:
        system_id, label, side = item
    else:
        raise ValueError(
            f"Invalid input or output format: {item}. "
            "Expected: (system_id, label) or (system_id, label, side)"
        )

    side = side.upper()

    if side not in SIDE_MAP:
        raise ValueError(
            f"Invalid side '{side}'. Allowed sides are: {list(SIDE_MAP.keys())}"
        )

    return system_id, label, side


def validate_system_exists(system_id, systems, context):
    if system_id not in systems:
        raise ValueError(
            f"Unknown system '{system_id}' used as {context}. "
            f"Known systems are: {list(systems.keys())}"
        )


def create_small_cargo_xdsm():
    systems = {
        "opt": ("OPT", r"\text{Optimizer}"),
        "geo": ("FUNC", r"\text{Geometry}"),
        "mass": ("FUNC", r"\text{Mass}"),
        "aero": ("FUNC", r"\text{Aerodynamics}"),
        "prop": ("FUNC", r"\text{Propulsion}"),
        "traj": ("FUNC", r"\text{Trajectory}"),
    }

    connections = {
        ("opt", "geo"): r"S_w, b",
        ("opt", "prop"): r"f_{scale}",
        ("geo", "mass"): r"S_w, L_{fus}",
        ("geo", "aero"): r"S_w, AR",
        ("mass", "traj"): r"W_{OEW}",
        ("aero", "traj"): r"C_D",
        ("prop", "traj"): r"T, \dot{m}_f",
        ("traj", "opt"): r"W_{fuel}",
    }

    inputs = [
        ("opt", r"W_{cargo}, Range", "LEFT"),
    ]

    outputs = [
        ("opt", r"W_{fuel}^{*}", "RIGHT"),
        ("traj", r"W_{fuel}", "LEFT"),
    ]

    create_xdsm(
        filename="small_cargo_xdsm",
        systems=systems,
        connections=connections,
        inputs=inputs,
        outputs=outputs,
        build_pdf=True,
        quiet=False,
    )


def create_small_uav_xdsm():
    systems = {
        "opt": ("OPT", r"\text{Optimizer}"),
        "geo": ("FUNC", r"\text{Geometry}"),
        "mass": ("FUNC", r"\text{Mass Estimation}"),
        "engine": ("FUNC", r"\text{Engine Performance}"),
        "aero": ("FUNC", r"\text{Aerodynamics}"),
        "mission": ("FUNC", r"\text{Mission Analysis}"),
    }

    connections = {
        ("opt", "geo"): r"S_w, b, t/c",
        ("opt", "engine"): r"f_{scale}",
        ("geo", "mass"): r"S_w, L_{fus}",
        ("geo", "aero"): r"S_w, AR, t/c",
        ("mass", "mission"): r"W_{OEW}",
        ("engine", "mission"): r"T, \dot{m}_f",
        ("aero", "mission"): r"C_D",
        ("mission", "opt"): r"Range, Endurance",
    }

    inputs = [
        ("opt", r"Cargo, Altitude", "LEFT"),
    ]

    outputs = [
        ("mission", r"Range", "RIGHT"),
    ]

    create_xdsm(
        filename="small_uav_xdsm",
        systems=systems,
        connections=connections,
        inputs=inputs,
        outputs=outputs,
        build_pdf=True,
        quiet=False,
    )

def build_pdf_with_docker(tex_file_path):
    tex_file_path = Path(tex_file_path)

    command = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{tex_file_path.parent.resolve()}:/workdir",
        "-w",
        "/workdir",
        "texlive/texlive",
        "pdflatex",
        tex_file_path.name,
    ]

    print("\nBuilding PDF with Docker:")
    print(" ".join(command))

    subprocess.run(command, check=True)

    

def main():
    parser = argparse.ArgumentParser(description="Create pyXDSM diagrams.")
    parser.add_argument(
        "--script",
        help="Run an optimization setup script and infer an XDSM from its global problem object.",
    )
    parser.add_argument(
        "--prob-name",
        default="prob",
        help="Global variable name containing the AviaryProblem after the script runs.",
    )
    parser.add_argument("--filename", help="Output filename stem. Defaults to <script>_auto_xdsm.")
    parser.add_argument("--output-dir", default="xdsm_outputs")
    parser.add_argument("--no-pdf", action="store_true", help="Skip Docker PDF build.")
    parser.add_argument(
        "--examples",
        action="store_true",
        help="Generate the legacy hard-coded example XDSMs.",
    )

    args = parser.parse_args()

    if args.script:
        create_xdsm_from_script(
            script_path=args.script,
            prob_name=args.prob_name,
            filename=args.filename,
            output_dir=args.output_dir,
            build_pdf=not args.no_pdf,
            quiet=False,
        )
    elif args.examples:
        create_small_cargo_xdsm()
        create_small_uav_xdsm()
    else:
        parser.print_help()

    print("\nDone.")


if __name__ == "__main__":
    main()
