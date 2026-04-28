"""
Universal XDSM creator.

This script creates XDSM diagrams from a generic system and connection definition.

Run:
    python scripts/create_xdsm.py

Output:
    xdsm_outputs/small_cargo_xdsm.tex
    xdsm_outputs/small_uav_xdsm.tex
"""

from pathlib import Path
import subprocess
import pyxdsm

from pyxdsm.XDSM import XDSM, FUNC, LEFT, OPT, RIGHT, SOLVER


SYSTEM_TYPE_MAP = {
    "OPT": OPT,
    "FUNC": FUNC,
    "SOLVER": SOLVER,
}


SIDE_MAP = {
    "LEFT": LEFT,
    "RIGHT": RIGHT,
}


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
    import pyxdsm

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

        subprocess.run(command, check=True)

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

    

if __name__ == "__main__":
    create_small_cargo_xdsm()
    create_small_uav_xdsm()

    print("\nDone.")