# Universal XDSM Workflow

This guide describes the Aviary XDSM workflow built around `universal_xdsm_creator.py`.
The preferred workflow is automatic: run an Aviary/OpenMDAO optimization script, inspect
the configured problem, and generate a high-level XDSM without writing a new case-specific
XDSM function.

The tool can also compile the generated LaTeX into a PDF using Docker and TeXLive.

---

## What It Creates

All generated files are written to:

```text
xdsm_outputs/
```

For a filename such as `small_uav_auto_xdsm`, the expected files are:

```text
xdsm_outputs/small_uav_auto_xdsm.tex
xdsm_outputs/small_uav_auto_xdsm.tikz
xdsm_outputs/small_uav_auto_xdsm.pdf
```

The PDF is only created when Docker is available and the command is run without
`--no-pdf`.

---

## Requirements

Run the tool from the same Python environment used for Aviary:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe -m pip install pyxdsm
```

For PDF output, Docker Desktop must be installed, running, and accessible from the
terminal user.

Check Docker with:

```powershell
docker info
```

---

## Automatic XDSM From a Setup Script

The optimization script should expose the final `AviaryProblem` object as `prob`.
This is the common pattern:

```python
if __name__ == "__main__":
    prob = main()
```

Then run:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm
```

To generate only `.tex` and `.tikz` and skip Docker/PDF compilation:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm `
  --no-pdf
```

If a script stores the problem under a different global name, pass it explicitly:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script my_case.py `
  --prob-name my_prob `
  --filename my_case_auto_xdsm
```

---

## Example Commands

### Small UAV

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_uav/run_small_uav_mission.py `
  --filename small_uav_auto_xdsm
```

### Test Aircraft

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/test_aircraft/test_aircraft_run_optimization.py `
  --filename test_aircraft_auto_xdsm
```

### Small Cargo

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py `
  --script aviary/models/aircraft/small_cargo/run_small_cargo_optimization.py `
  --filename small_cargo_auto_xdsm
```

Add `--no-pdf` to any command when Docker is unavailable.

---

## How Automatic Inference Works

The automatic path runs the setup script and inspects the configured OpenMDAO model.
It reads:

* design variables
* objectives
* constraints
* promoted input and output names
* absolute OpenMDAO connections
* subsystem paths for geometry, mass, aerodynamics, propulsion, trajectory, and post-mission logic

It then groups low-level OpenMDAO systems into high-level XDSM blocks:

```text
Optimizer
Geometry
Mass
Aerodynamics
Propulsion
Mission / Trajectory
Post Mission
```

This avoids adding a new hand-written `create_my_case_xdsm()` function for each new
aircraft or mission setup.

---

## Docker PDF Compilation

When PDF output is enabled, the tool runs a Dockerized TeXLive command similar to:

```powershell
docker run --rm `
  -v C:\Software\Repository\Aviary_clean\xdsm_outputs:/workdir `
  -w /workdir `
  texlive/texlive `
  pdflatex small_uav_auto_xdsm.tex
```

The Docker container sees `xdsm_outputs/` as `/workdir`, compiles the `.tex`, and writes
the PDF back into `xdsm_outputs/`.

The tool also copies `diagram_styles.tex` into `xdsm_outputs/` so LaTeX does not depend
on an absolute Python site-package path.

---

## Manual XDSM Definitions

Manual case functions are still supported for custom diagrams. Define systems,
connections, inputs, and outputs, then call `create_xdsm()`:

```python
def create_my_case():
    systems = {
        "opt": ("OPT", r"\text{Optimizer}"),
        "geo": ("FUNC", r"\text{Geometry}"),
        "aero": ("FUNC", r"\text{Aerodynamics}"),
    }

    connections = {
        ("opt", "geo"): r"S_w, b",
        ("geo", "aero"): r"S_w, AR",
    }

    inputs = [("opt", r"W_{cargo}, R", "LEFT")]
    outputs = [("opt", r"J^*", "RIGHT")]

    create_xdsm(
        filename="my_case_xdsm",
        systems=systems,
        connections=connections,
        inputs=inputs,
        outputs=outputs,
        build_pdf=True,
    )
```

To generate the legacy hard-coded examples:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe universal_xdsm_creator.py --examples
```

---

## Troubleshooting

### `pyXDSM is not installed`

Install `pyxdsm` into the same environment used to run the tool:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe -m pip install pyxdsm
```

Installing with plain `pip install pyxdsm` may install into a different Python.

### Docker Cannot Connect

Typical error:

```text
failed to connect to the docker API
permission denied while trying to connect to the docker API
```

Fixes:

* Start Docker Desktop.
* Wait until Docker Desktop reports that it is running.
* Run `docker info` from the same terminal.
* Make sure the terminal user belongs to Docker's allowed user group.

If Docker is unavailable, rerun with:

```powershell
--no-pdf
```

The `.tex` and `.tikz` files will still be generated.

### No PDF Generated

Check whether the PDF exists:

```powershell
Get-ChildItem xdsm_outputs/*auto_xdsm*
```

If only `.tex` and `.tikz` exist, Docker or LaTeX compilation failed.

You can manually compile an existing generated file once Docker is running:

```powershell
docker run --rm `
  -v C:\Software\Repository\Aviary_clean\xdsm_outputs:/workdir `
  -w /workdir `
  texlive/texlive `
  pdflatex small_uav_auto_xdsm.tex
```

### `diagram_styles.tex not found`

Make sure `xdsm_outputs/diagram_styles.tex` exists. The tool copies it automatically
from the installed `pyxdsm` package.

---

## Summary

The automatic workflow is:

```text
Aviary setup script
  -> configured OpenMDAO problem
  -> inferred high-level XDSM
  -> .tex and .tikz
  -> optional Docker PDF
```

This keeps XDSM generation reusable across aircraft and mission scripts without
adding custom XDSM code to each setup.
