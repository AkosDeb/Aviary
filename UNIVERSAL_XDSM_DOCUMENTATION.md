# XDSM Visualization Workflow (pyXDSM + Docker LaTeX)

## Overview

This document describes the complete workflow for generating XDSM (eXtended Design Structure Matrix) diagrams using **pyXDSM** and compiling them into PDFs using a **Dockerized LaTeX environment**.

The system is designed to be:

* Portable across machines
* Independent of local LaTeX installations
* Compatible with Windows + Docker setups
* Clean and reproducible for documentation pipelines

---

## Workflow Architecture

```text
Define MDO structure (Python)
        ↓
Generate .tex + .tikz (pyXDSM)
        ↓
Patch dependencies (diagram_styles.tex)
        ↓
Compile via Docker (pdflatex)
        ↓
Final PDF diagram
```

---

## File Structure

After execution, all outputs are stored in:

```text
xdsm_outputs/
    small_cargo_xdsm.tex
    small_cargo_xdsm.tikz
    small_cargo_xdsm.pdf
    small_uav_xdsm.tex
    small_uav_xdsm.tikz
    small_uav_xdsm.pdf
    diagram_styles.tex
```

---

## How It Works

### 1. Define Systems

Each system represents a block in the XDSM:

```python
systems = {
    "opt": ("OPT", r"\text{Optimizer}"),
    "geo": ("FUNC", r"\text{Geometry}"),
    "aero": ("FUNC", r"\text{Aerodynamics}"),
}
```

Supported types:

* `OPT` → Optimizer
* `FUNC` → Functional discipline
* `SOLVER` → Solver block

---

### 2. Define Connections

Data flow between systems:

```python
connections = {
    ("opt", "geo"): r"S_w, b",
    ("geo", "aero"): r"S_w, AR",
}
```

---

### 3. Define Inputs / Outputs

External interfaces:

```python
inputs = [("opt", r"W_{cargo}, Range")]
outputs = [("opt", r"W_{fuel}^{*}", "RIGHT")]
```

---

### 4. Generate XDSM

Call:

```python
create_xdsm(
    filename="small_cargo_xdsm",
    systems=systems,
    connections=connections,
    inputs=inputs,
    outputs=outputs,
    build_pdf=True,
)
```

---

## Docker-Based PDF Compilation

### Why Docker

pyXDSM generates LaTeX files, but:

* Local LaTeX installations are inconsistent
* Windows paths break LaTeX imports
* Reproducibility is poor

Solution: use **Docker + TeXLive**

---

### Command Used Internally

```bash
docker run --rm \
  -v "<repo>/xdsm_outputs:/workdir" \
  -w /workdir \
  texlive/texlive \
  pdflatex small_cargo_xdsm.tex
```

---

## Critical Fixes Implemented

### 1. Path Issue (Windows → Docker)

Problem:

```latex
\input{C:/Users/.../pyxdsm/diagram_styles}
```

Docker cannot access local Python paths.

### Solution

* Copy `diagram_styles.tex` locally into `xdsm_outputs/`
* Replace absolute paths with:

```latex
\input{diagram_styles}
```

---

### 2. File Location Issue

Problem:

* pyXDSM embeds folder paths into `.tex`

Solution:

* Generate files in root
* Move them into `xdsm_outputs/`
* Patch `.tikz` references

---

### 3. Dependency Injection

Automatically copy:

```text
pyxdsm/diagram_styles.tex
```

into:

```text
xdsm_outputs/diagram_styles.tex
```

---

## Running the Tool

From repository root:

```bash
python universal_xdsm_creator.py
```

---

## Adding New XDSM Cases

Define a new function:

```python
def create_my_case():
    systems = {...}
    connections = {...}
    inputs = [...]
    outputs = [...]

    create_xdsm(
        filename="my_case",
        systems=systems,
        connections=connections,
        inputs=inputs,
        outputs=outputs,
    )
```

---

## Design Principles

### Centralized Logic

* All generation and compilation happens inside `create_xdsm()`

### Separation of Concerns

* Case functions define only data
* Core handles execution

### Reproducibility

* No dependency on local LaTeX
* Docker ensures consistent builds

### Portability

* Works across machines without configuration

---

## Troubleshooting

### Error: `diagram_styles.tex not found`

Cause:

* pyXDSM path not patched

Fix:

* Ensure file exists in `xdsm_outputs/`

---

### Docker fails

Check:

```bash
docker run hello-world
```

---

### No PDF generated

Check `.log` file:

```text
xdsm_outputs/small_cargo_xdsm.log
```

---

## Summary

This system provides a fully automated pipeline:

* Define MDO system → Python
* Generate XDSM → pyXDSM
* Fix dependencies → internal patching
* Compile → Docker LaTeX
* Output → PDF ready for documentation

No manual LaTeX handling required.
