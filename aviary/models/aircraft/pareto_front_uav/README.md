# Pareto Front UAV — Aspect Ratio Trade-off Study

Forked from `horizontal_small_uav`. Demonstrates multi-objective design space
exploration using a parametric aspect-ratio sweep with IPOPT inner optimisation.

---

## What it does

Wing reference area is held fixed at **Sref = 0.45 m²** throughout.  
For each aspect ratio in the sweep, wingspan is set to:

```
b = sqrt(AR × Sref)
```

IPOPT then maximises **range** by optimising the remaining design variables
(engine scaled SLS thrust, wing centre-of-pressure location) subject to the
same constraints as the baseline model (static margin, engine mass budget,
fuel budget).

The result is a **Pareto front** in the two-objective space:

| Objective | Direction | Driver |
|-----------|-----------|--------|
| Range (km) | maximise | lower induced drag at high AR |
| Wing structural mass (kg) | minimise | shorter span = lighter wing |

These objectives conflict: increasing AR reduces induced drag (CDi ∝ 1/AR) and
improves range, but lengthens the span and increases structural mass.  Every
point on the front is optimal — you cannot gain range without paying in wing
mass, and vice versa.

---

## Files

| File | Purpose |
|------|---------|
| `run_pareto_front_uav.py` | Main sweep script — run this |
| `pareto_front_uav.csv` | Aircraft input deck (Sref = 0.45 m², span overridden per AR point) |
| `phase_info.py` | Dymos phase definitions (climb → cruise → accel → dash) |

---

## How to run

```bash
python aviary/models/aircraft/pareto_front_uav/run_pareto_front_uav.py
```

Each AR point launches a full Aviary FALLOUT solve (~1–3 min each depending on
convergence). Points that do not converge (typically very low AR < 2) are
skipped and reported.

---

## Outputs

Written to `outputs/` at the repository root:

| Output | Description |
|--------|-------------|
| `pareto_front_uav_pareto.csv` | Table of AR, span, chord, range, wing mass, CDi |
| `pareto_front_uav_pareto.png` | Two-panel plot (see below) |

### Plot panels

**Left — Pareto front** (`wing_mass` vs `range`)  
Each dot is one AR value. The curve shows the trade-off boundary: designs
below-left of the curve are unachievable; designs above-right are dominated.

**Right — AR sensitivity** (triple y-axis)  
- Range (km) — blue, left axis  
- Wing structural mass (kg) — orange, right axis  
- Mean cruise CDi — green, offset right axis  

The CDi curve confirms the 1/AR relationship; the wing mass curve shows the
structural penalty; the range curve shows the net aerodynamic benefit.

---

## Key parameters (top of `run_pareto_front_uav.py`)

```python
WING_AREA_M2 = 0.45          # fixed Sref [m^2]
AR_SWEEP = np.array([...])   # AR values to evaluate
```

Adjust `AR_SWEEP` to change resolution or extend the range. Very low AR
(< 2) tends to cause convergence failures because the short chord pushes
the static-margin constraint to its limits.

---

## Relationship to `horizontal_small_uav`

The only structural difference is that `Aircraft.Wing.SPAN` is **not** a
design variable here — it is computed from AR and set as a fixed parameter
before each IPOPT solve.  All constraints, phase structure, and the
`SmallTurbojetModel` external subsystem are identical.
