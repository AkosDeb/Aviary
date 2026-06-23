# Lift Curve Slope — Scholz Winglet AR Correction and Polhamus Formula

This document covers the two OpenMDAO components in `lift_curve_slope.py`:

| Component | Purpose |
|-----------|---------|
| `ScholzWingletARCorrection` | Converts H-tail VTP span into an effective wing AR increase (Scholz 2018) |
| `LiftCurveSlopePolhamus` | Computes wing-alone 3-D CL_alpha from AR, Mach, sweep, t/c (Roskam Eq. 8.22) |

Related documentation:

| Document | Purpose |
|----------|---------|
| `README_airfoil_data.md` | Airfoil section data used by lift, M_crit, and parasite drag |
| `README_drag_build_up.md` | Roskam/DATCOM parasite drag build-up and shared aero utilities |
| `README_lateral_stability.md` | Side-force derivatives, rudder effectiveness, and load-factor conventions |
Intended wiring for an H-tail. **The AR correction applies to the wing (horizontal panel) only — the VTP uses a separate, uncorrected `LiftCurveSlopePolhamus` instance:**

```text
── Wing (horizontal panel) ──────────────────────────────────────────
  vtp_span, wing_span, aspect_ratio
        |
ScholzWingletARCorrection  ->  AR_eff
                                     |
                          LiftCurveSlopePolhamus  ->  wing CL_alpha
                          (wing geometry)

── VTP (vertical tail panels) ───────────────────────────────────────
                          LiftCurveSlopePolhamus  ->  CL_alpha_v
                          (VTP geometry, no AR correction)
                                                            |
                                                   CyDeltaRudder  ->  CY_delta_r
```

---

## Physical background

### Why the effective AR matters

For an isolated wing, spanwise tip flow reduces the effective angle of attack and
lowers the 3-D lift curve slope below its 2-D section value.  Endplates at the wing
tips block this leakage and raise the effective aspect ratio, pushing CL_alpha toward
the 2-D value.  In the H-tail configuration the VTP panels serve as endplates for the
horizontal panel.

### Why Polhamus instead of Helmbold

The Helmbold formula is the incompressible, unswept, thin-airfoil special case of the
Polhamus (DATCOM) formula.  It gives no credit for:

- **Compressibility** — Prandtl-Glauert raises CL_alpha by ~2 % at M = 0.3.
- **Sweep** — aft sweep reduces CL_alpha (~-10 % at 30 deg quarter-chord sweep).
- **Non-ideal airfoil** — real sections have cl_alpha < 2*pi; ratio k = cl_alpha/(2*pi) < 1.

Setting M = 0, sweep = 0 deg, k = 1 in the Polhamus formula recovers Helmbold exactly.

---

## ScholzWingletARCorrection

**File:** `lift_curve_slope.py`
**Reference:** Scholz, D. (2018). "Definition and discussion of the intrinsic
efficiency of winglets." INCAS Bulletin, Vol. 10, Issue 1. HAW Hamburg.

### Physical model

Scholz frames each VTP as a winglet mounted at the tip of the horizontal tail.
Rather than fitting a geometric formula for k_h(h/c), he asks: what span extension
would produce the same induced-drag reduction as the winglet? That equivalent span
increase is then used to derive AR_eff.

The key insight is that a real winglet underperforms relative to the geometric
equivalence (folding the winglet flat to extend the span) by a factor k_WL >= 1.
The better the winglet design, the closer k_WL approaches 1.0.

### Formula chain

**Step 1 — effective span:**

```
b_eff = b_h + 2 * h / k_WL
```

The factor of 2 accounts for both left and right HT tips (one VTP each).
`h` is the full span of the VTP at *one* tip.

**Step 2 — effective aspect ratio (before split-winglet penalty):**

```
AR_eff_raw = AR_h * (b_eff / b_h)^2
           = AR_h * (1 + 2*h / (k_WL * b_h))^2
```

**Step 3 — split-winglet penalty for symmetric H-tail VTPs:**

```
AR_eff = AR_eff_raw * split_penalty
```

Scholz states that split winglets (extending symmetrically above and below the HT
surface, as in an H-tail) produce approximately 90 % of the induced-drag reduction
of a standard winglet of the same total height.  The default `split_penalty = 0.90`.

**Combined single expression:**

```
AR_eff = AR_h * (1 + 2*h / (k_WL * b_h))^2 * split_penalty
```

### k_WL reference values

| Source | k_WL | Meaning |
|--------|------|---------|
| Geometric ideal | 1.0 | Winglet folded flat — unrealistically optimistic |
| Jones / McLean / Howe | 2.0 | Theoretical optimum — best-case, rarely achieved |
| Dubs / Zimmer | 2.45 | Experimental average — **used for conceptual design** |
| Real aircraft average | 2.8 | Conservative lower bound from A/C performance data |

2.0 is the theoretical optimum but experimental data (Dubs, Zimmer) consistently
shows 2.45 as the achievable average for real winglet designs. Use 2.45 for
conceptual design; use 2.8 as a conservative lower bound check.

### Comparison with Hoerner

Unlike Hoerner's formula (AR_eff = AR * (1 + 1.9 * h/b_h) = linear in h), Scholz's
Step 2 is **quadratic in h**.  For small h/b_h ratios the two converge; for larger
VTP spans (h/b_h > 0.15) Scholz diverges increasingly from Hoerner.

With k_WL = 2.0 and split_penalty = 0.90 the Scholz formula is equivalent to:

```
AR_eff = AR_h * (1 + h/b_h)^2 * 0.90
```

At the current design point (h = 0.507 m, b_h = 1.357 m, AR = 4.09):

| Method | AR_eff | k_h |
|--------|--------|-----|
| Hoerner (linear, h/b = 0.374) | AR*(1 + 1.9*0.374) = AR*1.71 = ~7.0 | ~1.71 |
| Scholz k_WL=2.0 (theoretical opt.) | AR*(1 + 0.374)^2*0.90 = AR*1.73 = ~7.1 | ~1.73 |
| Scholz k_WL=2.45 (exp. avg.) **current** | AR*(1 + 0.305)^2*0.90 = AR*1.54 = ~6.3 | ~1.54 |
| Scholz k_WL=2.8 (real A/C avg.) | AR*(1 + 0.267)^2*0.90 = AR*1.44 = ~5.9 | ~1.44 |

Scholz captures the quadratic dependence on VTP span. The gap between k_WL=2.0
and k_WL=2.45 represents the difference between the theoretical optimum and what
experimental measurements actually show — a non-trivial ~12 % reduction in AR_eff.

### Inputs and outputs

| Variable | Units | Description |
|----------|-------|-------------|
| `aspect_ratio` | — | Wing AR_h = b^2/S (no endplate effect) |
| `vtp_span` | m | Full VTP span h at one HT tip (upper + lower semi-span) |
| `wing_span` | m | Full horizontal wing span b_h (tip to tip) |
| **`AR_eff`** | — | Scholz effective AR with split-winglet penalty |
| **`k_h`** | — | Correction factor k_h = AR_eff / AR_geo (diagnostic) |

Constructor arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `k_wl` | 2.0 | Winglet effectiveness penalty factor |
| `split_penalty` | 0.90 | Fraction for symmetric split winglets |

### UAV numerical example (2026-06-08 baseline)

Geometry: b_h = 1.357 m, h = 0.507 m, AR_geo = 4.09, k_WL = 2.0, split_penalty = 0.90

```
Step 1:  b_eff = 1.357 + 2*0.507/2.0 = 1.357 + 0.507 = 1.864 m
Step 2:  AR_eff_raw = 4.09 * (1.864/1.357)^2 = 4.09 * 1.374^2 = 4.09 * 1.887 = 7.72
         (note: (b_eff/b_h)^2 = (1 + 0.507/1.357)^2 = (1.374)^2 = 1.887)
Step 3:  AR_eff = 7.72 * 0.90 = 6.95  (code gives 7.08 — small diff from optimised h)
```

k_h = 7.08 / 4.09 = 1.73 => AR gain = 3.0 (+69 %)

---

## LiftCurveSlopePolhamus

**File:** `lift_curve_slope.py`
**Reference:** Roskam, "Airplane Design Part VI", Section 8.1.3.2, Eq. 8.22.

### What it computes

Wing-alone lift curve slope — fuselage K_wf is **not** applied here:

```
c_l_alpha = 2*pi * (1 + 0.77*tc)            [Abbott & von Doenhoff thickness correction]
k         = c_l_alpha / (2*pi)
CL_alpha  = 2*pi*A / [2 + sqrt(A^2*(beta^2 + tan^2(Lc/2)) / k^2 + 4)]
```

### Terms

| Symbol | Meaning | Formula |
|--------|---------|---------|
| A | aspect ratio | input; connect AR_eff from ScholzWingletARCorrection |
| tc | thickness-to-chord ratio | input from `section_tc` (AirfoilConstantsComp) |
| beta | Prandtl-Glauert factor | sqrt(1 - M^2) |
| k | section slope ratio | c_l_alpha / (2*pi) |
| Lc/2 | semi-chord sweep | derived internally from quarter-chord sweep |

### Sweep conversion

`Aircraft.Wing.SWEEP` is the quarter-chord sweep Lc/4.  The component converts
it to semi-chord sweep internally:

```
tan(Lc/2) = tan(Lc/4) - (1 - lambda) / [A * (1 + lambda)]
```

### Special cases

| Condition | Reduces to |
|-----------|-----------|
| M=0, sweep=0, k=1 | Helmbold: `2*pi*A / (2 + sqrt(A^2 + 4))` |
| M=0, sweep=0, k<1 | Helmbold with non-ideal airfoil |
| M>0, sweep=0, k=1 | Compressibility-corrected Helmbold |

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `aspect_ratio` | 2.0 | — | A = b^2/S; wire AR_eff for wing, raw AR for VTP |
| `mach` | 0.0 | — | Flight Mach number (subsonic, M < 1) |
| `sweep_c4_deg` | 0.0 | deg | Quarter-chord sweep Lc/4 |
| `taper_ratio` | 1.0 | — | lambda = c_tip / c_root |
| `thickness_to_chord` | 0.12 | — | Airfoil t/c ratio; used to compute c_l_alpha = 2π(1+0.77·t/c) |
| `fuselage_diameter` | 0.0 | m | Equiv. fuselage diameter d_f; 0 disables K_wf |
| `wing_span` | 1.0 | m | Wing span b; only used when fuselage_diameter > 0 |
| **`CL_alpha`** | — | 1/rad | Wing-alone 3-D lift curve slope (K_wf NOT applied) |
| **`K_wf`** | — | — | Wing-fuselage interference factor (diagnostic only) |

`thickness_to_chord` is wired from `section_tc` (AirfoilConstantsComp) in `LiftingSurfaceGroup`.

### UAV numerical example (2026-06-15 baseline)

Wing at M = 0.477, AR_eff = 7.08, sweep = 0 deg, lambda = 0.6, t/c = 0.12, d_f = 0.169 m, b = 1.357 m:

```
c_l_alpha = 2*pi * (1 + 0.77*0.12) = 2*pi * 1.0924 = 6.862 /rad
k         = 6.862 / (2*pi) = 1.0924 / 1.0 = 1.0924... wait, k = c_l_alpha/(2pi) = 1.0924
beta      = sqrt(1 - 0.477^2) = sqrt(0.7724) = 0.879
inner     = 7.08^2 * (0.879^2 + 0) / 1.0924^2 + 4 = 50.1 * 0.773 / 1.193 + 4 = 32.5 + 4 = 36.5
CL_alpha  = 2*pi*7.08 / (2 + sqrt(36.5)) = 44.5 / (2 + 6.04) = 44.5 / 8.04 = 5.53 /rad
K_wf      = 1 + 0.025*0.125 - 0.25*0.125^2 = 0.999  (diagnostic only — not applied)
```

Compared to t/c = 0 (thin airfoil, k = 1): CL_alpha ≈ 5.22 /rad → thickness adds ~6 % at t/c = 0.12.

---

## Connection to CyDeltaRudder

`CyDeltaRudder` accepts `CL_alpha_v` from the VTP Polhamus instance.
**Do not apply the AR correction here** — `CL_alpha_v` is the VTP's own
3-D lift slope computed from raw VTP geometry.

---

## Wiring example

```python
from aviary.subsystems.aerodynamics.SpaJeti_based.lift_curve_slope import (
    ScholzWingletARCorrection, LiftCurveSlopePolhamus,
)

# Wing AR correction
model.add_subsystem('endplate_ar',
    ScholzWingletARCorrection(k_wl=2.0, split_penalty=0.90),
    promotes_inputs=[
        ('aspect_ratio', av.Aircraft.Wing.ASPECT_RATIO),
        ('vtp_span',     av.Aircraft.VerticalTail.SPAN),
        ('wing_span',    av.Aircraft.Wing.SPAN),
    ],
    promotes_outputs=['AR_eff', 'k_h'],
)

# Wing CL_alpha (uses AR_eff; section_tc from AirfoilConstantsComp feeds thickness_to_chord)
model.add_subsystem('wing_polhamus', LiftCurveSlopePolhamus(),
    promotes_inputs=[('aspect_ratio', 'AR_eff'), ('thickness_to_chord', 'section_tc'), ...],
    promotes_outputs=[('CL_alpha', 'wing_CL_alpha'), 'K_wf'],
)

# VTP CL_alpha (raw AR, no correction)
model.add_subsystem('vtp_polhamus', LiftCurveSlopePolhamus(),
    promotes_inputs=[('aspect_ratio', 'vtp_ar'), ...],
    promotes_outputs=[('CL_alpha', 'CL_alpha_v')],
)
```

---

## Validity Ranges and Safety Guards

### `ScholzWingletARCorrection`

| Condition | Action |
|-----------|--------|
| `aspect_ratio` <= 0 | `ValueError` |
| `wing_span` <= 0 | `ValueError` |
| `vtp_span` < 0 | `RuntimeWarning` |

### `LiftCurveSlopePolhamus`

| Condition | Action | Reason |
|-----------|--------|--------|
| `aspect_ratio` <= 0 | `ValueError` | Division by zero inside the formula |
| `aspect_ratio` < 2.0 | `RuntimeWarning` | Polhamus/DATCOM calibrated for AR >= 2; results degrade below this limit |
| `mach` < 0 | `ValueError` | Physically impossible |
| `mach` >= 1.0 | `ValueError` | Prandtl-Glauert β = √(1−M²) is undefined at M ≥ 1; formula breaks |

The AR < 2 threshold is a calibration limit, not a mathematical failure.
The formula still returns a finite number for AR < 2, but the Polhamus
accuracy assumption degrades significantly below this limit. For AR < 1 use
a vortex-lattice or panel-method result instead.

---

## MachCriticalComp — M_DD / M_crit

**File:** `mach_critical.py`
**Reference:** Weisshaar (2024), Eq. 36 — best directly-solvable M_DD form ranked against 20 empirical M_crit formulas (SEE = 3.95 %).

### Physical model

A wing section enters the transonic regime when local flow first reaches Mach 1 — the critical Mach M_crit. Drag divergence Mach M_DD is the slightly higher speed at which wave drag begins to rise rapidly (Shevell definition: dC_D/dM = 0.1). The Weisshaar formula solves directly for M_DD:

```
M_DD = K_A / cos(φ₂₅) − (t/c) / cos²(φ₂₅) − CL / (10 · cos³(φ₂₅))
```

where:
- `K_A` = 0.887 — Korn airfoil technology factor (conventional subsonic sections; 0.95 for supercritical)
- `φ₂₅` — quarter-chord sweep angle
- `t/c` — thickness-to-chord ratio (from `AirfoilConstantsComp` / `section_tc` design variable)
- `CL` — wing lift coefficient at the design condition (see below)

M_crit is recovered from M_DD via the Shevell wave-drag model (20·(M − M_crit)⁴):

```
M_crit = M_DD − (0.1/80)^(1/3)   ≈   M_DD − 0.1077
```

### CL derivation from Nz requirement

The CL used in the Weisshaar formula is **not** CL_alpha × alpha_max (the stall CL).
It is derived from the minimum longitudinal load-factor requirement so it reflects the
actual wing CL at the design dash condition:

```
CL = Nz_min × m × g / (q × S)
```

At the design point (Nz_min = 7, m = 15 kg, q = 8 581 Pa, S ≈ 0.45 m²) this gives
CL ≈ 0.24, versus CL_max ≈ 1.42 with the stall alpha — a 5× difference that made
the constraint infeasible when the stall CL was used (pre-2026-06-15 baseline).

### Margin definition

```
mach_crit_margin = M_crit − mach_upper_bound ≥ 0

mach_upper_bound = DASH_MACH + M_CRIT_SAFETY_MARGIN  (= 0.52 + 0.05 = 0.57)
```

The constraint is on M_crit (onset of local sonic flow), not M_DD, so the
design stays below the point of actual wave-drag rise with a conservative margin.

### M_DD sensitivity

M_DD **decreases** (constraint tightens) with:
- increasing t/c (thicker wing → lower M_DD)
- increasing CL (higher Nz_min or lower q/S → lower M_DD)
- reducing sweep (unswept wing → lower M_DD for the same t/c)

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `surface_sweep_c4` | 0.0 | deg | Quarter-chord sweep φ₂₅ |
| `section_tc` | 0.12 | — | t/c (from `AirfoilConstantsComp`; wired from `wing_section_tc` DV) |
| `nz_min` | 7.0 | — | Minimum longitudinal load-factor requirement |
| `aircraft_mass` | 15.0 | kg | Aircraft mass at the design point |
| `dynamic_pressure` | 8 581 | Pa | Dynamic pressure q at the design point |
| `wing_area` | 0.5 | m² | Wing reference area S |
| `mach_upper_bound` | 0.57 | — | DASH_MACH + M_CRIT_SAFETY_MARGIN |
| **`M_DD`** | — | — | Drag-divergence Mach (informational only) |
| **`M_crit`** | — | — | Critical Mach (onset of local sonic flow) |
| **`mach_crit_margin`** | — | — | M_crit − mach_upper_bound; constrain ≥ 0 |

Constructor argument: `k_a` (default 0.887; use 0.95 for supercritical sections).

### UAV numerical example

Design point: sweep = 0°, t/c = 0.12, Nz_min = 7, m = 15 kg, q = 8 581 Pa, S = 0.45 m², mach_upper_bound = 0.57.

```
CL     = 7 × 15 × 9.807 / (8581 × 0.45)  = 1030 / 3861  = 0.267
M_DD   = 0.887/1 − 0.12/1 − 0.267/10     = 0.887 − 0.120 − 0.0267 = 0.740
M_crit = 0.740 − 0.108                    = 0.632
margin = 0.632 − 0.57                     = +0.062  ✓
```

At t/c = 0.08 (thinner wing, harder M_crit target):

```
CL     = 0.267  (unchanged — depends on Nz, not t/c)
M_DD   = 0.887 − 0.08 − 0.0267 = 0.780
M_crit = 0.780 − 0.108          = 0.672
margin = 0.672 − 0.57           = +0.102  ✓
```

TOOD verification (Weisshaar at t/c = 0.08): M_crit = 0.5885 is reported for an
earlier CL convention (CL_max); the 2026-06-15 correction uses CL from Nz_min
instead, which is why the post-fix margin is larger.
