# Lift Curve Slope — Scholz Winglet AR Correction and Polhamus Formula

This document covers the two OpenMDAO components in `lift_curve_slope.py`:

| Component | Purpose |
|-----------|---------|
| `ScholzWingletARCorrection` | Converts H-tail VTP span into an effective wing AR increase (Scholz 2018) |
| `LiftCurveSlopePolhamus` | Computes 3-D CL_alpha from AR, Mach, sweep, section slope (Roskam Eq. 8.22) |

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

```
CL_alpha = K_wf * 2*pi*A / [2 + sqrt(A^2*(beta^2 + tan^2(Lc/2)) / k^2 + 4)]
```

### Terms

| Symbol | Meaning | Formula |
|--------|---------|---------|
| A | aspect ratio | input; connect AR_eff from ScholzWingletARCorrection |
| beta | Prandtl-Glauert factor | sqrt(1 - M^2) |
| k | section slope ratio | cl_alpha / (2*pi); k=1 for thin-airfoil theory |
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

### Wing-fuselage interference factor K_wf

```
K_wf = 1 + 0.025*(d_f/b) - 0.25*(d_f/b)^2
CL_alpha_corrected = K_wf * CL_alpha_Polhamus
```

For a slender UAV fuselage (d_f/b <= 0.15) the correction is below 0.2 % and
K_wf ~ 1.  It becomes relevant for d_f/b >= 0.2.

> **Apply to the wing instance only.** The VTP `LiftCurveSlopePolhamus`
> instance must leave `fuselage_diameter = 0` (K_wf = 1 by default).

### Inputs and outputs

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `aspect_ratio` | 2.0 | — | A = b^2/S; wire AR_eff for wing, raw AR for VTP |
| `mach` | 0.0 | — | Flight Mach number (subsonic, M < 1) |
| `sweep_c4_deg` | 0.0 | deg | Quarter-chord sweep Lc/4 |
| `taper_ratio` | 1.0 | — | lambda = c_tip / c_root |
| `section_lift_slope` | 2*pi | 1/rad | 2-D airfoil cl_alpha (thin-airfoil: 6.283 /rad) |
| `fuselage_diameter` | 0.0 | m | Equiv. fuselage diameter d_f; 0 disables K_wf |
| `wing_span` | 1.0 | m | Wing span b; only used when fuselage_diameter > 0 |
| **`CL_alpha`** | — | 1/rad | 3-D lift curve slope with K_wf applied |
| **`K_wf`** | — | — | Wing-fuselage interference factor (diagnostic) |

### UAV numerical example (2026-06-08 baseline)

Wing at M = 0.477, AR_eff = 7.08, sweep = 0 deg, lambda = 0.6, d_f = 0.169 m, b = 1.357 m:

```
beta  = sqrt(1 - 0.477^2) = sqrt(0.772) = 0.879
k     = 1.0  (thin-airfoil)
inner = 7.08^2 * (0.879^2 + 0) / 1^2 + 4 = 50.1 * 0.773 + 4 = 42.7
CL_alpha_Polhamus = 2*pi*7.08 / (2 + sqrt(42.7)) = 44.5 / 8.53 = 5.22 /rad
d_f/b = 0.169/1.357 = 0.125 -> K_wf = 1 + 0.025*0.125 - 0.25*0.125^2 = 0.999
CL_alpha = 0.999 * 5.22 = 5.21 /rad   (code gives 5.206 /rad)
```

---

## Connection to CyDeltaRudder

`CyDeltaRudder` accepts `CL_alpha_v` from the VTP Polhamus instance.
**Do not apply the AR correction here** — `CL_alpha_v` is the VTP's own
3-D lift slope computed from raw VTP geometry.

---

## Wiring example

```python
from aviary.subsystems.aerodynamics.flops_based.lift_curve_slope import (
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

# Wing CL_alpha (uses AR_eff)
model.add_subsystem('wing_polhamus', LiftCurveSlopePolhamus(),
    promotes_inputs=[('aspect_ratio', 'AR_eff'), ...],
    promotes_outputs=[('CL_alpha', 'wing_CL_alpha'), 'K_wf'],
)

# VTP CL_alpha (raw AR, no correction)
model.add_subsystem('vtp_polhamus', LiftCurveSlopePolhamus(),
    promotes_inputs=[('aspect_ratio', 'vtp_ar'), ...],
    promotes_outputs=[('CL_alpha', 'CL_alpha_v')],
)
```
