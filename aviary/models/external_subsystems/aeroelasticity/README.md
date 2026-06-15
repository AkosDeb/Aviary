# Preliminary Aeroelasticity External Subsystem

Preliminary aeroelastic constraints for gradient-based MDO with Aviary. All
components are OpenMDAO `ExplicitComponent` with analytical (CS) or finite-difference
partials, so they can be dropped directly into a DYMOS/OpenMDAO optimisation loop.

---

## What it computes

### Wing structure — `WingboxStructuralEstimate`
Thin-walled closed rectangular wingbox from wing geometry alone.

| Output | Description |
|--------|-------------|
| Bending stiffness EI | Euler-Bernoulli, skin + spar flanges |
| Torsional rigidity GJ | Bredt-Batho single-cell |
| Plunge spring constant k_h | 3EI/L³ cantilever tip |
| Torsional spring constant k_α | GJ/L cantilever |
| Elastic-axis fraction | Mid-point of front and rear spar |
| AC-to-EA fraction | Elastic axis − aerodynamic centre (0.25c) |
| Control hinge fraction | 1 − control chord fraction |
| Mass per unit span | Two-skin thin-wall (skin + spar webs) |
| Pitch inertia per unit span | 2·m_skin·(w²/12 + (h/2)²) + 2·m_spar·((w/2)² + h²/12) |
| Control inertia per unit span | ¼ m (control_radius)² |
| **Control static unbalance** | density × 8 × skin_t × cg_frac × r² (two-skin estimate) |

---

### Static aeroelasticity — `StaticAeroelastic`
Strip-theory lumped model, one representative section at the MAC.

| Output | Formula | Constraint |
|--------|---------|-----------|
| Divergence dynamic pressure | k_α / (e·c·Clα·L) | q_div > q_required |
| Divergence speed | √(2 q_div / ρ) | V_div > V_required |
| **Divergence speed margin** | V_div − V_required | ≥ 0 |
| Reversal dynamic pressure | −k_α Clδ / (c·Cmδ·Clα·L) | q_rev > q_required |
| Reversal speed | √(2 q_rev / ρ) | V_rev > V_required |
| **Reversal speed margin** | V_rev − V_required | ≥ 0 |
| Control effectiveness η | (1 − q/q_rev) / (1 − q/q_div) | η > 0 |

`q_required` = dynamic pressure at `REQUIRED_SPEED` (typically V_design × 1.15).

---

### Quasi-steady flutter screen — `QuasiSteadyFlutterScreen`
3-DOF lumped model: wing plunge (h), wing torsion (α), full-span control rotation (δ).

**Mass matrix** includes:
- Structural mass per span + VTP tip mass (H-wing configuration)
- Pitch static unbalance S_α
- Control static unbalance S_δ (from structural estimate, or provided directly)

**Aerodynamic model** — quasi-steady p-k:
- Stiffness terms: q × aero_matrix (from Clα, Cmα_EA, Chα and control derivatives)
- Velocity damping terms: 0.5 ρ V × aero_vel (from plunge-rate apparent AoA)
- cm_alpha is internally converted from AC convention to EA convention:
  `cm_alpha_EA = cl_alpha × e_frac + cm_alpha_AC`

| Output | Description |
|--------|-------------|
| Flutter speed | Bisection on max real part of state-matrix eigenvalues |
| Flutter dynamic pressure | 0.5 ρ V_f² |
| Flutter margin | V_f/V_design − 1 |
| **Flutter speed margin** | V_f − V_required |
| Max real eigenvalue at design | Smooth CS-differentiable stability metric for optimisation |

`MAX_REAL_EIGENVALUE_AT_DESIGN < 0` is the recommended optimisation constraint
(has exact CS derivatives). Flutter speed uses bisection so its CS gradient is zero;
use finite-difference if the flutter speed itself is the active constraint.

---

### P-K flutter analysis — `PKFlutterAnalysis`
Full reduced-frequency P-K iteration over a speed sweep. Same structural model as
`QuasiSteadyFlutterScreen`; the aerodynamic model is frequency-dependent.

**Aerodynamic model** — Theodorsen thin-airfoil strip theory (`theodorsen_strip_gaf`):
- `C(k) = H₁⁽²⁾(k) / (H₀⁽²⁾(k) + H₁⁽²⁾(k))` via `scipy.special.hankel2`
- Column h: `C(k)·(ik/b)` → aerodynamic damping from plunge rate (F-term) plus
  pseudo-stiffness shift (G-term)
- Column α: `C(k)·[1 + ik·(1/2−a)]` → circulatory stiffness + pitch-rate damping;
  `a = 2·ea_frac − 1` is the elastic-axis position from midchord
- Column δ: quasi-steady (Theodorsen-Garrick T-functions are the next upgrade)
- k=0 limit equals the quasi-steady aero matrix; k→∞ gives 50% circulatory lift
- **Swap point**: pass a custom `gaf_function` to `pk_modes_at_speed()` for
  DLM/AVL-derived matrices without changing any other code

**P-K iteration** (per mode, per speed point):
1. Seed k from dry structural frequencies
2. Compute Q(k) → build state matrix → solve eigenvalues
3. Update k = ω × (c/2) / V for the nearest mode
4. Repeat until |Δk| < tolerance (default 1e-4)

| Output | Description |
|--------|-------------|
| PK flutter speed | Speed where first modal damping crosses zero |
| PK flutter frequency | Frequency of the critical mode at flutter |
| PK flutter dynamic pressure | 0.5 ρ V_f² |
| **PK flutter speed margin** | V_f_pk − V_required |
| PK critical mode | Index of the first-to-flutter mode (0=plunge, 1=torsion, 2=control) |
| PK converged | 1.0 if all modes converged at the final speed point |
| PK mode damping (3-vector) | Modal damping ratio at the last speed sample |
| PK mode frequency (3-vector) | Modal frequency [Hz] at the last speed sample |

---

### VTP tip inertia — `VTPTipInertia`
Equivalent distributed mass/inertia of a vertical tail mounted at the wing tip (H-wing).
Centroid uses the correct tapered-surface formula: `z_c = H/3 × (1+2λ)/(1+λ)`.
Outputs lumped mass, pitch inertia, pitch static unbalance, and chordwise offset from
the wing elastic axis — all fed directly into the flutter components.

---

## Next upgrade path

| Item | Location | Description |
|------|----------|-------------|
| Control-surface unsteady GAF | `pk_flutter.py` `theodorsen_strip_gaf()` | Add Theodorsen-Garrick T-function correction to column δ |
| DLM / AVL matrices | `pk_flutter.py` `pk_modes_at_speed()` | Pass computed Q(k) as `gaf_function` to replace strip theory |

---

## Signal flow

```
WingboxStructuralEstimate
  │  EI, GJ, k_h, k_α, k_δ, m, I_α, I_δ, S_δ
  │
VTPTipInertia
  │  m_vtp, I_vtp, S_vtp
  │
  ├──► StaticAeroelastic  →  V_div_margin, V_rev_margin, η
  │
  ├──► QuasiSteadyFlutterScreen  →  V_f_margin, λ_max(design)
  │
  └──► PKFlutterAnalysis  →  V_f_pk_margin, modal g, ω
```

All speed margins and the design-point eigenvalue are suitable as optimisation
constraints. The P-K speed margin is the highest-fidelity flutter constraint in
this module.
