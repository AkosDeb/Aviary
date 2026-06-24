"""
Lift curve slope utilities for finite wings.

Components:
  ScholzWingletARCorrection — Scholz (2018) winglet-equivalent span increase for H-tail VTPs
  LiftCurveSlopePolhamus    — Polhamus/DATCOM 3-D CL_alpha (Roskam Part VI, Eq. 8.22)

Typical wiring for an H-tail wing (horizontal panel with VTP endplates):
    ScholzWingletARCorrection.AR_eff -> LiftCurveSlopePolhamus.aspect_ratio
"""

import warnings

import numpy as np
import openmdao.api as om


# ---------------------------------------------------------------------------
# Scholz (2018) winglet method — default parameters
# Reference: Scholz, D., "Definition and discussion of the intrinsic efficiency
#            of winglets", INCAS Bulletin, Vol. 10, Issue 1, 2018.
# ---------------------------------------------------------------------------
_K_WL_DEFAULT  = 2.0   # winglet penalty factor — McLean/Howe preliminary design value
_SPLIT_PENALTY = 0.90  # symmetric split-winglet (H-tail): 90 % of standard winglet


class ScholzWingletARCorrection(om.ExplicitComponent):
    """Effective AR correction for H-tail VTPs using Scholz (INCAS 2018) winglet theory.

    Each VTP panel is treated as a winglet mounted at the horizontal tail tip.
    Scholz's method converts winglet height into an equivalent span increase and
    derives AR_eff from that, penalising the result relative to the geometric ideal
    via the factor k_WL.

    Formula chain
    -------------
    Step 1 — effective span:
        b_eff = b_h + 2 * h / k_WL

    Step 2 — effective aspect ratio (before split-winglet penalty):
        AR_eff_raw = AR_h * (b_eff / b_h)^2
                   = AR_h * (1 + 2*h / (k_WL * b_h))^2

    Step 3 — split-winglet penalty for symmetric H-tail VTPs:
        AR_eff = AR_eff_raw * split_penalty

    where
        h             = full VTP span at one tip (upper semi-span + lower semi-span)
        b_h           = full horizontal wing span (tip to tip)
        k_WL          = winglet effectiveness penalty factor (constructor argument)
        split_penalty = 0.90 for symmetric up/down VTPs (Scholz: split winglets
                        produce ~90 % of the induced-drag reduction of a standard
                        winglet of the same total height)

    k_WL reference values
    ---------------------
        1.0   geometric ideal — winglet folded flat (unrealistically optimistic)
        2.0   McLean / Howe   — theoretical optimum (best-case, rarely achieved)
        2.45  Dubs / Zimmer   — based on measured experimental data
        2.8   real-aircraft average (conservative lower bound)

    Parameters
    ----------
    k_wl : float
        Winglet penalty factor (default 2.0).
    split_penalty : float
        Fraction applied for a symmetric split winglet (default 0.90).

    Outputs
    -------
    AR_eff : effective aspect ratio after Scholz correction and split penalty
    k_h    : AR_eff / AR_geo (diagnostic — consistent naming with prior component)

    References
    ----------
    Scholz, D. (2018). Definition and discussion of the intrinsic efficiency of
    winglets. INCAS Bulletin, 10(1). HAW Hamburg.
    """

    def __init__(self, k_wl=_K_WL_DEFAULT, split_penalty=_SPLIT_PENALTY, **kwargs):
        super().__init__(**kwargs)
        self._k_wl         = float(k_wl)
        self._split_penalty = float(split_penalty)

    def setup(self):
        self.add_input(
            'aspect_ratio', val=2.0, units='unitless',
            desc='Wing aspect ratio AR_h = b^2/S (no endplate effect)',
        )
        self.add_input(
            'vtp_span', val=0.31, units='m',
            desc='Full VTP span h at one HT tip (upper + lower semi-span combined) [m]',
        )
        self.add_input(
            'wing_span', val=1.8, units='m',
            desc='Full horizontal wing span b_h (tip to tip) [m]',
        )

        self.add_output(
            'AR_eff', val=4.0, units='unitless',
            desc='Scholz effective AR: AR_h * (1 + 2h/(k_WL*b_h))^2 * split_penalty',
        )
        self.add_output(
            'k_h', val=1.2, units='unitless',
            desc='AR correction factor k_h = AR_eff / AR_geo (diagnostic)',
        )

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        AR  = inputs['aspect_ratio']
        h   = inputs['vtp_span']
        b_h = inputs['wing_span']

        if np.any(AR <= 0.0):
            raise ValueError(
                f"ScholzWingletARCorrection: aspect_ratio must be > 0; got {float(AR)}."
            )
        if np.any(b_h <= 0.0):
            raise ValueError(
                f"ScholzWingletARCorrection: wing_span must be > 0; got {float(b_h)} m."
            )
        if np.any(h < 0.0):
            warnings.warn(
                f"ScholzWingletARCorrection: vtp_span = {float(h):.4f} m < 0. "
                "Negative VTP span is physically invalid.",
                RuntimeWarning,
                stacklevel=2,
            )

        span_ratio = 1.0 + 2.0 * h / (self._k_wl * b_h)
        k_h        = span_ratio ** 2 * self._split_penalty

        outputs['k_h']    = k_h
        outputs['AR_eff'] = k_h * AR


class LiftCurveSlopePolhamus(om.ExplicitComponent):
    """3-D lift curve slope using a Helmbold-Polhamus blend with optional
    wing-fuselage interference correction.

    At high aspect ratio (AR ≥ 4) the full Polhamus/DATCOM formula is used,
    which includes the quarter-chord sweep effect in the radical:

        C_L_alpha = K_wf · 2π A / [2 + √(A²(β² + tan²Λ_c/2) / k² + 4)]

    At low aspect ratio (AR ≤ 2) the sweep term is removed (Helmbold):

        C_L_alpha = K_wf · 2π A / [2 + √(A²β² / k² + 4)]

    The transition is a smooth tanh blend centred at AR = 3 with half-width ~1:

        w_H = 0.5 · (1 − tanh(2·(AR − 3)))

        inner = A²/k² · (β² + (1 − w_H) · tan²Λ_c/2) + 4

    At AR ≈ 1.2–2.0 (X-tail VTP panels) w_H ≈ 1 and the sweep term vanishes;
    Helmbold is better-calibrated against measured data at these low AR values
    (Polhamus/DATCOM is accurate for AR ≥ 4). At AR ≥ 5 (main wing) w_H ≈ 0
    and the full Polhamus formula is recovered.

    Variables:
        A        = aspect ratio
        β        = √(1 − M²)          Prandtl-Glauert compressibility factor
        k        = c_l_alpha / (2π)   section lift slope ratio (k=1 thin-airfoil)
        Λ_c/2    = semi-chord sweep angle (derived from quarter-chord sweep)
        K_wf     = wing-fuselage interference factor

    Wing-fuselage interference factor (Roskam Part VI, Eq. 8.17 / DATCOM 4.1.3.2):

        K_wf = 1 + 0.025 (d_f/b) − 0.25 (d_f/b)²

    K_wf = 1 when fuselage_diameter = 0 (default — correction disabled).

    Sweep conversion (quarter-chord → semi-chord):

        tan Λ_c/2 = tan Λ_c/4 − (1 − λ) / [A (1 + λ)]
    """

    def setup(self):
        self.add_input(
            'aspect_ratio', val=2.0, units='unitless',
            desc='Surface aspect ratio A = b² / S',
        )
        self.add_input(
            'mach', val=0.0, units='unitless',
            desc='Mach number M (subsonic only); sets Prandtl-Glauert factor β = √(1 − M²)',
        )
        self.add_input(
            'sweep_c4_deg', val=0.0, units='deg',
            desc='Quarter-chord sweep angle Λ_c/4 [deg]; '
                 'converted to semi-chord sweep internally using the taper ratio',
        )
        self.add_input(
            'taper_ratio', val=1.0, units='unitless',
            desc='Taper ratio λ = c_tip / c_root; needed to convert Λ_c/4 → Λ_c/2',
        )
        self.add_input(
            'section_lift_slope', val=2.0 * np.pi, units='unitless',
            desc='2-D airfoil lift-curve slope c_l_alpha [per rad, at the flight Mach number]. '
                 'Thin-airfoil theory gives 2π ≈ 6.283 /rad. '
                 'Typical symmetric sections: 5.5–5.9 /rad.',
        )
        self.add_input(
            'fuselage_diameter', val=0.0, units='m',
            desc='Equivalent fuselage diameter d_f [m] for the K_wf correction. '
                 'Set to 0 (default) to disable the correction (K_wf = 1). '
                 'For a non-circular fuselage use d_f = √(4·A_fus/π).',
        )
        self.add_input(
            'wing_span', val=1.0, units='m',
            desc='Wing span b [m]. Only used when fuselage_diameter > 0.',
        )

        self.add_output(
            'CL_alpha', val=4.0, units='unitless',
            desc='3-D lift curve slope C_L_alpha [per rad], Polhamus/DATCOM formula '
                 'with K_wf fuselage correction applied (Roskam Part VI, Eq. 8.22)',
        )
        self.add_output(
            'K_wf', val=1.0, units='unitless',
            desc='Wing-fuselage interference factor: '
                 'K_wf = 1 + 0.025(d_f/b) − 0.25(d_f/b)²',
        )

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        AR       = inputs['aspect_ratio']
        M        = inputs['mach']
        lam      = inputs['taper_ratio']
        cl_a_2d  = inputs['section_lift_slope']
        sweep_c4 = inputs['sweep_c4_deg'] * (np.pi / 180.0)
        d_f      = inputs['fuselage_diameter']
        b        = inputs['wing_span']

        if np.any(AR <= 0.0):
            raise ValueError(
                "LiftCurveSlopePolhamus: aspect_ratio must be > 0; "
                f"got min AR={float(np.min(AR.real)):.6g}."
            )
        if np.any(AR.real < 0.5):
            warnings.warn(
                f"LiftCurveSlopePolhamus: aspect_ratio min = {float(np.min(AR.real)):.3f} < 0.5. "
                "Below AR = 0.5 slender-body theory is more appropriate; "
                "Helmbold/Polhamus results are unreliable.",
                RuntimeWarning,
                stacklevel=2,
            )
        if np.any(M < 0.0):
            raise ValueError(
                "LiftCurveSlopePolhamus: Mach number must be >= 0; "
                f"got min M={float(np.min(M.real)):.6g}."
            )
        if np.any(M.real >= 1.0):
            raise ValueError(
                f"LiftCurveSlopePolhamus: Mach number must be < 1 (subsonic only); "
                f"got max M={float(np.max(M.real)):.6g}. The Prandtl-Glauert factor beta=sqrt(1-M^2) is "
                "undefined at M >= 1."
            )

        # Semi-chord sweep from quarter-chord sweep
        # tan(Λ_n) = tan(Λ_m) − 4(n−m)/AR · (1−λ)/(1+λ),  n=0.5, m=0.25 → 4·0.25 = 1
        tan_sweep_c2 = np.tan(sweep_c4) - (1.0 / AR) * (1.0 - lam) / (1.0 + lam)

        beta_sq = 1.0 - M ** 2

        # Section lift slope ratio: k=1 for thin-airfoil theory (2D c_l_alpha = 2π)
        k = cl_a_2d / (2.0 * np.pi)

        # Helmbold-Polhamus smooth blend on the sweep term.
        # w_H → 1 at low AR (Helmbold, sweep term removed): better calibrated for
        #   low-AR fins (AR ≈ 1–2) where sweep contributes little to span loading.
        # w_H → 0 at high AR (full Polhamus with sweep): accurate for AR ≥ 4.
        # tanh centred at AR=3, half-width ≈ 1.4; fully differentiable for CS.
        helmbold_weight = 0.5 * (1.0 - np.tanh(2.0 * (AR - 3.0)))
        inner = AR ** 2 * (beta_sq + (1.0 - helmbold_weight) * tan_sweep_c2 ** 2) / k ** 2 + 4.0

        r    = d_f / b  # zero when fuselage_diameter=0 → K_wf=1
        k_wf = 1.0 + 0.025 * r - 0.25 * r ** 2

        outputs['K_wf']     = k_wf
        outputs['CL_alpha'] = k_wf * 2.0 * np.pi * AR / (2.0 + np.sqrt(inner))
