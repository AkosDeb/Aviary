"""
Drag-divergence and critical Mach number for a lifting surface.

Formula
-------
Weisshaar (Eq. 36), as ranked in Weisshaar (2024) -- best directly-solvable M_DD form:

    M_DD = K_A / cos(phi_25) - (t/c) / cos^2(phi_25) - CL / (10 * cos^3(phi_25))

K_A = 0.887 (optimised technology factor, conventional subsonic airfoils, SEE = 3.95 %).

CL used in the formula is computed internally from the 3-D lift curve slope and the
maximum operating alpha (the same value used for the Nz constraint):

    CL = CL_alpha_3D * alpha_max_rad

This couples the M_crit check and the Nz load-factor check to the same alpha input,
so both constraints respond consistently when alpha_max is later computed from stall
physics (StallAlphaComp, see TOOD.md).

M_crit from Shevell wave-drag model (20*(M - M_crit)^4):
    dCD/dM = 0.1 at M_DD  =>  80*(M_DD - M_crit)^3 = 0.1
    M_crit = M_DD - (0.1/80)^(1/3)  ~  M_DD - 0.1077

Margin definition
-----------------
``mach_crit_margin = M_crit - mach_upper_bound``

  mach_upper_bound = DASH_MACH + M_CRIT_SAFETY_MARGIN

The constraint ``mach_crit_margin >= 0`` ensures the critical Mach stays above
the maximum mission Mach plus the safety buffer.  M_DD is retained as an
informational output; the constraint is intentionally set on M_crit so that the
design stays below the onset of local sonic flow, not merely below drag divergence.

Sensitivity note
----------------
M_DD is REDUCED by:
  - increasing t/c   (thicker wing -> lower M_DD; traded against higher CL_alpha)
  - increasing CL    (higher alpha or CL_alpha -> lower M_DD; couples to Nz constraint)
  - reducing sweep   (unswept wing -> lower M_DD for the same t/c)
"""

import numpy as np
import openmdao.api as om

# (0.1/80)^(1/3): delta between M_DD and M_crit from the Shevell CD_wave model
_M_DD_TO_MCRIT_DELTA = (0.1 / 80.0) ** (1.0 / 3.0)   # ~0.1077


class MachCriticalComp(om.ExplicitComponent):
    """Drag-divergence and critical Mach via Weisshaar Eq. 36.

    CL is derived from CL_alpha * alpha_max -- the same alpha used in the Nz
    constraint -- so both limits share a single hardcoded (or future stall-computed)
    alpha input.

    Inputs  (all at Group scope -- connect via promotes at add_subsystem)
    -------
    surface_sweep_c4 : float [deg]
        Quarter-chord sweep angle phi_25.
    section_tc : float [-]
        Thickness-to-chord ratio t/c (from AirfoilConstantsComp inside the Group).
    CL_alpha : float [/rad]
        3-D wing lift curve slope (= surface_CL_alpha from Polhamus at Group scope).
        Used to compute CL = CL_alpha * alpha_max_rad for the Weisshaar formula.
    alpha_max_deg : float [deg]
        Maximum operating angle of attack (shared with the Nz constraint).
        Hardcoded until StallAlphaComp replaces it (see TOOD.md).
    mach_upper_bound : float [-]
        Maximum mission Mach + safety buffer (= DASH_MACH + M_CRIT_SAFETY_MARGIN).
        Constraint: M_crit >= mach_upper_bound  =>  mach_crit_margin >= 0.

    Outputs
    -------
    M_DD : float [-]
        Drag-divergence Mach (Weisshaar Eq. 36, Shevell definition: dCD/dM = 0.1).
        Informational -- the constraint is on M_crit, not M_DD.
    M_crit : float [-]
        Critical Mach (onset of local sonic flow): M_DD - (0.1/80)^(1/3).
    mach_crit_margin : float [-]
        M_crit - mach_upper_bound.  Constrain >= 0.
    """

    def __init__(self, k_a: float = 0.887, **kwargs):
        """
        Parameters
        ----------
        k_a : float
            Airfoil technology factor (Korn constant).
            0.887 -- conventional subsonic (Weisshaar optimised).
            0.95  -- supercritical section.
        """
        super().__init__(**kwargs)
        self._k_a = float(k_a)

    def setup(self):
        self.add_input(
            'surface_sweep_c4', val=0.0, units='deg',
            desc='Quarter-chord sweep angle phi_25 [deg]',
        )
        self.add_input(
            'section_tc', val=0.12, units='unitless',
            desc='Thickness-to-chord ratio t/c (from AirfoilConstantsComp)',
        )
        self.add_input(
            'CL_alpha', val=5.0, units='unitless',
            desc=(
                '3-D wing lift curve slope [/rad] (surface_CL_alpha from Polhamus). '
                'Used with alpha_max_deg to compute the CL for the Weisshaar formula.'
            ),
        )
        self.add_input(
            'alpha_max_deg', val=12.0, units='deg',
            desc=(
                'Maximum operating angle of attack [deg]. '
                'Shared with LongitudinalLoadFactor (Nz constraint). '
                'Replace with StallAlphaComp output for physics-based alpha_stall.'
            ),
        )
        self.add_input(
            'mach_upper_bound', val=0.57, units='unitless',
            desc=(
                'Maximum mission Mach + safety buffer [= DASH_MACH + M_CRIT_SAFETY_MARGIN]. '
                'Constraint: M_crit >= mach_upper_bound.'
            ),
        )

        self.add_output(
            'M_DD', val=0.75, units='unitless',
            desc=(
                'Drag-divergence Mach (Weisshaar Eq. 36, K_A={:.3f}). '
                'Informational -- constraint is on M_crit.'
            ).format(self._k_a),
        )
        self.add_output(
            'M_crit', val=0.65, units='unitless',
            desc='Critical Mach: M_DD - (0.1/80)^(1/3)  ~  M_DD - 0.108',
        )
        self.add_output(
            'mach_crit_margin', val=0.1, units='unitless',
            desc='M_crit - mach_upper_bound.  Constrain >= 0.',
        )

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        phi_rad     = inputs['surface_sweep_c4'] * (np.pi / 180.0)
        tc          = inputs['section_tc']
        cl_alpha    = inputs['CL_alpha']
        alpha_rad   = inputs['alpha_max_deg'] * (np.pi / 180.0)
        mach_ub     = inputs['mach_upper_bound']

        cl   = cl_alpha * alpha_rad

        cos1 = np.cos(phi_rad)
        cos2 = cos1 ** 2
        cos3 = cos1 ** 3

        mdd    = self._k_a / cos1 - tc / cos2 - cl / (10.0 * cos3)
        m_crit = mdd - _M_DD_TO_MCRIT_DELTA

        outputs['M_DD']             = mdd
        outputs['M_crit']           = m_crit
        outputs['mach_crit_margin'] = m_crit - mach_ub
