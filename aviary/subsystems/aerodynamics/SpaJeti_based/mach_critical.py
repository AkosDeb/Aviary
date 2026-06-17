"""
Drag-divergence and critical Mach number for a lifting surface.

Formula
-------
Weisshaar (Eq. 36), as ranked in Weisshaar (2024) -- best directly-solvable M_DD form:

    M_DD = K_A / cos(phi_25) - (t/c) / cos^2(phi_25) - CL / (10 * cos^3(phi_25))

K_A = 0.887 (optimised technology factor, conventional subsonic airfoils, SEE = 3.95 %).

CL used in the formula
----------------------
CL is derived from the minimum longitudinal load-factor requirement (Nz_min), not from
CL_alpha * alpha_max.  Using CL_max would be overly conservative because at dash Mach
the actual CL is close to the level-flight value, not the stall CL.

    CL = Nz_min * m * g / (q * S)       [CL needed to achieve Nz_min]

The M_CRIT_SAFETY_MARGIN on mach_upper_bound (DASH_MACH + buffer) already provides
the safety buffer — no additional multiplier on CL is needed here.

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
  - increasing t/c   (thicker wing -> lower M_DD)
  - increasing CL    (higher Nz_min or lower q/S -> lower M_DD)
  - reducing sweep   (unswept wing -> lower M_DD for the same t/c)
"""

import numpy as np
import openmdao.api as om

_M_DD_TO_MCRIT_DELTA = (0.1 / 80.0) ** (1.0 / 3.0)   # ~0.1077
_G = 9.80665  # standard gravity [m/s²]


class MachCriticalComp(om.ExplicitComponent):
    """Drag-divergence and critical Mach via Weisshaar Eq. 36.

    CL is derived from the Nz load-factor requirement:
        CL = nz_min * aircraft_mass * g / (dynamic_pressure * wing_area)

    Inputs  (all at Group scope -- connect via promotes at add_subsystem)
    -------
    surface_sweep_c4 : float [deg]
        Quarter-chord sweep angle phi_25.
    section_tc : float [-]
        Thickness-to-chord ratio t/c (from AirfoilConstantsComp inside the Group).
    nz_min : float [-]
        Minimum longitudinal load factor requirement (e.g. 7.0).
    aircraft_mass : float [kg]
        Aircraft mass at the design point.
    dynamic_pressure : float [Pa]
        Dynamic pressure q at the design point.
    wing_area : float [m²]
        Wing reference area S.
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
            'nz_min', val=7.0, units='unitless',
            desc='Minimum longitudinal load factor requirement.',
        )
        self.add_input(
            'aircraft_mass', val=15.0, units='kg',
            desc='Aircraft mass at the design point [kg].',
        )
        self.add_input(
            'dynamic_pressure', val=8581.0, units='Pa',
            desc='Dynamic pressure q at the design point [Pa].',
        )
        self.add_input(
            'wing_area', val=0.5, units='m**2',
            desc='Wing reference area S [m²].',
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
        phi_rad = inputs['surface_sweep_c4'] * (np.pi / 180.0)
        tc      = inputs['section_tc']
        nz_min  = inputs['nz_min']
        mass    = inputs['aircraft_mass']
        q       = inputs['dynamic_pressure']
        S       = inputs['wing_area']
        mach_ub = inputs['mach_upper_bound']

        cl = nz_min * mass * _G / (q * S)

        cos1 = np.cos(phi_rad)
        cos2 = cos1 ** 2
        cos3 = cos1 ** 3

        mdd    = self._k_a / cos1 - tc / cos2 - cl / (10.0 * cos3)
        m_crit = mdd - _M_DD_TO_MCRIT_DELTA

        outputs['M_DD']             = mdd
        outputs['M_crit']           = m_crit
        outputs['mach_crit_margin'] = m_crit - mach_ub
