import numpy as np
import openmdao.api as om
import scipy.constants as _units

from aviary.variable_info.functions import add_aviary_input, add_aviary_option
from aviary.variable_info.variables import Aircraft, Dynamic
from aviary.subsystems.aerodynamics.aero_utils import (
    leading_edge_radius_ratio_naca_4_digit,
    leading_edge_reynolds_number_from_mach,
    leading_edge_suction_parameter_roskam,
    oswald_efficiency_roskam,
    wing_induced_drag_roskam,
)


class InducedDrag(om.ExplicitComponent):
    """Calculates induced drag."""

    def initialize(self):
        self.options.declare(
            'num_nodes', default=1, types=int, desc='Number of nodes along mission segment'
        )
        self.options.declare('gamma', default=1.4, desc='Ratio of specific heats for air.')

        add_aviary_option(self, Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION)

    def setup(self):
        nn = self.options['num_nodes']

        # Simulation inputs
        add_aviary_input(self, Dynamic.Atmosphere.MACH, shape=nn, units='unitless')
        add_aviary_input(self, Dynamic.Vehicle.LIFT, shape=(nn), units='lbf')
        add_aviary_input(self, Dynamic.Atmosphere.STATIC_PRESSURE, shape=nn, units='lbf/ft**2')

        # Aero design inputs
        add_aviary_input(self, Aircraft.Wing.AREA, units='ft**2')
        add_aviary_input(self, Aircraft.Wing.ASPECT_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.Wing.SPAN_EFFICIENCY_FACTOR, units='unitless')
        add_aviary_input(self, Aircraft.Wing.SWEEP, units='deg')
        add_aviary_input(self, Aircraft.Wing.TAPER_RATIO, units='unitless')

        # Declare outputs
        self.add_output(
            'induced_drag_coeff', shape=(nn), units='unitless', desc='Coefficient of induced drag'
        )

    def setup_partials(self):
        nn = self.options['num_nodes']

        row_col = np.arange(nn)
        self.declare_partials(
            'induced_drag_coeff',
            [
                Dynamic.Atmosphere.MACH,
                Dynamic.Vehicle.LIFT,
                Dynamic.Atmosphere.STATIC_PRESSURE,
            ],
            rows=row_col,
            cols=row_col,
        )

        wrt = [
            Aircraft.Wing.AREA,
            Aircraft.Wing.ASPECT_RATIO,
            Aircraft.Wing.SPAN_EFFICIENCY_FACTOR,
            Aircraft.Wing.SWEEP,
            Aircraft.Wing.TAPER_RATIO,
        ]

        self.declare_partials('induced_drag_coeff', wrt, rows=row_col, cols=np.zeros(nn))

    def compute(self, inputs, outputs):
        options = self.options
        gamma = options['gamma']
        mach, lift, P, Sref, AR, span_efficiency_factor, SW25, TR = inputs.values()

        CL = 2.0 * lift / (Sref * gamma * P * mach**2)

        redux = self.options[Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION]

        if redux:
            # Adjustment for extreme taper ratios.
            # Reference:
            # ----------
            # [1] DeYoung, John. "Advanced Supersonic Technology Concept Study Reference
            # Characteristics," NASA Contractor Report 132374.

            span_efficiency_0 = 1.0 + 0.1 * AR * (0.4226 * np.sqrt(AR) - 0.35 * TR - 0.143)
        else:
            span_efficiency_0 = 1.0

        if span_efficiency_factor <= 0.3:
            span_efficiency = span_efficiency_0 + span_efficiency_factor
        else:
            span_efficiency = span_efficiency_0 * span_efficiency_factor

        CDi = CL**2 / (np.pi * AR * span_efficiency)

        # If forward sweep, add Warner Robins Factor
        if SW25.real < 0.0:
            deg_to_rad = _units.degree

            TH = (1.0 - TR) / (1.0 + TR) / AR
            tan_sw = np.tan(SW25 / deg_to_rad)
            COSA = 1.0 / np.sqrt(1.0 + (tan_sw - 3.0 * TH) ** 2)
            COSB = 1.0 / np.sqrt(1.0 + (tan_sw + TH) ** 2)
            CAYT = (
                0.5
                * ((1.1 - 0.11 / (1.1 - mach * COSA)) / (1.1 - 0.11 / (1.1 - mach * COSB)) - 1.0)
                ** 2
            )

            CDi += CAYT * CL**2

        outputs['induced_drag_coeff'] = CDi

    def compute_partials(self, inputs, partials):
        options = self.options
        gamma = options['gamma']
        mach, lift, P, Sref, AR, span_efficiency_factor, SW25, TR = inputs.values()
        redux = self.options[Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION]

        if redux:
            sqrt_AR = np.sqrt(AR)
            span_efficiency_0 = 1.0 + 0.1 * AR * (0.4226 * sqrt_AR - 0.35 * TR - 0.143)
            dse0_dAR = 0.1 * (0.4226 * (sqrt_AR + 0.5 * AR / sqrt_AR) - 0.35 * TR - 0.143)
            dse0_dTR = -0.035 * AR

        else:
            span_efficiency_0 = 1.0

        if span_efficiency_factor <= 0.3:
            span_efficiency = span_efficiency_0 + span_efficiency_factor
        else:
            span_efficiency = span_efficiency_0 * span_efficiency_factor

        CL = 2.0 * lift / (Sref * gamma * P * mach**2)
        dCL_dL = 2.0 / (Sref * gamma * P * mach**2)
        dCL_dSref = -2.0 * lift / (Sref**2 * gamma * P * mach**2)
        dCL_dP = -2.0 * lift / (Sref * gamma * P**2 * mach**2)
        dCL_dmach = -4.0 * lift / (Sref * gamma * P * mach**3)

        dCDi_dCL = 2.0 * CL / (np.pi * AR * span_efficiency)
        dCDi_dAR = -(CL**2) / (np.pi * AR**2 * span_efficiency)
        dCDi_dspan = -(CL**2) / (np.pi * AR * span_efficiency**2)

        partials['induced_drag_coeff', Dynamic.Atmosphere.MACH] = dCDi_dCL * dCL_dmach
        partials['induced_drag_coeff', Dynamic.Vehicle.LIFT] = dCDi_dCL * dCL_dL
        partials['induced_drag_coeff', Dynamic.Atmosphere.STATIC_PRESSURE] = dCDi_dCL * dCL_dP
        partials['induced_drag_coeff', Aircraft.Wing.ASPECT_RATIO] = dCDi_dAR
        partials['induced_drag_coeff', Aircraft.Wing.SPAN_EFFICIENCY_FACTOR] = 0.0
        partials['induced_drag_coeff', Aircraft.Wing.SWEEP] = 0.0
        partials['induced_drag_coeff', Aircraft.Wing.TAPER_RATIO] = 0.0
        partials['induced_drag_coeff', Aircraft.Wing.AREA] = dCDi_dCL * dCL_dSref

        if span_efficiency_factor <= 0.3:
            partials['induced_drag_coeff', Aircraft.Wing.SPAN_EFFICIENCY_FACTOR] = dCDi_dspan
            if redux:
                partials['induced_drag_coeff', Aircraft.Wing.ASPECT_RATIO] += dCDi_dspan * dse0_dAR
                partials['induced_drag_coeff', Aircraft.Wing.TAPER_RATIO] = dCDi_dspan * dse0_dTR
        else:
            partials['induced_drag_coeff', Aircraft.Wing.SPAN_EFFICIENCY_FACTOR] = (
                dCDi_dspan * span_efficiency_0
            )
            if redux:
                partials['induced_drag_coeff', Aircraft.Wing.ASPECT_RATIO] += (
                    dCDi_dspan * dse0_dAR * span_efficiency_factor
                )
                partials['induced_drag_coeff', Aircraft.Wing.TAPER_RATIO] = (
                    dCDi_dspan * dse0_dTR * span_efficiency_factor
                )

        # If forward sweep, add Warner Robins Factor
        if SW25.real < 0.0:
            deg_to_rad = _units.degree

            fact1 = 1.0 - TR
            fact2 = 1.0 / (1.0 + TR)
            TH = fact1 * fact2 / AR
            dTH_dTR = -(fact2 + fact1 * fact2**2) / AR
            dTH_dAR = -fact1 * fact2 / AR**2

            tan_sw = np.tan(SW25 / deg_to_rad)
            dtansw_dsw = 1.0 / (deg_to_rad * np.cos(SW25 / deg_to_rad) ** 2)

            fact3 = 1.0 + (tan_sw - 3.0 * TH) ** 2
            COSA = 1.0 / np.sqrt(fact3)
            dCOSA_dtansw = -0.5 / fact3**1.5 * 2.0 * (tan_sw - 3.0 * TH)
            dCOSA_dTH = 0.5 / fact3**1.5 * 6.0 * (tan_sw - 3.0 * TH)

            fact4 = 1.0 + (tan_sw + TH) ** 2
            COSB = 1.0 / np.sqrt(fact4)
            dCOSB_dtansw = -0.5 / fact4**1.5 * 2.0 * (tan_sw + TH)
            dCOSB_dTH = -0.5 / fact4**1.5 * 2.0 * (tan_sw + TH)

            factA = 1.1 - mach * COSA
            factB = 1.1 - mach * COSB
            fact5 = 1.1 - 0.11 / factA
            fact6 = 1.1 - 0.11 / factB
            CAYT = 0.5 * (fact5 / fact6 - 1.0) ** 2
            dCAYT_dmach = (fact5 / fact6 - 1.0) * (
                -0.11 * COSA / (fact6 * factA**2) + 0.11 * fact5 * COSB / (factB**2 * fact6**2)
            )
            dCAYT_dCOSA = (fact5 / fact6 - 1.0) * (-0.11 * mach / (fact6 * factA**2))
            dCAYT_dCOSB = (fact5 / fact6 - 1.0) * (0.11 * fact5 * mach / (factB**2 * fact6**2))

            dCDi_dCAYT = CL**2
            dCDi_dCL = 2.0 * CAYT * CL

            partials['induced_drag_coeff', Dynamic.Atmosphere.MACH] += (
                dCDi_dCL * dCL_dmach + dCDi_dCAYT * dCAYT_dmach
            )
            partials['induced_drag_coeff', Dynamic.Vehicle.LIFT] += dCDi_dCL * dCL_dL
            partials['induced_drag_coeff', Aircraft.Wing.ASPECT_RATIO] += (
                dCDi_dCAYT * dTH_dAR * (dCAYT_dCOSA * dCOSA_dTH + dCAYT_dCOSB * dCOSB_dTH)
            )
            partials['induced_drag_coeff', Aircraft.Wing.SWEEP] += (
                dCDi_dCAYT * dtansw_dsw * (dCAYT_dCOSA * dCOSA_dtansw + dCAYT_dCOSB * dCOSB_dtansw)
            )
            partials['induced_drag_coeff', Dynamic.Atmosphere.STATIC_PRESSURE] += dCDi_dCL * dCL_dP
            partials['induced_drag_coeff', Aircraft.Wing.TAPER_RATIO] += (
                dCDi_dCAYT * dTH_dTR * (dCAYT_dCOSA * dCOSA_dTH + dCAYT_dCOSB * dCOSB_dTH)
            )
            partials['induced_drag_coeff', Aircraft.Wing.AREA] += dCDi_dCL * dCL_dSref


# ---------------------------------------------------------------------------
# Roskam-based induced drag  (local implementation, not wired into FLOPS polar)
# ---------------------------------------------------------------------------

class RoskamInducedDragComp(om.ExplicitComponent):
    """Wing induced drag from Roskam Part VI Section 4.2.1.2, Eq. 4.8 (no-twist).

    First term of the full Roskam expansion:

        CDi = C_Lw^2 / (pi * AR_eff * e)                  (Eq. 4.8, term 1)

    where

        C_Lw = CL                                          (trim factor not applied)
        e    = 1.1*(CL_alpha_w/AR_eff)
               / (R*(CL_alpha_w/AR_eff) + (1-R)*pi)        (Eq. 4.12)

    Excluded twist correction terms (Eq. 4.8, terms 2 and 3):
        + 2*pi * C_Lw * eps_t * V   (twist-CL cross term)
        + 4*pi^2 * eps_t^2 * w      (pure twist term)

    These are zero for an untwisted wing and are intentionally not implemented.
    When geometric twist is introduced, add:
        - eps_t  : design twist angle [rad] (root-to-tip, negative = washout)
        - V, w   : Roskam Part VI Appendix B integrals over the spanwise loading

    AR convention
    -------------
    AR_eff MUST be the output of ScholzWingletARCorrection, not the geometric AR.
    ALL aerodynamic calculations (lift, induced drag, pitch, stability derivatives)
    must use AR_eff to be consistent with the endplate model.

    Inputs
    ------
    CL        : aircraft total lift coefficient (scalar)
    AR_eff    : effective aspect ratio, Scholz endplate correction (scalar)
    CL_alpha_w: 3-D wing lift-curve slope [1/rad], computed at AR_eff (scalar)
    mach, static_pressure, temperature
              : atmosphere inputs used when chart-driven suction is enabled
    mean_aerodynamic_chord, thickness_to_chord, leading_edge_sweep, taper_ratio
              : wing geometry used to compute r_LE, Re_LER, and R

    Options
    -------
    compute_leading_edge_suction : bool (default False)
        If True, compute R from Roskam Figure 4.7 using live inputs.
    default_leading_edge_suction_parameter : float (default 0.98)
        Fixed fallback leading-edge suction parameter R when chart computation
        is disabled.

    Outputs
    -------
    CDi      : wing induced drag coefficient (scalar)
    e_oswald : Oswald span efficiency factor (scalar)
    leading_edge_radius : leading-edge radius [m]
    leading_edge_reynolds_number : Re_LER
    leading_edge_suction_parameter : R used in Eq. 4.12
    """

    def initialize(self):
        self.options.declare(
            'compute_leading_edge_suction',
            default=False,
            types=bool,
            desc='Compute Roskam Figure 4.7 leading-edge suction parameter from live inputs',
        )
        self.options.declare(
            'default_leading_edge_suction_parameter',
            default=0.98,
            types=float,
            desc='Fallback Roskam leading-edge suction parameter R',
        )

    def setup(self):
        self.add_input('CL',         val=0.5,   units='unitless',
                       desc='Aircraft total lift coefficient')
        self.add_input('AR_eff',     val=8.0,   units='unitless',
                       desc='Effective aspect ratio (Scholz endplate correction)')
        self.add_input('CL_alpha_w', val=5.5,   units='unitless',
                       desc='3-D wing lift-curve slope [1/rad] at AR_eff')
        self.add_input('mach', val=0.477, units='unitless',
                       desc='Mach number for leading-edge suction parameter')
        self.add_input('static_pressure', val=54048.0, units='Pa',
                       desc='Static pressure for Re_LER calculation')
        self.add_input('temperature', val=255.65, units='K',
                       desc='Static temperature for Re_LER calculation')
        self.add_input('mean_aerodynamic_chord', val=0.255, units='m',
                       desc='Wing mean aerodynamic chord')
        self.add_input('thickness_to_chord', val=0.15, units='unitless',
                       desc='Wing airfoil thickness-to-chord ratio')
        self.add_input('leading_edge_sweep', val=0.0, units='deg',
                       desc='Wing leading-edge sweep angle Lambda_LE')
        self.add_input('taper_ratio', val=0.6, units='unitless',
                       desc='Wing taper ratio lambda')

        self.add_output('CDi',      val=0.05,  units='unitless',
                        desc='Wing induced drag coefficient (Roskam Eq. 4.8, no twist)')
        self.add_output('e_oswald', val=1.0,   units='unitless',
                        desc='Oswald span efficiency factor (Roskam Eq. 4.12)')
        self.add_output('leading_edge_radius', val=0.0063, units='m',
                        desc='Leading-edge radius used for Re_LER')
        self.add_output('leading_edge_reynolds_number', val=4.0e4, units='unitless',
                        desc='Leading-edge Reynolds number Re_LER')
        self.add_output('leading_edge_suction_parameter', val=0.98, units='unitless',
                        desc='Leading-edge suction parameter R used in Eq. 4.12')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        cl        = float(inputs['CL'].ravel()[0])
        ar_eff    = float(inputs['AR_eff'].ravel()[0])
        cl_alpha  = float(inputs['CL_alpha_w'].ravel()[0])
        mach      = float(inputs['mach'].ravel()[0])
        pressure  = float(inputs['static_pressure'].ravel()[0])
        temp      = float(inputs['temperature'].ravel()[0])
        c_mac     = float(inputs['mean_aerodynamic_chord'].ravel()[0])
        tc        = float(inputs['thickness_to_chord'].ravel()[0])
        le_sweep_deg = float(inputs['leading_edge_sweep'].ravel()[0])
        taper     = float(inputs['taper_ratio'].ravel()[0])

        if ar_eff <= 0.0:
            raise ValueError(
                f'RoskamInducedDragComp: AR_eff must be > 0; got {ar_eff}.'
            )
        if cl_alpha <= 0.0:
            raise ValueError(
                f'RoskamInducedDragComp: CL_alpha_w must be > 0; got {cl_alpha}.'
            )
        if c_mac <= 0.0:
            raise ValueError(
                f'RoskamInducedDragComp: mean_aerodynamic_chord must be > 0; got {c_mac}.'
            )
        if taper < 0.0:
            raise ValueError(
                f'RoskamInducedDragComp: taper_ratio must be >= 0; got {taper}.'
            )

        leading_edge_radius = float(
            leading_edge_radius_ratio_naca_4_digit(tc) * c_mac
        )
        re_ler = leading_edge_reynolds_number_from_mach(
            mach,
            pressure,
            temp,
            leading_edge_radius,
        )
        if self.options['compute_leading_edge_suction']:
            r_suction = leading_edge_suction_parameter_roskam(
                re_ler,
                mach,
                np.deg2rad(le_sweep_deg),
                ar_eff,
                taper,
            )
        else:
            r_suction = self.options['default_leading_edge_suction_parameter']

        cdi, e = wing_induced_drag_roskam(
            cl,
            cl_alpha,
            ar_eff,
            leading_edge_suction_parameter=r_suction,
        )
        outputs['CDi']      = cdi
        outputs['e_oswald'] = e
        outputs['leading_edge_radius'] = leading_edge_radius
        outputs['leading_edge_reynolds_number'] = re_ler
        outputs['leading_edge_suction_parameter'] = r_suction


class FuselageLiftInducedDragComp(om.ExplicitComponent):
    """Roskam fuselage drag coefficient due to lift.

    Roskam Part VI Section 4.3.1.2, Eq. 4.33:

        CD_L_fus = 2*alpha**2*S_b_fus/S
                 + eta*c_d_c*alpha**3*S_plf_fus/S

    where alpha is the airplane/fuselage angle of attack in radians.  This
    component outputs a coefficient only and is intended to be report-only until
    the local drag build-up is connected to the mission polar.

    Base drag is intentionally not included here; add it later as a separate
    optional term.
    """

    def setup(self):
        self.add_input('aircraft_alpha', val=15.0, units='deg',
                       desc='Aircraft angle of attack used for fuselage lift drag')
        self.add_input('fuselage_base_area', val=0.001, units='m**2',
                       desc='Fuselage base area S_b_fus')
        self.add_input('fuselage_planform_area', val=0.25, units='m**2',
                       desc='Top-view fuselage planform area S_plf_fus')
        self.add_input('reference_area', val=0.45, units='m**2',
                       desc='Aircraft reference area S')
        self.add_input('eta_finite_cylinder', val=0.85, units='unitless',
                       desc='Finite-cylinder drag ratio eta; chart interpolation pending')
        self.add_input('crossflow_drag_coefficient', val=1.20, units='unitless',
                       desc='Infinite-cylinder crossflow drag coefficient c_d_c; chart interpolation pending')

        self.add_output('CDi_fus', val=0.0, units='unitless',
                        desc='Fuselage drag coefficient due to lift')
        self.add_output('CDi_fus_base_area_term', val=0.0, units='unitless',
                        desc='2*alpha^2*S_b_fus/S term')
        self.add_output('CDi_fus_planform_term', val=0.0, units='unitless',
                        desc='eta*c_d_c*alpha^3*S_plf_fus/S term')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        alpha = float(inputs['aircraft_alpha'].ravel()[0]) * np.pi / 180.0
        s_b = float(inputs['fuselage_base_area'].ravel()[0])
        s_plf = float(inputs['fuselage_planform_area'].ravel()[0])
        s_ref = float(inputs['reference_area'].ravel()[0])
        eta = float(inputs['eta_finite_cylinder'].ravel()[0])
        c_d_c = float(inputs['crossflow_drag_coefficient'].ravel()[0])

        if s_b < 0.0:
            raise ValueError(
                f'FuselageLiftInducedDragComp: fuselage_base_area must be >= 0; got {s_b}.'
            )
        if s_plf < 0.0:
            raise ValueError(
                f'FuselageLiftInducedDragComp: fuselage_planform_area must be >= 0; got {s_plf}.'
            )
        if s_ref <= 0.0:
            raise ValueError(
                f'FuselageLiftInducedDragComp: reference_area must be > 0; got {s_ref}.'
            )
        if eta < 0.0:
            raise ValueError(
                f'FuselageLiftInducedDragComp: eta_finite_cylinder must be >= 0; got {eta}.'
            )
        if c_d_c < 0.0:
            raise ValueError(
                'FuselageLiftInducedDragComp: crossflow_drag_coefficient must be '
                f'>= 0; got {c_d_c}.'
            )

        base_area_term = 2.0 * alpha**2 * s_b / s_ref
        planform_term = eta * c_d_c * alpha**3 * s_plf / s_ref

        outputs['CDi_fus_base_area_term'] = base_area_term
        outputs['CDi_fus_planform_term'] = planform_term
        outputs['CDi_fus'] = base_area_term + planform_term
