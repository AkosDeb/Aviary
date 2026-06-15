"""SpaJeti/Roskam induced-drag components."""

import numpy as np
import openmdao.api as om

from aviary.subsystems.aerodynamics.aero_utils import (
    leading_edge_radius_ratio_naca_4_digit,
    leading_edge_reynolds_number_from_mach,
    leading_edge_suction_parameter_roskam,
    wing_induced_drag_roskam,
)


class RoskamInducedDragComp(om.ExplicitComponent):
    """Wing induced drag from Roskam Part VI Section 4.2.1.2, Eq. 4.8."""

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
        self.add_input('CL', val=0.5, units='unitless',
                       desc='Aircraft total lift coefficient')
        self.add_input('AR_eff', val=8.0, units='unitless',
                       desc='Effective aspect ratio (Scholz endplate correction)')
        self.add_input('CL_alpha_w', val=5.5, units='unitless',
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

        self.add_output('CDi', val=0.05, units='unitless',
                        desc='Wing induced drag coefficient (Roskam Eq. 4.8, no twist)')
        self.add_output('e_oswald', val=1.0, units='unitless',
                        desc='Oswald span efficiency factor (Roskam Eq. 4.12)')
        self.add_output('leading_edge_radius', val=0.0063, units='m',
                        desc='Leading-edge radius used for Re_LER')
        self.add_output('leading_edge_reynolds_number', val=4.0e4, units='unitless',
                        desc='Leading-edge Reynolds number Re_LER')
        self.add_output('leading_edge_suction_parameter', val=0.98, units='unitless',
                        desc='Leading-edge suction parameter R used in Eq. 4.12')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        cl = float(inputs['CL'].ravel()[0])
        ar_eff = float(inputs['AR_eff'].ravel()[0])
        cl_alpha = float(inputs['CL_alpha_w'].ravel()[0])
        mach = float(inputs['mach'].ravel()[0])
        pressure = float(inputs['static_pressure'].ravel()[0])
        temp = float(inputs['temperature'].ravel()[0])
        c_mac = float(inputs['mean_aerodynamic_chord'].ravel()[0])
        tc = float(inputs['thickness_to_chord'].ravel()[0])
        le_sweep_deg = float(inputs['leading_edge_sweep'].ravel()[0])
        taper = float(inputs['taper_ratio'].ravel()[0])

        if ar_eff <= 0.0:
            raise ValueError(f'RoskamInducedDragComp: AR_eff must be > 0; got {ar_eff}.')
        if cl_alpha <= 0.0:
            raise ValueError(f'RoskamInducedDragComp: CL_alpha_w must be > 0; got {cl_alpha}.')
        if c_mac <= 0.0:
            raise ValueError(
                f'RoskamInducedDragComp: mean_aerodynamic_chord must be > 0; got {c_mac}.'
            )
        if taper < 0.0:
            raise ValueError(f'RoskamInducedDragComp: taper_ratio must be >= 0; got {taper}.')

        leading_edge_radius = float(leading_edge_radius_ratio_naca_4_digit(tc) * c_mac)
        re_ler = leading_edge_reynolds_number_from_mach(
            mach, pressure, temp, leading_edge_radius,
        )

        if self.options['compute_leading_edge_suction']:
            r_suction = leading_edge_suction_parameter_roskam(
                re_ler, mach, np.deg2rad(le_sweep_deg), ar_eff, taper,
            )
        else:
            r_suction = self.options['default_leading_edge_suction_parameter']

        cdi, e = wing_induced_drag_roskam(
            cl, cl_alpha, ar_eff, leading_edge_suction_parameter=r_suction,
        )
        outputs['CDi'] = cdi
        outputs['e_oswald'] = e
        outputs['leading_edge_radius'] = leading_edge_radius
        outputs['leading_edge_reynolds_number'] = re_ler
        outputs['leading_edge_suction_parameter'] = r_suction


_ETA_FIN_CYL_LD = np.array([2.0, 6.0, 12.0, 18.0, 28.0])
_ETA_FIN_CYL_ETA = np.array([0.52, 0.64, 0.71, 0.75, 0.79])
_CDC_MC = np.array([0.00, 0.25, 0.40, 0.50, 0.70])
_CDC_CDC = np.array([1.20, 1.20, 1.27, 1.37, 1.68])


class FuselageLiftInducedDragComp(om.ExplicitComponent):
    """Roskam fuselage drag coefficient due to lift."""

    def setup(self):
        self.add_input('aircraft_alpha', val=15.0, units='deg',
                       desc='Aircraft angle of attack used for fuselage lift drag')
        self.add_input('fuselage_base_area', val=0.001, units='m**2',
                       desc='Fuselage base area S_b_fus')
        self.add_input('fuselage_planform_area', val=0.25, units='m**2',
                       desc='Top-view fuselage planform area S_plf_fus')
        self.add_input('reference_area', val=0.45, units='m**2',
                       desc='Aircraft reference area S')
        self.add_input('fuselage_fineness_ratio', val=8.0, units='unitless',
                       desc='Fuselage fineness ratio l/d; used to interpolate eta from Fig 4.32')
        self.add_input('Mach', val=0.0, units='unitless',
                       desc='Freestream Mach number; combined with alpha to form Mc = M*sin(alpha)')

        self.add_output('CDi_fus', val=0.0, units='unitless',
                        desc='Fuselage drag coefficient due to lift')
        self.add_output('CDi_fus_base_area_term', val=0.0, units='unitless',
                        desc='2*alpha^2*S_b_fus/S term')
        self.add_output('CDi_fus_planform_term', val=0.0, units='unitless',
                        desc='eta*c_d_c*alpha^3*S_plf_fus/S term')
        self.add_output('eta_finite_cylinder', val=0.64, units='unitless',
                        desc='Interpolated finite-cylinder interference factor (Fig 4.32)')
        self.add_output('crossflow_drag_coefficient', val=1.20, units='unitless',
                        desc='Interpolated infinite-cylinder crossflow drag coeff (Fig 4.31)')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        alpha_rad = float(inputs['aircraft_alpha'].ravel()[0]) * np.pi / 180.0
        s_b = float(inputs['fuselage_base_area'].ravel()[0])
        s_plf = float(inputs['fuselage_planform_area'].ravel()[0])
        s_ref = float(inputs['reference_area'].ravel()[0])
        fin_ratio = float(inputs['fuselage_fineness_ratio'].ravel()[0])
        mach = float(inputs['Mach'].ravel()[0])

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

        eta = float(np.interp(fin_ratio, _ETA_FIN_CYL_LD, _ETA_FIN_CYL_ETA))
        c_d_c = float(np.interp(mach * np.sin(alpha_rad), _CDC_MC, _CDC_CDC))
        base_area_term = 2.0 * alpha_rad**2 * s_b / s_ref
        planform_term = eta * c_d_c * alpha_rad**3 * s_plf / s_ref

        outputs['eta_finite_cylinder'] = eta
        outputs['crossflow_drag_coefficient'] = c_d_c
        outputs['CDi_fus_base_area_term'] = base_area_term
        outputs['CDi_fus_planform_term'] = planform_term
        outputs['CDi_fus'] = base_area_term + planform_term
