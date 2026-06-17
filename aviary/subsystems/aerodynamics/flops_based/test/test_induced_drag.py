import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_partials, assert_near_equal
from openmdao.utils.testing_utils import use_tempdirs

from aviary.subsystems.aerodynamics.aero_utils import (
    leading_edge_radius_ratio_naca_4_digit,
    leading_edge_reynolds_number_from_mach,
    leading_edge_suction_parameter_roskam,
    wing_induced_drag_roskam,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.induced_drag import (
    FuselageLiftInducedDragComp,
    RoskamInducedDragComp,
)
from aviary.subsystems.aerodynamics.flops_based.induced_drag import InducedDrag
from aviary.variable_info.variables import Aircraft, Dynamic


@use_tempdirs
class InducedDragTest(unittest.TestCase):
    def test_roskam_induced_drag_uses_actual_cl_without_trim_factor(self):
        prob = om.Problem()
        prob.model.add_subsystem('drag', RoskamInducedDragComp(), promotes=['*'])
        prob.setup()

        cl = 0.5
        cl_alpha_w = 5.5
        ar_eff = 8.0
        leading_edge_suction_parameter = 0.98
        expected_cdi, expected_e = wing_induced_drag_roskam(
            cl,
            cl_alpha_w,
            ar_eff,
            leading_edge_suction_parameter=leading_edge_suction_parameter,
        )

        prob.set_val('CL', cl)
        prob.set_val('CL_alpha_w', cl_alpha_w)
        prob.set_val('AR_eff', ar_eff)
        prob.run_model()

        assert_near_equal(prob.get_val('CDi'), expected_cdi, 1e-12)
        assert_near_equal(prob.get_val('e_oswald'), expected_e, 1e-12)
        assert_near_equal(prob.get_val('leading_edge_suction_parameter'), 0.98, 1e-12)

    def test_roskam_induced_drag_can_compute_live_le_suction_diagnostics(self):
        prob = om.Problem()
        prob.model.add_subsystem(
            'drag',
            RoskamInducedDragComp(compute_leading_edge_suction=True),
            promotes=['*'],
        )
        prob.setup()

        prob.set_val('CL', 0.5)
        prob.set_val('CL_alpha_w', 5.5)
        prob.set_val('AR_eff', 8.0)
        prob.set_val('mach', 0.477)
        prob.set_val('static_pressure', 54048.0, units='Pa')
        prob.set_val('temperature', 255.65, units='K')
        prob.set_val('mean_aerodynamic_chord', 0.2552083333333333, units='m')
        prob.set_val('thickness_to_chord', 0.15)
        prob.set_val('leading_edge_sweep', 3.97, units='deg')
        prob.set_val('taper_ratio', 0.6)

        with self.assertWarns(RuntimeWarning):
            prob.run_model()

        radius_15 = float(prob.get_val('leading_edge_radius', units='m')[0])
        re_15 = float(prob.get_val('leading_edge_reynolds_number')[0])
        r_suction = prob.get_val('leading_edge_suction_parameter')

        expected_radius = (
            leading_edge_radius_ratio_naca_4_digit(0.15) * 0.2552083333333333
        )
        assert_near_equal(radius_15, expected_radius, 1e-12)
        assert_near_equal(re_15, 43763.49362902076, 1e-9)
        with self.assertWarns(RuntimeWarning):
            expected_r = leading_edge_suction_parameter_roskam(
                re_15,
                0.477,
                np.deg2rad(3.97),
                8.0,
                0.6,
            )
        assert_near_equal(r_suction, expected_r, 1e-12)

        prob.set_val('thickness_to_chord', 0.10)
        with self.assertWarns(RuntimeWarning):
            prob.run_model()

        assert prob.get_val('leading_edge_radius', units='m') < radius_15
        assert prob.get_val('leading_edge_reynolds_number') < re_15

    def test_leading_edge_suction_helpers(self):
        assert_near_equal(leading_edge_radius_ratio_naca_4_digit(0.12), 0.01586736, 1e-12)

        re_ler = leading_edge_reynolds_number_from_mach(
            mach=0.5,
            static_pressure_Pa=101325.0,
            temperature_K=288.15,
            leading_edge_radius_m=0.004,
        )
        assert_near_equal(re_ler, 46594.835362663594, 1e-9)

        with self.assertWarns(RuntimeWarning):
            r_main = leading_edge_suction_parameter_roskam(
                leading_edge_reynolds_number=1.0e4,
                mach=0.5,
                leading_edge_sweep_rad=np.deg2rad(45.0),
                aspect_ratio=8.0,
                taper_ratio=0.5,
            )
        assert 0.70 < r_main < 0.80

        with self.assertRaises(NotImplementedError):
            leading_edge_suction_parameter_roskam(
                leading_edge_reynolds_number=1.0e3,
                mach=0.5,
                leading_edge_sweep_rad=np.deg2rad(45.0),
                aspect_ratio=8.0,
                taper_ratio=0.5,
            )

        with self.assertWarns(RuntimeWarning):
            r_inset = leading_edge_suction_parameter_roskam(
                leading_edge_reynolds_number=1.0e5,
                mach=0.5,
                leading_edge_sweep_rad=0.0,
                aspect_ratio=8.0,
                taper_ratio=0.5,
            )
        assert_near_equal(r_inset, 0.958, 1e-12)

    def test_fuselage_lift_induced_drag(self):
        # fineness_ratio = 6.0 is an exact breakpoint -> eta = 0.64 (no interpolation)
        # Mach = 0.0 -> Mc = 0.0 -> c_d_c = 1.20 (first table entry)
        prob = om.Problem()
        prob.model.add_subsystem('drag', FuselageLiftInducedDragComp(), promotes=['*'])
        prob.setup()

        prob.set_val('aircraft_alpha', 15.0, units='deg')
        prob.set_val('fuselage_base_area', 0.0008343336047856176, units='m**2')
        prob.set_val('fuselage_planform_area', 0.228, units='m**2')
        prob.set_val('reference_area', 0.45, units='m**2')
        prob.set_val('fuselage_fineness_ratio', 6.0)
        prob.set_val('Mach', 0.0)
        prob.run_model()

        alpha = np.deg2rad(15.0)
        eta_expected   = 0.64   # exact table breakpoint l/d=6
        c_dc_expected  = 1.20   # Mc = 0*sin(15°) = 0 -> first table entry
        base_term      = 2.0 * alpha**2 * 0.0008343336047856176 / 0.45
        planform_term  = eta_expected * c_dc_expected * alpha**3 * 0.228 / 0.45

        assert_near_equal(prob.get_val('eta_finite_cylinder'),        eta_expected,  1e-12)
        assert_near_equal(prob.get_val('crossflow_drag_coefficient'), c_dc_expected, 1e-12)
        assert_near_equal(prob.get_val('CDi_fus_base_area_term'), base_term,    1e-12)
        assert_near_equal(prob.get_val('CDi_fus_planform_term'),  planform_term, 1e-12)
        assert_near_equal(prob.get_val('CDi_fus'), base_term + planform_term,   1e-12)

    def test_fuselage_lift_induced_drag_interpolated(self):
        # fineness_ratio = 9.0  ->  eta = 0.64 + (9-6)/(12-6)*(0.71-0.64) = 0.675
        # Mach = 0.5, alpha = 15 deg  ->  Mc = 0.5*sin(15°) ~ 0.1294
        #   Mc is in [0, 0.25] interval -> c_d_c = 1.2 (flat segment)
        prob = om.Problem()
        prob.model.add_subsystem('drag', FuselageLiftInducedDragComp(), promotes=['*'])
        prob.setup()

        prob.set_val('aircraft_alpha', 15.0, units='deg')
        prob.set_val('fuselage_base_area', 0.001, units='m**2')
        prob.set_val('fuselage_planform_area', 0.228, units='m**2')
        prob.set_val('reference_area', 0.45, units='m**2')
        prob.set_val('fuselage_fineness_ratio', 9.0)
        prob.set_val('Mach', 0.5)
        prob.run_model()

        alpha      = np.deg2rad(15.0)
        eta_exp    = 0.64 + (9.0 - 6.0) / (12.0 - 6.0) * (0.71 - 0.64)   # 0.675
        Mc         = 0.5 * np.sin(alpha)
        c_dc_exp   = np.interp(Mc, [0.0, 0.25, 0.4, 0.5, 0.7],
                                    [1.2, 1.2, 1.27, 1.37, 1.68])

        assert_near_equal(prob.get_val('eta_finite_cylinder'),        eta_exp,  1e-10)
        assert_near_equal(prob.get_val('crossflow_drag_coefficient'), c_dc_exp, 1e-10)

    def test_derivs(self):
        P = 2.60239151
        Sref = 1370.0

        CL = np.array([0.3, 0.35, 0.4, 0.45, 0.5, 0.55])
        mach = np.array([0.4, 0.45, 0.5, 0.55, 0.6, 0.85])
        lift = 0.5 * CL * Sref * 1.4 * P * mach**2

        nn = len(CL)

        prob = om.Problem()

        options = {}
        options[Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION] = False

        prob.model.add_subsystem(
            'induced_drag', InducedDrag(num_nodes=nn, **options), promotes=['*']
        )
        prob.setup(force_alloc_complex=True)

        prob.set_val(Dynamic.Atmosphere.MACH, val=mach)
        prob.set_val(Dynamic.Vehicle.LIFT, val=lift)
        prob.set_val(Dynamic.Atmosphere.STATIC_PRESSURE, val=P)
        prob.set_val(Aircraft.Wing.AREA, val=Sref)
        prob.set_val(Aircraft.Wing.SWEEP, val=-25.03)
        prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.278)
        prob.set_val(Aircraft.Wing.SPAN_EFFICIENCY_FACTOR, 0.7)

        prob.set_val(Aircraft.Wing.ASPECT_RATIO, val=11.05)

        prob.run_model()

        derivs = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(derivs, atol=1e-12, rtol=8e-12)

        assert_near_equal(
            prob.get_val('induced_drag_coeff'),
            [0.00370367, 0.00504111, 0.0065843, 0.00833326, 0.01028797, 0.01244845],
            1e-6,
        )

    def test_derivs_span_eff_redux(self):
        P = 2.60239151
        Sref = 1370.0

        CL = np.array([0.3, 0.35, 0.4, 0.45, 0.5, 0.55])
        mach = np.array([0.4, 0.45, 0.5, 0.55, 0.6, 0.85])
        lift = 0.5 * CL * Sref * 1.4 * P * mach**2

        nn = len(CL)

        # High factor

        prob = om.Problem()

        options = {}
        options[Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION] = True

        prob.model.add_subsystem('drag', InducedDrag(num_nodes=nn, **options), promotes=['*'])
        prob.setup(force_alloc_complex=True)

        prob.set_val(Dynamic.Atmosphere.MACH, val=mach)
        prob.set_val(Dynamic.Vehicle.LIFT, val=lift)
        prob.set_val(Dynamic.Atmosphere.STATIC_PRESSURE, val=P)
        prob.set_val(Aircraft.Wing.AREA, val=Sref)
        prob.set_val(Aircraft.Wing.SWEEP, val=-25.10)
        prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.312)
        prob.set_val(Aircraft.Wing.SPAN_EFFICIENCY_FACTOR, 0.528)

        prob.set_val(Aircraft.Wing.ASPECT_RATIO, val=11.05)

        prob.run_model()

        derivs = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(derivs, atol=1e-12, rtol=8e-12)

        assert_near_equal(
            prob.get_val('induced_drag_coeff'),
            [0.00216084, 0.00294208, 0.00384454, 0.00486925, 0.00601801, 0.00748097],
            1e-6,
        )

        # Low factor.

        prob = om.Problem(model=om.Group())

        options = {}
        options[Aircraft.Wing.SPAN_EFFICIENCY_REDUCTION] = True

        prob.model.add_subsystem('drag', InducedDrag(num_nodes=nn, **options), promotes=['*'])
        prob.setup(force_alloc_complex=True)

        prob.set_val(Dynamic.Atmosphere.MACH, val=mach)
        prob.set_val(Dynamic.Vehicle.LIFT, val=lift)
        prob.set_val(Dynamic.Atmosphere.STATIC_PRESSURE, val=P)
        prob.set_val(Aircraft.Wing.AREA, val=Sref)
        prob.set_val(Aircraft.Wing.SWEEP, val=-25.10)
        prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.312)
        prob.set_val(Aircraft.Wing.SPAN_EFFICIENCY_FACTOR, 0.528)

        prob.set_val(Aircraft.Wing.ASPECT_RATIO, val=11.05)

        prob.run_model()

        derivs = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(derivs, atol=1e-12, rtol=8e-12)

        assert_near_equal(
            prob.get_val('induced_drag_coeff'),
            [0.00216084, 0.00294208, 0.00384454, 0.00486925, 0.00601801, 0.00748097],
            1e-6,
        )


if __name__ == '__main__':
    unittest.main()
