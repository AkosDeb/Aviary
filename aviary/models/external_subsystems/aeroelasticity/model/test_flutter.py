import unittest

import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.flutter import QuasiSteadyFlutterScreen
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestFlutterAnalysis(unittest.TestCase):
    def make_problem(self, name):
        prob = om.Problem(name=name, reports=False)
        prob.model.add_subsystem('flutter', QuasiSteadyFlutterScreen(num_speed_samples=30))
        prob.setup()
        prob.set_val(f'flutter.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'flutter.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'flutter.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'flutter.{AE.DESIGN_SPEED}', 80.0, units='m/s')
        prob.set_val(f'flutter.{AE.REQUIRED_SPEED}', 1.15 * 550.0 / 3.6, units='m/s')
        prob.set_val(f'flutter.{AE.FLUTTER_MAX_SPEED}', 250.0, units='m/s')
        prob.set_val(f'flutter.{AE.MASS_PER_UNIT_SPAN}', 1.5, units='kg/m')
        prob.set_val(f'flutter.{AE.PITCH_INERTIA_PER_UNIT_SPAN}', 0.05, units='kg*m')
        prob.set_val(f'flutter.{AE.CONTROL_INERTIA_PER_UNIT_SPAN}', 0.005, units='kg*m')
        prob.set_val(f'flutter.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', 0.0, units='kg/m')
        prob.set_val(f'flutter.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', 0.0, units='kg*m')
        prob.set_val(f'flutter.{AE.VTP_PITCH_STATIC_UNBALANCE}', 0.0, units='kg')
        prob.set_val(f'flutter.{AE.BENDING_STIFFNESS_PLUNGE}', 2.0e5, units='N/m')
        prob.set_val(f'flutter.{AE.TORSIONAL_STIFFNESS}', 1.5e4, units='N*m/rad')
        prob.set_val(f'flutter.{AE.CONTROL_STIFFNESS}', 5.0e3, units='N*m/rad')
        return prob

    def test_outputs_are_finite_and_margin_tracks_speed(self):
        prob = self.make_problem('test_flutter_outputs')
        prob.run_model()

        flutter_speed = prob.get_val(f'flutter.{AE.FLUTTER_SPEED}', units='m/s')
        flutter_q = prob.get_val(f'flutter.{AE.FLUTTER_DYNAMIC_PRESSURE}', units='Pa')
        flutter_margin = prob.get_val(f'flutter.{AE.FLUTTER_MARGIN}')
        flutter_speed_margin = prob.get_val(f'flutter.{AE.FLUTTER_SPEED_MARGIN}', units='m/s')

        self.assertGreater(flutter_speed, 0.0)
        self.assertGreater(flutter_q, 0.0)
        self.assertAlmostEqual(float(flutter_margin[0]), float(flutter_speed[0]) / 80.0 - 1.0)
        self.assertAlmostEqual(
            float(flutter_speed_margin[0]),
            float(flutter_speed[0]) - 1.15 * 550.0 / 3.6,
        )

    def test_default_design_point_is_stable(self):
        prob = self.make_problem('test_flutter_design_point_stability')
        prob.run_model()

        design_eig = float(
            prob.get_val(f'flutter.{AE.MAX_REAL_EIGENVALUE_AT_DESIGN}', units='1/s')[0]
        )
        flutter_speed = float(prob.get_val(f'flutter.{AE.FLUTTER_SPEED}', units='m/s')[0])

        self.assertLess(design_eig, 0.0)
        self.assertGreater(flutter_speed, 80.0)

    def test_vtp_mass_changes_flutter_result(self):
        no_vtp = self.make_problem('test_flutter_without_vtp')
        with_vtp = self.make_problem('test_flutter_with_vtp')
        with_vtp.set_val(f'flutter.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', 0.2, units='kg/m')
        with_vtp.set_val(f'flutter.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', 0.02, units='kg*m')
        with_vtp.set_val(f'flutter.{AE.VTP_PITCH_STATIC_UNBALANCE}', -0.02, units='kg')

        no_vtp.run_model()
        with_vtp.run_model()

        no_vtp_eig = float(
            no_vtp.get_val(f'flutter.{AE.MAX_REAL_EIGENVALUE_AT_DESIGN}', units='1/s')[0]
        )
        with_vtp_eig = float(
            with_vtp.get_val(f'flutter.{AE.MAX_REAL_EIGENVALUE_AT_DESIGN}', units='1/s')[0]
        )
        self.assertNotAlmostEqual(no_vtp_eig, with_vtp_eig)


if __name__ == '__main__':
    unittest.main()
