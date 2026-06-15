import unittest

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.pk_flutter import (
    PKFlutterAnalysis,
    placeholder_pk_aero_matrices,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestPKFlutterAnalysis(unittest.TestCase):
    def make_problem(self, name):
        prob = om.Problem(name=name, reports=False)
        prob.model.add_subsystem(
            'pk',
            PKFlutterAnalysis(num_speed_samples=12, pk_iterations=12),
        )
        prob.setup()
        prob.set_val(f'pk.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'pk.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'pk.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'pk.{AE.REQUIRED_SPEED}', 1.15 * 550.0 / 3.6, units='m/s')
        prob.set_val(f'pk.{AE.FLUTTER_MAX_SPEED}', 250.0, units='m/s')
        prob.set_val(f'pk.{AE.MASS_PER_UNIT_SPAN}', 1.5, units='kg/m')
        prob.set_val(f'pk.{AE.PITCH_INERTIA_PER_UNIT_SPAN}', 0.05, units='kg*m')
        prob.set_val(f'pk.{AE.CONTROL_INERTIA_PER_UNIT_SPAN}', 0.005, units='kg*m')
        prob.set_val(f'pk.{AE.BENDING_STIFFNESS_PLUNGE}', 2.0e5, units='N/m')
        prob.set_val(f'pk.{AE.TORSIONAL_STIFFNESS}', 1.5e4, units='N*m/rad')
        prob.set_val(f'pk.{AE.CONTROL_STIFFNESS}', 5.0e3, units='N*m/rad')
        return prob

    def test_placeholder_aero_depends_on_reduced_frequency(self):
        q_real_0, q_imag_0 = placeholder_pk_aero_matrices(
            0.0, 0.8, 0.4, 2.0 * np.pi, 2.5, 0.7, -0.6, -0.02, -0.3
        )
        q_real_1, q_imag_1 = placeholder_pk_aero_matrices(
            1.0, 0.8, 0.4, 2.0 * np.pi, 2.5, 0.7, -0.6, -0.02, -0.3
        )

        self.assertGreater(np.linalg.norm(q_real_0), np.linalg.norm(q_real_1))
        self.assertAlmostEqual(np.linalg.norm(q_imag_0), 0.0)
        self.assertGreater(np.linalg.norm(q_imag_1), 0.0)

    def test_pk_outputs_are_finite(self):
        prob = self.make_problem('test_pk_outputs')
        prob.run_model()

        speed = prob.get_val(f'pk.{AE.PK_FLUTTER_SPEED}', units='m/s')
        freq = prob.get_val(f'pk.{AE.PK_FLUTTER_FREQUENCY}', units='Hz')
        mode_freq = prob.get_val(f'pk.{AE.PK_MODE_FREQUENCY}', units='Hz')
        mode_damping = prob.get_val(f'pk.{AE.PK_MODE_DAMPING}')
        speed_margin = prob.get_val(f'pk.{AE.PK_FLUTTER_SPEED_MARGIN}', units='m/s')

        self.assertGreater(float(speed[0]), 0.0)
        self.assertGreaterEqual(float(freq[0]), 0.0)
        self.assertEqual(mode_freq.shape, (3,))
        self.assertEqual(mode_damping.shape, (3,))
        self.assertTrue(np.all(np.isfinite(mode_freq)))
        self.assertTrue(np.all(np.isfinite(mode_damping)))
        self.assertAlmostEqual(
            float(speed_margin[0]),
            float(speed[0]) - 1.15 * 550.0 / 3.6,
        )

    def test_vtp_mass_changes_pk_modal_result(self):
        no_vtp = self.make_problem('test_pk_without_vtp')
        with_vtp = self.make_problem('test_pk_with_vtp')
        with_vtp.set_val(f'pk.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', 0.2, units='kg/m')
        with_vtp.set_val(f'pk.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', 0.02, units='kg*m')
        with_vtp.set_val(f'pk.{AE.VTP_PITCH_STATIC_UNBALANCE}', -0.02, units='kg')

        no_vtp.run_model()
        with_vtp.run_model()

        no_vtp_freq = no_vtp.get_val(f'pk.{AE.PK_MODE_FREQUENCY}', units='Hz')
        with_vtp_freq = with_vtp.get_val(f'pk.{AE.PK_MODE_FREQUENCY}', units='Hz')
        self.assertGreater(np.linalg.norm(no_vtp_freq - with_vtp_freq), 0.0)


if __name__ == '__main__':
    unittest.main()
