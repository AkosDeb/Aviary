import unittest

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.pk_flutter import (
    PKFlutterAnalysis,
    build_structural_matrices,
    pk_modes_at_speed,
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

    def test_pk_helpers_preserve_complex_step_perturbations(self):
        M, C, _ = build_structural_matrices(
            1.5 + 1.0e-40j, 0.05, 0.005, 0.0, 0.0, 2.0e5, 1.5e4, 5.0e3, 0.02
        )
        q_real, q_imag = placeholder_pk_aero_matrices(
            0.25, 0.8 + 1.0e-40j, 0.4, 2.0 * np.pi, 2.5, 0.7, -0.6, -0.02, -0.3
        )

        self.assertNotEqual(np.imag(M[0, 0]), 0.0)
        self.assertTrue(np.any(np.imag(C) != 0.0))
        self.assertTrue(np.any(np.imag(q_real) != 0.0))
        self.assertTrue(np.any(np.imag(q_imag) != 0.0))

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

    def test_dry_modal_frequencies_are_plausible(self):
        M, C, K = build_structural_matrices(
            1.5, 0.05, 0.005, 0.0, 0.0, 2.0e5, 1.5e4, 5.0e3, 0.02
        )
        chord = 0.4
        aero_args = (0.8, chord, 2.0 * np.pi, 2.5, 0.7, -0.6, -0.02, -0.3)
        freqs, _, _ = pk_modes_at_speed(
            speed=1.0, rho=1.225, chord=chord,
            mass_matrix=M, damping_matrix=C, stiffness_matrix=K,
            aero_args=aero_args, num_modes=3,
        )

        # Specific Hz targets are NOT asserted here — independent beam/FE calibration
        # gives torsion ≈ 148 Hz and control ≈ 990 Hz for the SpaJeti geometry, which
        # do not match the simplified inertia values used in this test.  Tight bounds
        # are deferred until the inertia inputs are traced to the structural box chain.
        for i, f in enumerate(freqs):
            self.assertGreater(f, 1.0, msg=f'Mode {i}: {f:.2f} Hz below 1 Hz')
            self.assertLess(f, 2000.0, msg=f'Mode {i}: {f:.2f} Hz above 2000 Hz')

        self.assertLess(freqs[0], freqs[1])
        self.assertLess(freqs[1], freqs[2])


class TestPKConvergenceFlag(unittest.TestCase):
    def _setup_spajeti(self, prob):
        prob.set_val(f'pk.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'pk.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'pk.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'pk.{AE.FLUTTER_MAX_SPEED}', 250.0, units='m/s')
        prob.set_val(f'pk.{AE.MASS_PER_UNIT_SPAN}', 1.5, units='kg/m')
        prob.set_val(f'pk.{AE.PITCH_INERTIA_PER_UNIT_SPAN}', 0.05, units='kg*m')
        prob.set_val(f'pk.{AE.CONTROL_INERTIA_PER_UNIT_SPAN}', 0.005, units='kg*m')
        prob.set_val(f'pk.{AE.BENDING_STIFFNESS_PLUNGE}', 2.0e5, units='N/m')
        prob.set_val(f'pk.{AE.TORSIONAL_STIFFNESS}', 1.5e4, units='N*m/rad')
        prob.set_val(f'pk.{AE.CONTROL_STIFFNESS}', 5.0e3, units='N*m/rad')

    def test_pk_converged_is_one_for_well_conditioned_case(self):
        # Flutter_max_speed set to 20 m/s — far below any realistic flutter speed for
        # SpaJeti geometry.  Weak aero coupling guarantees k-iteration converges at
        # every speed point so PK_CONVERGED must be 1.0.
        prob = om.Problem(name='test_pk_conv_good', reports=False)
        prob.model.add_subsystem('pk', PKFlutterAnalysis(num_speed_samples=10))
        prob.setup()
        self._setup_spajeti(prob)
        prob.set_val(f'pk.{AE.FLUTTER_MAX_SPEED}', 20.0, units='m/s')
        prob.run_model()
        self.assertEqual(float(prob.get_val(f'pk.{AE.PK_CONVERGED}')[0]), 1.0)

    def test_pk_converged_is_zero_when_insufficient_iterations(self):
        prob = om.Problem(name='test_pk_conv_tight', reports=False)
        prob.model.add_subsystem(
            'pk',
            PKFlutterAnalysis(num_speed_samples=5, pk_iterations=1, pk_tolerance=1.0e-20),
        )
        prob.setup()
        self._setup_spajeti(prob)
        prob.run_model()
        self.assertEqual(float(prob.get_val(f'pk.{AE.PK_CONVERGED}')[0]), 0.0)


if __name__ == '__main__':
    unittest.main()
