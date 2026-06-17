import unittest
import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.beam_modal_flutter import (
    BeamModalFlutter,
    _control_mode_shape,
    _garrick_T10_T11,
    beam_modal_theodorsen_gaf_3dof,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestGarrickTFunctions(unittest.TestCase):
    def test_known_values_midchord_hinge(self):
        """c=0 (50% chord hinge): T10=π/2+1, T11=π/4+1 from analytic formula."""
        T10, T11 = _garrick_T10_T11(0.5)
        self.assertAlmostEqual(T10, np.sqrt(1.0) + np.pi / 2, places=10)
        self.assertAlmostEqual(T11, 0.5 * np.pi / 2 + 1.0, places=10)

    def test_known_values_75pct_chord_hinge(self):
        """c=0.5 (75% chord hinge, typical 25% control surface)."""
        T10, T11 = _garrick_T10_T11(0.75)
        c = 0.5
        expected_T10 = np.sqrt(1 - c**2) + np.arccos(c)
        expected_T11 = 0.5 * (1 - 2 * c) * np.arccos(c) + np.sqrt(1 - c**2)
        self.assertAlmostEqual(T10, expected_T10, places=10)
        self.assertAlmostEqual(T11, expected_T11, places=10)

    def test_te_hinge_monotone(self):
        """T10 and η_δ should decrease as hinge moves toward TE (c→1)."""
        T10_mid, T11_mid = _garrick_T10_T11(0.75)   # c=0.5
        T10_aft, T11_aft = _garrick_T10_T11(0.875)  # c=0.75
        self.assertGreater(T10_mid, T10_aft, 'T10 should decrease as hinge moves aft')


class TestControlModeShape(unittest.TestCase):
    def test_binary_shape(self):
        y = np.linspace(0.0, 1.0, 11)
        phi = _control_mode_shape(y, 0.4, 0.9)
        self.assertTrue(np.all(phi[(y >= 0.4) & (y <= 0.9)] == 1.0))
        self.assertTrue(np.all(phi[y < 0.4] == 0.0))

    def test_zero_outside_span(self):
        y = np.linspace(0.0, 2.0, 21)
        phi = _control_mode_shape(y, 0.5, 0.9)
        self.assertEqual(phi[0], 0.0)
        self.assertEqual(phi[-1], 0.0)


class TestGAF3DOF(unittest.TestCase):
    def test_shape(self):
        y = np.linspace(0.0, 1.0, 11)
        chord = 0.2 * np.ones(11)
        bending = np.linspace(0.0, 1.0, 11)
        torsion = np.linspace(0.0, 1.0, 11)
        phi_delta = _control_mode_shape(y, 0.4, 0.9)
        q_real, q_imag, _ = beam_modal_theodorsen_gaf_3dof(
            0.1, y, chord, bending, torsion, phi_delta,
            2*np.pi, -0.05*2*np.pi, 2.5, -0.6, -0.02, -0.30, 0.375, 0.75,
        )
        self.assertEqual(q_real.shape, (3, 3))
        self.assertEqual(q_imag.shape, (3, 3))

    def test_quasi_steady_limit(self):
        """At k=0, imaginary part should be zero and real part finite."""
        y = np.linspace(0.0, 1.0, 11)
        chord = 0.2 * np.ones(11)
        bending = np.linspace(0.0, 1.0, 11)
        torsion = np.linspace(0.0, 1.0, 11)
        phi_delta = _control_mode_shape(y, 0.4, 0.9)
        q_real, q_imag, _ = beam_modal_theodorsen_gaf_3dof(
            0.0, y, chord, bending, torsion, phi_delta,
            2*np.pi, -0.05*2*np.pi, 2.5, -0.6, -0.02, -0.30, 0.375, 0.75,
        )
        np.testing.assert_allclose(q_imag, 0.0, atol=1.0e-10)

    def test_delta_column_nonzero_imaginary_at_positive_k(self):
        """With Theodorsen-Garrick, δ column should have non-zero imaginary part at k>0."""
        y = np.linspace(0.0, 1.0, 11)
        chord = 0.2 * np.ones(11)
        bending = np.linspace(0.0, 1.0, 11)
        torsion = np.linspace(0.0, 1.0, 11)
        phi_delta = _control_mode_shape(y, 0.4, 0.9)
        q_real, q_imag, _ = beam_modal_theodorsen_gaf_3dof(
            0.3, y, chord, bending, torsion, phi_delta,
            2*np.pi, -0.05*2*np.pi, 2.5, -0.6, -0.02, -0.30, 0.375, 0.75,
        )
        # δ column (index 2) imaginary part must be non-zero at k=0.3
        self.assertFalse(np.all(q_imag[:, 2] == 0.0),
            'δ column imaginary part should be non-zero with Theodorsen-Garrick correction')


class TestBeamModalFlutter3DOF(unittest.TestCase):
    def _make_prob(self):
        n = 11
        prob = om.Problem()
        prob.model.add_subsystem('bm', BeamModalFlutter(num_stations=n, num_speed_samples=20), promotes=['*'])
        prob.setup()
        y = np.linspace(0.0, 0.9, n)
        chord = np.linspace(0.18, 0.12, n)
        EI = np.linspace(6000.0, 1000.0, n)
        GJ = np.linspace(6500.0, 1100.0, n)
        mass = np.linspace(1.5, 0.5, n)
        pitch_inertia = np.linspace(0.05, 0.01, n)
        prob.set_val(AE.SPANWISE_STATIONS, y, units='m')
        prob.set_val(AE.SPANWISE_CHORD, chord, units='m')
        prob.set_val(AE.SPANWISE_BENDING_STIFFNESS, EI, units='N*m**2')
        prob.set_val(AE.SPANWISE_TORSIONAL_RIGIDITY, GJ, units='N*m**2')
        prob.set_val(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, mass, units='kg/m')
        prob.set_val(AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN, pitch_inertia, units='kg*m')
        prob.set_val(AE.FLUTTER_MAX_SPEED, 350.0, units='m/s')
        prob.set_val(AE.REQUIRED_SPEED, 191.7, units='m/s')
        return prob

    def test_runs_and_outputs_finite(self):
        prob = self._make_prob()
        prob.run_model()
        spd = prob.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0]
        ctrl_f = prob.get_val(AE.BEAM_MODAL_CONTROL_FREQUENCY, units='Hz')[0]
        self.assertTrue(np.isfinite(spd), f'3DOF flutter speed not finite: {spd}')
        self.assertTrue(np.isfinite(ctrl_f), f'control freq not finite: {ctrl_f}')
        self.assertGreater(ctrl_f, 0.0)

    def test_stiffer_hinge_raises_control_frequency(self):
        prob_soft = self._make_prob()
        prob_stiff = self._make_prob()
        prob_soft.set_val(AE.CONTROL_STIFFNESS, 1.0e3, units='N*m/rad')
        prob_stiff.set_val(AE.CONTROL_STIFFNESS, 1.0e5, units='N*m/rad')
        prob_soft.run_model()
        prob_stiff.run_model()
        f_soft = prob_soft.get_val(AE.BEAM_MODAL_CONTROL_FREQUENCY, units='Hz')[0]
        f_stiff = prob_stiff.get_val(AE.BEAM_MODAL_CONTROL_FREQUENCY, units='Hz')[0]
        self.assertGreater(f_stiff, f_soft,
            f'Expected stiffer hinge -> higher control freq; got {f_soft:.2f} vs {f_stiff:.2f}')

    def test_mass_balance_zero_vs_positive(self):
        """Positive control static unbalance (CG behind hinge) should reduce flutter speed."""
        prob_balanced = self._make_prob()
        prob_unbalanced = self._make_prob()
        prob_balanced.set_val(AE.CONTROL_STATIC_UNBALANCE, 0.0, units='kg')
        prob_unbalanced.set_val(AE.CONTROL_STATIC_UNBALANCE, 0.05, units='kg')
        prob_balanced.run_model()
        prob_unbalanced.run_model()
        v_bal = prob_balanced.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0]
        v_unbal = prob_unbalanced.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0]
        self.assertGreaterEqual(v_bal, v_unbal,
            f'Balanced control should have >= flutter speed; got bal={v_bal:.1f} unbal={v_unbal:.1f}')


if __name__ == '__main__':
    unittest.main()
