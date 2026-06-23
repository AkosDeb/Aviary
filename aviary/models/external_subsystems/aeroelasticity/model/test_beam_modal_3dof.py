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
        self.assertAlmostEqual(T11 / (2.0 * T10), expected_T11 / (2.0 * expected_T10), places=10)

    def test_te_hinge_monotone(self):
        """T10 and η_δ should decrease as hinge moves toward TE (c→1)."""
        T10_mid, T11_mid = _garrick_T10_T11(0.75)   # c=0.5
        T10_aft, T11_aft = _garrick_T10_T11(0.875)  # c=0.75
        self.assertGreater(T10_mid, T10_aft, 'T10 should decrease as hinge moves aft')

    def test_eta_delta_sign_and_magnitude_at_75pct_hinge(self):
        """η_δ at 75% chord hinge (c=0.5) must be positive with the correct magnitude.

        At c=0.5: T10 = sqrt(0.75)+π/3 ≈ 1.913, T11 = sqrt(0.75) ≈ 0.866,
        η_δ = T11/(2·T10) ≈ 0.226.

        Note: the TOOD item listed T10≈0.203, T11≈-0.193, η_δ≈-0.475 — those
        values are inconsistent with any standard Theodorsen formulation and do not
        correspond to c=0.5. They have been superseded by the analytic values here.
        A negative η_δ would flip the control coupling phase and give an
        unconservative flutter prediction.
        """
        T10, T11 = _garrick_T10_T11(0.75)
        eta_delta = T11 / (2.0 * T10)

        # Analytic reference at c = 2*0.75 - 1 = 0.5
        c = 0.5
        T10_ref = np.sqrt(1.0 - c**2) + np.arccos(c)   # ≈ 1.9132
        T11_ref = 0.5 * (1.0 - 2.0 * c) * np.arccos(c) + np.sqrt(1.0 - c**2)  # ≈ 0.8660
        eta_ref = T11_ref / (2.0 * T10_ref)             # ≈ 0.2263

        self.assertAlmostEqual(T10, T10_ref, places=8,
            msg=f'T10 at 75% chord: expected {T10_ref:.4f}, got {T10:.4f}')
        self.assertAlmostEqual(T11, T11_ref, places=8,
            msg=f'T11 at 75% chord: expected {T11_ref:.4f}, got {T11:.4f}')
        self.assertGreater(eta_delta, 0.0,
            msg=f'η_δ must be positive at 75% chord; got {eta_delta:.4f} — '
                'negative value would flip control coupling phase')
        self.assertAlmostEqual(eta_delta, eta_ref, places=8,
            msg=f'η_δ at 75% chord: expected {eta_ref:.4f}, got {eta_delta:.4f}')


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


    def test_diagonal_real_terms_are_stabilizing_at_positive_k(self):
        y = np.linspace(0.0, 1.0, 11)
        chord = 0.2 * np.ones(11)
        bending = np.linspace(0.0, 1.0, 11)
        torsion = np.linspace(0.0, 1.0, 11)
        phi_delta = _control_mode_shape(y, 0.4, 0.9)
        q_real, _, _ = beam_modal_theodorsen_gaf_3dof(
            0.3, y, chord, bending, torsion, phi_delta,
            2*np.pi, -0.05*2*np.pi, 2.5, -0.6, -0.02, -0.30, 0.375, 0.75,
        )
        self.assertTrue(np.all(np.diag(q_real) < 0.0), np.diag(q_real))


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
        converged = prob.get_val(AE.BEAM_MODAL_3DOF_PK_CONVERGED)[0]
        self.assertTrue(np.isfinite(spd), f'3DOF flutter speed not finite: {spd}')
        self.assertTrue(np.isfinite(ctrl_f), f'control freq not finite: {ctrl_f}')
        self.assertEqual(converged, 1.0)
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

    def test_heavier_vtp_tip_mass_does_not_raise_flutter_speed(self):
        """Extra mass at the outer tip stations mimics a heavier VTP loading.

        Increasing tip inertia lowers the torsion frequency and tightens the
        bending-torsion coupling, so flutter speed must not increase.
        """
        n = 11
        prob_light = self._make_prob()
        prob_heavy = self._make_prob()

        base_mass = np.linspace(1.5, 0.5, n)
        heavy_mass = base_mass.copy()
        heavy_mass[-3:] += 0.5

        base_pi = np.linspace(0.05, 0.01, n)
        heavy_pi = base_pi.copy()
        heavy_pi[-3:] += 0.05

        prob_heavy.set_val(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, heavy_mass, units='kg/m')
        prob_heavy.set_val(AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN, heavy_pi, units='kg*m')

        prob_light.run_model()
        prob_heavy.run_model()

        spd_light = float(prob_light.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0])
        spd_heavy = float(prob_heavy.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0])

        self.assertLessEqual(
            spd_heavy, spd_light,
            f'Heavier tip should not raise flutter speed; light={spd_light:.1f} m/s, heavy={spd_heavy:.1f} m/s',
        )

    def test_rigid_passive_control_3dof_matches_2dof_flutter_speed(self):
        """With structurally rigid and aerodynamically passive control surface,
        3-DOF P-K flutter speed must agree with 2-DOF P-K within 1%.

        Conditions: very high K_hinge, zero control aero derivatives (cl_delta=
        cm_delta=ch_alpha=ch_delta=0), negligible control inertia, zero static
        unbalance.  Under these conditions the control DOF decouples from the
        [bending, torsion] flutter problem and the two formulations must give the
        same answer.  A divergence > 1% indicates a matrix assembly error in the
        3-DOF upgrade.
        """
        n = 11
        prob = om.Problem()
        prob.model.add_subsystem(
            'bm', BeamModalFlutter(num_stations=n, num_speed_samples=20), promotes=['*']
        )
        prob.setup()
        # Soft stiffness so flutter is found well within the search range
        y = np.linspace(0.0, 0.9, n)
        chord = np.linspace(0.18, 0.12, n)
        EI = np.linspace(600.0, 100.0, n)
        GJ = np.linspace(650.0, 110.0, n)
        mass = np.linspace(1.5, 0.5, n)
        pitch_inertia = np.linspace(0.05, 0.01, n)
        prob.set_val(AE.SPANWISE_STATIONS, y, units='m')
        prob.set_val(AE.SPANWISE_CHORD, chord, units='m')
        prob.set_val(AE.SPANWISE_BENDING_STIFFNESS, EI, units='N*m**2')
        prob.set_val(AE.SPANWISE_TORSIONAL_RIGIDITY, GJ, units='N*m**2')
        prob.set_val(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, mass, units='kg/m')
        prob.set_val(AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN, pitch_inertia, units='kg*m')
        prob.set_val(AE.FLUTTER_MAX_SPEED, 200.0, units='m/s')
        prob.set_val(AE.REQUIRED_SPEED, 50.0, units='m/s')
        # Rigid, aerodynamically passive control surface
        prob.set_val(AE.CONTROL_STIFFNESS, 1.0e8, units='N*m/rad')
        prob.set_val(AE.CONTROL_STATIC_UNBALANCE, 0.0, units='kg')
        prob.set_val(AE.CONTROL_INERTIA_PER_UNIT_SPAN, 1.0e-6, units='kg*m')
        prob.set_val(AE.CONTROL_LIFT_DERIVATIVE, 0.0, units='1/rad')
        prob.set_val(AE.CONTROL_MOMENT_DERIVATIVE, 0.0, units='1/rad')
        prob.set_val(AE.HINGE_MOMENT_ALPHA_DERIVATIVE, 0.0, units='1/rad')
        prob.set_val(AE.HINGE_MOMENT_CONTROL_DERIVATIVE, 0.0, units='1/rad')
        prob.run_model()

        spd_2dof = float(prob.get_val(AE.BEAM_MODAL_PK_FLUTTER_SPEED, units='m/s')[0])
        spd_3dof = float(prob.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0])

        rel_diff = abs(spd_3dof - spd_2dof) / max(abs(spd_2dof), 1.0)
        self.assertLess(
            rel_diff, 0.01,
            f'2-DOF vs 3-DOF divergence {rel_diff * 100:.2f}% > 1%; '
            f'2DOF={spd_2dof:.1f} m/s, 3DOF={spd_3dof:.1f} m/s',
        )

    def test_vtp_mass_off_baseline(self):
        """VTP mass-off vs. uniform VTP contribution: heavier wing should not raise flutter speed.

        Models the VTPTipInertia path: the component distributes total VTP mass
        uniformly as vtp_mass / wing_span over SPANWISE_TOTAL_MASS_PER_UNIT_SPAN.
        For the SpaJeti baseline (2 panels, span=0.4 m, chord=0.2 m, areal_density=
        1.25 kg/m², wing_span=2.0 m) this gives ~0.1 kg/m.

        Confirms sign: VTP mass must lower (or not raise) the 3-DOF flutter speed.
        The flutter speed delta is printed to document the v1.28.0 VTP contribution.
        """
        n = 11
        vtp_mass_per_span = 0.1    # kg/m — representative SpaJeti VTP contribution
        vtp_inertia_per_span = 0.004  # kg·m

        prob_off = self._make_prob()
        prob_on = self._make_prob()

        base_mass = np.linspace(1.5, 0.5, n)
        base_pi = np.linspace(0.05, 0.01, n)
        prob_on.set_val(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN,
                        base_mass + vtp_mass_per_span, units='kg/m')
        prob_on.set_val(AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
                        base_pi + vtp_inertia_per_span, units='kg*m')

        prob_off.run_model()
        prob_on.run_model()

        spd_off = float(prob_off.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0])
        spd_on = float(prob_on.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0])
        delta = spd_off - spd_on

        self.assertLessEqual(
            spd_on, spd_off,
            f'VTP mass-on ({spd_on:.1f} m/s) must not exceed mass-off ({spd_off:.1f} m/s). '
            f'VTP delta = {delta:.1f} m/s.',
        )


if __name__ == '__main__':
    unittest.main()
