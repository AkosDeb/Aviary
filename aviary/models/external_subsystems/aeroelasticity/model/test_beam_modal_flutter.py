import unittest

import numpy as np
import openmdao.api as om
from aviary.variable_info.variables import Aircraft

from aviary.models.external_subsystems.aeroelasticity.model.beam_modal_flutter import (
    BeamModalFlutter,
)
from aviary.models.external_subsystems.aeroelasticity.model.spanwise_wingbox import (
    SpanwiseWingboxProperties,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestBeamModalFlutter(unittest.TestCase):
    def test_modal_frequencies_are_positive(self):
        prob = om.Problem(name='test_beam_modal_flutter_freq', reports=False)
        prob.model.add_subsystem('beam', BeamModalFlutter(num_stations=31))
        prob.setup()

        y = np.linspace(0.0, 1.5, 31)
        prob.set_val(f'beam.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'beam.{AE.SPANWISE_CHORD}', 0.3 * np.ones_like(y), units='m')
        prob.set_val(f'beam.{AE.SPANWISE_BENDING_STIFFNESS}', 2.0e4 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TORSIONAL_RIGIDITY}', 1.0e4 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN}', 1.5 * np.ones_like(y), units='kg/m')
        prob.set_val(
            f'beam.{AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN}',
            0.02 * np.ones_like(y),
            units='kg*m',
        )
        prob.run_model()

        self.assertGreater(prob.get_val(f'beam.{AE.BEAM_MODAL_BENDING_FREQUENCY}', units='Hz')[0], 0.0)
        self.assertGreater(prob.get_val(f'beam.{AE.BEAM_MODAL_TORSION_FREQUENCY}', units='Hz')[0], 0.0)
        self.assertGreater(prob.get_val(f'beam.{AE.BEAM_MODAL_FLUTTER_SPEED}', units='m/s')[0], 0.0)
        self.assertGreater(prob.get_val(f'beam.{AE.BEAM_MODAL_PK_FLUTTER_SPEED}', units='m/s')[0], 0.0)
        self.assertGreaterEqual(prob.get_val(f'beam.{AE.BEAM_MODAL_PK_CONVERGED}')[0], 0.0)

    def test_stiffer_beam_increases_dry_frequencies(self):
        prob = om.Problem(name='test_beam_modal_flutter_stiffness', reports=False)
        prob.model.add_subsystem('beam', BeamModalFlutter(num_stations=21))
        prob.setup()

        y = np.linspace(0.0, 1.0, 21)
        prob.set_val(f'beam.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'beam.{AE.SPANWISE_BENDING_STIFFNESS}', 1.0e3 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TORSIONAL_RIGIDITY}', 1.0e3 * np.ones_like(y), units='N*m**2')
        prob.run_model()
        f_b_low = prob.get_val(f'beam.{AE.BEAM_MODAL_BENDING_FREQUENCY}', units='Hz').copy()
        f_t_low = prob.get_val(f'beam.{AE.BEAM_MODAL_TORSION_FREQUENCY}', units='Hz').copy()

        prob.set_val(f'beam.{AE.SPANWISE_BENDING_STIFFNESS}', 4.0e3 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TORSIONAL_RIGIDITY}', 4.0e3 * np.ones_like(y), units='N*m**2')
        prob.run_model()
        f_b_high = prob.get_val(f'beam.{AE.BEAM_MODAL_BENDING_FREQUENCY}', units='Hz')
        f_t_high = prob.get_val(f'beam.{AE.BEAM_MODAL_TORSION_FREQUENCY}', units='Hz')

        self.assertGreater(f_b_high[0], f_b_low[0])
        self.assertGreater(f_t_high[0], f_t_low[0])

    def test_pk_modal_outputs_have_two_modes(self):
        prob = om.Problem(name='test_beam_modal_flutter_pk_outputs', reports=False)
        prob.model.add_subsystem('beam', BeamModalFlutter(num_stations=21, num_speed_samples=12))
        prob.setup()

        y = np.linspace(0.0, 1.2, 21)
        prob.set_val(f'beam.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'beam.{AE.SPANWISE_CHORD}', 0.25 * np.ones_like(y), units='m')
        prob.set_val(f'beam.{AE.SPANWISE_BENDING_STIFFNESS}', 1.5e4 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TORSIONAL_RIGIDITY}', 8.0e3 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN}', 1.2 * np.ones_like(y), units='kg/m')
        prob.set_val(
            f'beam.{AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN}',
            0.015 * np.ones_like(y),
            units='kg*m',
        )
        prob.run_model()

        damping = prob.get_val(f'beam.{AE.BEAM_MODAL_PK_MODE_DAMPING}')
        frequency = prob.get_val(f'beam.{AE.BEAM_MODAL_PK_MODE_FREQUENCY}', units='Hz')

        self.assertEqual(damping.shape, (2,))
        self.assertEqual(frequency.shape, (2,))
        self.assertTrue(np.all(np.isfinite(damping)))
        self.assertTrue(np.all(frequency > 0.0))


class TestTCSensitivitySweep(unittest.TestCase):
    def test_flutter_speed_and_bending_frequency_increase_monotonically_with_tc(self):
        """t/c sensitivity sweep from 0.05 to 0.18 in steps of 0.01.

        SpanwiseWingboxProperties computes EI and GJ at each thickness ratio;
        BeamModalFlutter records dry bending frequency and 2-DOF P-K flutter speed.

        Both quantities must increase monotonically with t/c — thicker wing is
        stiffer, so flutter margin grows. Any non-monotonic step in the 2-DOF
        result indicates an error in the GJ/EI approximation.

        Note: BEAM_MODAL_3DOF_PK_FLUTTER_SPEED is NOT asserted monotone because
        the 3-DOF control-surface coupling can become dominant at mid-range t/c
        (pitch inertia scaling shifts the control mode into the torsion band),
        which is expected physical behaviour rather than a model error.
        """
        n = 11
        tc_values = np.round(np.arange(0.05, 0.185, 0.01), 3)  # 0.05 … 0.18

        box_prob = om.Problem(name='tc_sweep_box', reports=False)
        box_prob.model.add_subsystem('box', SpanwiseWingboxProperties(num_stations=n))
        box_prob.setup()
        box_prob.set_val(f'box.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        box_prob.set_val(f'box.{Aircraft.Wing.SPAN}', 2.0, units='m')
        box_prob.set_val(f'box.{Aircraft.Wing.TAPER_RATIO}', 1.0)

        flutter_prob = om.Problem(name='tc_sweep_flutter', reports=False)
        flutter_prob.model.add_subsystem(
            'bm', BeamModalFlutter(num_stations=n, num_speed_samples=10), promotes=['*']
        )
        flutter_prob.setup()
        flutter_prob.set_val(AE.FLUTTER_MAX_SPEED, 350.0, units='m/s')
        flutter_prob.set_val(AE.REQUIRED_SPEED, 100.0, units='m/s')

        bending_freqs = []
        pk2_flutter_speeds = []

        for tc in tc_values:
            box_prob.set_val(f'box.{AE.STRUCTURAL_THICKNESS_TO_CHORD}', tc)
            box_prob.run_model()

            y = box_prob.get_val(f'box.{AE.SPANWISE_STATIONS}', units='m')
            chord = box_prob.get_val(f'box.{AE.SPANWISE_CHORD}', units='m')
            EI = box_prob.get_val(f'box.{AE.SPANWISE_BENDING_STIFFNESS}', units='N*m**2')
            GJ = box_prob.get_val(f'box.{AE.SPANWISE_TORSIONAL_RIGIDITY}', units='N*m**2')
            mass = box_prob.get_val(f'box.{AE.SPANWISE_MASS_PER_UNIT_SPAN}', units='kg/m')
            inertia = box_prob.get_val(
                f'box.{AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN}', units='kg*m'
            )

            flutter_prob.set_val(AE.SPANWISE_STATIONS, y, units='m')
            flutter_prob.set_val(AE.SPANWISE_CHORD, chord, units='m')
            flutter_prob.set_val(AE.SPANWISE_BENDING_STIFFNESS, EI, units='N*m**2')
            flutter_prob.set_val(AE.SPANWISE_TORSIONAL_RIGIDITY, GJ, units='N*m**2')
            flutter_prob.set_val(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, mass, units='kg/m')
            flutter_prob.set_val(
                AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN, inertia, units='kg*m'
            )
            flutter_prob.run_model()

            bending_freqs.append(
                float(flutter_prob.get_val(AE.BEAM_MODAL_BENDING_FREQUENCY, units='Hz')[0])
            )
            pk2_flutter_speeds.append(
                float(flutter_prob.get_val(AE.BEAM_MODAL_PK_FLUTTER_SPEED, units='m/s')[0])
            )

        bending_freqs = np.array(bending_freqs)
        pk2_flutter_speeds = np.array(pk2_flutter_speeds)

        for i in range(len(tc_values) - 1):
            self.assertLessEqual(
                bending_freqs[i], bending_freqs[i + 1],
                f'Bending frequency not monotone at t/c '
                f'{tc_values[i]:.2f}→{tc_values[i+1]:.2f}: '
                f'{bending_freqs[i]:.2f}→{bending_freqs[i+1]:.2f} Hz',
            )
            self.assertLessEqual(
                pk2_flutter_speeds[i], pk2_flutter_speeds[i + 1],
                f'2-DOF P-K flutter speed not monotone at t/c '
                f'{tc_values[i]:.2f}→{tc_values[i+1]:.2f}: '
                f'{pk2_flutter_speeds[i]:.1f}→{pk2_flutter_speeds[i+1]:.1f} m/s',
            )


if __name__ == '__main__':
    unittest.main()
