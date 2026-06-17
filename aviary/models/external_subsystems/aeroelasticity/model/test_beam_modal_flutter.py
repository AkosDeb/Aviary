import unittest

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.beam_modal_flutter import (
    BeamModalFlutter,
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


if __name__ == '__main__':
    unittest.main()
