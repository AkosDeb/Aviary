import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.spanwise_mass_distribution import (
    SpanwiseMassDistribution,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestSpanwiseMassDistribution(unittest.TestCase):
    def test_added_mass_is_conserved(self):
        prob = om.Problem(name='test_spanwise_mass_distribution', reports=False)
        prob.model.add_subsystem('mass', SpanwiseMassDistribution(num_stations=21))
        prob.setup()

        y = np.linspace(0.0, 1.0, 21)
        structure_mass = 0.4 * np.ones_like(y)
        prob.set_val(f'mass.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'mass.{AE.SPANWISE_CHORD}', 0.3 * np.ones_like(y), units='m')
        prob.set_val(
            f'mass.{AE.SPANWISE_MASS_PER_UNIT_SPAN}',
            structure_mass,
            units='kg/m',
        )
        prob.set_val(f'mass.{AE.VTP_TIP_MASS}', 0.10, units='kg')
        prob.set_val(f'mass.{AE.ENGINE_MASS}', 0.20, units='kg')
        prob.set_val(f'mass.{AE.FUEL_MASS}', 0.30, units='kg')
        prob.set_val(f'mass.{AE.FUEL_SPAN_FRACTION}', 0.5)
        prob.set_val(f'mass.{AE.SERVO_MASS}', 0.05, units='kg')
        prob.set_val(f'mass.{AE.ELEVON_MASS}', 0.15, units='kg')
        prob.set_val(f'mass.{AE.ELEVON_SPAN_START_FRACTION}', 0.5)
        prob.set_val(f'mass.{AE.ELEVON_SPAN_END_FRACTION}', 1.0)
        prob.run_model()

        total = prob.get_val(f'mass.{AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN}', units='kg/m')
        expected = (
            np.trapezoid(structure_mass, y)
            + 0.10
            + 0.20
            + 0.30
            + 0.05
            + 0.15
        )

        assert_near_equal(np.trapezoid(total, y), expected, tolerance=2.0e-2)
        assert_near_equal(
            prob.get_val(f'mass.{AE.SPANWISE_TOTAL_HALF_WING_MASS}', units='kg'),
            expected,
            tolerance=2.0e-2,
        )

    def test_elevon_aft_of_elastic_axis_increases_pitch_inertia(self):
        prob = om.Problem(name='test_spanwise_mass_distribution_inertia', reports=False)
        prob.model.add_subsystem('mass', SpanwiseMassDistribution(num_stations=9))
        prob.setup()

        y = np.linspace(0.0, 1.0, 9)
        prob.set_val(f'mass.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'mass.{AE.SPANWISE_CHORD}', 0.5 * np.ones_like(y), units='m')
        prob.set_val(f'mass.{AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN}', np.zeros_like(y), units='kg*m')
        prob.set_val(f'mass.{AE.ELEVON_MASS}', 0.2, units='kg')
        prob.set_val(f'mass.{AE.ELEVON_CG_FRACTION}', 0.80)
        prob.run_model()

        inertia = prob.get_val(
            f'mass.{AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN}',
            units='kg*m',
        )
        self.assertGreater(np.max(inertia), 0.0)


if __name__ == '__main__':
    unittest.main()
