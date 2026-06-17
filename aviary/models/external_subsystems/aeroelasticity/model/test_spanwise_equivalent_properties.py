import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.spanwise_equivalent_properties import (
    SpanwiseEquivalentProperties,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestSpanwiseEquivalentProperties(unittest.TestCase):
    def test_uniform_properties_reduce_to_same_scalar_values(self):
        prob = om.Problem(name='test_spanwise_equivalent_uniform', reports=False)
        prob.model.add_subsystem('equiv', SpanwiseEquivalentProperties(num_stations=51))
        prob.setup()

        y = np.linspace(0.0, 2.0, 51)
        prob.set_val(f'equiv.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'equiv.{AE.SPANWISE_BENDING_STIFFNESS}', 1000.0 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'equiv.{AE.SPANWISE_TORSIONAL_RIGIDITY}', 400.0 * np.ones_like(y), units='N*m**2')
        prob.set_val(f'equiv.{AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN}', 2.5 * np.ones_like(y), units='kg/m')
        prob.set_val(
            f'equiv.{AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN}',
            0.12 * np.ones_like(y),
            units='kg*m',
        )
        prob.run_model()

        assert_near_equal(
            prob.get_val(f'equiv.{AE.BENDING_STIFFNESS}', units='N*m**2'),
            1000.0,
            tolerance=3.0e-4,
        )
        assert_near_equal(prob.get_val(f'equiv.{AE.TORSIONAL_RIGIDITY}', units='N*m**2'), 400.0)
        assert_near_equal(prob.get_val(f'equiv.{AE.MASS_PER_UNIT_SPAN}', units='kg/m'), 2.5)
        assert_near_equal(prob.get_val(f'equiv.{AE.PITCH_INERTIA_PER_UNIT_SPAN}', units='kg*m'), 0.12)
        assert_near_equal(prob.get_val(f'equiv.{AE.TORSIONAL_STIFFNESS}', units='N*m/rad'), 200.0)
        assert_near_equal(
            prob.get_val(f'equiv.{AE.BENDING_STIFFNESS_PLUNGE}', units='N/m'),
            375.0,
            tolerance=3.0e-4,
        )

    def test_soft_tip_reduces_equivalent_bending_stiffness(self):
        prob = om.Problem(name='test_spanwise_equivalent_soft_tip', reports=False)
        prob.model.add_subsystem('equiv', SpanwiseEquivalentProperties(num_stations=51))
        prob.setup()

        y = np.linspace(0.0, 1.0, 51)
        EI = np.ones_like(y) * 1000.0
        EI[y > 0.7] = 100.0
        prob.set_val(f'equiv.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'equiv.{AE.SPANWISE_BENDING_STIFFNESS}', EI, units='N*m**2')
        prob.run_model()

        self.assertLess(prob.get_val(f'equiv.{AE.BENDING_STIFFNESS}', units='N*m**2')[0], 1000.0)


if __name__ == '__main__':
    unittest.main()
