import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.schrenk_lift_distribution import (
    SchrenkLiftDistribution,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestSchrenkLiftDistribution(unittest.TestCase):
    def test_schrenk_load_conservation_and_tip_boundary(self):
        prob = om.Problem(name='test_schrenk_lift_distribution', reports=False)
        prob.model.add_subsystem('loads', SchrenkLiftDistribution(num_stations=41))
        prob.setup()

        prob.set_val(f'loads.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'loads.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'loads.{Aircraft.Wing.TAPER_RATIO}', 0.5)
        prob.set_val(f'loads.{AE.AIR_DENSITY}', 1.225, units='kg/m**3')
        prob.set_val(f'loads.{AE.DESIGN_SPEED}', 100.0, units='m/s')
        prob.set_val(f'loads.{AE.LOAD_FACTOR}', 1.15)
        prob.set_val(f'loads.{AE.LOAD_LIFT_COEFFICIENT}', 0.8)
        prob.set_val(f'loads.{AE.AERO_CENTER_TO_EA_FRACTION}', 0.125)
        prob.set_val(f'loads.{AE.BENDING_STIFFNESS}', 2.0e5, units='N*m**2')
        prob.set_val(f'loads.{AE.TORSIONAL_RIGIDITY}', 1.0e4, units='N*m**2')
        prob.run_model()

        y = prob.get_val(f'loads.{AE.SPANWISE_STATIONS}', units='m')
        lift_per_span = prob.get_val(
            f'loads.{AE.SCHRENK_LIFT_PER_UNIT_SPAN}', units='N/m'
        )
        shear = prob.get_val(f'loads.{AE.SCHRENK_SHEAR_FORCE}', units='N')
        bending = prob.get_val(f'loads.{AE.SCHRENK_BENDING_MOMENT}', units='N*m')
        torque = prob.get_val(f'loads.{AE.SCHRENK_TORQUE}', units='N*m')

        total_lift = 1.15 * 0.5 * 1.225 * 100.0**2 * 0.8 * 0.8
        half_lift_from_distribution = np.trapezoid(lift_per_span, y)

        assert_near_equal(
            prob.get_val(f'loads.{AE.CONSTANT_TOTAL_LIFT}', units='N'),
            total_lift,
        )
        assert_near_equal(half_lift_from_distribution, 0.5 * total_lift, tolerance=1e-11)
        assert_near_equal(shear[0], 0.5 * total_lift, tolerance=1e-11)
        assert_near_equal(shear[-1], 0.0)
        assert_near_equal(bending[-1], 0.0)
        assert_near_equal(torque[-1], 0.0)

        constant_root_bending = (0.5 * total_lift / 1.0) * 1.0**2 / 2.0
        self.assertGreater(prob.get_val(f'loads.{AE.ROOT_BENDING_MOMENT}', units='N*m')[0], 0.0)
        self.assertLess(
            prob.get_val(f'loads.{AE.ROOT_BENDING_MOMENT}', units='N*m')[0],
            constant_root_bending,
        )


if __name__ == '__main__':
    unittest.main()
