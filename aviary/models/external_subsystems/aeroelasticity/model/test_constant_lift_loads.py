import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.constant_lift_loads import (
    ConstantLiftLoads,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestConstantLiftLoads(unittest.TestCase):
    def test_uniform_half_wing_loads(self):
        prob = om.Problem(name='test_constant_lift_loads', reports=False)
        prob.model.add_subsystem('loads', ConstantLiftLoads())
        prob.setup(force_alloc_complex=True)

        prob.set_val(f'loads.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'loads.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'loads.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'loads.{AE.AIR_DENSITY}', 1.225, units='kg/m**3')
        prob.set_val(f'loads.{AE.DESIGN_SPEED}', 100.0, units='m/s')
        prob.set_val(f'loads.{AE.LOAD_FACTOR}', 1.15)
        prob.set_val(f'loads.{AE.LOAD_LIFT_COEFFICIENT}', 0.8)
        prob.set_val(f'loads.{AE.AERO_CENTER_TO_EA_FRACTION}', 0.125)
        prob.set_val(f'loads.{AE.BENDING_STIFFNESS}', 2.0e5, units='N*m**2')
        prob.set_val(f'loads.{AE.TORSIONAL_RIGIDITY}', 1.0e4, units='N*m**2')
        prob.run_model()

        total_lift = 1.15 * 0.5 * 1.225 * 100.0**2 * 0.8 * 0.8
        lift_per_span = total_lift / 2.0
        chord = 0.4
        semispan = 1.0
        root_bending = lift_per_span * semispan**2 / 2.0
        root_torque = lift_per_span * semispan * 0.125 * chord
        tip_deflection = lift_per_span * semispan**4 / (8.0 * 2.0e5)
        tip_twist = root_torque * semispan / 1.0e4

        assert_near_equal(
            prob.get_val(f'loads.{AE.CONSTANT_TOTAL_LIFT}', units='N'),
            total_lift,
        )
        assert_near_equal(
            prob.get_val(f'loads.{AE.CONSTANT_LIFT_PER_UNIT_SPAN}', units='N/m'),
            lift_per_span,
        )
        assert_near_equal(
            prob.get_val(f'loads.{AE.ROOT_BENDING_MOMENT}', units='N*m'),
            root_bending,
        )
        assert_near_equal(prob.get_val(f'loads.{AE.ROOT_TORQUE}', units='N*m'), root_torque)
        assert_near_equal(prob.get_val(f'loads.{AE.TIP_DEFLECTION}', units='m'), tip_deflection)
        assert_near_equal(prob.get_val(f'loads.{AE.TIP_TWIST}', units='rad'), tip_twist)


if __name__ == '__main__':
    unittest.main()
