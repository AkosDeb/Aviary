import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.strength_margins import (
    StrengthMargins,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestStrengthMargins(unittest.TestCase):
    def test_root_stress_and_margins(self):
        prob = om.Problem(name='test_strength_margins', reports=False)
        prob.model.add_subsystem('strength', StrengthMargins())
        prob.setup()

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 100.0, units='N*m')
        prob.set_val(f'strength.{AE.ROOT_TORQUE}', 20.0, units='N*m')
        prob.set_val(f'strength.{AE.BENDING_INERTIA}', 2.0e-6, units='m**4')
        prob.set_val(f'strength.{AE.WINGBOX_HEIGHT}', 0.10, units='m')
        prob.set_val(f'strength.{AE.WINGBOX_AREA}', 0.02, units='m**2')
        prob.set_val(f'strength.{AE.SKIN_THICKNESS}', 0.002, units='m')
        prob.set_val(f'strength.{AE.SPAR_THICKNESS}', 0.001, units='m')
        prob.set_val(f'strength.{AE.TENSILE_ALLOWABLE}', 150.0e6, units='Pa')
        prob.set_val(f'strength.{AE.COMPRESSIVE_ALLOWABLE}', 120.0e6, units='Pa')
        prob.set_val(f'strength.{AE.SHEAR_ALLOWABLE}', 60.0e6, units='Pa')
        prob.set_val(f'strength.{AE.STRENGTH_SAFETY_FACTOR}', 1.5)
        prob.run_model()

        bending_stress = 100.0 * 0.05 / 2.0e-6
        shear_flow = 20.0 / (2.0 * 0.02)
        torsion_stress = shear_flow / 0.001
        bending_margin = 120.0e6 / (1.5 * bending_stress) - 1.0
        torsion_margin = 60.0e6 / (1.5 * torsion_stress) - 1.0

        assert_near_equal(
            prob.get_val(f'strength.{AE.ROOT_BENDING_STRESS}', units='Pa'),
            bending_stress,
        )
        assert_near_equal(
            prob.get_val(f'strength.{AE.TORSION_SHEAR_STRESS}', units='Pa'),
            torsion_stress,
        )
        assert_near_equal(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}'), bending_margin)
        assert_near_equal(prob.get_val(f'strength.{AE.TORSION_STRENGTH_MARGIN}'), torsion_margin)
        assert_near_equal(prob.get_val(f'strength.{AE.MIN_STRENGTH_MARGIN}'), bending_margin)

    def test_margin_decreases_with_higher_load(self):
        prob = om.Problem(name='test_strength_margin_load_sensitivity', reports=False)
        prob.model.add_subsystem('strength', StrengthMargins())
        prob.setup()

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 50.0, units='N*m')
        prob.run_model()
        low_load_margin = float(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}')[0])

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 100.0, units='N*m')
        prob.run_model()
        high_load_margin = float(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}')[0])

        self.assertLess(high_load_margin, low_load_margin)


if __name__ == '__main__':
    unittest.main()
