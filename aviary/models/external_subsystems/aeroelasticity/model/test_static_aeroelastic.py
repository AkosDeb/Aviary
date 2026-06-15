import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.static_aeroelastic import (
    StaticAeroelastic,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestStaticAeroelastic(unittest.TestCase):
    def test_divergence_reversal_and_effectiveness(self):
        prob = om.Problem(name='test_static_aeroelastic', reports=False)
        prob.model.add_subsystem('static', StaticAeroelastic())
        prob.setup(force_alloc_complex=True)

        prob.set_val(f'static.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'static.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'static.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'static.{AE.TORSIONAL_STIFFNESS}', 5000.0, units='N*m/rad')
        prob.set_val(f'static.{AE.AERO_CENTER_TO_EA_FRACTION}', 0.125)
        prob.set_val(f'static.{AE.LIFT_CURVE_SLOPE}', 2.0 * np.pi, units='unitless')
        prob.set_val(f'static.{AE.CONTROL_LIFT_DERIVATIVE}', 2.5, units='1/rad')
        prob.set_val(f'static.{AE.CONTROL_MOMENT_DERIVATIVE}', -0.60, units='1/rad')
        prob.set_val(f'static.{AE.DESIGN_SPEED}', 80.0, units='m/s')
        prob.set_val(f'static.{AE.REQUIRED_SPEED}', 1.15 * 550.0 / 3.6, units='m/s')
        prob.run_model()

        chord = 0.4
        semispan = 0.5 * 2.0
        q_div_expected = 5000.0 / (0.125 * chord**2 * 2.0 * np.pi * semispan)
        q_rev_expected = -5000.0 * 2.5 / (chord**2 * -0.60 * 2.0 * np.pi * semispan)
        q_design = 0.5 * 1.225 * 80.0**2
        effectiveness_expected = (
            (1.0 - q_design / q_rev_expected) / (1.0 - q_design / q_div_expected)
        )

        assert_near_equal(
            prob.get_val(f'static.{AE.DIVERGENCE_DYNAMIC_PRESSURE}', units='Pa'),
            q_div_expected,
        )
        assert_near_equal(
            prob.get_val(f'static.{AE.REVERSAL_DYNAMIC_PRESSURE}', units='Pa'),
            q_rev_expected,
        )
        assert_near_equal(
            prob.get_val(f'static.{AE.CONTROL_EFFECTIVENESS}'),
            effectiveness_expected,
        )
        assert_near_equal(
            prob.get_val(f'static.{AE.DIVERGENCE_SPEED_MARGIN}', units='m/s'),
            (2.0 * q_div_expected / 1.225) ** 0.5 - 1.15 * 550.0 / 3.6,
        )
        assert_near_equal(
            prob.get_val(f'static.{AE.REVERSAL_SPEED_MARGIN}', units='m/s'),
            (2.0 * q_rev_expected / 1.225) ** 0.5 - 1.15 * 550.0 / 3.6,
        )


if __name__ == '__main__':
    unittest.main()
