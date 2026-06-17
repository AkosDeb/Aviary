import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.spanwise_beam_response import (
    SpanwiseBeamResponse,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestSpanwiseBeamResponse(unittest.TestCase):
    def test_constant_moment_and_torque_match_closed_form(self):
        prob = om.Problem(name='test_spanwise_beam_response', reports=False)
        prob.model.add_subsystem('beam', SpanwiseBeamResponse(num_stations=11))
        prob.setup()

        length = 2.0
        moment = 120.0
        torque = 30.0
        EI = 600.0
        GJ = 150.0
        y = np.linspace(0.0, length, 11)

        prob.set_val(f'beam.{AE.SPANWISE_STATIONS}', y, units='m')
        prob.set_val(f'beam.{AE.SCHRENK_BENDING_MOMENT}', moment * np.ones_like(y), units='N*m')
        prob.set_val(f'beam.{AE.SCHRENK_TORQUE}', torque * np.ones_like(y), units='N*m')
        prob.set_val(f'beam.{AE.SPANWISE_BENDING_STIFFNESS}', EI * np.ones_like(y), units='N*m**2')
        prob.set_val(f'beam.{AE.SPANWISE_TORSIONAL_RIGIDITY}', GJ * np.ones_like(y), units='N*m**2')
        prob.run_model()

        expected_slope_tip = (moment / EI) * length
        expected_deflection_tip = 0.5 * (moment / EI) * length**2
        expected_twist_tip = (torque / GJ) * length

        assert_near_equal(
            prob.get_val(f'beam.{AE.SPANWISE_SLOPE}', units='rad')[-1],
            expected_slope_tip,
        )
        assert_near_equal(
            prob.get_val(f'beam.{AE.SCHRENK_TIP_DEFLECTION}', units='m'),
            expected_deflection_tip,
        )
        assert_near_equal(
            prob.get_val(f'beam.{AE.SCHRENK_TIP_TWIST}', units='rad'),
            expected_twist_tip,
        )

    def test_deflection_and_twist_start_from_clamped_root(self):
        prob = om.Problem(name='test_spanwise_beam_response_root_bc', reports=False)
        prob.model.add_subsystem('beam', SpanwiseBeamResponse(num_stations=5))
        prob.setup()
        prob.run_model()

        assert_near_equal(prob.get_val(f'beam.{AE.SPANWISE_SLOPE}', units='rad')[0], 0.0)
        assert_near_equal(prob.get_val(f'beam.{AE.SPANWISE_DEFLECTION}', units='m')[0], 0.0)
        assert_near_equal(prob.get_val(f'beam.{AE.SPANWISE_TWIST}', units='rad')[0], 0.0)


if __name__ == '__main__':
    unittest.main()
