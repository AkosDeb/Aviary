import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.static_aeroelastic import (
    StaticAeroelastic,
)
from aviary.models.external_subsystems.aeroelasticity.model.structural_box import (
    WingboxStructuralEstimate,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestWingboxStructuralEstimate(unittest.TestCase):
    def test_two_spar_closed_box_defaults(self):
        prob = om.Problem()
        prob.model.add_subsystem('box', WingboxStructuralEstimate())
        prob.setup(force_alloc_complex=True)

        prob.set_val(f'box.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'box.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'box.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'box.{Aircraft.Wing.THICKNESS_TO_CHORD}', 0.12)
        prob.run_model()

        assert_near_equal(prob.get_val(f'box.{AE.ELASTIC_AXIS_FRACTION}'), 0.375)
        assert_near_equal(prob.get_val(f'box.{AE.CONTROL_HINGE_FRACTION}'), 0.75)
        assert_near_equal(prob.get_val(f'box.{AE.AERO_CENTER_TO_EA_FRACTION}'), 0.125)

        self.assertGreater(prob.get_val(f'box.{AE.BENDING_STIFFNESS}', units='N*m**2'), 0.0)
        self.assertGreater(prob.get_val(f'box.{AE.TORSIONAL_RIGIDITY}', units='N*m**2'), 0.0)
        self.assertGreater(prob.get_val(f'box.{AE.TORSIONAL_STIFFNESS}', units='N*m/rad'), 0.0)

        partials = prob.check_partials(method='cs', out_stream=None)
        for comp_partials in partials.values():
            for partial in comp_partials.values():
                self.assertLess(partial['abs error'].forward, 1.0e-6)


class TestStaticAeroelastic(unittest.TestCase):
    def test_divergence_reversal_and_effectiveness(self):
        prob = om.Problem()
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

        partials = prob.check_partials(method='cs', out_stream=None)
        for comp_partials in partials.values():
            for partial in comp_partials.values():
                self.assertLess(partial['abs error'].forward, 1.0e-6)


if __name__ == '__main__':
    unittest.main()
