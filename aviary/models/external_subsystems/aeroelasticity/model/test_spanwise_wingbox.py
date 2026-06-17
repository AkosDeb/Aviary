import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.spanwise_wingbox import (
    SpanwiseWingboxProperties,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestSpanwiseWingboxProperties(unittest.TestCase):
    def test_tapered_wing_root_properties_exceed_tip(self):
        prob = om.Problem(name='test_spanwise_wingbox_taper', reports=False)
        prob.model.add_subsystem('box', SpanwiseWingboxProperties(num_stations=5))
        prob.setup(force_alloc_complex=True)

        prob.set_val(f'box.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'box.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'box.{Aircraft.Wing.TAPER_RATIO}', 0.5)
        prob.set_val(f'box.{AE.STRUCTURAL_THICKNESS_TO_CHORD}', 0.12)
        prob.run_model()

        y = prob.get_val(f'box.{AE.SPANWISE_STATIONS}', units='m')
        chord = prob.get_val(f'box.{AE.SPANWISE_CHORD}', units='m')
        ei = prob.get_val(f'box.{AE.SPANWISE_BENDING_STIFFNESS}', units='N*m**2')
        gj = prob.get_val(f'box.{AE.SPANWISE_TORSIONAL_RIGIDITY}', units='N*m**2')
        mass = prob.get_val(f'box.{AE.SPANWISE_MASS_PER_UNIT_SPAN}', units='kg/m')

        assert_near_equal(y[0], 0.0)
        assert_near_equal(y[-1], 1.0)
        self.assertGreater(chord[0], chord[-1])
        self.assertGreater(ei[0], ei[-1])
        self.assertGreater(gj[0], gj[-1])
        self.assertGreater(mass[0], mass[-1])

    def test_structural_tc_increases_spanwise_stiffness(self):
        prob = om.Problem(name='test_spanwise_wingbox_tc', reports=False)
        prob.model.add_subsystem('box', SpanwiseWingboxProperties(num_stations=7))
        prob.setup()

        prob.set_val(f'box.{AE.STRUCTURAL_THICKNESS_TO_CHORD}', 0.05)
        prob.run_model()
        ei_low = prob.get_val(f'box.{AE.SPANWISE_BENDING_STIFFNESS}', units='N*m**2').copy()
        gj_low = prob.get_val(f'box.{AE.SPANWISE_TORSIONAL_RIGIDITY}', units='N*m**2').copy()

        prob.set_val(f'box.{AE.STRUCTURAL_THICKNESS_TO_CHORD}', 0.18)
        prob.run_model()
        ei_high = prob.get_val(f'box.{AE.SPANWISE_BENDING_STIFFNESS}', units='N*m**2').copy()
        gj_high = prob.get_val(f'box.{AE.SPANWISE_TORSIONAL_RIGIDITY}', units='N*m**2').copy()

        self.assertTrue((ei_high > ei_low).all())
        self.assertTrue((gj_high > gj_low).all())


if __name__ == '__main__':
    unittest.main()
