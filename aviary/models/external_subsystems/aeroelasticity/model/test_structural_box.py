import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.structural_box import (
    WingboxStructuralEstimate,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestWingboxStructuralEstimate(unittest.TestCase):
    def test_two_spar_closed_box_defaults(self):
        prob = om.Problem(name='test_wingbox_structural_estimate', reports=False)
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
        self.assertGreater(prob.get_val(f'box.{AE.MASS_PER_UNIT_SPAN}', units='kg/m'), 0.0)

    def test_material_modulus_changes_stiffness(self):
        prob = om.Problem(name='test_wingbox_material_stiffness', reports=False)
        prob.model.add_subsystem('box', WingboxStructuralEstimate())
        prob.setup()

        prob.set_val(f'box.{AE.YOUNGS_MODULUS}', 70.0e9, units='Pa')
        prob.set_val(f'box.{AE.SHEAR_MODULUS}', 27.0e9, units='Pa')
        prob.run_model()
        high_ei = float(prob.get_val(f'box.{AE.BENDING_STIFFNESS}', units='N*m**2')[0])
        high_gj = float(prob.get_val(f'box.{AE.TORSIONAL_RIGIDITY}', units='N*m**2')[0])

        prob.set_val(f'box.{AE.YOUNGS_MODULUS}', 10.0e9, units='Pa')
        prob.set_val(f'box.{AE.SHEAR_MODULUS}', 4.0e9, units='Pa')
        prob.run_model()
        low_ei = float(prob.get_val(f'box.{AE.BENDING_STIFFNESS}', units='N*m**2')[0])
        low_gj = float(prob.get_val(f'box.{AE.TORSIONAL_RIGIDITY}', units='N*m**2')[0])

        self.assertGreater(high_ei, low_ei)
        self.assertGreater(high_gj, low_gj)


if __name__ == '__main__':
    unittest.main()
