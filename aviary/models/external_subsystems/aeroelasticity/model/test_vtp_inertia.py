import unittest

import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.vtp_inertia import VTPTipInertia
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestVTPTipInertia(unittest.TestCase):
    def make_problem(self):
        prob = om.Problem(name='test_vtp_tip_inertia', reports=False)
        prob.model.add_subsystem('vtp', VTPTipInertia())
        prob.setup(force_alloc_complex=True)

        prob.set_val(f'vtp.{Aircraft.Wing.AREA}', 0.8, units='m**2')
        prob.set_val(f'vtp.{Aircraft.Wing.SPAN}', 2.0, units='m')
        prob.set_val(f'vtp.{Aircraft.Wing.TAPER_RATIO}', 1.0)
        prob.set_val(f'vtp.{AE.ELASTIC_AXIS_FRACTION}', 0.375)
        prob.set_val(f'vtp.{Aircraft.VerticalTail.MASS}', 0.2, units='kg')
        prob.set_val(f'vtp.{Aircraft.VerticalTail.SPAN}', 0.4, units='m')
        prob.set_val(f'vtp.{Aircraft.VerticalTail.ROOT_CHORD}', 0.2, units='m')
        prob.set_val(f'vtp.{Aircraft.VerticalTail.TAPER_RATIO}', 1.0)
        prob.set_val(f'vtp.{Aircraft.VerticalTail.SWEEP}', 0.0, units='deg')
        prob.set_val(f'vtp.{AE.VTP_ROOT_LE_X_OFFSET}', 0.0, units='m')
        return prob

    def test_symmetric_vtp_adds_mass_inertia_and_static_unbalance(self):
        prob = self.make_problem()
        prob.run_model()

        m_eq = prob.get_val(f'vtp.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', units='kg/m')
        inertia = prob.get_val(f'vtp.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', units='kg*m')
        dx = prob.get_val(f'vtp.{AE.VTP_DX_TO_ELASTIC_AXIS}', units='m')
        z_abs = prob.get_val(f'vtp.{AE.VTP_CG_Z_ABS}', units='m')

        self.assertAlmostEqual(float(m_eq[0]), 0.1)
        self.assertGreater(float(inertia[0]), 0.0)
        self.assertLess(float(dx[0]), 0.0)
        self.assertGreater(float(z_abs[0]), 0.0)

    def test_more_vtp_mass_increases_equivalent_mass_and_inertia(self):
        prob = self.make_problem()
        prob.set_val(f'vtp.{Aircraft.VerticalTail.MASS}', 0.1, units='kg')
        prob.run_model()
        low_mass = float(
            prob.get_val(f'vtp.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', units='kg/m')[0]
        )
        low_inertia = float(
            prob.get_val(f'vtp.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', units='kg*m')[0]
        )

        prob.set_val(f'vtp.{Aircraft.VerticalTail.MASS}', 0.3, units='kg')
        prob.run_model()
        high_mass = float(
            prob.get_val(f'vtp.{AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN}', units='kg/m')[0]
        )
        high_inertia = float(
            prob.get_val(f'vtp.{AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN}', units='kg*m')[0]
        )

        self.assertGreater(high_mass, low_mass)
        self.assertGreater(high_inertia, low_inertia)

    def test_root_le_x_offset_moves_vtp_cg_aft(self):
        prob = self.make_problem()
        prob.set_val(f'vtp.{AE.VTP_ROOT_LE_X_OFFSET}', 0.0, units='m')
        prob.run_model()
        baseline_dx = float(prob.get_val(f'vtp.{AE.VTP_DX_TO_ELASTIC_AXIS}', units='m')[0])

        prob.set_val(f'vtp.{AE.VTP_ROOT_LE_X_OFFSET}', 0.05, units='m')
        prob.run_model()
        shifted_dx = float(prob.get_val(f'vtp.{AE.VTP_DX_TO_ELASTIC_AXIS}', units='m')[0])

        self.assertAlmostEqual(shifted_dx - baseline_dx, 0.05)


if __name__ == '__main__':
    unittest.main()
