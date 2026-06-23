import unittest

import aviary.api as av
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.aeroelasticity_builder import (
    AeroelasticityGroup,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestAeroelasticityGroupFlutterSwitch(unittest.TestCase):
    @staticmethod
    def _set_aircraft_defaults(prob):
        prob.model.set_input_defaults(
            av.Aircraft.Wing.TAPER_RATIO,
            val=0.6,
            units='unitless',
        )
        prob.model.set_input_defaults(
            av.Aircraft.VerticalTail.TAPER_RATIO,
            val=0.4,
            units='unitless',
        )

    def test_beam_modal_mode_builds_beam_modal_flutter(self):
        prob = om.Problem(
            model=AeroelasticityGroup(flutter_model='beam_modal_3dof_pk'),
            reports=False,
        )
        self._set_aircraft_defaults(prob)
        prob.setup()

        self.assertIn('beam_modal_flutter', prob.model._subsystems_allprocs)
        output_names = {
            meta['prom_name']
            for meta in prob.model.get_io_metadata(iotypes='output').values()
        }
        self.assertIn(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, output_names)

    def test_legacy_scalar_mode_skips_beam_modal_flutter(self):
        prob = om.Problem(
            model=AeroelasticityGroup(flutter_model='legacy_scalar'),
            reports=False,
        )
        self._set_aircraft_defaults(prob)
        prob.setup()

        self.assertNotIn('beam_modal_flutter', prob.model._subsystems_allprocs)
        output_names = {
            meta['prom_name']
            for meta in prob.model.get_io_metadata(iotypes='output').values()
        }
        self.assertIn(AE.MAX_REAL_EIGENVALUE_AT_DESIGN, output_names)
        self.assertNotIn(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, output_names)

    def test_beam_modal_group_baseline_outputs_are_plausible(self):
        prob = om.Problem(
            model=AeroelasticityGroup(flutter_model='beam_modal_3dof_pk'),
            reports=False,
        )
        self._set_aircraft_defaults(prob)
        prob.setup()

        prob.set_val(av.Aircraft.Wing.AREA, 0.45, units='m**2')
        prob.set_val(av.Aircraft.Wing.SPAN, 1.8, units='m')
        prob.set_val(av.Aircraft.Wing.TAPER_RATIO, 0.6)
        prob.set_val(av.Aircraft.VerticalTail.SPAN, 0.31, units='m')
        prob.set_val(av.Aircraft.VerticalTail.ROOT_CHORD, 0.258, units='m')
        prob.set_val(av.Aircraft.VerticalTail.TAPER_RATIO, 0.4)
        prob.run_model()

        bending_frequency = prob.get_val(AE.BEAM_MODAL_BENDING_FREQUENCY, units='Hz')[0]
        torsion_frequency = prob.get_val(AE.BEAM_MODAL_TORSION_FREQUENCY, units='Hz')[0]
        control_frequency = prob.get_val(AE.BEAM_MODAL_CONTROL_FREQUENCY, units='Hz')[0]

        self.assertGreater(bending_frequency, 0.0)
        self.assertGreater(torsion_frequency, bending_frequency)
        self.assertGreater(control_frequency, torsion_frequency)
        self.assertEqual(prob.get_val(AE.BEAM_MODAL_3DOF_PK_CONVERGED)[0], 1.0)
        self.assertGreater(
            prob.get_val(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, units='m/s')[0],
            0.0,
        )
        self.assertAlmostEqual(prob.get_val(AE.ELASTIC_AXIS_FRACTION)[0], 0.375)
        self.assertGreater(prob.get_val(AE.VTP_TIP_MASS, units='kg')[0], 0.0)
        self.assertGreater(prob.get_val(AE.VTP_TIP_PITCH_INERTIA, units='kg*m**2')[0], 0.0)


if __name__ == '__main__':
    unittest.main()
