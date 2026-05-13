import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_partials, assert_near_equal
from openmdao.utils.testing_utils import use_tempdirs

from aviary.subsystems.propulsion.small_turbojet.model.small_turbojet_mission import (
    SmallTurbojetMission,
)
from aviary.subsystems.propulsion.small_turbojet.model.small_turbojet_premission import (
    SmallTurbojetPreMission,
)
from aviary.subsystems.propulsion.small_turbojet.small_turbojet_builder import SmallTurbojetModel
from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft, Dynamic


class TestSmallTurbojet(unittest.TestCase):
    def test_builder_precheck(self):
        engine = SmallTurbojetModel()

        self.assertFalse(engine._precheck_complete)

        engine.build_pre_mission(aviary_inputs=None)

        self.assertTrue(engine._precheck_complete)

    def test_builder_precheck_can_be_disabled(self):
        engine = SmallTurbojetModel(run_precheck=False)

        engine.build_pre_mission(aviary_inputs=None)

        self.assertFalse(engine._precheck_complete)

    @use_tempdirs
    def test_premission_regressions(self):
        prob = om.Problem()
        prob.model.add_subsystem('small_turbojet', SmallTurbojetPreMission(), promotes=['*'])

        prob.setup(force_alloc_complex=True)
        prob.set_val(SmallTurbojetVariables.DIAMETER, 0.15, units='m')
        prob.set_val(SmallTurbojetVariables.LENGTH, 0.45, units='m')

        prob.run_model()

        assert_near_equal(
            prob.get_val(Aircraft.Engine.SCALED_SLS_THRUST, units='N'),
            353.4291735288517,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.MASS, units='kg'),
            9.542587685278995,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.EGT, units='K'),
            910.0,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)'),
            3.065e-5,
            tolerance=1e-12,
        )

        partial_data = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(partial_data, atol=1e-12, rtol=1e-12)

    @use_tempdirs
    def test_premission_to_mission(self):
        nn = 3
        throttle = np.array([0.0, 0.5, 1.0])

        prob = om.Problem()
        prob.model.add_subsystem('premission', SmallTurbojetPreMission(), promotes=['*'])
        prob.model.add_subsystem('mission', SmallTurbojetMission(num_nodes=nn), promotes=['*'])

        prob.setup(force_alloc_complex=True)
        prob.set_val(SmallTurbojetVariables.DIAMETER, 0.15, units='m')
        prob.set_val(SmallTurbojetVariables.LENGTH, 0.45, units='m')
        prob.set_val(Dynamic.Vehicle.Propulsion.THROTTLE, throttle)

        prob.run_model()

        thrust_max = 353.4291735288517
        sfc = 3.065e-5
        thrust = throttle * thrust_max

        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.THRUST_MAX, units='N'),
            np.full(nn, thrust_max),
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.THRUST, units='N'),
            thrust,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE, units='kg/s'),
            -thrust * sfc,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.TEMPERATURE_T4, units='K'),
            np.full(nn, 910.0),
            tolerance=1e-12,
        )

        partial_data = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(partial_data, atol=1e-12, rtol=1e-12)


if __name__ == '__main__':
    unittest.main()
