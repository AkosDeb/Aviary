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

    def test_builder_design_vars_and_bus(self):
        engine = SmallTurbojetModel(run_precheck=False)

        design_vars = engine.get_design_vars()
        self.assertEqual(
            set(design_vars),
            {
                f'pre_mission.propulsion.{SmallTurbojetVariables.DIAMETER}',
                f'pre_mission.propulsion.{SmallTurbojetVariables.LENGTH}',
            },
        )
        self.assertEqual(engine.get_parameters(), {})

        bus_vars = engine.get_pre_mission_bus_variables()
        self.assertEqual(
            set(bus_vars),
            {
                Aircraft.Engine.SCALED_SLS_THRUST,
                SmallTurbojetVariables.SFC,
            },
        )
        self.assertEqual(
            bus_vars[Aircraft.Engine.SCALED_SLS_THRUST]['mission_name'],
            Aircraft.Engine.SCALED_SLS_THRUST,
        )
        self.assertEqual(
            bus_vars[SmallTurbojetVariables.SFC]['mission_name'],
            SmallTurbojetVariables.SFC,
        )

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
            401.373473537979,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.MASS, units='kg'),
            5.10023850489127,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.MAX_RPM, units='rpm'),
            81451.765791041,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)'),
            4.03132702439889e-5,
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

        thrust_max = 401.373473537979
        sfc = 4.03132702439889e-5
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
        partial_data = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(partial_data, atol=1e-12, rtol=1e-12)


if __name__ == '__main__':
    unittest.main()
