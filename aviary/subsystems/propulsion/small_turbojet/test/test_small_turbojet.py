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

_T_REF = 394.046052632   # N — training-mean thrust used for sanity checks


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
            {f'pre_mission.propulsion.{Aircraft.Engine.SCALED_SLS_THRUST}'},
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
        prob.set_val(Aircraft.Engine.SCALED_SLS_THRUST, _T_REF, units='N')

        prob.run_model()

        # Diameter is computed by MaxDiameter; check it's in a plausible range.
        d_mm = prob.get_val(SmallTurbojetVariables.DIAMETER, units='m').item() * 1000.0
        self.assertTrue(50.0 < d_mm < 300.0, f'diameter={d_mm:.1f} mm outside plausible range')

        # RPM, mass, and SFC must be finite and physically plausible.
        rpm = prob.get_val(SmallTurbojetVariables.MAX_RPM, units='rpm').item()
        self.assertTrue(np.isfinite(rpm) and rpm > 10_000 and rpm < 300_000,
                        f'rpm_max={rpm:.0f} outside plausible range')

        mass = prob.get_val(SmallTurbojetVariables.MASS, units='kg').item()
        self.assertTrue(np.isfinite(mass) and 0.1 < mass < 100.0,
                        f'mass={mass:.3f} kg outside plausible range')

        sfc = prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)').item()
        self.assertTrue(np.isfinite(sfc) and 1e-6 < sfc < 5e-4,
                        f'SFC={sfc:.2e} outside plausible range')

        partial_data = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(partial_data, atol=1e-8, rtol=1e-8)

    @use_tempdirs
    def test_premission_to_mission(self):
        nn = 3
        throttle = np.array([0.0, 0.5, 1.0])

        prob = om.Problem()
        prob.model.add_subsystem('premission', SmallTurbojetPreMission(), promotes=['*'])
        prob.model.add_subsystem('mission', SmallTurbojetMission(num_nodes=nn), promotes=['*'])

        prob.setup(force_alloc_complex=True)
        prob.set_val(Aircraft.Engine.SCALED_SLS_THRUST, _T_REF, units='N')
        prob.set_val(Dynamic.Vehicle.Propulsion.THROTTLE, throttle)

        prob.run_model()

        thrust_max = prob.get_val(Aircraft.Engine.SCALED_SLS_THRUST, units='N').item()
        sfc = prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)').item()
        thrust_vec = throttle * thrust_max

        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.THRUST_MAX, units='N'),
            np.full(nn, thrust_max),
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.THRUST, units='N'),
            thrust_vec,
            tolerance=1e-12,
        )
        assert_near_equal(
            prob.get_val(Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE, units='kg/s'),
            -thrust_vec * sfc,
            tolerance=1e-12,
        )

        partial_data = prob.check_partials(out_stream=None, method='cs')
        assert_check_partials(partial_data, atol=1e-8, rtol=1e-8)


if __name__ == '__main__':
    unittest.main()
