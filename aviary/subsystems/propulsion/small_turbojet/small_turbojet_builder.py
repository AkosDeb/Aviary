import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.engine_model import EngineModel
from aviary.subsystems.propulsion.small_turbojet.model.small_turbojet_mission import (
    SmallTurbojetMission,
)
from aviary.subsystems.propulsion.small_turbojet.model.small_turbojet_premission import (
    SmallTurbojetPreMission,
)
from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft, Dynamic


class SmallTurbojetModel(EngineModel):
    """
    EngineModel wrapper for a simplified small turbojet regression model.

    The wrapper is the object Aviary receives through load_external_subsystems().
    It is responsible for creating the OpenMDAO systems used in pre-mission and
    mission, and for telling Aviary which design variables, fixed parameters, and
    timeseries outputs this engine provides.
    """

    _default_name = 'small_turbojet'
    compute_max_values = True

    def __init__(self, name=None, options=None, run_precheck=True):
        self.run_precheck = run_precheck
        self._precheck_complete = False
        super().__init__(name=name, options=options)

    def build_pre_mission(self, aviary_inputs, subsystem_options=None):
        self._run_precheck()
        return SmallTurbojetPreMission()

    def build_mission(self, num_nodes, aviary_inputs, user_options, subsystem_options):
        self._run_precheck()
        return SmallTurbojetMission(num_nodes=num_nodes)

    def _run_precheck(self):
        if self._precheck_complete or not self.run_precheck:
            return

        throttle = np.array([0.0, 0.5, 1.0])

        prob = om.Problem()
        prob.model.add_subsystem('premission', SmallTurbojetPreMission(), promotes=['*'])
        prob.model.add_subsystem(
            'mission',
            SmallTurbojetMission(num_nodes=len(throttle)),
            promotes=['*'],
        )

        prob.setup()
        prob.set_val(Aircraft.Engine.SCALED_SLS_THRUST, 394.0, units='N')
        prob.set_val(Dynamic.Vehicle.Propulsion.THROTTLE, throttle)
        prob.run_model()

        max_thrust = prob.get_val(Aircraft.Engine.SCALED_SLS_THRUST, units='N')
        mass = prob.get_val(SmallTurbojetVariables.MASS, units='kg')
        sfc = prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)')
        thrust = prob.get_val(Dynamic.Vehicle.Propulsion.THRUST, units='N')
        fuel_flow = prob.get_val(
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            units='kg/s',
        )

        checks = {
            Aircraft.Engine.SCALED_SLS_THRUST: max_thrust,
            SmallTurbojetVariables.MASS: mass,
            SmallTurbojetVariables.SFC: sfc,
            Dynamic.Vehicle.Propulsion.THRUST: thrust,
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE: fuel_flow,
        }
        for name, value in checks.items():
            if not np.all(np.isfinite(value)):
                raise RuntimeError(
                    f'SmallTurbojetModel precheck failed: {name} contains non-finite values.'
                )

        if np.any(max_thrust <= 0.0):
            raise RuntimeError('SmallTurbojetModel precheck failed: max thrust must be positive.')
        if np.any(mass <= 0.0):
            raise RuntimeError('SmallTurbojetModel precheck failed: engine mass must be positive.')
        if np.any(sfc <= 0.0):
            raise RuntimeError('SmallTurbojetModel precheck failed: SFC must be positive.')
        if not np.all(np.diff(thrust) >= 0.0):
            raise RuntimeError(
                'SmallTurbojetModel precheck failed: thrust must increase with throttle.'
            )
        if np.any(fuel_flow > 0.0):
            raise RuntimeError(
                'SmallTurbojetModel precheck failed: negative fuel flow output must not be positive.'
            )

        self._precheck_complete = True

    def get_design_vars(self, aviary_inputs=None):
        return {
            f'pre_mission.propulsion.{Aircraft.Engine.SCALED_SLS_THRUST}': {
                'units': 'N',
                'lower': 20.0,
                'upper': 2000.0,
                'ref': 394.0,
            },
        }

    def get_parameters(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        return {}

    def get_pre_mission_bus_variables(self, aviary_inputs=None, mission_info=None):
        return {
            Aircraft.Engine.SCALED_SLS_THRUST: {
                'mission_name': Aircraft.Engine.SCALED_SLS_THRUST,
                'units': 'N',
            },
            SmallTurbojetVariables.SFC: {
                'mission_name': SmallTurbojetVariables.SFC,
                'units': 'kg/(N*s)',
            },
        }

    def get_timeseries(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        return [
            Dynamic.Vehicle.Propulsion.THRUST,
            Dynamic.Vehicle.Propulsion.THRUST_MAX,
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
        ]
