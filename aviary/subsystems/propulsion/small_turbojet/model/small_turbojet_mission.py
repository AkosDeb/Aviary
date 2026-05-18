import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft, Dynamic


class SmallTurbojetMission(om.ExplicitComponent):
    """
    Mission performance model for the simplified small turbojet.

    For now, the engine is assumed to provide identical static performance at
    all speeds and altitudes. Throttle linearly scales thrust, and fuel flow is
    computed from thrust and the pre-mission SFC regression.
    """

    def initialize(self):
        self.options.declare('num_nodes', types=int)

    def setup(self):
        nn = self.options['num_nodes']

        self.add_input(
            Dynamic.Vehicle.Propulsion.THROTTLE,
            val=np.ones(nn),
            units='unitless',
        )
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST, val=350.0, units='N')
        self.add_input(SmallTurbojetVariables.SFC, val=3.0e-5, units='kg/(N*s)')

        self.add_output(Dynamic.Vehicle.Propulsion.THRUST, val=np.zeros(nn), units='N')
        self.add_output(Dynamic.Vehicle.Propulsion.THRUST_MAX, val=np.zeros(nn), units='N')
        self.add_output(
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            val=np.zeros(nn),
            units='kg/s',
        )

        rows = np.arange(nn)
        cols = np.arange(nn)
        scalar_cols = np.zeros(nn, dtype=int)

        self.declare_partials(
            Dynamic.Vehicle.Propulsion.THRUST,
            Dynamic.Vehicle.Propulsion.THROTTLE,
            rows=rows,
            cols=cols,
        )
        self.declare_partials(
            Dynamic.Vehicle.Propulsion.THRUST,
            Aircraft.Engine.SCALED_SLS_THRUST,
            rows=rows,
            cols=scalar_cols,
        )
        self.declare_partials(
            Dynamic.Vehicle.Propulsion.THRUST_MAX,
            Aircraft.Engine.SCALED_SLS_THRUST,
            rows=rows,
            cols=scalar_cols,
            val=1.0,
        )
        self.declare_partials(
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            Dynamic.Vehicle.Propulsion.THROTTLE,
            rows=rows,
            cols=cols,
        )
        self.declare_partials(
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            Aircraft.Engine.SCALED_SLS_THRUST,
            rows=rows,
            cols=scalar_cols,
        )
        self.declare_partials(
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            SmallTurbojetVariables.SFC,
            rows=rows,
            cols=scalar_cols,
        )

    def compute(self, inputs, outputs):
        throttle = inputs[Dynamic.Vehicle.Propulsion.THROTTLE]
        max_thrust = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        sfc = inputs[SmallTurbojetVariables.SFC]

        thrust = throttle * max_thrust

        outputs[Dynamic.Vehicle.Propulsion.THRUST] = thrust
        outputs[Dynamic.Vehicle.Propulsion.THRUST_MAX] = max_thrust
        outputs[Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE] = -thrust * sfc

    def compute_partials(self, inputs, partials):
        throttle = inputs[Dynamic.Vehicle.Propulsion.THROTTLE]
        max_thrust = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        sfc = inputs[SmallTurbojetVariables.SFC]

        partials[
            Dynamic.Vehicle.Propulsion.THRUST, Dynamic.Vehicle.Propulsion.THROTTLE
        ] = max_thrust
        partials[
            Dynamic.Vehicle.Propulsion.THRUST, Aircraft.Engine.SCALED_SLS_THRUST
        ] = throttle

        partials[
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            Dynamic.Vehicle.Propulsion.THROTTLE,
        ] = -max_thrust * sfc
        partials[
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            Aircraft.Engine.SCALED_SLS_THRUST,
        ] = -throttle * sfc
        partials[
            Dynamic.Vehicle.Propulsion.FUEL_FLOW_RATE_NEGATIVE,
            SmallTurbojetVariables.SFC,
        ] = -throttle * max_thrust
