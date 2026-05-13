import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft


class SmallTurbojetPreMission(om.ExplicitComponent):
    """
    Static design regressions for the simplified small turbojet.

    This component runs before the mission. It converts geometry-level design
    variables into engine properties that are constant during the mission.
    The equations here are placeholders; replace their coefficients/forms with
    your trained regressions when ready.
    """

    def setup(self):
        self.add_input(SmallTurbojetVariables.DIAMETER, val=0.15, units='m')
        self.add_input(SmallTurbojetVariables.LENGTH, val=0.45, units='m')

        self.add_output(Aircraft.Engine.SCALED_SLS_THRUST, val=350.0, units='N')
        self.add_output(SmallTurbojetVariables.MASS, val=10.0, units='kg')
        self.add_output(SmallTurbojetVariables.EGT, val=900.0, units='K')
        self.add_output(SmallTurbojetVariables.SFC, val=3.0e-5, units='kg/(N*s)')

        self.declare_partials(
            Aircraft.Engine.SCALED_SLS_THRUST,
            SmallTurbojetVariables.DIAMETER,
        )
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.LENGTH)
        self.declare_partials(SmallTurbojetVariables.EGT, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.EGT, SmallTurbojetVariables.LENGTH)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.LENGTH)

    def compute(self, inputs, outputs):
        diameter = inputs[SmallTurbojetVariables.DIAMETER]
        length = inputs[SmallTurbojetVariables.LENGTH]

        inlet_area = 0.25 * np.pi * diameter**2
        engine_volume = inlet_area * length

        outputs[Aircraft.Engine.SCALED_SLS_THRUST] = 20000.0 * inlet_area
        outputs[SmallTurbojetVariables.MASS] = 1200.0 * engine_volume
        outputs[SmallTurbojetVariables.EGT] = 850.0 + 250.0 * diameter + 50.0 * length
        outputs[SmallTurbojetVariables.SFC] = (
            3.2e-5 - 1.5e-5 * diameter + 2.0e-6 * length
        )

    def compute_partials(self, inputs, partials):
        diameter = inputs[SmallTurbojetVariables.DIAMETER]
        length = inputs[SmallTurbojetVariables.LENGTH]

        d_area_d_diameter = 0.5 * np.pi * diameter
        inlet_area = 0.25 * np.pi * diameter**2

        partials[
            Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.DIAMETER
        ] = 20000.0 * d_area_d_diameter
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER] = (
            1200.0 * length * d_area_d_diameter
        )
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.LENGTH] = (
            1200.0 * inlet_area
        )

        partials[SmallTurbojetVariables.EGT, SmallTurbojetVariables.DIAMETER] = 250.0
        partials[SmallTurbojetVariables.EGT, SmallTurbojetVariables.LENGTH] = 50.0

        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER] = -1.5e-5
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.LENGTH] = 2.0e-6
