import numpy as np
import openmdao.api as om

from aviary.variable_info.variables import Aircraft


class PropulsionLocationComp(om.ExplicitComponent):
    """Engine CG station for an aft-mounted pusher configuration.

    The engine sits at the aft fuselage; its longitudinal station is a fixed fraction
    of fuselage length from the nose:

        engine_x = -(fuselage_length × engine_station_fraction)   [m, FORWARD frame]

    Negative because the aft fuselage is behind the nose origin in the x-positive-FORWARD
    body frame.  engine_z = 0 (engine on fuselage centreline).

    Fuel CG is handled separately by FuelTankComp.

    Inputs
    ------
    Aircraft.Fuselage.LENGTH   m    Fuselage structural length
    engine_station_fraction    —    Engine CG as fraction of fuselage length from nose
                                    (default 0.85 — aft-mounted pusher)

    Outputs
    -------
    engine_x   m   Engine CG x in body frame (negative = aft of nose, FORWARD frame)
    engine_z   m   Engine CG z (0 = fuselage centreline)
    """

    def setup(self):
        self.add_input(Aircraft.Fuselage.LENGTH, val=2.0, units='m')
        self.add_input('engine_station_fraction', val=0.85, units='unitless',
                       desc='Engine CG as fraction of fuselage length from nose')

        self.add_output('engine_x', val=-1.70, units='m',
                        desc='Engine CG x in body frame (negative = aft of nose)')
        self.add_output('engine_z', val=0.0, units='m',
                        desc='Engine z on fuselage centreline')

    def setup_partials(self):
        self.declare_partials(
            'engine_x',
            [Aircraft.Fuselage.LENGTH, 'engine_station_fraction'],
            method='cs',
        )

    def compute(self, inputs, outputs):
        def _f(key):
            return np.asarray(inputs[key]).item()

        outputs['engine_x'] = -_f(Aircraft.Fuselage.LENGTH) * _f('engine_station_fraction')
        outputs['engine_z'] = 0.0
