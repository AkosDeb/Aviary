import numpy as np
import openmdao.api as om


class FuelTankComp(om.ExplicitComponent):
    """Fuel CG for a two-tank fuselage layout (front + rear).

    The fuel CG is the weighted average of two discrete tank stations:

        fuel_x = fuel_distribution * front_tank_x + (1 - fuel_distribution) * rear_tank_x

    This is a purely geometric result — independent of fuel_mass.  The fuel's
    contribution to the aircraft CG in SpaJetiCGEstimator is automatically zero
    when fuel_mass = 0, because the estimator multiplies fuel_mass by fuel_x.

    Body frame: x positive forward, nose at origin, z positive down.

    Inputs
    ------
    fuel_distribution   —   Fraction of fuel in the front tank (0=all rear, 1=all front).
                            Default 0.5.
    front_tank_x        m   Front tank centroid x in body frame (negative = aft of nose).
                            Default −0.85 m (near wing LE, ~42.5 % of a 2 m fuselage).
    rear_tank_x         m   Rear tank centroid x in body frame (negative = aft of nose).
                            Default −0.95 m (near engine station, ~47.5 %).

    Outputs
    -------
    fuel_x   m   Fuel CG x in body frame (weighted average of two tanks)
    fuel_z   m   Fuel CG z (= 0, fuselage centreline)

    Numerical example — UAV baseline (front_tank_x = −0.85, rear_tank_x = −0.95,
                                       fuel_distribution = 0.5)
    -----------------------------------------------------------------------
    fuel_x = 0.5 * (−0.85) + 0.5 * (−0.95) = −0.900 m
    fuel_z = 0.000 m
    """

    def setup(self):
        self.add_input(
            'fuel_distribution', val=0.5, units='unitless',
            desc='Fraction of fuel in front tank (0 = all rear, 1 = all front)',
        )
        self.add_input(
            'front_tank_x', val=-0.85, units='m',
            desc='Front tank centroid x in forward body frame',
        )
        self.add_input(
            'rear_tank_x', val=-0.95, units='m',
            desc='Rear tank centroid x in forward body frame',
        )

        self.add_output(
            'fuel_x', val=-0.90, units='m',
            desc='Fuel CG x in body frame (weighted average of front and rear tanks)',
        )
        self.add_output(
            'fuel_z', val=0.0, units='m',
            desc='Fuel CG z on fuselage centreline',
        )

    def setup_partials(self):
        self.declare_partials(
            'fuel_x',
            ['fuel_distribution', 'front_tank_x', 'rear_tank_x'],
            method='cs',
        )

    def compute(self, inputs, outputs):
        def _f(key):
            return np.asarray(inputs[key]).item()

        d = _f('fuel_distribution')
        fx = _f('front_tank_x')
        rx = _f('rear_tank_x')
        outputs['fuel_x'] = d * fx + (1.0 - d) * rx
        outputs['fuel_z'] = 0.0
