import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class SchrenkLiftDistribution(om.ExplicitComponent):
    """Half-wing Schrenk lift distribution and equivalent cantilever loads.

    Schrenk's approximation averages a chord-proportional loading with an
    elliptical loading. The resulting shape is normalized to the requested total
    lift, then integrated over the half-span for shear, bending moment, torque,
    tip deflection, and tip twist.

    This is a differentiable conceptual-design load backend. It is intended to
    replace the current constant lift distribution after review, but is kept as
    a separate component for now.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')
        self.add_input(AE.DESIGN_SPEED, val=550.0 / 3.6, units='m/s')
        self.add_input(AE.LOAD_FACTOR, val=1.0)
        self.add_input(AE.LOAD_LIFT_COEFFICIENT, val=1.0)
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125, units='unitless')
        self.add_input(AE.BENDING_STIFFNESS, val=1.0e5, units='N*m**2')
        self.add_input(AE.TORSIONAL_RIGIDITY, val=1.0e5, units='N*m**2')

        self.add_output(AE.SPANWISE_STATIONS, val=np.zeros(n), units='m')
        self.add_output(AE.SPANWISE_CHORD, val=np.ones(n), units='m')
        self.add_output(AE.SCHRENK_LIFT_PER_UNIT_SPAN, val=np.ones(n), units='N/m')
        self.add_output(AE.SCHRENK_SHEAR_FORCE, val=np.ones(n), units='N')
        self.add_output(AE.SCHRENK_BENDING_MOMENT, val=np.ones(n), units='N*m')
        self.add_output(AE.SCHRENK_TORQUE, val=np.ones(n), units='N*m')

        self.add_output(AE.CONSTANT_TOTAL_LIFT, val=1.0, units='N')
        self.add_output(AE.ROOT_BENDING_MOMENT, val=1.0, units='N*m')
        self.add_output(AE.ROOT_TORQUE, val=1.0, units='N*m')
        self.add_output(AE.TIP_DEFLECTION, val=0.0, units='m')
        self.add_output(AE.TIP_TWIST, val=0.0, units='rad')
        self.add_output(AE.TIP_TWIST_DEG, val=0.0, units='deg')

        self.declare_partials('*', '*', method='fd')

    @staticmethod
    def _cumtrapz_from_tip(y, value):
        """Return integral from each station to the tip using trapezoids."""
        integ = np.zeros_like(value)
        for idx in range(len(value) - 2, -1, -1):
            dy = y[idx + 1] - y[idx]
            integ[idx] = integ[idx + 1] + 0.5 * (value[idx] + value[idx + 1]) * dy
        return integ

    @staticmethod
    def _trapz(y, value):
        return np.sum(0.5 * (value[:-1] + value[1:]) * (y[1:] - y[:-1]))

    def compute(self, inputs, outputs):
        n = self.options['num_stations']

        area = inputs[Aircraft.Wing.AREA]
        span = inputs[Aircraft.Wing.SPAN]
        taper = inputs[Aircraft.Wing.TAPER_RATIO]
        rho = inputs[AE.AIR_DENSITY]
        speed = inputs[AE.DESIGN_SPEED]
        load_factor = inputs[AE.LOAD_FACTOR]
        load_cl = inputs[AE.LOAD_LIFT_COEFFICIENT]
        e_frac = inputs[AE.AERO_CENTER_TO_EA_FRACTION]
        EI = inputs[AE.BENDING_STIFFNESS]
        GJ = inputs[AE.TORSIONAL_RIGIDITY]

        semispan = 0.5 * span
        y = np.linspace(0.0, semispan, n)

        root_chord = 2.0 * area / (span * (1.0 + taper))
        tip_chord = root_chord * taper
        chord = root_chord + (tip_chord - root_chord) * y / np.maximum(semispan, 1.0e-12)

        half_area = 0.5 * area
        ellipse_shape = np.sqrt(np.maximum(1.0 - (y / np.maximum(semispan, 1.0e-12)) ** 2, 0.0))
        ellipse_integral = self._trapz(y, ellipse_shape)
        ellipse_equivalent_chord = ellipse_shape * half_area / np.maximum(ellipse_integral, 1.0e-30)

        schrenk_shape = 0.5 * (chord + ellipse_equivalent_chord)
        schrenk_integral = self._trapz(y, schrenk_shape)

        q_dyn = 0.5 * rho * speed**2
        total_lift = load_factor * q_dyn * area * load_cl
        half_lift = 0.5 * total_lift
        lift_per_span = half_lift * schrenk_shape / np.maximum(schrenk_integral, 1.0e-30)

        shear = self._cumtrapz_from_tip(y, lift_per_span)
        bending = self._cumtrapz_from_tip(y, shear)

        torque_load = lift_per_span * e_frac * chord
        torque = self._cumtrapz_from_tip(y, torque_load)

        tip_deflection = self._trapz(y, bending * (semispan - y)) / np.maximum(EI, 1.0e-30)
        tip_twist = self._trapz(y, torque) / np.maximum(GJ, 1.0e-30)

        outputs[AE.SPANWISE_STATIONS] = y
        outputs[AE.SPANWISE_CHORD] = chord
        outputs[AE.SCHRENK_LIFT_PER_UNIT_SPAN] = lift_per_span
        outputs[AE.SCHRENK_SHEAR_FORCE] = shear
        outputs[AE.SCHRENK_BENDING_MOMENT] = bending
        outputs[AE.SCHRENK_TORQUE] = torque
        outputs[AE.CONSTANT_TOTAL_LIFT] = total_lift
        outputs[AE.ROOT_BENDING_MOMENT] = bending[0]
        outputs[AE.ROOT_TORQUE] = torque[0]
        outputs[AE.TIP_DEFLECTION] = tip_deflection
        outputs[AE.TIP_TWIST] = tip_twist
        outputs[AE.TIP_TWIST_DEG] = tip_twist * (180.0 / np.pi)
