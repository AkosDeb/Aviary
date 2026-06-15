import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class ConstantLiftLoads(om.ExplicitComponent):
    """Half-wing cantilever loads from a constant spanwise lift distribution."""

    def setup(self):
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')
        self.add_input(AE.DESIGN_SPEED, val=550.0 / 3.6, units='m/s')
        self.add_input(AE.LOAD_FACTOR, val=1.0)
        self.add_input(AE.LOAD_LIFT_COEFFICIENT, val=1.0)
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125)
        self.add_input(AE.BENDING_STIFFNESS, val=1.0e5, units='N*m**2')
        self.add_input(AE.TORSIONAL_RIGIDITY, val=1.0e5, units='N*m**2')

        self.add_output(AE.CONSTANT_LIFT_PER_UNIT_SPAN, val=1.0, units='N/m')
        self.add_output(AE.CONSTANT_TOTAL_LIFT, val=1.0, units='N')
        self.add_output(AE.ROOT_BENDING_MOMENT, val=1.0, units='N*m')
        self.add_output(AE.ROOT_TORQUE, val=1.0, units='N*m')
        self.add_output(AE.TIP_DEFLECTION, val=0.0, units='m')
        self.add_output(AE.TIP_TWIST, val=0.0, units='rad')
        self.add_output(AE.TIP_TWIST_DEG, val=0.0, units='deg')

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
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

        root_chord = 2.0 * area / (span * (1.0 + taper))
        chord = (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / (1.0 + taper)
        semispan = 0.5 * span

        q_dyn = 0.5 * rho * speed**2
        total_lift = load_factor * q_dyn * area * load_cl
        lift_per_span = total_lift / span
        root_bending = lift_per_span * semispan**2 / 2.0
        root_torque = lift_per_span * semispan * e_frac * chord
        tip_deflection = lift_per_span * semispan**4 / (8.0 * EI)
        tip_twist = root_torque * semispan / GJ

        outputs[AE.CONSTANT_TOTAL_LIFT] = total_lift
        outputs[AE.CONSTANT_LIFT_PER_UNIT_SPAN] = lift_per_span
        outputs[AE.ROOT_BENDING_MOMENT] = root_bending
        outputs[AE.ROOT_TORQUE] = root_torque
        outputs[AE.TIP_DEFLECTION] = tip_deflection
        outputs[AE.TIP_TWIST] = tip_twist
        outputs[AE.TIP_TWIST_DEG] = tip_twist * (180.0 / np.pi)
