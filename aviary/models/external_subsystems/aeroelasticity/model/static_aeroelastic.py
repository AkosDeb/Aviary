import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class StaticAeroelastic(om.ExplicitComponent):
    """Preliminary divergence and full-span control reversal estimates."""

    def setup(self):
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.TORSIONAL_STIFFNESS, val=1.0e4, units='N*m/rad')
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125)
        self.add_input(AE.LIFT_CURVE_SLOPE, val=2.0 * np.pi, units='unitless')
        self.add_input(AE.CONTROL_LIFT_DERIVATIVE, val=2.5, units='1/rad')
        self.add_input(AE.CONTROL_MOMENT_DERIVATIVE, val=-0.60, units='1/rad')
        self.add_input(AE.DESIGN_SPEED, val=150.0, units='m/s')
        self.add_input(AE.REQUIRED_SPEED, val=1.15 * 550.0 / 3.6, units='m/s')
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')

        self.add_output(AE.DIVERGENCE_DYNAMIC_PRESSURE, val=1.0e5, units='Pa')
        self.add_output(AE.DIVERGENCE_SPEED, val=400.0, units='m/s')
        self.add_output(AE.DIVERGENCE_SPEED_MARGIN, val=200.0, units='m/s')
        self.add_output(AE.REVERSAL_DYNAMIC_PRESSURE, val=1.0e5, units='Pa')
        self.add_output(AE.REVERSAL_SPEED, val=400.0, units='m/s')
        self.add_output(AE.REVERSAL_SPEED_MARGIN, val=200.0, units='m/s')
        self.add_output(AE.CONTROL_EFFECTIVENESS, val=1.0)

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        area = inputs[Aircraft.Wing.AREA]
        span = inputs[Aircraft.Wing.SPAN]
        taper = inputs[Aircraft.Wing.TAPER_RATIO]
        k_torsion = inputs[AE.TORSIONAL_STIFFNESS]
        static_margin_arm = inputs[AE.AERO_CENTER_TO_EA_FRACTION]
        cl_alpha = inputs[AE.LIFT_CURVE_SLOPE]
        cl_delta = inputs[AE.CONTROL_LIFT_DERIVATIVE]
        cm_delta = inputs[AE.CONTROL_MOMENT_DERIVATIVE]
        design_speed = inputs[AE.DESIGN_SPEED]
        required_speed = inputs[AE.REQUIRED_SPEED]
        rho = inputs[AE.AIR_DENSITY]

        root_chord = 2.0 * area / (span * (1.0 + taper))
        chord = (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / (1.0 + taper)
        semispan = 0.5 * span

        # Strip-theory lumped model: q_div = k_torsion / (e * c * Clα * L) where
        # e = static_margin_arm * chord and L = semispan (cantilever length).
        q_div = k_torsion / (static_margin_arm * chord**2 * cl_alpha * semispan)
        q_rev = -k_torsion * cl_delta / (chord**2 * cm_delta * cl_alpha * semispan)
        q_design = 0.5 * rho * design_speed**2

        outputs[AE.DIVERGENCE_DYNAMIC_PRESSURE] = q_div
        outputs[AE.REVERSAL_DYNAMIC_PRESSURE] = q_rev

        # Guard: non-positive q means no divergence/reversal in forward flight (e.g.,
        # EA ahead of AC for divergence, or reversed control sign for reversal).
        # Replace with a large sentinel so sqrt and downstream ratios stay finite.
        q_div_safe = np.where(q_div > 0.0, q_div, 1.0e15)
        q_rev_safe = np.where(q_rev > 0.0, q_rev, 1.0e15)

        outputs[AE.DIVERGENCE_SPEED] = np.sqrt(2.0 * q_div_safe / rho)
        outputs[AE.REVERSAL_SPEED] = np.sqrt(2.0 * q_rev_safe / rho)
        outputs[AE.DIVERGENCE_SPEED_MARGIN] = outputs[AE.DIVERGENCE_SPEED] - required_speed
        outputs[AE.REVERSAL_SPEED_MARGIN] = outputs[AE.REVERSAL_SPEED] - required_speed

        # Guard effectiveness denominator: blows up if design speed exactly equals
        # divergence speed. A small floor keeps the optimiser from seeing NaN.
        eff_numer = 1.0 - q_design / q_rev_safe
        eff_denom = 1.0 - q_design / q_div_safe
        eff_denom_safe = np.where(np.abs(eff_denom) > 1.0e-12, eff_denom, 1.0e-12)
        outputs[AE.CONTROL_EFFECTIVENESS] = eff_numer / eff_denom_safe
