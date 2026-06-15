import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class StrengthMargins(om.ExplicitComponent):
    """Preliminary root stress margins for the closed wingbox.

    This is intentionally simple and hard to misuse:

    * It is a root-section screen, not a full finite-element model.
    * It assumes the constant-lift component provides root bending moment and
      root torque for a half-wing cantilever.
    * Bending stress uses sigma = M * z / I with z = half wingbox height.
    * Torsion uses Bredt closed-cell shear flow q = T / (2 * enclosed_area).
      The stress is checked in both skin and spar walls and the worse value is
      reported.
    * Margins are reported as allowable / (safety_factor * stress) - 1.

    Positive margin means the preliminary screen passes. Negative margin means
    the wingbox fails this simplified check. Do not use this as certification
    evidence; replace the allowables and load model with tested data before any
    real structural decision.
    """

    def setup(self):
        self.add_input(AE.ROOT_BENDING_MOMENT, val=1.0, units='N*m')
        self.add_input(AE.ROOT_TORQUE, val=1.0, units='N*m')
        self.add_input(AE.BENDING_INERTIA, val=1.0e-6, units='m**4')
        self.add_input(AE.WINGBOX_HEIGHT, val=0.05, units='m')
        self.add_input(AE.WINGBOX_AREA, val=0.01, units='m**2')
        self.add_input(AE.SKIN_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.SPAR_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.TENSILE_ALLOWABLE, val=140.0e6, units='Pa')
        self.add_input(AE.COMPRESSIVE_ALLOWABLE, val=140.0e6, units='Pa')
        self.add_input(AE.SHEAR_ALLOWABLE, val=90.0e6, units='Pa')
        self.add_input(AE.STRENGTH_SAFETY_FACTOR, val=1.5)

        self.add_output(AE.ROOT_BENDING_STRESS, val=0.0, units='Pa')
        self.add_output(AE.TORSION_SHEAR_STRESS, val=0.0, units='Pa')
        self.add_output(AE.BENDING_STRENGTH_MARGIN, val=1.0)
        self.add_output(AE.TORSION_STRENGTH_MARGIN, val=1.0)
        self.add_output(AE.MIN_STRENGTH_MARGIN, val=1.0)

        # abs/max/min and guard logic make this a finite-difference component.
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        bending_moment = inputs[AE.ROOT_BENDING_MOMENT]
        root_torque = inputs[AE.ROOT_TORQUE]
        bending_inertia = np.maximum(inputs[AE.BENDING_INERTIA], 1.0e-30)
        half_height = 0.5 * inputs[AE.WINGBOX_HEIGHT]
        enclosed_area = np.maximum(inputs[AE.WINGBOX_AREA], 1.0e-30)
        skin_thickness = np.maximum(inputs[AE.SKIN_THICKNESS], 1.0e-12)
        spar_thickness = np.maximum(inputs[AE.SPAR_THICKNESS], 1.0e-12)
        tensile_allowable = np.maximum(inputs[AE.TENSILE_ALLOWABLE], 1.0e-30)
        compressive_allowable = np.maximum(inputs[AE.COMPRESSIVE_ALLOWABLE], 1.0e-30)
        shear_allowable = np.maximum(inputs[AE.SHEAR_ALLOWABLE], 1.0e-30)
        safety_factor = np.maximum(inputs[AE.STRENGTH_SAFETY_FACTOR], 1.0e-12)

        bending_stress = np.abs(bending_moment) * half_height / bending_inertia
        shear_flow = np.abs(root_torque) / (2.0 * enclosed_area)
        skin_shear_stress = shear_flow / skin_thickness
        spar_shear_stress = shear_flow / spar_thickness
        torsion_shear_stress = np.maximum(skin_shear_stress, spar_shear_stress)

        bending_allowable = np.minimum(tensile_allowable, compressive_allowable)
        stress_floor = 1.0e-12

        outputs[AE.ROOT_BENDING_STRESS] = bending_stress
        outputs[AE.TORSION_SHEAR_STRESS] = torsion_shear_stress
        outputs[AE.BENDING_STRENGTH_MARGIN] = (
            bending_allowable / (safety_factor * np.maximum(bending_stress, stress_floor)) - 1.0
        )
        outputs[AE.TORSION_STRENGTH_MARGIN] = (
            shear_allowable
            / (safety_factor * np.maximum(torsion_shear_stress, stress_floor))
            - 1.0
        )
        outputs[AE.MIN_STRENGTH_MARGIN] = np.minimum(
            outputs[AE.BENDING_STRENGTH_MARGIN],
            outputs[AE.TORSION_STRENGTH_MARGIN],
        )
