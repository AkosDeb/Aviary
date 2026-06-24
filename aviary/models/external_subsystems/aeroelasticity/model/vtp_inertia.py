import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class VTPTipInertia(om.ExplicitComponent):
    """Map tail-owned physical inertia equivalents to legacy VTP tip outputs."""

    def setup(self):
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(
            'tail_physical_tip_mass_equivalent',
            val=0.2,
            units='kg',
            desc='Tail geometry owned equivalent mass applied at the wing tip.',
        )
        self.add_input(
            'tail_physical_tip_pitch_inertia_equivalent',
            val=0.01,
            units='kg*m**2',
            desc='Tail geometry owned pitch inertia equivalent about the wing tip.',
        )
        self.add_input(
            'tail_physical_cg_x_from_wing_tip_le',
            val=0.0,
            units='m',
            desc='Tail physical CG x offset aft from the wing tip leading edge.',
        )
        self.add_input(
            'tail_physical_cg_z_abs',
            val=0.2,
            units='m',
            desc='Absolute panel centroid z offset used by the legacy VTP diagnostics.',
        )
        self.add_input(
            'tail_physical_dx_to_elastic_axis',
            val=0.0,
            units='m',
            desc='Tail physical CG x offset from the wing tip elastic axis.',
        )

        self.add_output(AE.VTP_TIP_MASS, val=0.0, units='kg')
        self.add_output(AE.VTP_TIP_PITCH_INERTIA, val=0.0, units='kg*m**2')
        self.add_output(AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN, val=0.0, units='kg/m')
        self.add_output(AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN, val=0.0, units='kg*m')
        self.add_output(AE.VTP_PITCH_STATIC_UNBALANCE, val=0.0, units='kg')
        self.add_output(AE.VTP_CG_X_FROM_WING_TIP_LE, val=0.0, units='m')
        self.add_output(AE.VTP_CG_Z_ABS, val=0.0, units='m')
        self.add_output(AE.VTP_DX_TO_ELASTIC_AXIS, val=0.0, units='m')

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        wing_span = inputs[Aircraft.Wing.SPAN]
        tip_mass = inputs['tail_physical_tip_mass_equivalent']
        tip_pitch_inertia = inputs['tail_physical_tip_pitch_inertia_equivalent']
        x_from_wing_tip_le = inputs['tail_physical_cg_x_from_wing_tip_le']
        z_abs = inputs['tail_physical_cg_z_abs']
        dx_to_ea = inputs['tail_physical_dx_to_elastic_axis']

        wing_span_safe = np.maximum(wing_span, 1.0e-12)
        equivalent_mass_per_span = tip_mass / wing_span_safe
        pitch_inertia_per_span = tip_pitch_inertia / wing_span_safe
        static_unbalance = equivalent_mass_per_span * dx_to_ea

        outputs[AE.VTP_TIP_MASS] = tip_mass
        outputs[AE.VTP_TIP_PITCH_INERTIA] = tip_pitch_inertia
        outputs[AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN] = equivalent_mass_per_span
        outputs[AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN] = pitch_inertia_per_span
        outputs[AE.VTP_PITCH_STATIC_UNBALANCE] = static_unbalance
        outputs[AE.VTP_CG_X_FROM_WING_TIP_LE] = x_from_wing_tip_le
        outputs[AE.VTP_CG_Z_ABS] = z_abs
        outputs[AE.VTP_DX_TO_ELASTIC_AXIS] = dx_to_ea
