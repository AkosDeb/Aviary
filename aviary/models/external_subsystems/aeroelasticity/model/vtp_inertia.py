import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class VTPTipInertia(om.ExplicitComponent):
    """Estimate H-wing VTP tip mass properties relative to the wing elastic axis.

    The spanwise beam model represents one half wing. For the SpaJeti H-wing,
    one half wing carries two mirrored VTP panels at its tip: one up and one
    down. Their vertical static offsets cancel, but their pitch inertias add.
    """

    def setup(self):
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.ELASTIC_AXIS_FRACTION, val=0.375)

        self.add_input(Aircraft.VerticalTail.MASS, val=0.15, units='kg')
        self.add_input(Aircraft.VerticalTail.SPAN, val=0.30, units='m')
        self.add_input(Aircraft.VerticalTail.ROOT_CHORD, val=0.20, units='m')
        self.add_input(Aircraft.VerticalTail.TAPER_RATIO, val=1.0)
        self.add_input(Aircraft.VerticalTail.SWEEP, val=0.0, units='deg')
        self.add_input(AE.VTP_CHORDWISE_CG_FRACTION, val=0.40)
        self.add_input(AE.VTP_ROOT_LE_X_OFFSET, val=0.0, units='m')
        self.add_input(AE.VTP_AREAL_DENSITY, val=1.2, units='kg/m**2')
        self.add_input(AE.VTP_TIP_PANEL_COUNT, val=2.0, units='unitless')

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
        wing_area = inputs[Aircraft.Wing.AREA]
        wing_span = inputs[Aircraft.Wing.SPAN]
        wing_taper = inputs[Aircraft.Wing.TAPER_RATIO]
        elastic_axis = inputs[AE.ELASTIC_AXIS_FRACTION]

        vtp_span = inputs[Aircraft.VerticalTail.SPAN]
        vtp_root_chord = inputs[Aircraft.VerticalTail.ROOT_CHORD]
        vtp_taper = inputs[Aircraft.VerticalTail.TAPER_RATIO]
        vtp_sweep = inputs[Aircraft.VerticalTail.SWEEP]
        chordwise_cg = inputs[AE.VTP_CHORDWISE_CG_FRACTION]
        root_le_x_offset = inputs[AE.VTP_ROOT_LE_X_OFFSET]
        areal_density = inputs[AE.VTP_AREAL_DENSITY]
        panel_count = inputs[AE.VTP_TIP_PANEL_COUNT]

        wing_root_chord = 2.0 * wing_area / (wing_span * (1.0 + wing_taper))
        wing_tip_chord = wing_root_chord * wing_taper

        z_centroid_abs = (
            vtp_span / 3.0 * (1.0 + 2.0 * vtp_taper) / (1.0 + vtp_taper)
        )
        vtp_mac = (
            2.0
            / 3.0
            * vtp_root_chord
            * (1.0 + vtp_taper + vtp_taper**2)
            / (1.0 + vtp_taper)
        )

        x_c4 = z_centroid_abs * np.tan(vtp_sweep * (np.pi / 180.0))
        x_cg = root_le_x_offset + x_c4 + (chordwise_cg - 0.25) * vtp_mac
        x_ea_tip = elastic_axis * wing_tip_chord
        dx_to_ea = x_cg - x_ea_tip

        panel_area = vtp_span * vtp_root_chord * (1.0 + vtp_taper) / 2.0
        panel_mass = panel_area * areal_density
        vtp_mass = panel_mass * panel_count

        # Approximate each trapezoidal panel as a plate with MAC by span
        # dimensions, then use the parallel-axis theorem to the wing EA.
        panel_own_inertia_y = panel_mass * (vtp_mac**2 + vtp_span**2) / 12.0
        pitch_inertia_total = panel_count * (
            panel_own_inertia_y + panel_mass * (dx_to_ea**2 + z_centroid_abs**2)
        )

        # Guard: zero wing span would cause division by zero.
        wing_span_safe = np.maximum(wing_span, 1.0e-12)
        equivalent_mass_per_span = vtp_mass / wing_span_safe
        pitch_inertia = pitch_inertia_total / wing_span_safe
        static_unbalance = equivalent_mass_per_span * dx_to_ea

        outputs[AE.VTP_TIP_MASS] = vtp_mass
        outputs[AE.VTP_TIP_PITCH_INERTIA] = pitch_inertia_total
        outputs[AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN] = equivalent_mass_per_span
        outputs[AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN] = pitch_inertia
        outputs[AE.VTP_PITCH_STATIC_UNBALANCE] = static_unbalance
        outputs[AE.VTP_CG_X_FROM_WING_TIP_LE] = x_cg
        outputs[AE.VTP_CG_Z_ABS] = z_centroid_abs
        outputs[AE.VTP_DX_TO_ELASTIC_AXIS] = dx_to_ea
