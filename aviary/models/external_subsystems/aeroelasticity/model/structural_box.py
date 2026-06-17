import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class WingboxStructuralEstimate(om.ExplicitComponent):
    """Thin-walled closed wingbox estimate for preliminary aeroelastic constraints.

    The model uses an equivalent chord based on trapezoidal wing area/span and a closed
    rectangular cell between the front and rear spars. Torsion uses the Bredt-Batho
    thin-walled single-cell approximation.
    """

    def setup(self):
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.STRUCTURAL_THICKNESS_TO_CHORD, val=0.12, units='unitless')

        self.add_input(AE.FRONT_SPAR_FRACTION, val=0.15, units='unitless')
        self.add_input(AE.REAR_SPAR_FRACTION, val=0.60, units='unitless')
        self.add_input(AE.AERODYNAMIC_CENTER_FRACTION, val=0.25, units='unitless')
        self.add_input(AE.CONTROL_CHORD_FRACTION, val=0.25, units='unitless')
        self.add_input(AE.SKIN_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.SPAR_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.BOX_HEIGHT_FRACTION, val=0.80, units='unitless')
        self.add_input(AE.YOUNGS_MODULUS, val=70.0e9, units='Pa')
        self.add_input(AE.SHEAR_MODULUS, val=27.0e9, units='Pa')
        self.add_input(AE.MATERIAL_DENSITY, val=2700.0, units='kg/m**3')
        self.add_input(AE.CONTROL_CG_FROM_HINGE_FRACTION, val=0.5, units='unitless')

        self.add_output(AE.ELASTIC_AXIS_FRACTION, val=0.375)
        self.add_output(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125)
        self.add_output(AE.CONTROL_HINGE_FRACTION, val=0.75)
        self.add_output(AE.WINGBOX_WIDTH, val=0.2, units='m')
        self.add_output(AE.WINGBOX_HEIGHT, val=0.05, units='m')
        self.add_output(AE.WINGBOX_AREA, val=0.01, units='m**2')
        self.add_output(AE.BENDING_INERTIA, val=1.0e-6, units='m**4')
        self.add_output(AE.TORSION_CONSTANT, val=1.0e-6, units='m**4')
        self.add_output(AE.BENDING_STIFFNESS, val=1.0e5, units='N*m**2')
        self.add_output(AE.TORSIONAL_RIGIDITY, val=1.0e5, units='N*m**2')
        self.add_output(AE.TORSIONAL_STIFFNESS, val=1.0e5, units='N*m/rad')
        self.add_output(AE.BENDING_STIFFNESS_PLUNGE, val=1.0e5, units='N/m')
        self.add_output(AE.MASS_PER_UNIT_SPAN, val=1.0, units='kg/m')
        self.add_output(AE.PITCH_INERTIA_PER_UNIT_SPAN, val=0.01, units='kg*m')
        self.add_output(AE.CONTROL_INERTIA_PER_UNIT_SPAN, val=0.001, units='kg*m')
        self.add_output(AE.CONTROL_STATIC_UNBALANCE, val=0.0, units='kg')

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        area = inputs[Aircraft.Wing.AREA]
        span = inputs[Aircraft.Wing.SPAN]
        taper = inputs[Aircraft.Wing.TAPER_RATIO]
        tc = inputs[AE.STRUCTURAL_THICKNESS_TO_CHORD]

        front_spar = inputs[AE.FRONT_SPAR_FRACTION]
        rear_spar = inputs[AE.REAR_SPAR_FRACTION]
        aerodynamic_center = inputs[AE.AERODYNAMIC_CENTER_FRACTION]
        control_chord = inputs[AE.CONTROL_CHORD_FRACTION]
        skin_thickness = inputs[AE.SKIN_THICKNESS]
        spar_thickness = inputs[AE.SPAR_THICKNESS]
        box_height_fraction = inputs[AE.BOX_HEIGHT_FRACTION]
        youngs_modulus = inputs[AE.YOUNGS_MODULUS]
        shear_modulus = inputs[AE.SHEAR_MODULUS]
        density = inputs[AE.MATERIAL_DENSITY]

        root_chord = 2.0 * area / (span * (1.0 + taper))
        chord = (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / (1.0 + taper)
        # Guard: zero semispan would cause division by zero in stiffness outputs.
        semispan = np.maximum(0.5 * span, 1.0e-12)

        elastic_axis = 0.5 * (front_spar + rear_spar)
        hinge = 1.0 - control_chord
        box_width = (rear_spar - front_spar) * chord
        box_height = box_height_fraction * tc * chord
        enclosed_area = box_width * box_height

        bending_inertia = (
            2.0 * box_width * skin_thickness * (0.5 * box_height) ** 2
            + 2.0 * spar_thickness * box_height**3 / 12.0
        )
        # Guard: zero wall thickness causes division by zero in Bredt-Batho formula.
        skin_t_safe = np.maximum(skin_thickness, 1.0e-12)
        spar_t_safe = np.maximum(spar_thickness, 1.0e-12)
        wall_flexibility = 2.0 * box_width / skin_t_safe + 2.0 * box_height / spar_t_safe
        # Guard: wall_flexibility = 0 only when box has zero dimensions (degenerate).
        wall_flexibility_safe = np.maximum(wall_flexibility, 1.0e-30)
        torsion_constant = 4.0 * enclosed_area**2 / wall_flexibility_safe
        wall_area = 2.0 * box_width * skin_thickness + 2.0 * box_height * spar_thickness
        mass_per_span = density * wall_area
        control_radius = 0.5 * (1.0 - hinge) * chord

        EI = youngs_modulus * bending_inertia
        GJ = shear_modulus * torsion_constant

        outputs[AE.ELASTIC_AXIS_FRACTION] = elastic_axis
        outputs[AE.AERO_CENTER_TO_EA_FRACTION] = elastic_axis - aerodynamic_center
        outputs[AE.CONTROL_HINGE_FRACTION] = hinge
        outputs[AE.WINGBOX_WIDTH] = box_width
        outputs[AE.WINGBOX_HEIGHT] = box_height
        outputs[AE.WINGBOX_AREA] = enclosed_area
        outputs[AE.BENDING_INERTIA] = bending_inertia
        outputs[AE.TORSION_CONSTANT] = torsion_constant
        outputs[AE.BENDING_STIFFNESS] = EI
        outputs[AE.TORSIONAL_RIGIDITY] = GJ
        outputs[AE.TORSIONAL_STIFFNESS] = GJ / semispan
        outputs[AE.BENDING_STIFFNESS_PLUNGE] = 3.0 * EI / semispan**3
        outputs[AE.MASS_PER_UNIT_SPAN] = mass_per_span
        # Top/bottom skins at z = ±box_height/2; left/right spars at x = ±box_width/2.
        # Elastic axis coincides with the box centroid for this symmetric section.
        m_skin = density * box_width * skin_thickness
        m_spar = density * box_height * spar_thickness
        outputs[AE.PITCH_INERTIA_PER_UNIT_SPAN] = (
            2.0 * m_skin * (box_width**2 / 12.0 + (0.5 * box_height)**2)
            + 2.0 * m_spar * ((0.5 * box_width)**2 + box_height**2 / 12.0)
        )
        outputs[AE.CONTROL_INERTIA_PER_UNIT_SPAN] = (
            0.25 * mass_per_span * control_radius**2
        )
        control_cg = inputs[AE.CONTROL_CG_FROM_HINGE_FRACTION]
        # Two-skin thin-wall model: m_ctrl = 4*density*skin_t*r, cg_offset = 2*cg*r.
        outputs[AE.CONTROL_STATIC_UNBALANCE] = (
            density * 8.0 * skin_thickness * control_cg * control_radius**2
        )
