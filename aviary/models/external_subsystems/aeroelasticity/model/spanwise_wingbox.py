import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class SpanwiseWingboxProperties(om.ExplicitComponent):
    """Spanwise thin-wall wingbox properties for the half wing.

    This component is the first Step-3 bridge from a single equivalent wingbox to
    a station-based beam model. It evaluates the same closed rectangular cell
    formulas as ``WingboxStructuralEstimate`` at spanwise stations from root to
    tip, using local trapezoidal chord.

    It is intentionally analysis-only for now: downstream static/flutter
    components still consume the old equivalent scalar properties until the beam
    model is ready.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0, units='unitless')
        self.add_input(AE.STRUCTURAL_THICKNESS_TO_CHORD, val=0.12, units='unitless')
        self.add_input(AE.FRONT_SPAR_FRACTION, val=0.15, units='unitless')
        self.add_input(AE.REAR_SPAR_FRACTION, val=0.60, units='unitless')
        self.add_input(AE.SKIN_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.SPAR_THICKNESS, val=0.0015, units='m')
        self.add_input(AE.BOX_HEIGHT_FRACTION, val=0.80, units='unitless')
        self.add_input(AE.YOUNGS_MODULUS, val=70.0e9, units='Pa')
        self.add_input(AE.SHEAR_MODULUS, val=27.0e9, units='Pa')
        self.add_input(AE.MATERIAL_DENSITY, val=2700.0, units='kg/m**3')

        self.add_output(AE.SPANWISE_STATIONS, val=np.zeros(n), units='m')
        self.add_output(AE.SPANWISE_CHORD, val=np.ones(n), units='m')
        self.add_output(AE.SPANWISE_WINGBOX_WIDTH, val=np.ones(n), units='m')
        self.add_output(AE.SPANWISE_WINGBOX_HEIGHT, val=np.ones(n), units='m')
        self.add_output(AE.SPANWISE_BENDING_STIFFNESS, val=np.ones(n), units='N*m**2')
        self.add_output(AE.SPANWISE_TORSIONAL_RIGIDITY, val=np.ones(n), units='N*m**2')
        self.add_output(AE.SPANWISE_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_output(
            AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN,
            val=np.ones(n),
            units='kg*m',
        )

        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        area = inputs[Aircraft.Wing.AREA]
        span = inputs[Aircraft.Wing.SPAN]
        taper = inputs[Aircraft.Wing.TAPER_RATIO]
        tc = inputs[AE.STRUCTURAL_THICKNESS_TO_CHORD]
        front_spar = inputs[AE.FRONT_SPAR_FRACTION]
        rear_spar = inputs[AE.REAR_SPAR_FRACTION]
        skin_thickness = inputs[AE.SKIN_THICKNESS]
        spar_thickness = inputs[AE.SPAR_THICKNESS]
        box_height_fraction = inputs[AE.BOX_HEIGHT_FRACTION]
        youngs_modulus = inputs[AE.YOUNGS_MODULUS]
        shear_modulus = inputs[AE.SHEAR_MODULUS]
        density = inputs[AE.MATERIAL_DENSITY]

        n = self.options['num_stations']
        semispan = 0.5 * span
        semispan_safe = np.maximum(semispan, 1.0e-12)
        y = np.linspace(0.0, 1.0, n) * semispan

        root_chord = 2.0 * area / (span * (1.0 + taper))
        tip_chord = root_chord * taper
        chord = root_chord + (tip_chord - root_chord) * y / semispan_safe

        box_width = (rear_spar - front_spar) * chord
        box_height = box_height_fraction * tc * chord
        enclosed_area = box_width * box_height

        bending_inertia = (
            2.0 * box_width * skin_thickness * (0.5 * box_height) ** 2
            + 2.0 * spar_thickness * box_height**3 / 12.0
        )

        skin_t_safe = np.maximum(skin_thickness, 1.0e-12)
        spar_t_safe = np.maximum(spar_thickness, 1.0e-12)
        wall_flexibility = 2.0 * box_width / skin_t_safe + 2.0 * box_height / spar_t_safe
        torsion_constant = 4.0 * enclosed_area**2 / np.maximum(wall_flexibility, 1.0e-30)

        wall_area = 2.0 * box_width * skin_thickness + 2.0 * box_height * spar_thickness
        mass_per_span = density * wall_area

        m_skin = density * box_width * skin_thickness
        m_spar = density * box_height * spar_thickness
        pitch_inertia_per_span = (
            2.0 * m_skin * (box_width**2 / 12.0 + (0.5 * box_height)**2)
            + 2.0 * m_spar * ((0.5 * box_width)**2 + box_height**2 / 12.0)
        )

        outputs[AE.SPANWISE_STATIONS] = y
        outputs[AE.SPANWISE_CHORD] = chord
        outputs[AE.SPANWISE_WINGBOX_WIDTH] = box_width
        outputs[AE.SPANWISE_WINGBOX_HEIGHT] = box_height
        outputs[AE.SPANWISE_BENDING_STIFFNESS] = youngs_modulus * bending_inertia
        outputs[AE.SPANWISE_TORSIONAL_RIGIDITY] = shear_modulus * torsion_constant
        outputs[AE.SPANWISE_MASS_PER_UNIT_SPAN] = mass_per_span
        outputs[AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN] = pitch_inertia_per_span
