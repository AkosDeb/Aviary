import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.beam_modal_flutter import (
    BeamModalFlutter,
)
from aviary.models.external_subsystems.aeroelasticity.materials.material_selector import (
    MaterialSelector,
)
from aviary.models.external_subsystems.aeroelasticity.model.constant_lift_loads import (
    ConstantLiftLoads,
)
from aviary.models.external_subsystems.aeroelasticity.model.schrenk_lift_distribution import (
    SchrenkLiftDistribution,
)
from aviary.models.external_subsystems.aeroelasticity.model.spanwise_beam_response import (
    SpanwiseBeamResponse,
)
from aviary.models.external_subsystems.aeroelasticity.model.spanwise_equivalent_properties import (
    SpanwiseEquivalentProperties,
)
from aviary.models.external_subsystems.aeroelasticity.model.spanwise_mass_distribution import (
    SpanwiseMassDistribution,
)
from aviary.models.external_subsystems.aeroelasticity.model.flutter import QuasiSteadyFlutterScreen
from aviary.models.external_subsystems.aeroelasticity.model.pk_flutter import PKFlutterAnalysis
from aviary.models.external_subsystems.aeroelasticity.model.static_aeroelastic import (
    StaticAeroelastic,
)
from aviary.models.external_subsystems.aeroelasticity.model.strength_margins import (
    StrengthMargins,
)
from aviary.models.external_subsystems.aeroelasticity.model.spanwise_wingbox import (
    SpanwiseWingboxProperties,
)
from aviary.models.external_subsystems.aeroelasticity.model.structural_box import (
    WingboxStructuralEstimate,
)
from aviary.models.external_subsystems.aeroelasticity.model.vtp_inertia import VTPTipInertia
from aviary.subsystems.subsystem_builder import SubsystemBuilder
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class AeroelasticityGroup(om.Group):
    """Premission aeroelasticity group for structural estimate and static checks."""

    def initialize(self):
        self.options.declare('material_name', default='aluminum_6061_t6', types=str)

    def setup(self):
        self.set_input_defaults(AE.DESIGN_SPEED, val=550.0 / 3.6, units='m/s')
        self.set_input_defaults(
            AE.REQUIRED_SPEED,
            val=1.15 * 550.0 / 3.6,
            units='m/s',
        )
        self.set_input_defaults(AE.VTP_AREAL_DENSITY, val=1.2, units='kg/m**2')
        self.set_input_defaults(AE.VTP_TIP_PANEL_COUNT, val=2.0, units='unitless')
        self.set_input_defaults(AE.ENGINE_SPAN_FRACTION, val=0.0, units='unitless')
        self.set_input_defaults(AE.ENGINE_X_OFFSET_TO_EA, val=0.0, units='m')
        self.set_input_defaults(AE.FUEL_SPAN_FRACTION, val=0.35, units='unitless')
        self.set_input_defaults(AE.FUEL_X_OFFSET_TO_EA, val=0.0, units='m')
        self.set_input_defaults(AE.SERVO_MASS, val=0.05, units='kg')
        self.set_input_defaults(AE.SERVO_SPAN_FRACTION, val=0.85, units='unitless')
        self.set_input_defaults(AE.SERVO_X_OFFSET_TO_EA, val=0.03, units='m')
        self.set_input_defaults(AE.ELEVON_MASS, val=0.15, units='kg')
        self.set_input_defaults(AE.ELEVON_SPAN_START_FRACTION, val=0.45, units='unitless')
        self.set_input_defaults(AE.ELEVON_SPAN_END_FRACTION, val=0.95, units='unitless')
        self.set_input_defaults(AE.ELEVON_CG_FRACTION, val=0.80, units='unitless')

        self.add_subsystem(
            'material',
            MaterialSelector(material_name=self.options['material_name']),
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'structure',
            WingboxStructuralEstimate(),
            promotes_inputs=['*'],
            promotes_outputs=[
                AE.ELASTIC_AXIS_FRACTION,
                AE.AERO_CENTER_TO_EA_FRACTION,
                AE.CONTROL_HINGE_FRACTION,
                AE.WINGBOX_WIDTH,
                AE.WINGBOX_HEIGHT,
                AE.WINGBOX_AREA,
                AE.BENDING_INERTIA,
                AE.TORSION_CONSTANT,
                AE.CONTROL_INERTIA_PER_UNIT_SPAN,
                AE.CONTROL_STATIC_UNBALANCE,
            ],
        )
        self.add_subsystem(
            'spanwise_structure',
            SpanwiseWingboxProperties(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'schrenk_loads',
            SchrenkLiftDistribution(),
            promotes_inputs=['*'],
            promotes_outputs=[
                AE.SCHRENK_LIFT_PER_UNIT_SPAN,
                AE.SCHRENK_SHEAR_FORCE,
                AE.SCHRENK_BENDING_MOMENT,
                AE.SCHRENK_TORQUE,
            ],
        )
        self.add_subsystem(
            'spanwise_beam',
            SpanwiseBeamResponse(),
            promotes_inputs=[
                AE.SPANWISE_STATIONS,
                AE.SCHRENK_BENDING_MOMENT,
                AE.SCHRENK_TORQUE,
                AE.SPANWISE_BENDING_STIFFNESS,
                AE.SPANWISE_TORSIONAL_RIGIDITY,
            ],
            promotes_outputs=[
                AE.SPANWISE_SLOPE,
                AE.SPANWISE_DEFLECTION,
                AE.SPANWISE_TWIST,
                AE.SPANWISE_TWIST_DEG,
                AE.SCHRENK_TIP_DEFLECTION,
                AE.SCHRENK_TIP_TWIST,
                AE.SCHRENK_TIP_TWIST_DEG,
            ],
        )
        self.add_subsystem(
            'vtp_inertia',
            VTPTipInertia(),
            promotes_inputs=['*'],
            promotes_outputs=[
                AE.VTP_TIP_MASS,
                AE.VTP_TIP_PITCH_INERTIA,
                AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN,
                AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN,
                AE.VTP_PITCH_STATIC_UNBALANCE,
                AE.VTP_CG_X_FROM_WING_TIP_LE,
                AE.VTP_CG_Z_ABS,
                AE.VTP_DX_TO_ELASTIC_AXIS,
            ],
        )
        self.add_subsystem(
            'spanwise_mass',
            SpanwiseMassDistribution(),
            promotes_inputs=[
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_CHORD,
                AE.SPANWISE_MASS_PER_UNIT_SPAN,
                AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN,
                AE.ELASTIC_AXIS_FRACTION,
                AE.VTP_TIP_MASS,
                AE.VTP_TIP_PITCH_INERTIA,
                AE.ENGINE_MASS,
                AE.ENGINE_SPAN_FRACTION,
                AE.ENGINE_X_OFFSET_TO_EA,
                AE.FUEL_MASS,
                AE.FUEL_SPAN_FRACTION,
                AE.FUEL_X_OFFSET_TO_EA,
                AE.SERVO_MASS,
                AE.SERVO_SPAN_FRACTION,
                AE.SERVO_X_OFFSET_TO_EA,
                AE.ELEVON_MASS,
                AE.ELEVON_SPAN_START_FRACTION,
                AE.ELEVON_SPAN_END_FRACTION,
                AE.ELEVON_CG_FRACTION,
            ],
            promotes_outputs=[
                AE.SPANWISE_ADDED_MASS_PER_UNIT_SPAN,
                AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN,
                AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
                AE.SPANWISE_TOTAL_HALF_WING_MASS,
            ],
        )
        self.add_subsystem(
            'spanwise_equivalent',
            SpanwiseEquivalentProperties(),
            promotes_inputs=[
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_BENDING_STIFFNESS,
                AE.SPANWISE_TORSIONAL_RIGIDITY,
                AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN,
                AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
            ],
            promotes_outputs=[
                AE.BENDING_STIFFNESS,
                AE.TORSIONAL_RIGIDITY,
                AE.TORSIONAL_STIFFNESS,
                AE.BENDING_STIFFNESS_PLUNGE,
                AE.MASS_PER_UNIT_SPAN,
                AE.PITCH_INERTIA_PER_UNIT_SPAN,
            ],
        )
        self.add_subsystem(
            'beam_modal_flutter',
            BeamModalFlutter(),
            promotes_inputs=[
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_CHORD,
                AE.SPANWISE_BENDING_STIFFNESS,
                AE.SPANWISE_TORSIONAL_RIGIDITY,
                AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN,
                AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
                AE.AIR_DENSITY,
                AE.DESIGN_SPEED,
                AE.REQUIRED_SPEED,
                AE.FLUTTER_MAX_SPEED,
                AE.STRUCTURAL_DAMPING_RATIO,
                AE.LIFT_CURVE_SLOPE,
                AE.AERO_CENTER_TO_EA_FRACTION,
                AE.CONTROL_MOMENT_ALPHA_DERIVATIVE,
                AE.ELEVON_SPAN_START_FRACTION,
                AE.ELEVON_SPAN_END_FRACTION,
                AE.CONTROL_INERTIA_PER_UNIT_SPAN,
                AE.CONTROL_STATIC_UNBALANCE,
                AE.CONTROL_STIFFNESS,
                AE.CONTROL_LIFT_DERIVATIVE,
                AE.CONTROL_MOMENT_DERIVATIVE,
                AE.HINGE_MOMENT_ALPHA_DERIVATIVE,
                AE.HINGE_MOMENT_CONTROL_DERIVATIVE,
                AE.CONTROL_HINGE_FRACTION,
            ],
            promotes_outputs=[
                AE.BEAM_MODAL_BENDING_FREQUENCY,
                AE.BEAM_MODAL_TORSION_FREQUENCY,
                AE.BEAM_MODAL_FLUTTER_SPEED,
                AE.BEAM_MODAL_FLUTTER_SPEED_MARGIN,
                AE.BEAM_MODAL_MAX_REAL_EIGENVALUE_AT_DESIGN,
                AE.BEAM_MODAL_CRITICAL_MODE,
                AE.BEAM_MODAL_PK_FLUTTER_SPEED,
                AE.BEAM_MODAL_PK_FLUTTER_FREQUENCY,
                AE.BEAM_MODAL_PK_FLUTTER_SPEED_MARGIN,
                AE.BEAM_MODAL_PK_MODE_DAMPING,
                AE.BEAM_MODAL_PK_MODE_FREQUENCY,
                AE.BEAM_MODAL_PK_CONVERGED,
                AE.BEAM_MODAL_CONTROL_FREQUENCY,
                AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED,
                AE.BEAM_MODAL_3DOF_PK_FLUTTER_FREQUENCY,
                AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN,
                AE.BEAM_MODAL_3DOF_PK_CONVERGED,
                AE.BEAM_MODAL_3DOF_PK_MODE_DAMPING,
                AE.BEAM_MODAL_3DOF_PK_MODE_FREQUENCY,
            ],
        )
        self.add_subsystem(
            'flutter',
            QuasiSteadyFlutterScreen(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'pk_flutter',
            PKFlutterAnalysis(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'static',
            StaticAeroelastic(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'constant_loads',
            ConstantLiftLoads(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'strength',
            StrengthMargins(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )


class AeroelasticityBuilder(SubsystemBuilder):
    """Builder for preliminary H-wing aeroelasticity checks."""

    _default_name = 'aeroelasticity'

    def __init__(self, name=None, meta_data=None, material_name='aluminum_6061_t6'):
        self.material_name = material_name
        super().__init__(name, meta_data)

    def build_pre_mission(self, aviary_inputs, subsystem_options=None):
        material_name = self.material_name
        if subsystem_options and 'material_name' in subsystem_options:
            material_name = subsystem_options['material_name']
        return AeroelasticityGroup(material_name=material_name)
