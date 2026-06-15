import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.materials.material_selector import (
    MaterialSelector,
)
from aviary.models.external_subsystems.aeroelasticity.model.constant_lift_loads import (
    ConstantLiftLoads,
)
from aviary.models.external_subsystems.aeroelasticity.model.flutter import QuasiSteadyFlutterScreen
from aviary.models.external_subsystems.aeroelasticity.model.pk_flutter import PKFlutterAnalysis
from aviary.models.external_subsystems.aeroelasticity.model.static_aeroelastic import (
    StaticAeroelastic,
)
from aviary.models.external_subsystems.aeroelasticity.model.strength_margins import (
    StrengthMargins,
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

        self.add_subsystem(
            'material',
            MaterialSelector(material_name=self.options['material_name']),
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'structure',
            WingboxStructuralEstimate(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )
        self.add_subsystem(
            'vtp_inertia',
            VTPTipInertia(),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
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
