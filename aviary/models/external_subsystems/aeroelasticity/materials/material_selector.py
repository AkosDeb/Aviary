import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.materials.catalog import get_material
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class MaterialSelector(om.ExplicitComponent):
    """Outputs material properties from the aeroelasticity material catalog."""

    def initialize(self):
        self.options.declare('material_name', default='aluminum_6061_t6', types=str)

    def setup(self):
        self.add_output(AE.YOUNGS_MODULUS, val=68.9e9, units='Pa')
        self.add_output(AE.SHEAR_MODULUS, val=26.0e9, units='Pa')
        self.add_output(AE.MATERIAL_DENSITY, val=2700.0, units='kg/m**3')
        self.add_output(AE.TENSILE_ALLOWABLE, val=140.0e6, units='Pa')
        self.add_output(AE.COMPRESSIVE_ALLOWABLE, val=140.0e6, units='Pa')
        self.add_output(AE.SHEAR_ALLOWABLE, val=90.0e6, units='Pa')

    def compute(self, inputs, outputs):
        material = get_material(self.options['material_name'])

        outputs[AE.YOUNGS_MODULUS] = material.youngs_modulus
        outputs[AE.SHEAR_MODULUS] = material.shear_modulus
        outputs[AE.MATERIAL_DENSITY] = material.density
        outputs[AE.TENSILE_ALLOWABLE] = material.tensile_allowable
        outputs[AE.COMPRESSIVE_ALLOWABLE] = material.compressive_allowable
        outputs[AE.SHEAR_ALLOWABLE] = material.shear_allowable
