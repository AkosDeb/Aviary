import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.materials.catalog import (
    MATERIALS,
    get_material,
)
from aviary.models.external_subsystems.aeroelasticity.materials.material_selector import (
    MaterialSelector,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class TestMaterialCatalog(unittest.TestCase):
    def test_catalog_contains_requested_materials(self):
        expected = {
            'aluminum_6061_t6',
            'gfrp_quasi_iso',
            'cfrp_quasi_iso',
            'injection_molded_gfrp',
            'printed_pa_cf',
            'printed_peek_cf',
            'printed_pekk_cf',
        }
        self.assertTrue(expected.issubset(MATERIALS))

    def test_unknown_material_raises_useful_error(self):
        with self.assertRaisesRegex(ValueError, 'Valid choices'):
            get_material('plywood_from_the_moon')


class TestMaterialSelector(unittest.TestCase):
    def test_outputs_selected_material_properties(self):
        prob = om.Problem(name='test_material_selector', reports=False)
        prob.model.add_subsystem(
            'mat',
            MaterialSelector(material_name='cfrp_quasi_iso'),
        )
        prob.setup()
        prob.run_model()

        material = get_material('cfrp_quasi_iso')
        assert_near_equal(
            prob.get_val(f'mat.{AE.YOUNGS_MODULUS}', units='Pa'),
            material.youngs_modulus,
        )
        assert_near_equal(
            prob.get_val(f'mat.{AE.SHEAR_MODULUS}', units='Pa'),
            material.shear_modulus,
        )
        assert_near_equal(
            prob.get_val(f'mat.{AE.MATERIAL_DENSITY}', units='kg/m**3'),
            material.density,
        )
        assert_near_equal(
            prob.get_val(f'mat.{AE.TENSILE_ALLOWABLE}', units='Pa'),
            material.tensile_allowable,
        )
        assert_near_equal(
            prob.get_val(f'mat.{AE.COMPRESSIVE_ALLOWABLE}', units='Pa'),
            material.compressive_allowable,
        )
        assert_near_equal(
            prob.get_val(f'mat.{AE.SHEAR_ALLOWABLE}', units='Pa'),
            material.shear_allowable,
        )


if __name__ == '__main__':
    unittest.main()
