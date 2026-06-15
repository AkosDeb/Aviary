import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal
from openmdao.utils.testing_utils import use_tempdirs

from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    SuperellipseFuselageGeometry,
    superellipse_area,
)


@use_tempdirs
class SuperellipseFuselageGeometryTest(unittest.TestCase):
    def test_superellipse_area_matches_ellipse_at_n_equal_2(self):
        assert_near_equal(
            superellipse_area(width=0.20, height=0.10, exponent=2.0),
            3.141592653589793 * 0.10 * 0.05,
            1e-12,
        )

    def test_geometry_outputs_are_reasonable_for_spajeti_shape(self):
        prob = om.Problem()
        prob.model.add_subsystem('fus', SuperellipseFuselageGeometry(), promotes=['*'])
        prob.setup()

        prob.set_val('fuselage_length', 2.0, units='m')
        prob.set_val('max_width', 0.15, units='m')
        prob.set_val('max_height', 0.15, units='m')
        prob.set_val('nose_length_fraction', 0.20)
        prob.set_val('tail_length_fraction', 0.35)
        prob.set_val('base_width_fraction', 0.20)
        prob.set_val('base_height_fraction', 0.20)
        prob.set_val('superellipse_exponent', 4.0)

        prob.run_model()

        assert_near_equal(prob.get_val('fuselage_planform_area', units='m**2'), 0.228, 1e-12)
        assert_near_equal(prob.get_val('fuselage_base_area', units='m**2'), 0.0008343336047856176, 1e-12)
        assert_near_equal(prob.get_val('fuselage_base_diameter', units='m'), 0.03259304433639925, 1e-12)
        assert_near_equal(prob.get_val('fuselage_wetted_area', units='m**2'), 0.8034338150771664, 1e-12)
        assert_near_equal(prob.get_val('fuselage_equivalent_diameter', units='m'), 0.16296522168199623, 1e-12)
        assert_near_equal(prob.get_val('fuselage_fineness_ratio'), 12.27255717114121, 1e-12)
        assert_near_equal(prob.get_val('fuselage_volume', units='m**3'), 0.02826245970914936, 1e-12)
        assert_near_equal(prob.get_val('fuselage_centroid_x', units='m'), 0.9367255425881771, 1e-12)

    def test_geometry_changes_with_base_fraction(self):
        prob = om.Problem()
        prob.model.add_subsystem('fus', SuperellipseFuselageGeometry(), promotes=['*'])
        prob.setup()
        prob.run_model()

        base_area_small = float(prob.get_val('fuselage_base_area', units='m**2')[0])
        wetted_small = float(prob.get_val('fuselage_wetted_area', units='m**2')[0])

        prob.set_val('base_width_fraction', 0.5)
        prob.set_val('base_height_fraction', 0.5)
        prob.run_model()

        self.assertGreater(prob.get_val('fuselage_base_area', units='m**2')[0], base_area_small)
        self.assertGreater(prob.get_val('fuselage_wetted_area', units='m**2')[0], wetted_small)

    def test_invalid_fraction_sum_raises(self):
        prob = om.Problem()
        prob.model.add_subsystem('fus', SuperellipseFuselageGeometry(), promotes=['*'])
        prob.setup()
        prob.set_val('nose_length_fraction', 0.7)
        prob.set_val('tail_length_fraction', 0.5)

        with self.assertRaises(ValueError):
            prob.run_model()


if __name__ == '__main__':
    unittest.main()
