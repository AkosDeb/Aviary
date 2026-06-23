import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal
from openmdao.utils.testing_utils import use_tempdirs

from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    SuperellipseFuselageGeometry,
    _fuselage_x_distribution,
    superellipse_exponent_distribution,
    superellipse_width_height_distribution,
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

        assert_near_equal(prob.get_val('fuselage_planform_area', units='m**2'), 0.23781487181263372, 1e-12)
        assert_near_equal(prob.get_val('fuselage_base_area', units='m**2'), 0.0008343336047856176, 1e-12)
        assert_near_equal(prob.get_val('fuselage_base_diameter', units='m'), 0.03259304433639925, 1e-12)
        assert_near_equal(prob.get_val('fuselage_wetted_area', units='m**2'), 0.8374241284621304, 1e-12)
        assert_near_equal(prob.get_val('fuselage_equivalent_diameter', units='m'), 0.16296522168199623, 1e-12)
        assert_near_equal(prob.get_val('fuselage_fineness_ratio'), 12.27255717114121, 1e-12)
        assert_near_equal(prob.get_val('fuselage_volume', units='m**3'), 0.029335170052871506, 1e-12)
        assert_near_equal(prob.get_val('fuselage_centroid_x', units='m'), 0.9079262422314408, 1e-12)

    def test_ellipsoid_nose_uses_circular_sections_and_blended_exponent(self):
        x_norm = [0.0, 0.0375, 0.075, 0.125, 0.175]
        widths, heights = superellipse_width_height_distribution(
            x_norm,
            0.15,
            0.15,
            0.20,
            0.35,
            0.20,
            0.20,
            nose_type='ellipsoid',
            nose_aspect_ratio=2.0,
            fuselage_length=2.0,
        )
        exponents = superellipse_exponent_distribution(
            x_norm,
            0.20,
            0.35,
            4.0,
            nose_type='ellipsoid',
            nose_aspect_ratio=2.0,
            max_width=0.15,
            fuselage_length=2.0,
        )

        self.assertEqual(widths[0], 0.0)
        assert_near_equal(widths[1], heights[1], 1e-12)
        assert_near_equal(widths[2], 0.15, 1e-12)
        assert_near_equal(heights[2], 0.15, 1e-12)
        assert_near_equal(exponents[0], 2.0, 1e-12)
        assert_near_equal(exponents[2], 2.0, 1e-12)
        assert_near_equal(exponents[3], 3.0, 1e-12)
        assert_near_equal(exponents[4], 4.0, 1e-12)

    def test_ellipsoid_nose_transition_grid_is_densified(self):
        uniform = _fuselage_x_distribution(
            81,
            'power_law',
            2.0,
            0.15,
            2.0,
        )
        dense = _fuselage_x_distribution(
            81,
            'ellipsoid',
            2.0,
            0.15,
            2.0,
        )

        nose = 2.0 * 0.15 / (2.0 * 2.0)
        blend_end = nose + 0.10
        uniform_count = ((uniform > nose) & (uniform < blend_end)).sum()
        dense_count = ((dense > nose) & (dense < blend_end)).sum()

        self.assertGreater(len(dense), len(uniform))
        self.assertGreaterEqual(dense_count, 2 * uniform_count)

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

    def test_equivalent_diameter_uses_flared_base_when_it_is_largest_section(self):
        prob = om.Problem()
        prob.model.add_subsystem('fus', SuperellipseFuselageGeometry(), promotes=['*'])
        prob.setup()

        prob.set_val('max_width', 0.30, units='m')
        prob.set_val('max_height', 0.30, units='m')
        prob.set_val('base_width_fraction', 0.35 / 0.30)
        prob.set_val('base_height_fraction', 0.35 / 0.30)
        prob.run_model()

        assert_near_equal(
            prob.get_val('fuselage_max_cross_section_area', units='m**2'),
            prob.get_val('fuselage_base_area', units='m**2'),
            1e-12,
        )

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
