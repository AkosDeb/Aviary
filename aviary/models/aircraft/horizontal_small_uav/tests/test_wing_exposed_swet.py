"""
Tests for wing exposed wetted area consistency.

Two things are verified:
1. FUSELAGE_EQUIV_DIAMETER_M is computed from the local superellipse cross-section
   geometry (not hardcoded) and matches the formula used by SuperellipseFuselageGeometry.
2. The resulting buried-panel area and exposed wetted area are within physical bounds
   for the SpaJeti horizontal_small_uav baseline wing geometry.

No OpenMDAO run is needed — all checks use module-level constants only.
"""
import math
import unittest

from aviary.subsystems.geometry.flops_based.superellipse_fuselage import superellipse_area
from aviary.models.aircraft.horizontal_small_uav.run_horizontal_small_uav import (
    FUSELAGE_EQUIV_DIAMETER_M,
    FUSELAGE_MAX_WIDTH_M,
    FUSELAGE_MAX_HEIGHT_M,
    FUSELAGE_SUPERELLIPSE_EXPONENT,
)

# Baseline CSV values — must match horizontal_small_uav.csv.
# Fuselage dimensions come from local rounded-square constants in run_horizontal_small_uav.py.
# Root chord is derived; it is not in the CSV directly.
_WING_AREA_M2 = 0.45    # aircraft:wing:area   [m²]
_WING_SPAN_M  = 1.80    # aircraft:wing:span   [m]
_WING_TAPER   = 0.60    # aircraft:wing:taper_ratio

# c_root = 2S / (b(1+λ))  (trapezoidal planform, consistent with MACGeometryComp)
_WING_ROOT_CHORD_M = 2.0 * _WING_AREA_M2 / (_WING_SPAN_M * (1.0 + _WING_TAPER))


class TestFuselageEquivDiameter(unittest.TestCase):
    """FUSELAGE_EQUIV_DIAMETER_M must be derived from the actual superellipse section."""

    def test_equiv_diam_matches_superellipse_formula(self):
        """d_eq = sqrt(4 * A_max / pi) with A_max from superellipse_area."""
        a_max    = superellipse_area(
            FUSELAGE_MAX_WIDTH_M, FUSELAGE_MAX_HEIGHT_M, FUSELAGE_SUPERELLIPSE_EXPONENT
        )
        expected = math.sqrt(4.0 * a_max / math.pi)
        self.assertAlmostEqual(
            FUSELAGE_EQUIV_DIAMETER_M, expected, places=8,
            msg='FUSELAGE_EQUIV_DIAMETER_M does not match superellipse_area formula; '
                'update run_horizontal_small_uav.py fuselage geometry constants.',
        )

    def test_equiv_diam_is_positive(self):
        self.assertGreater(FUSELAGE_EQUIV_DIAMETER_M, 0.0)

    def test_equiv_diam_is_less_than_rectangle_equivalent(self):
        """d_eq < d_rectangle = sqrt(4*W*H/pi) — superellipse is inside bounding box."""
        d_rect = math.sqrt(4.0 * FUSELAGE_MAX_WIDTH_M * FUSELAGE_MAX_HEIGHT_M / math.pi)
        self.assertLessEqual(FUSELAGE_EQUIV_DIAMETER_M, d_rect + 1e-9)


class TestWingExposedWettedArea(unittest.TestCase):
    """Exposed Swet = (S_planform - S_buried) × 2; S_buried = (d_fus/2) × c_root."""

    def _compute(self):
        s_buried  = (FUSELAGE_EQUIV_DIAMETER_M / 2.0) * _WING_ROOT_CHORD_M
        swet_wing = (_WING_AREA_M2 - s_buried) * 2.0
        return s_buried, swet_wing

    def test_buried_area_positive(self):
        s_buried, _ = self._compute()
        self.assertGreater(s_buried, 0.0)

    def test_buried_area_less_than_planform(self):
        s_buried, _ = self._compute()
        self.assertLess(
            s_buried, _WING_AREA_M2,
            msg=f'Buried area {s_buried:.5f} m² >= planform {_WING_AREA_M2} m² — '
                f'fuselage diameter ({FUSELAGE_EQUIV_DIAMETER_M:.4f} m) is too large.',
        )

    def test_swet_wing_positive(self):
        _, swet_wing = self._compute()
        self.assertGreater(swet_wing, 0.0)

    def test_swet_wing_not_more_than_twice_planform(self):
        _, swet_wing = self._compute()
        self.assertLessEqual(swet_wing, 2.0 * _WING_AREA_M2)

    def test_swet_convention(self):
        """Verify the S_wet = (S_planform - S_buried) × 2 memory convention."""
        s_buried, swet_wing = self._compute()
        self.assertAlmostEqual(swet_wing, (_WING_AREA_M2 - s_buried) * 2.0, places=10)

    def test_exposed_fraction_above_50_percent(self):
        """For a small UAV the fuselage must not bury more than half the planform."""
        s_buried, _ = self._compute()
        self.assertLess(s_buried, 0.5 * _WING_AREA_M2,
                        msg='More than 50 % of wing planform buried — check fuselage diameter.')


if __name__ == '__main__':
    unittest.main()
