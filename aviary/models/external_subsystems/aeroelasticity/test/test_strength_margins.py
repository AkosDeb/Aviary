import unittest

import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.models.external_subsystems.aeroelasticity.model.constant_lift_loads import (
    ConstantLiftLoads,
)
from aviary.models.external_subsystems.aeroelasticity.model.strength_margins import (
    StrengthMargins,
)
from aviary.models.external_subsystems.aeroelasticity.model.structural_box import (
    WingboxStructuralEstimate,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class TestStrengthMargins(unittest.TestCase):
    def test_root_stress_and_margins(self):
        prob = om.Problem(name='test_strength_margins', reports=False)
        prob.model.add_subsystem('strength', StrengthMargins())
        prob.setup()

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 100.0, units='N*m')
        prob.set_val(f'strength.{AE.ROOT_TORQUE}', 20.0, units='N*m')
        prob.set_val(f'strength.{AE.BENDING_INERTIA}', 2.0e-6, units='m**4')
        prob.set_val(f'strength.{AE.WINGBOX_HEIGHT}', 0.10, units='m')
        prob.set_val(f'strength.{AE.WINGBOX_AREA}', 0.02, units='m**2')
        prob.set_val(f'strength.{AE.SKIN_THICKNESS}', 0.002, units='m')
        prob.set_val(f'strength.{AE.SPAR_THICKNESS}', 0.001, units='m')
        prob.set_val(f'strength.{AE.TENSILE_ALLOWABLE}', 150.0e6, units='Pa')
        prob.set_val(f'strength.{AE.COMPRESSIVE_ALLOWABLE}', 120.0e6, units='Pa')
        prob.set_val(f'strength.{AE.SHEAR_ALLOWABLE}', 60.0e6, units='Pa')
        prob.set_val(f'strength.{AE.STRENGTH_SAFETY_FACTOR}', 1.5)
        prob.run_model()

        bending_stress = 100.0 * 0.05 / 2.0e-6
        shear_flow = 20.0 / (2.0 * 0.02)
        torsion_stress = shear_flow / 0.001
        bending_margin = 120.0e6 / (1.5 * bending_stress) - 1.0
        torsion_margin = 60.0e6 / (1.5 * torsion_stress) - 1.0

        assert_near_equal(
            prob.get_val(f'strength.{AE.ROOT_BENDING_STRESS}', units='Pa'),
            bending_stress,
        )
        assert_near_equal(
            prob.get_val(f'strength.{AE.TORSION_SHEAR_STRESS}', units='Pa'),
            torsion_stress,
        )
        assert_near_equal(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}'), bending_margin)
        assert_near_equal(prob.get_val(f'strength.{AE.TORSION_STRENGTH_MARGIN}'), torsion_margin)
        assert_near_equal(prob.get_val(f'strength.{AE.MIN_STRENGTH_MARGIN}'), bending_margin)

    def test_margin_decreases_with_higher_load(self):
        prob = om.Problem(name='test_strength_margin_load_sensitivity', reports=False)
        prob.model.add_subsystem('strength', StrengthMargins())
        prob.setup()

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 50.0, units='N*m')
        prob.run_model()
        low_load_margin = float(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}')[0])

        prob.set_val(f'strength.{AE.ROOT_BENDING_MOMENT}', 100.0, units='N*m')
        prob.run_model()
        high_load_margin = float(prob.get_val(f'strength.{AE.BENDING_STRENGTH_MARGIN}')[0])

        self.assertLess(high_load_margin, low_load_margin)


class TestStrengthMarginsSpaJetiRegression(unittest.TestCase):
    """Full-chain regression for the SpaJeti baseline geometry.

    Chains WingboxStructuralEstimate → ConstantLiftLoads → StrengthMargins.

    Stress values pin the physics formulas and must stay tight.
    Margin checks are derived from those stresses using the same allowables set
    below, so this test passes when allowables are intentionally updated but
    fails if any load or stress formula changes unexpectedly.
    """

    def test_spajeti_baseline_stress_and_margins(self):
        prob = om.Problem(name='test_spajeti_strength_regression', reports=False)
        prob.model.add_subsystem('box', WingboxStructuralEstimate(), promotes=['*'])
        prob.model.add_subsystem('loads', ConstantLiftLoads(), promotes=['*'])
        prob.model.add_subsystem('strength', StrengthMargins(), promotes=['*'])
        prob.setup()

        # SpaJeti wing geometry
        prob.set_val(Aircraft.Wing.AREA, 0.8, units='m**2')
        prob.set_val(Aircraft.Wing.SPAN, 2.0, units='m')
        prob.set_val(Aircraft.Wing.TAPER_RATIO, 1.0)
        prob.set_val(Aircraft.Wing.THICKNESS_TO_CHORD, 0.12)

        # Manoeuvre design point
        prob.set_val(AE.DESIGN_SPEED, 100.0, units='m/s')
        prob.set_val(AE.LOAD_FACTOR, 3.5)
        prob.set_val(AE.LOAD_LIFT_COEFFICIENT, 0.8)

        # Material allowables (Al 2024-T3 representative)
        tensile = 140.0e6
        compressive = 140.0e6
        shear = 90.0e6
        sf = 1.5
        prob.set_val(AE.TENSILE_ALLOWABLE, tensile, units='Pa')
        prob.set_val(AE.COMPRESSIVE_ALLOWABLE, compressive, units='Pa')
        prob.set_val(AE.SHEAR_ALLOWABLE, shear, units='Pa')
        prob.set_val(AE.STRENGTH_SAFETY_FACTOR, sf)

        prob.run_model()

        # Expected stresses derived analytically from the same geometry and load.
        # Default spar fractions: front=0.15, rear=0.60, box_height_fraction=0.80.
        # Default wall thicknesses: skin_t = spar_t = 0.0015 m.
        chord = 2.0 * 0.8 / (2.0 * (1.0 + 1.0))          # 0.4 m (rectangular wing)
        box_width = (0.60 - 0.15) * chord                  # 0.18 m
        box_height = 0.80 * 0.12 * chord                   # 0.0384 m
        half_height = 0.5 * box_height
        enclosed_area = box_width * box_height
        skin_t = spar_t = 0.0015
        bending_inertia = (
            2.0 * box_width * skin_t * half_height**2
            + 2.0 * spar_t * box_height**3 / 12.0
        )
        e_frac = 0.5 * (0.15 + 0.60) - 0.25               # elastic_axis - aero_center = 0.125
        q_dyn = 0.5 * 1.225 * 100.0**2
        semispan = 1.0
        lift_per_span = 3.5 * q_dyn * 0.8 * 0.8 / 2.0
        root_bending = lift_per_span * semispan**2 / 2.0
        root_torque = lift_per_span * semispan * e_frac * chord

        expected_bending_stress = root_bending * half_height / bending_inertia
        shear_flow = abs(root_torque) / (2.0 * enclosed_area)
        expected_torsion_stress = max(shear_flow / skin_t, shear_flow / spar_t)

        bending_allowable = min(tensile, compressive)
        expected_bending_margin = bending_allowable / (sf * expected_bending_stress) - 1.0
        expected_torsion_margin = shear / (sf * expected_torsion_stress) - 1.0
        expected_min_margin = min(expected_bending_margin, expected_torsion_margin)

        assert_near_equal(
            prob.get_val(AE.ROOT_BENDING_STRESS, units='Pa'),
            expected_bending_stress,
            tolerance=1.0e-5,
        )
        assert_near_equal(
            prob.get_val(AE.TORSION_SHEAR_STRESS, units='Pa'),
            expected_torsion_stress,
            tolerance=1.0e-5,
        )
        assert_near_equal(
            prob.get_val(AE.BENDING_STRENGTH_MARGIN),
            expected_bending_margin,
            tolerance=1.0e-5,
        )
        assert_near_equal(
            prob.get_val(AE.TORSION_STRENGTH_MARGIN),
            expected_torsion_margin,
            tolerance=1.0e-5,
        )
        assert_near_equal(
            prob.get_val(AE.MIN_STRENGTH_MARGIN),
            expected_min_margin,
            tolerance=1.0e-5,
        )


if __name__ == '__main__':
    unittest.main()
