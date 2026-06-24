import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.subsystems.mass.spajeti_based.cg_estimator import SpaJetiCGEstimator
from aviary.subsystems.mass.spajeti_based.fuselage_structural_mass import FuselageStructuralMass
from aviary.subsystems.mass.spajeti_based.mass_group import SpaJetiMassGroup
from aviary.subsystems.mass.spajeti_based.propulsion_location import PropulsionLocationComp
from aviary.subsystems.mass.spajeti_based.vtp_structural_mass import VTPStructuralMass
from aviary.subsystems.mass.spajeti_based.wing_structural_mass import WingStructuralMass
from aviary.variable_info.variables import Aircraft


def _make_problem(component, force_alloc_complex=False):
    prob = om.Problem()
    prob.model.add_subsystem('comp', component, promotes=['*'])
    prob.setup(force_alloc_complex=force_alloc_complex)
    return prob


def _scalar(value):
    return float(np.asarray(value).item())


def test_cg_estimator_matches_hand_weighted_average_and_empty_excludes_fuel():
    prob = _make_problem(SpaJetiCGEstimator())
    values = {
        'wing_structural_mass': (1.8, 'kg'),
        'wing_x_cg': (-0.88, 'm'),
        'wing_z_cg': (0.01, 'm'),
        'fuselage_structural_mass': (2.16, 'kg'),
        'fuselage_x_cg': (-0.90, 'm'),
        'fuselage_z_cg': (0.0, 'm'),
        'vtp_structural_mass': (0.30, 'kg'),
        'vtp_x_cg': (-0.87, 'm'),
        'vtp_z_cg': (0.0, 'm'),
        'engine_mass': (2.0, 'kg'),
        'engine_x': (-1.70, 'm'),
        'engine_z': (0.0, 'm'),
        'fuel_mass': (1.25, 'kg'),
        'fuel_x': (-0.90, 'm'),
        'fuel_z': (0.0, 'm'),
        'avionics_mass': (0.25, 'kg'),
        'avionics_x': (-0.30, 'm'),
        'avionics_z': (0.0, 'm'),
    }
    for name, (value, units) in values.items():
        prob.set_val(name, value, units=units)

    prob.run_model()

    masses = np.array([1.8, 2.16, 0.30, 2.0, 1.25, 0.25])
    x_pos = np.array([-0.88, -0.90, -0.87, -1.70, -0.90, -0.30])
    z_pos = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert prob.get_val('aircraft_x_cg', units='m') == pytest.approx(
        np.dot(masses, x_pos) / masses.sum()
    )
    assert prob.get_val('aircraft_z_cg', units='m') == pytest.approx(
        np.dot(masses, z_pos) / masses.sum()
    )
    assert prob.get_val('aircraft_empty_mass', units='kg') == pytest.approx(
        masses.sum() - 1.25
    )
    assert prob.get_val('aircraft_total_mass', units='kg') == pytest.approx(masses.sum())


def test_wing_mass_increases_when_spanwise_semispan_grows():
    prob = _make_problem(WingStructuralMass(num_stations=5))
    chord = np.ones(5) * 0.30
    m_per_span = np.ones(5) * 0.80

    prob.set_val(AE.SPANWISE_CHORD, chord, units='m')
    prob.set_val(AE.SPANWISE_MASS_PER_UNIT_SPAN, m_per_span, units='kg/m')
    prob.set_val(AE.FRONT_SPAR_FRACTION, 0.15)
    prob.set_val(AE.REAR_SPAR_FRACTION, 0.60)
    prob.set_val('non_structural_mass_fraction', 0.25)
    prob.set_val('wing_x_apex', -0.80, units='m')

    prob.set_val(AE.SPANWISE_STATIONS, np.linspace(0.0, 0.75, 5), units='m')
    prob.run_model()
    mass_short = _scalar(prob.get_val('wing_structural_mass', units='kg'))

    prob.set_val(AE.SPANWISE_STATIONS, np.linspace(0.0, 1.00, 5), units='m')
    prob.run_model()
    mass_long = _scalar(prob.get_val('wing_structural_mass', units='kg'))

    assert mass_long > mass_short
    assert mass_long / mass_short == pytest.approx(1.00 / 0.75)


def test_wing_cg_uses_body_fixed_x_forward_frame():
    prob = _make_problem(WingStructuralMass(num_stations=5))
    prob.set_val(AE.SPANWISE_STATIONS, np.linspace(0.0, 0.9, 5), units='m')
    prob.set_val(AE.SPANWISE_CHORD, np.ones(5) * 0.30, units='m')
    prob.set_val(AE.SPANWISE_MASS_PER_UNIT_SPAN, np.ones(5), units='kg/m')
    prob.set_val(AE.FRONT_SPAR_FRACTION, 0.15)
    prob.set_val(AE.REAR_SPAR_FRACTION, 0.60)
    prob.set_val('wing_x_apex', -0.80, units='m')

    prob.run_model()

    box_mid = 0.5 * (0.15 + 0.60)
    expected_x_cg = -0.80 - box_mid * 0.30
    assert prob.get_val('wing_x_cg', units='m') == pytest.approx(expected_x_cg)
    assert prob.get_val('wing_x_cg', units='m') < 0.0


def test_vtp_adapter_passes_tail_physical_mass_and_cg_to_legacy_names():
    prob = _make_problem(VTPStructuralMass())
    prob.set_val('tail_physical_structural_mass', 0.20, units='kg')
    prob.set_val('tail_physical_x_cg', -0.92, units='m')
    prob.set_val('tail_physical_z_cg', 0.03, units='m')
    prob.run_model()
    mass_short = _scalar(prob.get_val('vtp_structural_mass', units='kg'))

    prob.set_val('tail_physical_structural_mass', 0.40, units='kg')
    prob.set_val('tail_physical_x_cg', -0.97, units='m')
    prob.set_val('tail_physical_z_cg', -0.01, units='m')
    prob.run_model()
    mass_tall = _scalar(prob.get_val('vtp_structural_mass', units='kg'))

    assert mass_tall > mass_short
    assert mass_tall / mass_short == pytest.approx(2.0)
    assert prob.get_val('vtp_x_cg', units='m') == pytest.approx(-0.97)
    assert prob.get_val('vtp_z_cg', units='m') == pytest.approx(-0.01)


def test_vtp_adapter_cg_translates_with_tail_physical_cg():
    prob = _make_problem(VTPStructuralMass())
    prob.set_val('tail_physical_x_cg', -0.80, units='m')
    prob.run_model()
    x0 = _scalar(prob.get_val('vtp_x_cg', units='m'))

    prob.set_val('tail_physical_x_cg', -0.70, units='m')
    prob.run_model()
    x1 = _scalar(prob.get_val('vtp_x_cg', units='m'))

    assert x1 - x0 == pytest.approx(0.10)


def test_propulsion_location_uses_x_forward_body_frame():
    prob = _make_problem(PropulsionLocationComp())
    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val('engine_station_fraction', 0.85)

    prob.run_model()

    assert prob.get_val('engine_x', units='m') == pytest.approx(-1.70)
    assert prob.get_val('engine_z', units='m') == pytest.approx(0.0)


def test_spajeti_mass_group_baseline_outputs_have_consistent_signs_and_totals():
    prob = om.Problem(model=SpaJetiMassGroup(num_wing_stations=5))
    prob.setup()

    prob.set_val(AE.SPANWISE_STATIONS, np.linspace(0.0, 0.90, 5), units='m')
    prob.set_val(AE.SPANWISE_CHORD, np.linspace(0.30, 0.24, 5), units='m')
    prob.set_val(AE.SPANWISE_MASS_PER_UNIT_SPAN, np.ones(5) * 0.80, units='kg/m')
    prob.set_val(AE.FRONT_SPAR_FRACTION, 0.15)
    prob.set_val(AE.REAR_SPAR_FRACTION, 0.60)
    prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.8)
    prob.set_val(Aircraft.Wing.SWEEP, 0.0, units='deg')
    prob.set_val('wing_x_apex', -0.80, units='m')
    prob.set_val('wing_z_apex', 0.0, units='m')
    prob.set_val('non_structural_mass_fraction', 0.25)

    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.30, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_HEIGHT, 0.30, units='m')
    prob.set_val('fuselage_wetted_area', 1.43, units='m**2')
    prob.set_val('fuselage_centroid_x', 0.90, units='m')
    prob.set_val('fuselage_areal_density', 1.95, units='kg/m**2')

    prob.set_val('tail_physical_structural_mass', 0.48, units='kg')
    prob.set_val('tail_physical_x_cg', -0.92, units='m')
    prob.set_val('tail_physical_z_cg', 0.0, units='m')

    prob.set_val('engine_mass', 2.0, units='kg')
    prob.set_val('fuel_mass', 1.0, units='kg')
    prob.set_val('avionics_mass', 0.25, units='kg')
    prob.set_val('avionics_x', -0.30, units='m')
    prob.set_val('avionics_z', 0.0, units='m')

    prob.run_model()

    empty = _scalar(prob.get_val('aircraft_empty_mass', units='kg'))
    total = _scalar(prob.get_val('aircraft_total_mass', units='kg'))
    fuel = _scalar(prob.get_val('fuel_mass', units='kg'))

    assert prob.get_val('wing_x_cg', units='m') < 0.0
    assert prob.get_val('fuselage_x_cg', units='m') < 0.0
    assert prob.get_val('vtp_x_cg', units='m') < 0.0
    assert prob.get_val('engine_x', units='m') < 0.0
    assert prob.get_val('fuel_x', units='m') < 0.0
    assert total == pytest.approx(empty + fuel)
    assert prob.get_val('aircraft_x_cg', units='m') < 0.0

    assert _scalar(prob.get_val('wing_structural_mass', units='kg')) == pytest.approx(1.80)
    assert _scalar(prob.get_val('fuselage_structural_mass', units='kg')) == pytest.approx(
        2.79, rel=0.05
    )
    assert _scalar(prob.get_val('vtp_structural_mass', units='kg')) == pytest.approx(0.48)
    assert 7.2 <= empty <= 7.4
    assert _scalar(prob.get_val('aircraft_x_cg', units='m')) == pytest.approx(-1.07, rel=0.03)


def test_cs_declared_mass_component_partials_pass_against_finite_difference():
    cases = [
        (WingStructuralMass(num_stations=5), _seed_wing),
        (FuselageStructuralMass(), _seed_fuselage),
        (VTPStructuralMass(), _seed_vtp),
        (PropulsionLocationComp(), _seed_propulsion),
        (SpaJetiCGEstimator(), _seed_cg),
    ]

    for component, seed in cases:
        prob = _make_problem(component, force_alloc_complex=True)
        seed(prob)
        prob.run_model()
        partials = prob.check_partials(method='fd', out_stream=None)
        assert_check_partials(partials, atol=1.0e-5, rtol=1.0e-5)


def test_fuselage_structural_mass_sensitivity_is_physical():
    prob = _make_problem(FuselageStructuralMass())
    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.30, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_HEIGHT, 0.30, units='m')
    prob.set_val('fuselage_wetted_area', 1.43, units='m**2')
    prob.set_val('fuselage_centroid_x', 0.90, units='m')
    prob.set_val('fuselage_areal_density', 1.95, units='kg/m**2')
    prob.run_model()
    baseline_mass = _scalar(prob.get_val('fuselage_structural_mass', units='kg'))
    baseline_x_cg = _scalar(prob.get_val('fuselage_x_cg', units='m'))

    prob.set_val('fuselage_wetted_area', 1.57, units='m**2')
    prob.set_val('fuselage_centroid_x', 0.99, units='m')
    prob.run_model()
    larger_area_mass = _scalar(prob.get_val('fuselage_structural_mass', units='kg'))
    aft_centroid_x_cg = _scalar(prob.get_val('fuselage_x_cg', units='m'))

    prob.set_val('fuselage_wetted_area', 1.43, units='m**2')
    prob.set_val('fuselage_areal_density', 2.10, units='kg/m**2')
    prob.run_model()
    denser_mass = _scalar(prob.get_val('fuselage_structural_mass', units='kg'))

    assert baseline_mass == pytest.approx(2.79, rel=0.05)
    assert larger_area_mass > baseline_mass
    assert denser_mass > baseline_mass
    assert aft_centroid_x_cg < baseline_x_cg
    assert baseline_x_cg < 0.0


def _seed_wing(prob):
    prob.set_val(AE.SPANWISE_STATIONS, np.linspace(0.0, 0.90, 5), units='m')
    prob.set_val(AE.SPANWISE_CHORD, np.linspace(0.30, 0.24, 5), units='m')
    prob.set_val(AE.SPANWISE_MASS_PER_UNIT_SPAN, np.ones(5) * 0.80, units='kg/m')
    prob.set_val(AE.FRONT_SPAR_FRACTION, 0.15)
    prob.set_val(AE.REAR_SPAR_FRACTION, 0.60)
    prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.8)
    prob.set_val(Aircraft.Wing.SWEEP, 0.0, units='deg')
    prob.set_val('wing_x_apex', -0.80, units='m')
    prob.set_val('wing_z_apex', 0.0, units='m')
    prob.set_val('non_structural_mass_fraction', 0.25)


def _seed_fuselage(prob):
    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.30, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_HEIGHT, 0.30, units='m')
    prob.set_val('fuselage_wetted_area', 1.43, units='m**2')
    prob.set_val('fuselage_centroid_x', 0.90, units='m')
    prob.set_val('fuselage_areal_density', 1.95, units='kg/m**2')


def _seed_vtp(prob):
    prob.set_val('tail_physical_structural_mass', 0.32, units='kg')
    prob.set_val('tail_physical_x_cg', -0.92, units='m')
    prob.set_val('tail_physical_z_cg', 0.0, units='m')


def _seed_propulsion(prob):
    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val('engine_station_fraction', 0.85)


def _seed_cg(prob):
    values = {
        'wing_structural_mass': (1.8, 'kg'),
        'wing_x_cg': (-0.88, 'm'),
        'wing_z_cg': (0.01, 'm'),
        'fuselage_structural_mass': (2.16, 'kg'),
        'fuselage_x_cg': (-0.90, 'm'),
        'fuselage_z_cg': (0.0, 'm'),
        'vtp_structural_mass': (0.30, 'kg'),
        'vtp_x_cg': (-0.87, 'm'),
        'vtp_z_cg': (0.0, 'm'),
        'engine_mass': (2.0, 'kg'),
        'engine_x': (-1.70, 'm'),
        'engine_z': (0.0, 'm'),
        'fuel_mass': (1.25, 'kg'),
        'fuel_x': (-0.90, 'm'),
        'fuel_z': (0.0, 'm'),
        'avionics_mass': (0.25, 'kg'),
        'avionics_x': (-0.30, 'm'),
        'avionics_z': (0.0, 'm'),
    }
    for name, (value, units) in values.items():
        prob.set_val(name, value, units=units)
