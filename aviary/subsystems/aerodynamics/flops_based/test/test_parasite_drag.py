import numpy as np
import openmdao.api as om
import pytest

from aviary.subsystems.aerodynamics.aero_utils import (
    airfoil_thickness_location_parameter,
    dynamic_viscosity_sutherland,
    exposed_wetted_area_lifting_surface,
    flat_plate_skin_friction_coeff,
    form_factor_datcom_body,
    form_factor_lifting_surface,
    form_factor_raymer_fuselage,
    lifting_surface_correction_factor,
    reynolds_number_from_mach,
    roughness_height,
    roughness_cutoff_reynolds,
    wing_fuselage_interference_factor,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.parasite_drag import RoskamParasiteDragBuildUp


def test_atmosphere_and_flat_plate_helpers():
    mu = dynamic_viscosity_sutherland(288.15)
    re = reynolds_number_from_mach(
        mach=0.3,
        static_pressure_Pa=101325.0,
        temperature_K=288.15,
        characteristic_length_m=1.0,
    )
    cf = flat_plate_skin_friction_coeff(1.0e6, mach=0.3, laminar_fraction=0.0)

    assert mu == pytest.approx(1.7892976260350732e-5, rel=0.0, abs=1e-12)
    assert re == pytest.approx(6.989225304399539e6, rel=0.0, abs=1e-9)
    assert cf == pytest.approx(0.004433494542035681, rel=0.0, abs=1e-12)


def test_exposed_wetted_area_helper():
    assert exposed_wetted_area_lifting_surface(0.45, 0.05) == pytest.approx(0.80)
    assert exposed_wetted_area_lifting_surface(
        np.array([0.45, 0.20]),
        np.array([0.05, 0.00]),
    ) == pytest.approx(np.array([0.80, 0.40]))

    with pytest.raises(ValueError, match='buried_planform_area cannot exceed'):
        exposed_wetted_area_lifting_surface(0.45, 0.50)


def test_form_factor_helpers():
    ff_surface = form_factor_lifting_surface(
        thickness_to_chord=0.12,
        max_thickness_location_over_chord=0.4,
    )
    ff_datcom_body = form_factor_datcom_body(8.0)
    ff_raymer_fuselage = form_factor_raymer_fuselage(8.0)
    re_cutoff_subsonic = roughness_cutoff_reynolds(1.0, roughness_height('aluminum'), mach=0.5)
    re_cutoff_supersonic = roughness_cutoff_reynolds(1.0, roughness_height('aluminum'), mach=1.2)

    assert ff_surface == pytest.approx(1.260736, rel=0.0, abs=1e-12)
    assert ff_datcom_body == pytest.approx(1.1371875, rel=0.0, abs=1e-12)
    assert ff_raymer_fuselage == pytest.approx(1.140970869120796, rel=0.0, abs=1e-12)
    assert re_cutoff_subsonic == pytest.approx(6.994933605582026e6, rel=0.0, abs=1e-6)
    assert re_cutoff_supersonic == pytest.approx(1.0006977467666801e7, rel=0.0, abs=1e-6)


def test_lifting_surface_form_factor_uses_max_thickness_location_over_chord():
    ff_naca_6_series_like = form_factor_lifting_surface(
        thickness_to_chord=0.12,
        max_thickness_location_over_chord=0.4,
    )
    ff_naca_4_digit_like = form_factor_lifting_surface(
        thickness_to_chord=0.12,
        max_thickness_location_over_chord=0.3,
    )

    assert ff_naca_6_series_like == pytest.approx(1.260736, rel=0.0, abs=1e-12)
    assert ff_naca_4_digit_like == pytest.approx(1.164736, rel=0.0, abs=1e-12)
    assert ff_naca_6_series_like > ff_naca_4_digit_like


def test_lifting_surface_corrections_and_interference_helpers():
    assert airfoil_thickness_location_parameter(0.3) == pytest.approx(1.2, rel=0.0, abs=1e-12)
    assert airfoil_thickness_location_parameter(0.3001) == pytest.approx(2.0, rel=0.0, abs=1e-12)
    assert lifting_surface_correction_factor(0.5, 1.0) == pytest.approx(
        1.13,
        rel=0.0,
        abs=1e-12,
    )
    assert lifting_surface_correction_factor(0.8, 0.8) == pytest.approx(1.21, rel=0.0, abs=1e-12)
    assert wing_fuselage_interference_factor(1.0e7, 0.6) == pytest.approx(0.999, rel=0.0, abs=1e-12)
    assert wing_fuselage_interference_factor(3.0e7, 0.85) == pytest.approx(0.97, rel=0.0, abs=1e-12)


def test_component_build_up(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    prob = om.Problem()
    prob.model.add_subsystem(
        'parasite',
        RoskamParasiteDragBuildUp(
            num_nodes=2,
            component_names=('wing', 'fuselage'),
            component_kinds=('lifting_surface', 'fuselage'),
        ),
        promotes=['*'],
    )

    prob.setup()

    prob.set_val('mach', np.array([0.3, 0.5]))
    prob.set_val('static_pressure', np.array([101325.0, 54048.0]), units='Pa')
    prob.set_val('temperature', np.array([288.15, 255.65]), units='K')
    prob.set_val('reference_area', 0.45, units='m**2')
    prob.set_val('wetted_area', np.array([0.95, 2.0]), units='m**2')
    prob.set_val('characteristic_length', np.array([0.255, 2.0]), units='m')
    prob.set_val('thickness_to_chord', np.array([0.15, 0.0]))
    prob.set_val('fineness_ratio', np.array([0.0, 10.0]))
    prob.set_val('interference_factor', np.array([1.05, 1.0]))
    prob.set_val('leakage_protuberance_factor', np.array([0.02, 0.05]))

    prob.run_model()

    cd0_component = prob.get_val('CD0_component')
    cd0 = prob.get_val('CD0')
    r_ls = prob.get_val('lifting_surface_correction_factor')

    assert cd0_component.shape == (2, 2)
    assert cd0.shape == (2,)
    assert r_ls.shape == (2, 2)
    assert np.all(r_ls[:, 0] > 1.0)
    assert r_ls[:, 1] == pytest.approx(np.ones(2), rel=0.0, abs=1e-12)
    assert cd0 == pytest.approx(np.sum(cd0_component, axis=1), rel=0.0, abs=1e-14)
    assert np.all(cd0_component > 0.0)
    assert np.all(cd0 > 0.0)
    assert 'drag' not in prob.model._outputs
