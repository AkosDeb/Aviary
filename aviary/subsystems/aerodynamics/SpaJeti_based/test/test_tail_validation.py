"""
Tail cant-angle validation suite — TOOD.md §Tail validation.

Six validation items:
  1. TailCantRotation cant-angle sweep, invariant, and check_partials.
  2. Mission drag completeness — CDI_tail=0 and wrong alpha denominator documented.
  3. XTailGeometry Swet convention: exposed Swet excludes fuselage-buried panel area.
  4. VTPSurface wiring — surface_ar -> tail_panel_ar (physical), not aero AR.
  5. Ny/Nz sign and magnitude: CY_beta_tail < 0, CL_alpha_tail > 0 at φ=45°.
  6. Polhamus low-AR operating point documented (panel AR ≈ 1.2–2.0).
"""

import pathlib

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials

from aviary.subsystems.aerodynamics.SpaJeti_based.cy_beta_vtp import TailCantRotation
from aviary.subsystems.aerodynamics.SpaJeti_based.parasite_drag import RoskamParasiteDragBuildUp
from aviary.subsystems.aerodynamics.SpaJeti_based.roskam_aero_group import _GeomArrayAssembler
from aviary.subsystems.geometry.spajeti_based.tail_geometry import XTailGeometry
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Shared UAV design-point values (NACA 0012 X-tail, ISA 5000 m dash)
# ---------------------------------------------------------------------------
_N        = 4       # panel count
_PHI_DEG  = 45.0    # nominal cant angle [deg]
_CL_A     = 1.72    # 3-D panel CL_alpha from Polhamus [1/rad]
_S_PANEL  = 0.08    # physical panel area [m²]
_S_REF    = 0.45    # wing reference area [m²]
_AUTHORITY = _N * _CL_A * _S_PANEL / _S_REF   # ≈ 1.2213 [1/rad]

# Standard input set for XTailGeometry (UAV design baseline)
_GEOM_INPUTS = {
    Aircraft.VerticalTail.SPAN:        (0.31,  'm'),
    Aircraft.VerticalTail.ROOT_CHORD:  (0.275, 'm'),
    Aircraft.VerticalTail.TAPER_RATIO: (0.6,   'unitless'),
    Aircraft.VerticalTail.SWEEP:       (20.0,  'deg'),
    Aircraft.VerticalTail.THICKNESS_TO_CHORD: (0.12, 'unitless'),
    Aircraft.Wing.SPAN:                (1.8,   'm'),
    Aircraft.Wing.AREA:                (0.45,  'm**2'),
    Aircraft.Wing.TAPER_RATIO:         (0.5,   'unitless'),
    Aircraft.Wing.SWEEP:               (5.0,   'deg'),
    Aircraft.Fuselage.MAX_HEIGHT:      (0.30,  'm'),
    Aircraft.Fuselage.MAX_WIDTH:       (0.16,  'm'),
}


def _make_tail_cant_prob(phi_deg, force_alloc_complex=False):
    """Return a configured, run_model'd TailCantRotation problem."""
    prob = om.Problem()
    prob.model.add_subsystem('comp', TailCantRotation(), promotes=['*'])
    prob.setup(force_alloc_complex=force_alloc_complex)
    prob.set_val('tail_cant_angle',          phi_deg)
    prob.set_val('tail_panel_count',         float(_N))
    prob.set_val('CL_alpha_panel',           _CL_A)
    prob.set_val('tail_physical_panel_area', _S_PANEL)
    prob.set_val('wing_ref_area',            _S_REF)
    prob.run_model()
    return prob


def _make_geom_prob(phi_deg):
    """Return a configured, run_model'd XTailGeometry problem."""
    prob = om.Problem()
    prob.model.add_subsystem('geom', XTailGeometry(num_panels=4), promotes=['*'])
    prob.setup()
    for var, (val, unit) in _GEOM_INPUTS.items():
        prob.set_val(var, val, units=unit)
    prob.set_val('tail_cant_angle', phi_deg, units='deg')
    prob.run_model()
    return prob


# ===========================================================================
# 1.  TailCantRotation — cant-angle sweep, invariant, check_partials
# ===========================================================================

@pytest.mark.parametrize('phi_deg', [0.0, 30.0, 45.0, 60.0, 90.0])
def test_cant_rotation_authority_invariant(phi_deg):
    """−CY_beta_tail + CL_alpha_tail == authority for all cant angles.

    Derivation: authority = N*CL_a*S_p/S_ref, CY = -authority*sin²(φ),
    CL = authority*cos²(φ).  −CY + CL = authority*(sin²+cos²) = authority.
    """
    prob = _make_tail_cant_prob(phi_deg)
    cy = prob.get_val('CY_beta_tail')[0]
    cl = prob.get_val('CL_alpha_tail')[0]
    np.testing.assert_allclose(
        -cy + cl, _AUTHORITY, rtol=1e-10,
        err_msg=f'φ={phi_deg}°: invariant −CY+CL ≠ authority',
    )


def test_cant_rotation_pure_vtp():
    """φ=90°: all side force, zero longitudinal lift slope."""
    prob = _make_tail_cant_prob(90.0)
    np.testing.assert_allclose(prob.get_val('CL_alpha_tail'), 0.0, atol=1e-12)
    np.testing.assert_allclose(prob.get_val('CY_beta_tail'), -_AUTHORITY, rtol=1e-10)


def test_cant_rotation_pure_htp():
    """φ=0°: all longitudinal lift slope, zero side force."""
    prob = _make_tail_cant_prob(0.0)
    np.testing.assert_allclose(prob.get_val('CY_beta_tail'), 0.0, atol=1e-12)
    np.testing.assert_allclose(prob.get_val('CL_alpha_tail'), _AUTHORITY, rtol=1e-10)


def test_cant_rotation_x_tail_equal_split():
    """φ=45°: |CY_beta_tail| == CL_alpha_tail == authority/2."""
    prob = _make_tail_cant_prob(45.0)
    cy = prob.get_val('CY_beta_tail')[0]
    cl = prob.get_val('CL_alpha_tail')[0]
    np.testing.assert_allclose(abs(cy), cl, rtol=1e-10,
        err_msg='at 45°: |CY| must equal CL')
    np.testing.assert_allclose(cl, _AUTHORITY / 2.0, rtol=1e-10)


@pytest.mark.parametrize('phi_deg', [0.0, 45.0, 90.0])
def test_cant_rotation_check_partials(phi_deg):
    """Analytic / complex-step partial agreement at cant-angle endpoints and 45°."""
    prob = _make_tail_cant_prob(phi_deg, force_alloc_complex=True)
    data = prob.check_partials(method='cs', out_stream=None, compact_print=True)
    assert_check_partials(data, atol=1e-8, rtol=1e-6)


# ===========================================================================
# 2.  Mission drag completeness — known gaps documented
# ===========================================================================

def test_mission_induced_drag_gap_documented():
    """Tail CDI is absent from _RoskamMissionInducedDrag.

    KNOWN GAP: CDI_tail = 0 in the mission polar.  This test guards against
    silent regression.  When tail CDI is implemented, update this test to
    verify a non-zero contribution.
    """
    roskam = (
        pathlib.Path(__file__).resolve().parents[1] / 'roskam_aero_group.py'
    )
    src = roskam.read_text(encoding='utf-8')
    assert 'CDI_tail' not in src, (
        "'CDI_tail' found in roskam_aero_group.py — update this test to verify "
        "a non-zero tail induced-drag contribution"
    )


def test_fuselage_alpha_uses_cl_alpha_total():
    """_FuselageMissionLiftDrag uses CL_alpha_total (wing + tail) for alpha.

    Gap resolved in v1.38.0: CL_alpha_total replaces wing_CL_alpha so the
    fuselage alpha estimate accounts for the canted-tail contribution.
    """
    roskam = (
        pathlib.Path(__file__).resolve().parents[1] / 'roskam_aero_group.py'
    )
    src = roskam.read_text(encoding='utf-8')
    fus_start = src.find('class _FuselageMissionLiftDrag')
    assert fus_start != -1, '_FuselageMissionLiftDrag class not found'
    fus_block = src[fus_start:fus_start + 1500]
    assert 'CL_alpha_total' in fus_block, (
        '_FuselageMissionLiftDrag must use CL_alpha_total, not wing_CL_alpha'
    )
    assert 'wing_CL_alpha' not in fus_block, (
        'wing_CL_alpha must be removed from _FuselageMissionLiftDrag'
    )


# ===========================================================================
# 3.  XTailGeometry — wetted-area convention
# ===========================================================================

def test_swet_excludes_fuselage_buried_panel_area():
    """tail_physical_wetted_area excludes panel area buried in the fuselage."""
    prob = _make_geom_prob(45.0)
    s_panel = prob.get_val('tail_physical_panel_area')[0]
    s_wet = prob.get_val('tail_physical_wetted_area')[0]
    root_chord = _GEOM_INPUTS[Aircraft.VerticalTail.ROOT_CHORD][0]
    taper = _GEOM_INPUTS[Aircraft.VerticalTail.TAPER_RATIO][0]
    panel_span = _GEOM_INPUTS[Aircraft.VerticalTail.SPAN][0]
    half_width = _GEOM_INPUTS[Aircraft.Fuselage.MAX_WIDTH][0] / 2.0
    half_height = _GEOM_INPUTS[Aircraft.Fuselage.MAX_HEIGHT][0] / 2.0
    sin45 = np.sin(np.deg2rad(45.0))
    cos45 = np.cos(np.deg2rad(45.0))
    buried_span = 1.0 / np.sqrt((cos45 / half_width) ** 2 + (sin45 / half_height) ** 2)
    buried_panel_area = root_chord * (
        buried_span - 0.5 * (1.0 - taper) * buried_span**2 / panel_span
    )
    expected_swet = 2.0 * (_N * s_panel - _N * buried_panel_area)
    np.testing.assert_allclose(
        s_wet, expected_swet, rtol=1e-10,
        err_msg='Swet convention broken: expected 2*N*(S_panel - S_buried)',
    )


@pytest.mark.parametrize('phi_deg', [0.0, 30.0, 45.0, 60.0, 90.0])
def test_tail_fuselage_cutout_cant_angle_invariant(phi_deg):
    """Fuselage cutout area depends on panel count/chord/tc, not cant angle."""
    ref = _make_geom_prob(45.0).get_val('tail_fuselage_cutout_area')[0]
    prob = _make_geom_prob(phi_deg)
    cutout = prob.get_val('tail_fuselage_cutout_area')[0]
    swet = prob.get_val('tail_physical_wetted_area')[0]
    np.testing.assert_allclose(
        cutout, ref, rtol=1e-10,
        err_msg=f'Cutout changed at phi={phi_deg} deg; only exposed Swet should change',
    )
    assert swet >= 0.0


def test_lifting_surface_drag_characteristic_lengths_are_mac():
    """Roskam drag arrays use MAC for lifting-surface Reynolds lengths."""
    prob = om.Problem()
    prob.model.add_subsystem('asm', _GeomArrayAssembler(), promotes=['*'])
    prob.setup()

    prob.set_val(Aircraft.Wing.WETTED_AREA, 0.80, units='m**2')
    prob.set_val(Aircraft.Wing.AREA, 0.45, units='m**2')
    prob.set_val(Aircraft.Wing.SPAN, 1.80, units='m')
    prob.set_val('wing_c_mac', 0.25463, units='m')
    prob.set_val(Aircraft.Wing.THICKNESS_TO_CHORD, 0.15)
    prob.set_val(Aircraft.Wing.SWEEP, 5.0, units='deg')
    prob.set_val('tail_drag_wetted_area', 0.64, units='m**2')
    prob.set_val('tail_drag_characteristic_length', 0.18333, units='m')
    prob.set_val(Aircraft.VerticalTail.THICKNESS_TO_CHORD, 0.12)
    prob.set_val(Aircraft.VerticalTail.SWEEP, 20.0, units='deg')
    prob.set_val('fuselage_exposed_wetted_area', 1.9, units='m**2')
    prob.set_val(Aircraft.Fuselage.LENGTH, 2.0, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.15, units='m')
    prob.set_val('fuselage_fineness_ratio', 10.0)

    prob.run_model()

    char_lengths = prob.get_val('char_length_arr', units='m')
    np.testing.assert_allclose(char_lengths[0], 0.25463, rtol=1e-12)
    np.testing.assert_allclose(char_lengths[1], 0.18333, rtol=1e-12)
    np.testing.assert_allclose(char_lengths[2], 2.0, rtol=1e-12)


def test_tail_parasite_drag_uses_tail_interference_without_fuselage_interference():
    """The tail CD0 path applies R_tail but not fuselage R_wf."""
    prob = om.Problem()
    prob.model.add_subsystem(
        'pd',
        RoskamParasiteDragBuildUp(
            num_nodes=1,
            component_names=('tail',),
            component_kinds=('vtp',),
        ),
        promotes=['*'],
    )
    prob.setup()

    prob.set_val('mach', np.array([0.5]))
    prob.set_val('static_pressure', np.array([54048.0]), units='Pa')
    prob.set_val('temperature', np.array([255.65]), units='K')
    prob.set_val('reference_area', 0.45, units='m**2')
    prob.set_val('wetted_area', np.array([0.64]), units='m**2')
    prob.set_val('characteristic_length', np.array([0.274]), units='m')
    prob.set_val('thickness_to_chord', np.array([0.12]))
    prob.set_val('sweep_at_max_thickness', np.array([20.0]), units='deg')
    prob.set_val('fuselage_length', 2.0, units='m')
    prob.set_val('tail_interference_factor', 1.04)

    prob.run_model()

    cd0_tail = prob.get_val('CD0_component')[0, 0]
    cf = prob.get_val('skin_friction_coefficient')[0, 0]
    ff = prob.get_val('form_factor')[0, 0]
    r_ls = prob.get_val('lifting_surface_correction_factor')[0, 0]
    expected = cf * ff * r_ls * 1.04 * 0.64 / 0.45
    np.testing.assert_allclose(cd0_tail, expected, rtol=1e-12)


# ===========================================================================
# 4.  VTPSurface wiring — physical panel AR reaches Polhamus
# ===========================================================================

def test_vtp_surface_wires_physical_panel_ar():
    """Run script must map surface_ar -> 'tail_panel_ar', not tail_aero_vertical_ar.

    Using the physical panel AR (b²/S_panel) in Polhamus is correct.
    Using the projected aero AR (b_vert²/S_aero_vert = panel_ar at all angles)
    would give the same number here but relies on the wrong definition.
    """
    run_script = (
        pathlib.Path(__file__).resolve().parents[5]
        / 'aviary' / 'models' / 'aircraft' / 'horizontal_small_uav'
        / 'run_horizontal_small_uav.py'
    )
    src = run_script.read_text(encoding='utf-8')

    # surface_ar -> 'tail_panel_ar' must appear in the VTPSurface block.
    assert "'tail_panel_ar'" in src, (
        "run script does not wire surface_ar -> 'tail_panel_ar'"
    )
    # The VTPSurface block must NOT use tail_aero_vertical_ar for surface_ar.
    vtp_block_start = src.find("'vtp_surface'")
    assert vtp_block_start != -1, "'vtp_surface' subsystem not found in run script"
    vtp_block = src[vtp_block_start:vtp_block_start + 600]
    assert 'tail_aero_vertical_ar' not in vtp_block, (
        "VTPSurface wired to tail_aero_vertical_ar — must use tail_panel_ar instead"
    )


# ===========================================================================
# 5.  Sign and magnitude smoke test at design point
# ===========================================================================

def test_cy_beta_negative_cl_alpha_positive_at_design_point():
    """At φ=45° CY_beta_tail < 0 (restoring side force) and CL_alpha_tail > 0."""
    prob = _make_tail_cant_prob(_PHI_DEG)
    assert prob.get_val('CY_beta_tail')[0] < 0.0, 'CY_beta_tail must be negative'
    assert prob.get_val('CL_alpha_tail')[0] > 0.0, 'CL_alpha_tail must be positive'


def test_cl_alpha_tail_adds_to_wing():
    """CL_alpha_tail > 0 must increase total CL_alpha above wing-alone value.

    Validates the physics: a canted tail (φ < 90°) contributes positive
    longitudinal lift slope, so CL_alpha_total > wing_CL_alpha.
    """
    prob = _make_tail_cant_prob(_PHI_DEG)
    wing_cl_alpha = 4.5   # representative NACA 4415 3-D value [1/rad]
    cl_alpha_tail = prob.get_val('CL_alpha_tail')[0]
    assert cl_alpha_tail > 0.0
    assert wing_cl_alpha + cl_alpha_tail > wing_cl_alpha


def test_authority_numerical_value():
    """Cross-check the UAV design-point authority against the hand-computed value.

    N=4, φ=45°, CL_alpha_panel=1.72/rad, S_panel=0.08m², S_ref=0.45m²:
      authority = 4 * 1.72 * 0.08 / 0.45 = 1.22133…/rad
      CY_beta_tail = −0.61067/rad,  CL_alpha_tail = +0.61067/rad
    """
    prob = _make_tail_cant_prob(45.0)
    expected_authority = 4.0 * 1.72 * 0.08 / 0.45
    cy = prob.get_val('CY_beta_tail')[0]
    cl = prob.get_val('CL_alpha_tail')[0]
    np.testing.assert_allclose(cl, expected_authority / 2.0, rtol=1e-10)
    np.testing.assert_allclose(cy, -expected_authority / 2.0, rtol=1e-10)


# ===========================================================================
# 6.  Polhamus low-AR operating point documented
# ===========================================================================

def test_polhamus_low_ar_operating_range():
    """Panel AR must stay in [0.5, 4] — Polhamus low-AR regime.

    At the UAV design point, tail_panel_ar ≈ 1.2–2.0.  Polhamus/DATCOM is
    well-calibrated for AR > 4; at this operating point errors of 10–20% vs
    measured data are expected.  Helmbold's formula is more accurate here and
    should be considered as a future improvement (see TOOD.md §Tail validation).

    Bounds rationale:
      AR < 0.5: flat-plate approximation breaks down — need slender-body theory.
      AR > 4.0: Polhamus is well-calibrated — low-AR warning is no longer needed.
    """
    prob = _make_geom_prob(45.0)
    panel_ar = prob.get_val('tail_panel_ar')[0]
    assert 0.5 <= panel_ar < 4.0, (
        f'Panel AR={panel_ar:.3f} outside low-AR Polhamus operating range [0.5, 4). '
        'Update test if AR is intentionally changed.'
    )


def test_panel_ar_uses_physical_span():
    """tail_panel_ar = b_panel² / S_panel — based on physical (not projected) span."""
    prob = _make_geom_prob(45.0)
    span    = prob.get_val(Aircraft.VerticalTail.SPAN)[0]     # physical panel span
    s_panel = prob.get_val('tail_physical_panel_area')[0]
    panel_ar_expected = span**2 / s_panel
    panel_ar_actual   = prob.get_val('tail_panel_ar')[0]
    np.testing.assert_allclose(panel_ar_actual, panel_ar_expected, rtol=1e-10,
        err_msg='tail_panel_ar must equal b_panel²/S_panel')
