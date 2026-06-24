import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_near_equal

from aviary.subsystems.geometry.spajeti_based.tail_geometry import (
    SURFACE_HORIZONTAL,
    SURFACE_VERTICAL,
    TailGeometryGroup,
    X_TAIL,
    XTailGeometry,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


def test_x_tail_geometry_projects_diagonal_panels():
    prob = om.Problem()
    prob.model.add_subsystem('tail', XTailGeometry(), promotes=['*'])
    prob.setup(force_alloc_complex=True)

    prob.set_val(Aircraft.Fuselage.MAX_HEIGHT, 0.30, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.16, units='m')
    prob.set_val(Aircraft.VerticalTail.SPAN, 0.40, units='m')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, 0.20, units='m')
    prob.set_val(Aircraft.VerticalTail.TAPER_RATIO, 0.50, units='unitless')
    prob.set_val(Aircraft.VerticalTail.SWEEP, 0.0, units='deg')
    prob.set_val(Aircraft.VerticalTail.THICKNESS_TO_CHORD, 0.12, units='unitless')
    prob.set_val(Aircraft.Wing.SPAN, 1.80, units='m')
    prob.set_val(Aircraft.Wing.AREA, 0.45, units='m**2')
    prob.set_val(Aircraft.Wing.TAPER_RATIO, 0.8, units='unitless')
    prob.set_val(Aircraft.Wing.SWEEP, 5.0, units='deg')
    prob.set_val('wing_x_apex_fwd', -0.80, units='m')
    prob.set_val('wing_z_apex', 0.02, units='m')
    prob.set_val('tail_volume_reference_x_cg', -0.70, units='m')
    prob.set_val(AE.ELASTIC_AXIS_FRACTION, 0.375, units='unitless')
    prob.set_val(AE.VTP_CHORDWISE_CG_FRACTION, 0.40, units='unitless')
    prob.set_val(AE.VTP_ROOT_LE_X_OFFSET, 0.0, units='m')
    prob.set_val(AE.VTP_TIP_PANEL_COUNT, 2.0, units='unitless')
    prob.set_val('tail_cant_angle', 45.0, units='deg')
    prob.set_val('tail_areal_density', 1.5, units='kg/m**2')

    prob.run_model()

    panel_area = 0.40 * 0.20 * (1.0 + 0.50) / 2.0
    sin45 = np.sin(np.pi / 4.0)
    expected_total_vertical_area = 4.0 * panel_area * sin45 ** 2
    expected_total_physical_area = 4.0 * panel_area
    half_width = 0.16 / 2.0
    half_height = 0.30 / 2.0
    buried_span = 1.0 / np.sqrt((sin45 / half_width) ** 2 + (sin45 / half_height) ** 2)
    buried_panel_area = 0.20 * (
        buried_span - 0.5 * (1.0 - 0.50) * buried_span**2 / 0.40
    )
    expected_exposed_area = expected_total_physical_area - 4.0 * buried_panel_area
    expected_tail_cutout = 4.0 * 0.6843 * 0.12 * 0.20**2
    expected_vertical_span = 0.40 * sin45
    structural_mass = expected_total_physical_area * 1.5
    panel_mac = 2.0 / 3.0 * 0.20 * (1.0 + 0.50 + 0.50**2) / (1.0 + 0.50)
    z_centroid = 0.40 / 3.0 * (1.0 + 2.0 * 0.50) / (1.0 + 0.50)
    wing_root_chord = 2.0 * 0.45 / (1.80 * (1.0 + 0.8))
    wing_mac = 2.0 / 3.0 * wing_root_chord * (1.0 + 0.8 + 0.8**2) / (1.0 + 0.8)
    wing_tip_chord = wing_root_chord * 0.8
    wing_tip_le_x = -0.80 - 0.5 * 1.80 * np.tan(np.deg2rad(5.0))
    x_from_tip_le = (0.40 - 0.25) * panel_mac
    tail_aero_center_x = wing_tip_le_x
    tail_moment_arm = -0.70 - tail_aero_center_x
    expected_horizontal_volume = expected_total_vertical_area * tail_moment_arm / (0.45 * wing_mac)
    expected_vertical_volume = expected_total_vertical_area * tail_moment_arm / (0.45 * 1.80)
    dx_to_ea = x_from_tip_le - 0.375 * wing_tip_chord
    panel_mass = structural_mass / 2.0
    expected_tip_inertia = 2.0 * (
        panel_mass * (panel_mac**2 + 0.40**2) / 12.0
        + panel_mass * (dx_to_ea**2 + z_centroid**2)
    )

    assert XTailGeometry.tail_type == X_TAIL
    assert XTailGeometry.surface_types['vertical'] == SURFACE_VERTICAL
    assert XTailGeometry.surface_types['horizontal'] == SURFACE_HORIZONTAL

    assert_near_equal(prob.get_val('tail_panel_area', units='m**2'), panel_area)
    assert_near_equal(prob.get_val('tail_physical_panel_area', units='m**2'), panel_area)
    assert_near_equal(
        prob.get_val('tail_physical_total_area', units='m**2'),
        expected_total_physical_area,
    )
    assert_near_equal(
        prob.get_val('tail_physical_structural_mass', units='kg'),
        structural_mass,
    )
    assert_near_equal(prob.get_val('tail_physical_x_cg', units='m'), wing_tip_le_x - x_from_tip_le)
    assert_near_equal(prob.get_val('tail_physical_z_cg', units='m'), 0.02)
    assert_near_equal(
        prob.get_val('tail_physical_cg_x_from_wing_tip_le', units='m'),
        x_from_tip_le,
    )
    assert_near_equal(prob.get_val('tail_physical_cg_z_abs', units='m'), z_centroid)
    assert_near_equal(prob.get_val('tail_physical_dx_to_elastic_axis', units='m'), dx_to_ea)
    assert_near_equal(
        prob.get_val('tail_physical_tip_mass_equivalent', units='kg'),
        structural_mass,
    )
    assert_near_equal(
        prob.get_val('tail_physical_tip_pitch_inertia_equivalent', units='kg*m**2'),
        expected_tip_inertia,
    )
    assert_near_equal(
        prob.get_val('tail_aero_vertical_area', units='m**2'),
        expected_total_vertical_area,
    )
    assert_near_equal(
        prob.get_val('tail_aero_horizontal_area', units='m**2'),
        expected_total_vertical_area,
    )
    assert_near_equal(
        prob.get_val('tail_aero_vertical_span', units='m'),
        expected_vertical_span,
    )
    assert_near_equal(prob.get_val('tail_aero_center_x', units='m'), tail_aero_center_x)
    assert_near_equal(
        prob.get_val('tail_aero_horizontal_moment_arm', units='m'),
        tail_moment_arm,
    )
    assert_near_equal(
        prob.get_val('tail_aero_vertical_moment_arm', units='m'),
        tail_moment_arm,
    )
    assert_near_equal(
        prob.get_val('tail_aero_horizontal_volume_coefficient'),
        expected_horizontal_volume,
    )
    assert_near_equal(
        prob.get_val('tail_aero_vertical_volume_coefficient'),
        expected_vertical_volume,
    )
    assert_near_equal(
        prob.get_val('tail_total_wetted_area', units='m**2'),
        2.0 * expected_exposed_area,
    )
    assert_near_equal(
        prob.get_val('tail_physical_wetted_area', units='m**2'),
        2.0 * expected_exposed_area,
    )
    assert_near_equal(
        prob.get_val('tail_drag_wetted_area', units='m**2'),
        2.0 * expected_exposed_area,
    )
    assert_near_equal(
        prob.get_val('tail_fuselage_cutout_area', units='m**2'),
        expected_tail_cutout,
    )
    assert_near_equal(
        prob.get_val('tail_drag_characteristic_length', units='m'),
        panel_mac,
    )
    assert_near_equal(prob.get_val('tail_drag_interference_factor'), 1.04)
    assert_near_equal(
        prob.get_val('fuselage_vtp_span_ratio'),
        0.30 / expected_vertical_span,
    )
    assert_near_equal(prob.get_val('wing_ref_area', units='m**2'), 0.45)
    assert_near_equal(prob.get_val('wing_half_span', units='m'), 0.90)


def test_tail_geometry_group_selects_x_tail_backend():
    prob = om.Problem()
    prob.model.add_subsystem(
        'tail',
        TailGeometryGroup(tail_type=X_TAIL),
        promotes=['*'],
    )
    prob.setup(force_alloc_complex=True)

    prob.set_val(Aircraft.Fuselage.MAX_HEIGHT, 0.30, units='m')
    prob.set_val(Aircraft.Fuselage.MAX_WIDTH, 0.16, units='m')
    prob.set_val(Aircraft.VerticalTail.SPAN, 0.40, units='m')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, 0.20, units='m')
    prob.set_val(Aircraft.VerticalTail.TAPER_RATIO, 0.50, units='unitless')
    prob.set_val(Aircraft.VerticalTail.SWEEP, 0.0, units='deg')
    prob.set_val(Aircraft.VerticalTail.THICKNESS_TO_CHORD, 0.12, units='unitless')
    prob.set_val(Aircraft.Wing.SPAN, 1.80, units='m')
    prob.set_val(Aircraft.Wing.AREA, 0.45, units='m**2')
    prob.set_val('tail_cant_angle', 45.0, units='deg')

    prob.run_model()

    panel_area = 0.40 * 0.20 * (1.0 + 0.50) / 2.0
    expected_total_physical_area = 4.0 * panel_area

    assert_near_equal(
        prob.get_val('tail_physical_total_area', units='m**2'),
        expected_total_physical_area,
    )
