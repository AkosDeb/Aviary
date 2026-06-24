"""Tail geometry adapters for SpaJeti aircraft layouts.

The components in this module own layout-specific tail details and expose stable
equivalent geometry to downstream aero, mass, and reporting modules.
"""

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.functions import add_aviary_input
from aviary.variable_info.variables import Aircraft


X_TAIL = 'x_tail'
SURFACE_VERTICAL = 'vertical_equivalent'
SURFACE_HORIZONTAL = 'horizontal_equivalent'
TAIL_GEOMETRY_TYPES = (X_TAIL,)


class XTailGeometry(om.ExplicitComponent):
    """Equivalent horizontal and vertical geometry for a symmetric X-tail.

    A single diagonal panel is described by the vertical-tail Aviary shape inputs.
    The panel is then rotated by ``tail_cant_angle`` from the horizontal plane:

        vertical projected span   = panel_span * sin(cant)
        horizontal projected span = panel_span * cos(cant)

    Equivalent aerodynamic areas use the squared direction cosines.  One factor
    projects the body-axis flow perturbation onto the canted panel normal; the
    second projects the resulting panel lift back onto the body-axis force.
    Physical wetted area remains based on the true panel area.

    The component separates physical geometry from aerodynamic-equivalent
    geometry:

    * ``tail_physical_*`` outputs describe real material for mass, inertia,
      wetted area, and visualization.
    * ``tail_aero_*`` outputs describe equivalent horizontal/vertical surfaces
      for stability and control models.
    * ``tail_drag_*`` outputs describe the exposed tail geometry contract for
      parasite drag build-up.

    Existing ``tail_vertical_*``, ``tail_horizontal_*``, and ``vtp_*`` names are
    compatibility aliases consumed by the current Roskam path.
    """

    tail_type = X_TAIL
    surface_types = {
        'vertical': SURFACE_VERTICAL,
        'horizontal': SURFACE_HORIZONTAL,
    }

    def initialize(self):
        self.options.declare(
            'num_panels',
            default=4,
            types=int,
            desc='Number of diagonal panels in the symmetric X-tail.',
        )
        self.options.declare(
            'tail_type',
            default=X_TAIL,
            values=(X_TAIL,),
            desc='Layout type identifier used by future tail-API selection logic.',
        )

    def setup(self):
        add_aviary_input(self, Aircraft.Fuselage.MAX_HEIGHT, units='m')
        add_aviary_input(self, Aircraft.Fuselage.MAX_WIDTH, units='m')

        add_aviary_input(self, Aircraft.VerticalTail.SPAN, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.ROOT_CHORD, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.TAPER_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.VerticalTail.SWEEP, units='deg')
        add_aviary_input(self, Aircraft.VerticalTail.THICKNESS_TO_CHORD, units='unitless')

        add_aviary_input(self, Aircraft.Wing.SPAN, units='m')
        add_aviary_input(self, Aircraft.Wing.AREA, units='m**2')
        add_aviary_input(self, Aircraft.Wing.TAPER_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.Wing.SWEEP, units='deg')

        self.add_input(
            'tail_cant_angle',
            val=45.0,
            units='deg',
            desc='X-tail panel cant angle from horizontal plane.',
        )
        self.add_input(
            'tail_areal_density',
            val=1.2,
            units='kg/m**2',
            desc='Tail structural areal density applied to true physical panel area.',
        )
        self.add_input(
            'tail_airfoil_cross_section_area_coeff',
            val=0.6843,
            units='unitless',
            desc='Airfoil cross-section area coefficient used for fuselage skin cutouts.',
        )
        self.add_input(
            'wing_x_apex_fwd',
            val=-0.80,
            units='m',
            desc='Wing root leading-edge x in the mass/aeroelastic x-positive-forward frame.',
        )
        self.add_input(
            'wing_z_apex',
            val=0.0,
            units='m',
            desc='Wing root leading-edge z in the body frame, positive down.',
        )
        self.add_input(
            'tail_volume_reference_x_cg',
            val=-0.80,
            units='m',
            desc='Reference aircraft CG x used for tail moment arms; x positive forward.',
        )
        self.add_input(AE.ELASTIC_AXIS_FRACTION, val=0.375, units='unitless')
        self.add_input(AE.VTP_CHORDWISE_CG_FRACTION, val=0.40, units='unitless')
        self.add_input(AE.VTP_ROOT_LE_X_OFFSET, val=0.0, units='m')
        self.add_input(AE.VTP_TIP_PANEL_COUNT, val=2.0, units='unitless')

        # Physical geometry: true material layout.
        self.add_output('tail_physical_panel_area', val=0.08, units='m**2')
        self.add_output('tail_physical_total_area', val=0.32, units='m**2')
        self.add_output('tail_physical_wetted_area', val=0.64, units='m**2')
        self.add_output('tail_physical_structural_mass', val=0.38, units='kg')
        self.add_output('tail_physical_x_cg', val=-0.90, units='m')
        self.add_output('tail_physical_z_cg', val=0.0, units='m')
        self.add_output('tail_physical_cg_x_from_wing_tip_le', val=0.0, units='m')
        self.add_output('tail_physical_cg_z_abs', val=0.2, units='m')
        self.add_output('tail_physical_dx_to_elastic_axis', val=0.0, units='m')
        self.add_output('tail_physical_tip_mass_equivalent', val=0.38, units='kg')
        self.add_output('tail_physical_tip_pitch_inertia_equivalent', val=0.01, units='kg*m**2')

        # Parasite-drag geometry contract.
        self.add_output(
            'tail_drag_wetted_area',
            val=0.64,
            units='m**2',
            desc='Exposed tail wetted area used by parasite drag build-up.',
        )
        self.add_output(
            'tail_drag_characteristic_length',
            val=0.25,
            units='m',
            desc='Physical tail panel mean aerodynamic chord used for drag Reynolds number.',
        )
        self.add_output(
            'tail_drag_interference_factor',
            val=1.04,
            units='unitless',
            desc='Tail junction/interference factor used by parasite drag build-up.',
        )

        # Backward-compatible physical panel names.
        self.add_output('tail_panel_area', val=0.08, units='m**2')
        self.add_output('tail_panel_ar', val=1.2, units='unitless')
        self.add_output('tail_panel_avg_chord', val=0.25, units='m')
        self.add_output('tail_panel_count', val=float(self.options['num_panels']), units='unitless')

        # Aerodynamic-equivalent geometry.
        self.add_output('tail_aero_vertical_area', val=0.16, units='m**2')
        self.add_output('tail_aero_vertical_ar', val=0.6, units='unitless')
        self.add_output('tail_aero_vertical_span', val=0.22, units='m')
        self.add_output('tail_aero_horizontal_area', val=0.16, units='m**2')
        self.add_output('tail_aero_horizontal_ar', val=0.6, units='unitless')
        self.add_output('tail_aero_horizontal_span', val=0.22, units='m')
        self.add_output('tail_aero_center_x', val=-0.90, units='m')
        self.add_output('tail_aero_horizontal_moment_arm', val=0.10, units='m')
        self.add_output('tail_aero_vertical_moment_arm', val=0.10, units='m')
        self.add_output('tail_aero_horizontal_volume_coefficient', val=0.05, units='unitless')
        self.add_output('tail_aero_vertical_volume_coefficient', val=0.02, units='unitless')

        self.add_output('tail_total_wetted_area', val=0.64, units='m**2')
        self.add_output('tail_fuselage_cutout_area', val=0.0, units='m**2')

        self.add_output(
            'fuselage_vtp_span_ratio',
            val=0.8,
            units='unitless',
            desc='Fuselage depth over vertical-equivalent tail span.',
        )
        self.add_output('wing_ref_area', val=0.45, units='m**2')
        self.add_output('wing_half_span', val=0.9, units='m')

    def setup_partials(self):
        shape_inputs = [
            Aircraft.VerticalTail.SPAN,
            Aircraft.VerticalTail.ROOT_CHORD,
            Aircraft.VerticalTail.TAPER_RATIO,
            Aircraft.VerticalTail.THICKNESS_TO_CHORD,
            Aircraft.Fuselage.MAX_HEIGHT,
            Aircraft.Fuselage.MAX_WIDTH,
            'tail_cant_angle',
            'tail_airfoil_cross_section_area_coeff',
        ]
        outputs = [
            'tail_physical_panel_area',
            'tail_physical_total_area',
            'tail_physical_wetted_area',
            'tail_drag_wetted_area',
            'tail_drag_characteristic_length',
            'tail_panel_area',
            'tail_panel_ar',
            'tail_panel_avg_chord',
            'tail_aero_vertical_area',
            'tail_aero_vertical_ar',
            'tail_aero_vertical_span',
            'tail_aero_horizontal_area',
            'tail_aero_horizontal_ar',
            'tail_aero_horizontal_span',
            'tail_total_wetted_area',
            'tail_fuselage_cutout_area',
        ]
        self.declare_partials(outputs, shape_inputs, method='cs')
        self.declare_partials(
            [
                'tail_aero_center_x',
                'tail_aero_horizontal_moment_arm',
                'tail_aero_vertical_moment_arm',
                'tail_aero_horizontal_volume_coefficient',
                'tail_aero_vertical_volume_coefficient',
            ],
            '*',
            method='cs',
        )
        self.declare_partials(
            'tail_physical_structural_mass',
            [
                Aircraft.VerticalTail.SPAN,
                Aircraft.VerticalTail.ROOT_CHORD,
                Aircraft.VerticalTail.TAPER_RATIO,
                'tail_areal_density',
            ],
            method='cs',
        )
        self.declare_partials(
            'tail_physical_tip_mass_equivalent',
            [
                Aircraft.VerticalTail.SPAN,
                Aircraft.VerticalTail.ROOT_CHORD,
                Aircraft.VerticalTail.TAPER_RATIO,
                'tail_areal_density',
            ],
            method='cs',
        )
        self.declare_partials(
            [
                'tail_physical_x_cg',
                'tail_physical_z_cg',
                'tail_physical_cg_x_from_wing_tip_le',
                'tail_physical_cg_z_abs',
                'tail_physical_dx_to_elastic_axis',
                'tail_physical_tip_pitch_inertia_equivalent',
            ],
            '*',
            method='cs',
        )
        self.declare_partials(
            'fuselage_vtp_span_ratio',
            [Aircraft.Fuselage.MAX_HEIGHT, Aircraft.VerticalTail.SPAN, 'tail_cant_angle'],
            method='cs',
        )
        self.declare_partials('wing_ref_area', Aircraft.Wing.AREA, method='cs')
        self.declare_partials('wing_half_span', Aircraft.Wing.SPAN, method='cs')
        self.declare_partials('tail_panel_count', '*', dependent=False)
        self.declare_partials('tail_drag_interference_factor', '*', dependent=False)

    def compute(self, inputs, outputs):
        n_panels = float(self.options['num_panels'])
        if n_panels <= 0:
            raise ValueError('XTailGeometry: num_panels must be positive.')

        fus_height = inputs[Aircraft.Fuselage.MAX_HEIGHT]
        fus_width = inputs[Aircraft.Fuselage.MAX_WIDTH]
        panel_span = inputs[Aircraft.VerticalTail.SPAN]
        root_chord = inputs[Aircraft.VerticalTail.ROOT_CHORD]
        taper = inputs[Aircraft.VerticalTail.TAPER_RATIO]
        tail_tc = inputs[Aircraft.VerticalTail.THICKNESS_TO_CHORD]
        wing_span = inputs[Aircraft.Wing.SPAN]
        wing_area = inputs[Aircraft.Wing.AREA]
        wing_taper = inputs[Aircraft.Wing.TAPER_RATIO]
        wing_sweep_deg = inputs[Aircraft.Wing.SWEEP]
        wing_x_apex_fwd = inputs['wing_x_apex_fwd']
        wing_z_apex = inputs['wing_z_apex']
        tail_volume_reference_x_cg = inputs['tail_volume_reference_x_cg']
        elastic_axis = inputs[AE.ELASTIC_AXIS_FRACTION]
        chordwise_cg = inputs[AE.VTP_CHORDWISE_CG_FRACTION]
        root_le_x_offset = inputs[AE.VTP_ROOT_LE_X_OFFSET]
        panel_count = inputs[AE.VTP_TIP_PANEL_COUNT]
        areal_density = inputs['tail_areal_density']
        cutout_coeff = inputs['tail_airfoil_cross_section_area_coeff']
        cant_rad = inputs['tail_cant_angle'] * (np.pi / 180.0)
        wing_sweep_rad = wing_sweep_deg * (np.pi / 180.0)

        sin_cant = np.sin(cant_rad)
        cos_cant = np.cos(cant_rad)

        panel_area = panel_span * root_chord * (1.0 + taper) / 2.0
        total_physical_area = n_panels * panel_area
        panel_ar = panel_span ** 2 / panel_area
        panel_avg_chord = panel_area / panel_span
        panel_mac = (
            2.0 / 3.0 * root_chord
            * (1.0 + taper + taper**2)
            / (1.0 + taper)
        )

        vertical_span = panel_span * sin_cant
        horizontal_span = panel_span * cos_cant
        vertical_area = n_panels * panel_area * sin_cant ** 2
        horizontal_area = n_panels * panel_area * cos_cant ** 2

        half_width = 0.5 * fus_width
        half_height = 0.5 * fus_height
        radial_denom = (
            (cos_cant / half_width) ** 2
            + (sin_cant / half_height) ** 2
        ) ** 0.5
        buried_span = np.minimum(panel_span, 1.0 / radial_denom)
        buried_panel_area = root_chord * (
            buried_span
            - 0.5 * (1.0 - taper) * buried_span**2 / panel_span
        )
        buried_total_area = n_panels * buried_panel_area
        exposed_total_area = total_physical_area - buried_total_area
        tail_fuselage_cutout_area = n_panels * cutout_coeff * tail_tc * root_chord**2

        vertical_ar = vertical_span ** 2 / vertical_area
        horizontal_ar = horizontal_span ** 2 / horizontal_area
        structural_mass = total_physical_area * areal_density

        wing_root_chord = 2.0 * wing_area / (wing_span * (1.0 + wing_taper))
        wing_mac = (
            2.0 / 3.0 * wing_root_chord
            * (1.0 + wing_taper + wing_taper**2)
            / (1.0 + wing_taper)
        )
        wing_tip_chord = wing_root_chord * wing_taper
        wing_tip_le_x = wing_x_apex_fwd - 0.5 * wing_span * np.tan(wing_sweep_rad)

        z_centroid_abs = (
            panel_span / 3.0 * (1.0 + 2.0 * taper) / (1.0 + taper)
        )
        panel_c4_x = z_centroid_abs * np.tan(inputs[Aircraft.VerticalTail.SWEEP] * (np.pi / 180.0))
        tail_aero_center_x = wing_tip_le_x - (root_le_x_offset + panel_c4_x)
        x_cg_from_wing_tip_le = root_le_x_offset + panel_c4_x + (chordwise_cg - 0.25) * panel_mac
        x_ea_tip = elastic_axis * wing_tip_chord
        dx_to_ea = x_cg_from_wing_tip_le - x_ea_tip
        tail_x_cg = wing_tip_le_x - x_cg_from_wing_tip_le
        tail_z_cg = wing_z_apex
        tail_moment_arm = tail_volume_reference_x_cg - tail_aero_center_x

        horizontal_volume = horizontal_area * tail_moment_arm / (wing_area * wing_mac)
        vertical_volume = vertical_area * tail_moment_arm / (wing_area * wing_span)

        panel_mass = structural_mass / panel_count
        panel_own_inertia_y = panel_mass * (panel_mac**2 + panel_span**2) / 12.0
        tip_pitch_inertia = panel_count * (
            panel_own_inertia_y + panel_mass * (dx_to_ea**2 + z_centroid_abs**2)
        )

        outputs['tail_physical_panel_area'] = panel_area
        outputs['tail_physical_total_area'] = total_physical_area
        outputs['tail_physical_wetted_area'] = 2.0 * exposed_total_area
        outputs['tail_physical_structural_mass'] = structural_mass
        outputs['tail_physical_x_cg'] = tail_x_cg
        outputs['tail_physical_z_cg'] = tail_z_cg
        outputs['tail_physical_cg_x_from_wing_tip_le'] = x_cg_from_wing_tip_le
        outputs['tail_physical_cg_z_abs'] = z_centroid_abs
        outputs['tail_physical_dx_to_elastic_axis'] = dx_to_ea
        outputs['tail_physical_tip_mass_equivalent'] = structural_mass
        outputs['tail_physical_tip_pitch_inertia_equivalent'] = tip_pitch_inertia

        outputs['tail_drag_wetted_area'] = outputs['tail_physical_wetted_area']
        outputs['tail_drag_characteristic_length'] = panel_mac
        outputs['tail_drag_interference_factor'] = 1.04

        outputs['tail_panel_area'] = panel_area
        outputs['tail_panel_ar'] = panel_ar
        outputs['tail_panel_avg_chord'] = panel_avg_chord
        outputs['tail_panel_count'] = n_panels

        outputs['tail_aero_vertical_area'] = vertical_area
        outputs['tail_aero_vertical_ar'] = vertical_ar
        outputs['tail_aero_vertical_span'] = vertical_span
        outputs['tail_aero_horizontal_area'] = horizontal_area
        outputs['tail_aero_horizontal_ar'] = horizontal_ar
        outputs['tail_aero_horizontal_span'] = horizontal_span
        outputs['tail_aero_center_x'] = tail_aero_center_x
        outputs['tail_aero_horizontal_moment_arm'] = tail_moment_arm
        outputs['tail_aero_vertical_moment_arm'] = tail_moment_arm
        outputs['tail_aero_horizontal_volume_coefficient'] = horizontal_volume
        outputs['tail_aero_vertical_volume_coefficient'] = vertical_volume

        outputs['tail_total_wetted_area'] = outputs['tail_physical_wetted_area']
        outputs['tail_fuselage_cutout_area'] = tail_fuselage_cutout_area

        outputs['fuselage_vtp_span_ratio'] = fus_height / vertical_span
        outputs['wing_ref_area'] = wing_area
        outputs['wing_half_span'] = wing_span / 2.0


TAIL_GEOMETRY_BACKENDS = {
    X_TAIL: XTailGeometry,
}


class TailGeometryGroup(om.Group):
    """Select and expose the active SpaJeti tail geometry backend.

    This group is the public tail-geometry boundary used by aircraft models.
    Backends can use layout-specific internals, but must provide the stable
    physical and aero-equivalent tail outputs documented by ``XTailGeometry``.
    """

    def initialize(self):
        self.options.declare(
            'tail_type',
            default=X_TAIL,
            values=TAIL_GEOMETRY_TYPES,
            desc='Tail layout type used to select the geometry backend.',
        )
        self.options.declare(
            'backend_options',
            default=None,
            allow_none=True,
            types=dict,
            desc='Keyword options forwarded to the selected backend component.',
        )

    def setup(self):
        tail_type = self.options['tail_type']
        backend_cls = TAIL_GEOMETRY_BACKENDS[tail_type]
        backend_options = self.options['backend_options'] or {}

        self.add_subsystem(
            'geometry',
            backend_cls(tail_type=tail_type, **backend_options),
            promotes=['*'],
        )
