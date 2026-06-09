import numpy as np
import openmdao.api as om

from aviary.variable_info.functions import add_aviary_input, add_aviary_output
from aviary.variable_info.variables import Aircraft


class HTailGeometry(om.ExplicitComponent):
    """H-tail VTP geometry from first-principle design inputs.

    Primary design inputs are the full VTP panel span (b_v), root chord, taper ratio,
    and sweep.  Panel area and aspect ratio are derived from these so that the full VTP
    shape is driven by geometric design variables rather than by pre-computed area/AR.

    The VTP panel is assumed to be symmetric about its attachment point (HTP tip or
    equivalent), so b_v is the total panel span tip-to-tip.

    Geometry definitions
    --------------------
    b_v     : Aircraft.VerticalTail.SPAN  — full VTP panel span (tip to tip)
    r_i     : fuselage depth radius at VTP quarter-chord station
              = Aircraft.Fuselage.MAX_HEIGHT / 2  (constant cross-section assumed)
    S_v     : Aircraft.VerticalTail.AREA = b_v * c_root * (1 + lambda) / 2
    AR_v    : Aircraft.VerticalTail.ASPECT_RATIO = b_v² / S_v
    S_ref   : wing reference area = wing_area  (taper=1, sweep=0 assumption)

    Notes
    -----
    Aircraft.VerticalTail.SWEEP is accepted as a design input for future use (e.g.
    compressibility corrections, exposed-area calculations) but does not affect the
    current outputs.  No partials are declared for it.

    When integrating into the full model, remove the CSV entries for
    aircraft:vertical_tail:area and aircraft:vertical_tail:aspect_ratio — they will be
    provided by this component.  Add aircraft:vertical_tail:span and
    aircraft:vertical_tail:root_chord instead.
    """

    def setup(self):
        # fuselage depth at VTP quarter-chord station (constant cross-section)
        add_aviary_input(self, Aircraft.Fuselage.MAX_HEIGHT, units='m')

        # VTP direct design inputs
        add_aviary_input(self, Aircraft.VerticalTail.SPAN, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.ROOT_CHORD, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.TAPER_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.VerticalTail.SWEEP, units='deg')

        # Wing inputs for reference area and half-span
        add_aviary_input(self, Aircraft.Wing.SPAN, units='m')
        add_aviary_input(self, Aircraft.Wing.AREA, units='m**2')

        # VTP derived outputs — plain names to avoid conflicting with the FLOPS
        # pre-mission IndepVarComp that supplies aircraft:vertical_tail:area/AR
        # to the Dymos ODE.  Downstream custom aero components (CyBetaVtp,
        # CyDeltaRudder) consume these plain names directly.
        self.add_output('vtp_area',      val=0.08,  units='m**2',    desc='VTP panel area')
        self.add_output('vtp_ar',        val=1.2,   units='unitless', desc='VTP panel aspect ratio')
        self.add_output('vtp_avg_chord', val=0.257, units='m',        desc='VTP panel average chord')

        self.add_output(
            'fuselage_vtp_span_ratio',
            val=0.8,
            units='unitless',
            desc='2*r_i / b_v: fuselage diameter over full VTP panel span (b_v)',
        )
        self.add_output(
            'wing_ref_area',
            val=0.45,
            units='m**2',
            desc='Wing reference area: root_chord * span (taper=1, sweep=0 → equals wing area)',
        )
        self.add_output(
            'wing_half_span',
            val=0.9,
            units='m',
            desc='Wing half-span',
        )

    def setup_partials(self):
        vtp_shape = [
            Aircraft.VerticalTail.SPAN,
            Aircraft.VerticalTail.ROOT_CHORD,
            Aircraft.VerticalTail.TAPER_RATIO,
        ]
        self.declare_partials('vtp_area',      vtp_shape, method='cs')
        self.declare_partials('vtp_ar',        vtp_shape, method='cs')
        self.declare_partials('vtp_avg_chord', vtp_shape, method='cs')

        self.declare_partials(
            'fuselage_vtp_span_ratio',
            [Aircraft.Fuselage.MAX_HEIGHT, Aircraft.VerticalTail.SPAN],
            method='cs',
        )
        self.declare_partials('wing_ref_area', Aircraft.Wing.AREA, method='cs')
        self.declare_partials('wing_half_span', Aircraft.Wing.SPAN, method='cs')

    def compute(self, inputs, outputs):
        fus_height = inputs[Aircraft.Fuselage.MAX_HEIGHT]
        b_v = inputs[Aircraft.VerticalTail.SPAN]
        c_root = inputs[Aircraft.VerticalTail.ROOT_CHORD]
        tr = inputs[Aircraft.VerticalTail.TAPER_RATIO]
        wing_span = inputs[Aircraft.Wing.SPAN]
        wing_area = inputs[Aircraft.Wing.AREA]

        r_i = fus_height / 2.0
        s_v = b_v * c_root * (1.0 + tr) / 2.0
        ar_v = b_v ** 2 / s_v
        mean_chord = s_v / b_v  # = c_root * (1 + tr) / 2

        outputs['vtp_area']      = s_v
        outputs['vtp_ar']        = ar_v
        outputs['vtp_avg_chord'] = mean_chord
        outputs['fuselage_vtp_span_ratio'] = (2.0 * r_i) / b_v
        # For taper=1: root_chord = wing_area/wing_span, S_ref = root_chord*span = wing_area
        outputs['wing_ref_area'] = wing_area
        outputs['wing_half_span'] = wing_span / 2.0
