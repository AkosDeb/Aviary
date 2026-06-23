import warnings

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class WingStructuralMass(om.ExplicitComponent):
    """Wing mass and CG in the body-fixed frame (origin at nose tip, x forward).

    Mass is the integral of SPANWISE_MASS_PER_UNIT_SPAN (from SpanwiseWingboxProperties)
    over the semispan, doubled for both half-wings, with a non-structural knockdown factor
    for ribs, fasteners, and surface finish.

    The chordwise CG of each cross-section is computed from the structural layout directly —
    NOT from the elastic axis.  For a two-spar box with equal spar and skin thicknesses the
    mass-weighted chordwise centroid reduces exactly to the box midpoint:

        CG_fraction_from_LE = (FRONT_SPAR_FRACTION + REAR_SPAR_FRACTION) / 2

    The body-frame x of the wing CG (x positive forward, nose at x=0):

        x_cg_wing = wing_x_apex
                    - box_mid_frac × (∫chord(y)·m(y) dy / ∫m(y) dy)

    Assumptions:
        - Zero leading-edge sweep (LE straight across span → x_LE = wing_x_apex everywhere)
        - Taper ratio ∈ [0, 1]; hard stop if violated
        - Spar fractions constant along span

    wing_x_apex is negative (aft of nose origin).
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)
        self.options.declare('taper_ratio_min_warn', default=0.3, types=float)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(AE.SPANWISE_STATIONS, val=np.linspace(0.0, 1.0, n), units='m')
        self.add_input(AE.SPANWISE_CHORD, val=np.ones(n) * 0.18, units='m')
        self.add_input(AE.SPANWISE_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_input(AE.FRONT_SPAR_FRACTION, val=0.15, units='unitless')
        self.add_input(AE.REAR_SPAR_FRACTION, val=0.55, units='unitless')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0, units='unitless')
        self.add_input(Aircraft.Wing.SWEEP, val=0.0, units='deg')

        self.add_input('wing_x_apex', val=-0.7, units='m',
                       desc='x-position of wing root LE in body frame (negative = aft of nose)')
        self.add_input('wing_z_apex', val=0.0, units='m',
                       desc='z-position of wing root LE in body frame (positive = down)')
        self.add_input('non_structural_mass_fraction', val=0.25, units='unitless')

        self.add_output('wing_structural_mass', val=3.0, units='kg')
        self.add_output('wing_x_cg', val=-0.75, units='m')
        self.add_output('wing_z_cg', val=0.0, units='m')

        self.declare_partials(
            'wing_structural_mass',
            [
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_MASS_PER_UNIT_SPAN,
                'non_structural_mass_fraction',
            ],
            method='cs',
        )
        self.declare_partials(
            'wing_x_cg',
            [
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_CHORD,
                AE.SPANWISE_MASS_PER_UNIT_SPAN,
                AE.FRONT_SPAR_FRACTION,
                AE.REAR_SPAR_FRACTION,
                'wing_x_apex',
            ],
            method='cs',
        )
        self.declare_partials('wing_z_cg', 'wing_z_apex', method='cs')

    def compute(self, inputs, outputs):
        taper = float(np.real(np.asarray(inputs[Aircraft.Wing.TAPER_RATIO]).item()))
        sweep_deg = float(np.real(np.asarray(inputs[Aircraft.Wing.SWEEP]).item()))

        if taper < 0.0 or taper > 1.0:
            raise ValueError(
                f'WingStructuralMass: taper_ratio={taper:.3f} is outside [0, 1]. '
                'Forward-tapered wings (taper > 1) and negative taper are not supported.'
            )
        if sweep_deg < 0.0:
            raise ValueError(
                f'WingStructuralMass: sweep_angle={sweep_deg:.1f} deg is negative '
                '(forward sweep). The straight-LE CG formula requires zero or aft sweep.'
            )
        if taper < self.options['taper_ratio_min_warn']:
            warnings.warn(
                f'WingStructuralMass: taper_ratio={taper:.3f} is very low (<'
                f'{self.options["taper_ratio_min_warn"]}). CG estimate is still '
                'valid but the very small tip chord may reduce accuracy.',
                stacklevel=2,
            )

        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        chord = np.asarray(inputs[AE.SPANWISE_CHORD])
        m_per_span = np.asarray(inputs[AE.SPANWISE_MASS_PER_UNIT_SPAN])
        fs = np.asarray(inputs[AE.FRONT_SPAR_FRACTION]).item()
        rs = np.asarray(inputs[AE.REAR_SPAR_FRACTION]).item()
        fraction = np.asarray(inputs['non_structural_mass_fraction']).item()
        x_apex = np.asarray(inputs['wing_x_apex']).item()
        z_apex = np.asarray(inputs['wing_z_apex']).item()

        m_half = np.trapezoid(m_per_span, y)
        wing_mass = 2.0 * m_half * (1.0 + fraction)

        # Mass-weighted chordwise offset from LE to structural CG
        # (box_mid = midpoint between front and rear spar)
        box_mid_frac = 0.5 * (fs + rs)
        m_total_half = m_half if abs(np.real(m_half)) > 1.0e-12 else 1.0e-12
        c_mass_weighted = np.trapezoid(chord * m_per_span, y) / m_total_half
        # x positive forward; going from LE aft reduces x
        x_cg = x_apex - box_mid_frac * c_mass_weighted

        outputs['wing_structural_mass'] = wing_mass
        outputs['wing_x_cg'] = x_cg
        outputs['wing_z_cg'] = z_apex
