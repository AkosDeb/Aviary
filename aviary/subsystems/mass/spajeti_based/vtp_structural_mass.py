import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


class VTPStructuralMass(om.ExplicitComponent):
    """VTP structural mass and body-frame CG for an H-wing (dual-panel wingtip) aircraft.

    Uses the same geometry inputs as VTPTipInertia so both the flutter model and the
    weight estimation see a consistent VTP description when the optimizer changes
    Aircraft.VerticalTail.SPAN.

    Convention
    ----------
    AE.VTP_TIP_PANEL_COUNT = panels per wingtip (default 2 = one up + one down).
    num_wing_tips (option, default 2) = number of wingtips carrying VTP panels.
    Total VTP panels on the aircraft = VTP_TIP_PANEL_COUNT × num_wing_tips.

    Formula
    -------
    panel_area   = vtp_span × vtp_root_chord × (1 + vtp_taper) / 2
    vtp_structural_mass = panel_area × vtp_areal_density × panel_count × num_wing_tips

    Body-frame CG (x positive FORWARD, nose at origin, z positive DOWN)
    -------------------------------------------------------------------
    wing_tip_le_x = wing_x_apex − 0.5 × wing_span × tan(wing_sweep_rad)
    vtp_mac       = (2/3) × vtp_root_chord × (1 + taper + taper²) / (1 + taper)
    box_mid_frac  = (front_spar_frac + rear_spar_frac) / 2
    vtp_x_cg      = wing_tip_le_x − box_mid_frac × vtp_mac

    vtp_z_cg = wing_z_apex  (panels are symmetric: one up + one down, offsets cancel)

    Inputs
    ------
    Aircraft.VerticalTail.SPAN        m           VTP panel span (vertical)
    Aircraft.VerticalTail.ROOT_CHORD  m           VTP root chord at wingtip
    Aircraft.VerticalTail.TAPER_RATIO —           VTP taper ratio
    AE.VTP_AREAL_DENSITY              kg/m²       Composite areal density (shared with VTPTipInertia)
    AE.VTP_TIP_PANEL_COUNT            —           Panels per wingtip (shared with VTPTipInertia)
    Aircraft.Wing.SPAN                m           Full wing span (for wingtip x-position)
    Aircraft.Wing.SWEEP               deg         Wing leading-edge sweep
    wing_x_apex                       m           Wing root LE x (negative = aft of nose, FORWARD frame)
    wing_z_apex                       m           Wing root LE z (positive = down)
    AE.FRONT_SPAR_FRACTION            —           Front spar chordwise position
    AE.REAR_SPAR_FRACTION             —           Rear spar chordwise position

    Outputs
    -------
    vtp_structural_mass  kg   Total VTP mass for the full aircraft
    vtp_x_cg             m    VTP CG x-position in body frame (negative = aft of nose)
    vtp_z_cg             m    VTP CG z-position in body frame
    """

    def initialize(self):
        self.options.declare(
            'num_wing_tips', default=2, types=int,
            desc='Number of wingtips carrying VTP panels (2 for conventional H-wing)',
        )

    def setup(self):
        # VTP geometry (shared with VTPTipInertia)
        self.add_input(Aircraft.VerticalTail.SPAN, val=0.31, units='m')
        self.add_input(Aircraft.VerticalTail.ROOT_CHORD, val=0.20, units='m')
        self.add_input(Aircraft.VerticalTail.TAPER_RATIO, val=1.0, units='unitless')
        self.add_input(AE.VTP_AREAL_DENSITY, val=1.2, units='kg/m**2')
        self.add_input(AE.VTP_TIP_PANEL_COUNT, val=2.0, units='unitless')

        # Wing geometry for wingtip x-position
        self.add_input(Aircraft.Wing.SPAN, val=1.8, units='m')
        self.add_input(Aircraft.Wing.SWEEP, val=0.0, units='deg')
        self.add_input('wing_x_apex', val=-0.80, units='m',
                       desc='Wing root LE x in body frame (x positive FORWARD; negative = aft of nose)')
        self.add_input('wing_z_apex', val=0.0, units='m',
                       desc='Wing root LE z in body frame (positive = down)')

        # Structural box fractions (shared with WingStructuralMass)
        self.add_input(AE.FRONT_SPAR_FRACTION, val=0.15, units='unitless')
        self.add_input(AE.REAR_SPAR_FRACTION, val=0.55, units='unitless')

        self.add_output('vtp_structural_mass', val=0.25, units='kg',
                        desc='Total VTP structural mass for full aircraft')
        self.add_output('vtp_x_cg', val=-0.90, units='m',
                        desc='VTP CG x in body frame (negative = aft of nose)')
        self.add_output('vtp_z_cg', val=0.0, units='m',
                        desc='VTP CG z in body frame (0: up+down panels cancel)')

        self.declare_partials(
            'vtp_structural_mass',
            [
                Aircraft.VerticalTail.SPAN,
                Aircraft.VerticalTail.ROOT_CHORD,
                Aircraft.VerticalTail.TAPER_RATIO,
                AE.VTP_AREAL_DENSITY,
                AE.VTP_TIP_PANEL_COUNT,
            ],
            method='cs',
        )
        self.declare_partials(
            'vtp_x_cg',
            [
                Aircraft.VerticalTail.ROOT_CHORD,
                Aircraft.VerticalTail.TAPER_RATIO,
                Aircraft.Wing.SPAN,
                Aircraft.Wing.SWEEP,
                'wing_x_apex',
                AE.FRONT_SPAR_FRACTION,
                AE.REAR_SPAR_FRACTION,
            ],
            method='cs',
        )
        self.declare_partials('vtp_z_cg', 'wing_z_apex', method='cs')

    def compute(self, inputs, outputs):
        def _f(key):
            return np.asarray(inputs[key]).item()

        vtp_span       = _f(Aircraft.VerticalTail.SPAN)
        vtp_root_chord = _f(Aircraft.VerticalTail.ROOT_CHORD)
        vtp_taper      = _f(Aircraft.VerticalTail.TAPER_RATIO)
        areal_density  = _f(AE.VTP_AREAL_DENSITY)
        panel_count    = _f(AE.VTP_TIP_PANEL_COUNT)

        wing_span      = _f(Aircraft.Wing.SPAN)
        wing_sweep_deg = _f(Aircraft.Wing.SWEEP)
        wing_x_apex    = _f('wing_x_apex')
        wing_z_apex    = _f('wing_z_apex')

        front_spar     = _f(AE.FRONT_SPAR_FRACTION)
        rear_spar      = _f(AE.REAR_SPAR_FRACTION)

        n_tips = self.options['num_wing_tips']

        # Total VTP mass: one panel per tip × panel_count × n_tips
        panel_area = vtp_span * vtp_root_chord * (1.0 + vtp_taper) / 2.0
        vtp_mass   = panel_area * areal_density * panel_count * n_tips

        # VTP mean aerodynamic chord
        vtp_mac = (
            2.0 / 3.0 * vtp_root_chord
            * (1.0 + vtp_taper + vtp_taper**2)
            / (1.0 + vtp_taper)
        )

        # Wing tip LE x-position (x positive FORWARD; tip is further AFT than root if swept)
        wing_sweep_rad = wing_sweep_deg * (np.pi / 180.0)
        wing_tip_le_x  = wing_x_apex - 0.5 * wing_span * np.tan(wing_sweep_rad)

        # Structural box midpoint chordwise fraction
        box_mid_frac = (front_spar + rear_spar) / 2.0

        # VTP CG: wing tip LE minus box midpoint × MAC (more AFT = more negative)
        x_cg_vtp = wing_tip_le_x - box_mid_frac * vtp_mac

        # z-CG: one panel up + one down per tip; symmetric offsets cancel → wing z
        z_cg_vtp = wing_z_apex

        outputs['vtp_structural_mass'] = vtp_mass
        outputs['vtp_x_cg']            = x_cg_vtp
        outputs['vtp_z_cg']            = z_cg_vtp
