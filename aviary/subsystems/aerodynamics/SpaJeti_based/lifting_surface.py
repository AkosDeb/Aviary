"""
Modular lifting surface aerodynamic sub-chains.

Classes
-------
AirfoilConstantsComp
    IndepVarComp that exposes AirfoilData section properties as OpenMDAO
    outputs at Group scope.  Visible in N2; overridable via prob.set_val
    for parametric airfoil studies.

LiftingSurfaceGroup (om.Group)
    Base class.  Six-step setup sequence driven by overridable hooks:
      1. airfoil_consts          -- AirfoilConstantsComp -> section_* at Group scope
      2. _setup_ar_correction    -- AR_eff, k_eff  (identity by default)
      3. _setup_lift_curve_slope -- Polhamus CL_alpha -> surface_CL_alpha
      4. _setup_derivatives      -- surface-specific extras (empty by default)
      5. _setup_mach_critical    -- M_DD, M_crit, mach_crit_margin (empty by default;
                                     overridden in WingSurface via MachCriticalComp)
      6. _setup_mac_geometry     -- MAC chord, y_mac, x/z positions (empty by default;
                                     overridden in WingSurface via MACGeometryComp)

WingSurface (LiftingSurfaceGroup)
    Wing: ScholzWingletARCorrection -> Polhamus with K_wf fuselage factor.

VTPSurface (LiftingSurfaceGroup)
    VTP: physical-panel Polhamus + TailCantRotation (cant-angle decomposition).

Generic internal variable names
--------------------------------
These are used INSIDE each Group.  Map them to surface-type-specific Aviary
names at the call site via the promotes_inputs list of add_subsystem:

  surface_ar        -- geometric aspect ratio
  surface_span      -- full span [m]
  surface_sweep_c4  -- quarter-chord sweep [deg]
  surface_taper     -- taper ratio
  design_mach       -- Mach at the constraint/design flight condition

  WingSurface only:
    endplate_span   -- map to Aircraft.VerticalTail.SPAN at call site
    fuselage_diameter -- injected internally from SurfaceConfig (no call-site wiring needed)

  VTPSurface only (consumed from model scope, not via generic names):
    surface_ar   -> 'tail_panel_ar'  (physical panel AR from TailGeometryGroup)
    wing_ref_area, tail_physical_panel_area, tail_cant_angle, tail_panel_count

Section properties (Group scope only -- NOT promoted to model level by default)
--------------------------------------------------------------------------------
  section_cl_alpha  -- 2-D incompressible cl_alpha [/rad]
  section_cl_max    -- maximum 2-D lift coefficient
  section_cd_min    -- minimum profile drag coefficient
  section_cm_ac     -- pitching moment about AC (quarter-chord)
  section_tc        -- thickness-to-chord ratio
  section_camber    -- max camber ratio

  Future components inside the Group (M_crit, parasite drag, divergence speed,
  component mass) connect to these names directly -- no call-site wiring needed.

Outputs promoted to model scope (listed in call-site promotes_outputs)
-----------------------------------------------------------------------
  WingSurface:
    ('surface_CL_alpha', 'wing_CL_alpha')
    ('k_eff',            'k_h')    -- AR correction factor (Scholz k_h)
    'AR_eff'                        -- effective aspect ratio
    'K_wf'                          -- fuselage carry-through factor
    'M_DD'                          -- drag-divergence Mach (Weisshaar Eq. 36)
    'M_crit'                        -- critical Mach: M_DD - (0.1/80)^(1/3)  [informational]
    'mach_crit_margin'              -- M_crit - mach_upper_bound; constrain >= 0
    'wing_root_chord'               -- root chord c_r = 2*S/(b*(1+lambda))
    'wing_c_mac'                    -- mean aerodynamic chord
    'wing_y_mac'                    -- spanwise BL of MAC from centreline
    'wing_x_mac_le'                 -- x-station of MAC LE from nose (positive aft)
    'wing_z_mac_le'                 -- z-station of MAC LE from nose datum (positive down)
    'wing_x_mac_c4'                 -- x-station of MAC quarter-chord (aerodynamic centre x)
    'wing_le_sweep'                 -- leading-edge sweep angle Lambda_LE

  VTPSurface:
    ('surface_CL_alpha', 'CL_alpha_v')  -- physical panel lift slope [/rad]
    'CY_beta_tail'                      -- body-axis dCY/dbeta [/rad]
    'CL_alpha_tail'                     -- body-axis dCL/dalpha [/rad]
"""

import openmdao.api as om
from aviary.variable_info.variables import Aircraft

from aviary.subsystems.aerodynamics.SpaJeti_based.airfoil_data import AirfoilData
from aviary.subsystems.aerodynamics.SpaJeti_based.surface_config import SurfaceConfig
from aviary.subsystems.aerodynamics.SpaJeti_based.lift_curve_slope import (
    ScholzWingletARCorrection,
    LiftCurveSlopePolhamus,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.cy_beta_vtp import (
    CyBetaVtp, CyDeltaRudder, TailCantRotation,
)
from aviary.subsystems.aerodynamics.SpaJeti_based.mach_critical import MachCriticalComp
from aviary.subsystems.aerodynamics.SpaJeti_based.surface_geometry import MACGeometryComp


class AirfoilConstantsComp(om.IndepVarComp):
    """Exposes AirfoilData section properties as OpenMDAO independent variables.

    Using IndepVarComp makes values visible in N2 and overridable via
    prob.set_val for parametric airfoil studies.  Outputs are at Group scope
    only -- the containing LiftingSurfaceGroup does not promote them to model
    level by default.
    """

    def __init__(self, airfoil: AirfoilData, **kwargs):
        super().__init__(**kwargs)
        self._airfoil = airfoil

    def setup(self):
        a = self._airfoil
        self.add_output(
            'section_cl_alpha',
            val=float(a.cl_alpha_per_rad),
            units='unitless',
            desc=(
                f'{a.name} incompressible 2-D lift slope [/rad] at Re={a.re_ref:.0e}. '
                'DO NOT Prandtl-Glauert correct before passing to Polhamus -- '
                'the Polhamus formula handles 3-D compressibility via beta=sqrt(1-M^2).'
            ),
        )
        self.add_output('section_cl_max',
                        val=float(a.cl_max), units='unitless',
                        desc=f'{a.name} maximum 2-D lift coefficient at Re={a.re_ref:.0e}')
        self.add_output('section_cd_min',
                        val=float(a.cd_min), units='unitless',
                        desc=f'{a.name} minimum profile drag coefficient')
        self.add_output('section_cm_ac',
                        val=float(a.cm_ac), units='unitless',
                        desc=f'{a.name} pitching moment about AC (quarter-chord)')
        self.add_output('section_tc',
                        val=float(a.tc_ratio), units='unitless',
                        desc=f'{a.name} thickness-to-chord ratio t/c')
        self.add_output('section_camber',
                        val=float(a.camber_ratio), units='unitless',
                        desc=f'{a.name} maximum camber-to-chord ratio')
        self.add_output('section_max_thickness_location',
                        val=float(a.max_thickness_location), units='unitless',
                        desc=f'{a.name} maximum thickness location `(x/c)_m`')
        self.add_output('section_leading_edge_radius_ratio',
                        val=float(a.leading_edge_radius_ratio), units='unitless',
                        desc=f'{a.name} leading-edge radius ratio `r_LE/c`')


class LiftingSurfaceGroup(om.Group):
    """Base Group for a lifting surface aerodynamic sub-chain.

    Subclass and override the four hook methods to customise each surface type.
    See module docstring for variable name conventions.
    """

    def __init__(self, cfg: SurfaceConfig, **kwargs):
        super().__init__(**kwargs)
        self._cfg = cfg

    def setup(self):
        cfg = self._cfg

        # 1. Airfoil section constants -- available to all downstream components
        self.add_subsystem(
            'airfoil_consts',
            AirfoilConstantsComp(cfg.airfoil),
            promotes_outputs=[
                'section_cl_alpha', 'section_cl_max', 'section_cd_min',
                'section_cm_ac', 'section_tc', 'section_camber',
                'section_max_thickness_location', 'section_leading_edge_radius_ratio',
            ],
        )

        self._setup_ar_correction()
        self._setup_lift_curve_slope()
        self._setup_derivatives()
        self._setup_mach_critical()
        self._setup_mac_geometry()

    def _setup_ar_correction(self):
        """Default: AR_eff = surface_ar (identity), k_eff = 1.0 (no correction)."""
        self.add_subsystem(
            'ar_passthrough',
            om.ExecComp(
                'AR_eff = surface_ar',
                AR_eff={'val': 4.0, 'units': 'unitless'},
                surface_ar={'val': 4.0, 'units': 'unitless'},
            ),
            promotes=['*'],
        )
        self.add_subsystem(
            'k_eff_const',
            om.IndepVarComp('k_eff', val=1.0, units='unitless',
                            desc='AR correction factor (1.0 -- no endplate correction)'),
            promotes_outputs=['k_eff'],
        )

    def _setup_lift_curve_slope(self):
        """Polhamus CL_alpha with no fuselage correction (K_wf = 1 by default).

        Override in WingSurface to also wire fuselage_diameter and wing_span.
        """
        self.add_subsystem(
            'polhamus', LiftCurveSlopePolhamus(),
            promotes_inputs=[
                ('aspect_ratio',       'AR_eff'),
                ('sweep_c4_deg',       'surface_sweep_c4'),
                ('taper_ratio',        'surface_taper'),
                ('section_lift_slope', 'section_cl_alpha'),
                ('mach',               'design_mach'),
            ],
            promotes_outputs=[('CL_alpha', 'surface_CL_alpha')],
        )

    def _setup_derivatives(self):
        """Override in VTPSurface (and future HTPSurface) for surface-specific extras."""
        pass

    def _setup_mach_critical(self):
        """Override in WingSurface to add MachCriticalComp.

        Default: no-op.  VTP and HTP M_crit is less critical at subsonic design
        Mach numbers and omitted unless explicitly needed.
        """
        pass

    def _setup_mac_geometry(self):
        """Override in WingSurface (and future HTPSurface) to add MACGeometryComp.

        Default: no-op.
        Inputs required at Group scope: surface_area, surface_span, surface_taper,
        surface_sweep_c4, dihedral_deg, and surface-specific apex positions
        (wing_x_apex / wing_z_apex) promoted from model scope.
        """
        pass


class WingSurface(LiftingSurfaceGroup):
    """Wing lifting surface sub-chain.

    Extends the base Group with:
    - A ``fuselage_diameter`` group input with a SurfaceConfig default.
    - ``ScholzWingletARCorrection`` (Scholz INCAS 2018) for H-tail VTP endplate effect.
    - ``LiftCurveSlopePolhamus`` wired with ``fuselage_diameter`` and ``surface_span``
      to activate the K_wf fuselage carry-through correction.

    Extra generic input beyond the base set:
      ``endplate_span`` -- map to Aircraft.VerticalTail.SPAN at the call site.

    Extra outputs promoted to model scope (via call-site promotes_outputs):
      ``AR_eff``, ``('k_eff', 'k_h')``, ``K_wf``
    """

    def setup(self):
        # The run script can promote this input from SuperellipseFuselageGeometry's
        # canonical fuselage_equivalent_diameter.  The SurfaceConfig value remains
        # the fallback default for standalone component use.
        self.set_input_defaults(
            'fuselage_diameter',
            val=float(self._cfg.fuselage_diameter),
            units='m',
        )
        # Both ar_correction and polhamus promote 'wing_span' to 'surface_span'; their
        # component-level defaults differ (ScholzWingletARCorrection uses 1.8 m, Polhamus
        # uses 1.0 m).  Declare an authoritative Group-scope default to silence the
        # OpenMDAO ambiguity check.  The actual value is always overridden externally.
        self.set_input_defaults('surface_span', val=1.5, units='m')
        super().setup()

    def _setup_ar_correction(self):
        cfg = self._cfg
        self.add_subsystem(
            'ar_correction',
            ScholzWingletARCorrection(k_wl=cfg.k_wl, split_penalty=cfg.split_penalty),
            promotes_inputs=[
                ('aspect_ratio', 'surface_ar'),
                ('vtp_span',     'endplate_span'),
                ('wing_span',    'surface_span'),
            ],
            promotes_outputs=[
                ('AR_eff', 'AR_eff'),
                ('k_h',    'k_eff'),
            ],
        )

    def _setup_lift_curve_slope(self):
        """Wing Polhamus with fuselage_diameter (Group scope) and wing_span for K_wf."""
        self.add_subsystem(
            'polhamus', LiftCurveSlopePolhamus(),
            promotes_inputs=[
                ('aspect_ratio',       'AR_eff'),
                ('sweep_c4_deg',       'surface_sweep_c4'),
                ('taper_ratio',        'surface_taper'),
                ('section_lift_slope', 'section_cl_alpha'),
                ('mach',               'design_mach'),
                ('fuselage_diameter',  'fuselage_diameter'),
                ('wing_span',          'surface_span'),
            ],
            promotes_outputs=[
                ('CL_alpha', 'surface_CL_alpha'),
                'K_wf',
            ],
        )

    def _setup_mach_critical(self):
        """Wing M_DD and M_crit via Weisshaar Eq. 36 (K_A = 0.887).

        CL for Weisshaar is derived from the Nz_min requirement:
          CL_nz = nz_min * aircraft_mass * g / (dynamic_pressure * wing_area)
          CL    = CL_nz * mcrit_cl_safety_factor

        Group-scope inputs consumed here:
          section_tc            -- from AirfoilConstantsComp
          surface_sweep_c4      -- generic wing sweep input
          nz_min                -- from model scope (load_cond IndepVarComp)
          aircraft_mass         -- from model scope (load_cond IndepVarComp)
          dynamic_pressure      -- from model scope (load_cond IndepVarComp)
          Aircraft.Wing.AREA    -- from model scope (Aviary variable)
          mach_upper_bound      -- from model scope (load_cond IndepVarComp)

        Outputs promoted to model scope (add to call-site promotes_outputs):
          M_DD (informational), M_crit, mach_crit_margin = M_crit - mach_upper_bound
        """
        self.add_subsystem(
            'mach_critical',
            MachCriticalComp(k_a=0.887),
            promotes_inputs=[
                'surface_sweep_c4',
                'section_tc',
                'nz_min',
                'aircraft_mass',
                'dynamic_pressure',
                ('wing_area', Aircraft.Wing.AREA),
                'mach_upper_bound',
            ],
            promotes_outputs=['M_DD', 'M_crit', 'mach_crit_margin'],
        )

    def _setup_mac_geometry(self):
        """Wing MAC chord and body-frame position via MACGeometryComp.

        Body-fixed frame (origin at nose tip):
          x -- positive AFT (fuselage station)
          y -- positive STARBOARD
          z -- positive DOWN

        Apex inputs (wing_x_apex, wing_z_apex) are provided from model scope
        via the load_cond IndepVarComp (geometric inputs; can be promoted to
        design variables in a future CG/stability loop).

        Group-scope inputs consumed here:
          surface_area     -- must be promoted at call site (e.g. aircraft:wing:area)
          surface_span     -- generic span, already at Group scope
          surface_taper    -- generic taper, already at Group scope
          surface_sweep_c4 -- generic c/4 sweep, already at Group scope
          dihedral_deg     -- must be promoted at call site (e.g. aircraft:wing:dihedral)
          wing_x_apex      -- x-station of wing root LE from nose [m]
          wing_z_apex      -- z-station of wing root LE from nose datum [m]

        Outputs promoted to model scope (add to call-site promotes_outputs):
          wing_root_chord, wing_c_mac, wing_y_mac,
          wing_x_mac_le, wing_z_mac_le, wing_x_mac_c4
        """
        self.add_subsystem(
            'mac_geom',
            MACGeometryComp(),
            promotes_inputs=[
                'surface_area',
                'surface_span',
                'surface_taper',
                'surface_sweep_c4',
                'dihedral_deg',
                ('x_apex', 'wing_x_apex'),
                ('z_apex', 'wing_z_apex'),
            ],
            promotes_outputs=[
                ('root_chord', 'wing_root_chord'),
                ('c_mac',      'wing_c_mac'),
                ('y_mac',      'wing_y_mac'),
                ('x_mac_le',   'wing_x_mac_le'),
                ('z_mac_le',   'wing_z_mac_le'),
                ('x_mac_c4',   'wing_x_mac_c4'),
                ('leading_edge_sweep', 'wing_le_sweep'),
            ],
        )


class VTPSurface(LiftingSurfaceGroup):
    """VTP lifting surface sub-chain.

    Extends the base Group with:
    - ``LiftCurveSlopePolhamus`` on raw geometric AR (no endplate correction on the VTP).
    - ``CyBetaVtp``: passive side-force slope for the H-tail twin-panel layout.
    - ``CyDeltaRudder`` (optional): rudder effectiveness when
      ``SurfaceConfig.has_control_surface=True``.

    Generic inputs mapped at the call site:
      ``surface_ar``       -> 'tail_panel_ar'          (physical panel AR from TailGeometryGroup)
      ``surface_sweep_c4`` -> Aircraft.VerticalTail.SWEEP
      ``surface_taper``    -> Aircraft.VerticalTail.TAPER_RATIO

    Inputs consumed directly from model scope (promoted through the Group):
      wing_ref_area, tail_physical_panel_area, tail_cant_angle, tail_panel_count

    Outputs promoted to model scope (via call-site promotes_outputs):
      ``('surface_CL_alpha', 'CL_alpha_v')``, ``CY_beta_tail``, ``CL_alpha_tail``
    """

    def _setup_derivatives(self):
        self.add_subsystem(
            'cant_rotation', TailCantRotation(),
            promotes_inputs=[
                ('CL_alpha_panel', 'surface_CL_alpha'),
                'tail_physical_panel_area',
                'tail_cant_angle',
                'tail_panel_count',
                'wing_ref_area',
            ],
            promotes_outputs=['CY_beta_tail', 'CL_alpha_tail'],
        )
