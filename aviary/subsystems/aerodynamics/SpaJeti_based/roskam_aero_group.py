"""
Roskam/DATCOM mission aerodynamics group.

Drop-in replacement for FLOPS ``ComputedAeroGroup``.  Used via
``RoskamAeroBuilder`` which is registered with Aviary via
``prob.load_external_subsystems()``.

Component order inside the Roskam build-up: 0 = wing, 1 = VTP, 2 = fuselage.
"""

import numpy as np
import openmdao.api as om

from aviary.subsystems.aerodynamics.aero_common import DynamicPressure
from aviary.subsystems.aerodynamics.SpaJeti_based.parasite_drag import RoskamParasiteDragBuildUp
from aviary.variable_info.variables import Aircraft, Dynamic


_COMPONENT_NAMES = ('wing', 'vtp', 'fuselage')
_COMPONENT_KINDS = ('lifting_surface', 'vtp', 'fuselage')
_GRAV_METRIC = 9.80665
_ETA_FIN_CYL_LD = np.array([2.0, 6.0, 12.0, 18.0, 28.0])
_ETA_FIN_CYL_ETA = np.array([0.52, 0.64, 0.71, 0.75, 0.79])
_CDC_MC = np.array([0.00, 0.25, 0.40, 0.50, 0.70])
_CDC_CDC = np.array([1.20, 1.20, 1.27, 1.37, 1.68])


class _LiftEqualsWeightLocal(om.ExplicitComponent):
    """Local mission lift balance and CL calculation."""

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int)

    def setup(self):
        nn = self.options['num_nodes']
        self.add_input(Dynamic.Vehicle.MASS, val=np.ones(nn), units='kg')
        self.add_input(Dynamic.Atmosphere.DYNAMIC_PRESSURE, val=np.ones(nn), units='Pa')
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_output(Dynamic.Vehicle.LIFT, val=np.ones(nn), units='N')
        self.add_output('cl', val=np.ones(nn), units='unitless')
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        mass = inputs[Dynamic.Vehicle.MASS]
        q = inputs[Dynamic.Atmosphere.DYNAMIC_PRESSURE]
        s_ref = float(inputs[Aircraft.Wing.AREA].ravel()[0])
        if s_ref <= 0.0:
            raise ValueError(f'_LiftEqualsWeightLocal: wing reference area must be > 0; got {s_ref}.')
        if np.any(q <= 0.0):
            raise ValueError(f'_LiftEqualsWeightLocal: dynamic pressure must be > 0; got {q}.')

        lift = mass * _GRAV_METRIC
        outputs[Dynamic.Vehicle.LIFT] = lift
        outputs['cl'] = lift / (q * s_ref)


class _RoskamMissionInducedDrag(om.ExplicitComponent):
    """Mission-node wing induced drag using the SpaJeti/Roskam span-efficiency path."""

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int)

    def setup(self):
        nn = self.options['num_nodes']
        self.add_input('cl', val=np.ones(nn) * 0.5, units='unitless')
        self.add_input(Aircraft.Wing.ASPECT_RATIO, val=8.0, units='unitless')
        self.add_input(Aircraft.Wing.SPAN_EFFICIENCY_FACTOR, val=0.9, units='unitless')
        self.add_output('CDI', val=np.ones(nn) * 0.04, units='unitless')
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        cl = inputs['cl']
        ar_geo = float(inputs[Aircraft.Wing.ASPECT_RATIO].ravel()[0])
        e_span = float(inputs[Aircraft.Wing.SPAN_EFFICIENCY_FACTOR].ravel()[0])
        if ar_geo <= 0.0:
            raise ValueError(f'_RoskamMissionInducedDrag: aspect ratio must be > 0; got {ar_geo}.')
        if e_span <= 0.0:
            raise ValueError(
                f'_RoskamMissionInducedDrag: span efficiency factor must be > 0; got {e_span}.'
            )
        outputs['CDI'] = cl**2 / (np.pi * ar_geo * e_span)


class _FuselageMissionLiftDrag(om.ExplicitComponent):
    """Mission-node Roskam fuselage drag due to lift, Eq. 4.33."""

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int)

    def setup(self):
        nn = self.options['num_nodes']
        self.add_input('cl', val=np.ones(nn) * 0.5, units='unitless')
        self.add_input('wing_CL_alpha', val=5.0, units='unitless')
        self.add_input(Dynamic.Atmosphere.MACH, val=np.ones(nn) * 0.3, units='unitless')
        self.add_input('fuselage_base_area', val=0.001, units='m**2')
        self.add_input('fuselage_planform_area', val=0.25, units='m**2')
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input('fuselage_fineness_ratio', val=8.0, units='unitless')
        self.add_output('CDI_fus', val=np.zeros(nn), units='unitless')
        self.add_output('mission_alpha', val=np.zeros(nn), units='rad')
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        cl = inputs['cl']
        cl_alpha = float(inputs['wing_CL_alpha'].ravel()[0])
        mach = inputs[Dynamic.Atmosphere.MACH]
        s_b = float(inputs['fuselage_base_area'].ravel()[0])
        s_plf = float(inputs['fuselage_planform_area'].ravel()[0])
        s_ref = float(inputs[Aircraft.Wing.AREA].ravel()[0])
        fin_ratio = float(inputs['fuselage_fineness_ratio'].ravel()[0])

        if cl_alpha <= 0.0:
            raise ValueError(f'_FuselageMissionLiftDrag: wing_CL_alpha must be > 0; got {cl_alpha}.')
        if s_b < 0.0 or s_plf < 0.0:
            raise ValueError('_FuselageMissionLiftDrag: fuselage areas must be non-negative.')
        if s_ref <= 0.0:
            raise ValueError(f'_FuselageMissionLiftDrag: reference area must be > 0; got {s_ref}.')

        alpha_rad = cl / cl_alpha
        eta = np.interp(fin_ratio, _ETA_FIN_CYL_LD, _ETA_FIN_CYL_ETA)
        c_d_c = np.interp(mach * np.sin(alpha_rad), _CDC_MC, _CDC_CDC)
        base_area_term = 2.0 * alpha_rad**2 * s_b / s_ref
        planform_term = eta * c_d_c * alpha_rad**3 * s_plf / s_ref

        outputs['mission_alpha'] = alpha_rad
        outputs['CDI_fus'] = base_area_term + planform_term


class _GeomArrayAssembler(om.ExplicitComponent):
    """Pack per-component aircraft:* scalars into geometry arrays.

    Component order: 0 = wing, 1 = VTP, 2 = fuselage.

    Wing and VTP characteristic lengths are computed as area/span (average
    chord).  Fuselage fineness ratio is computed as length/max_width.
    These approximations are acceptable for preliminary design.
    """

    def setup(self):
        # ── Wing ──────────────────────────────────────────────────────────────
        self.add_input(Aircraft.Wing.WETTED_AREA, val=0.8, units='m**2')
        self.add_input(Aircraft.Wing.AREA, val=0.5, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.THICKNESS_TO_CHORD, val=0.15, units='unitless')
        self.add_input(Aircraft.Wing.SWEEP, val=0.0, units='deg')

        # ── VTP ───────────────────────────────────────────────────────────────
        # Use custom 'vtp_wetted_area' and 'vtp_area' (from HTailGeometry via
        # trajectory parameter) instead of the FLOPS static IndepVarComp values
        # aircraft:vertical_tail:wetted_area / area, which do not respond to the
        # VTP span design variable.
        self.add_input('vtp_wetted_area', val=0.32, units='m**2')
        self.add_input('vtp_area', val=0.05, units='m**2')
        self.add_input(Aircraft.VerticalTail.SPAN, val=0.31, units='m')
        self.add_input(Aircraft.VerticalTail.THICKNESS_TO_CHORD, val=0.12, units='unitless')
        self.add_input(Aircraft.VerticalTail.SWEEP, val=0.0, units='deg')

        # ── Fuselage ─────────────────────────────────────────────────────────
        # fuselage_exposed_wetted_area replaces the FLOPS/CSV static
        # aircraft:fuselage:wetted_area (which was hardcoded at 2.0 m² in the CSV
        # and never responded to wing DVs).  The exposed area is computed by
        # FuselageExposedWettedAreaComp: superellipse gross Swet minus
        # 2 × wing airfoil cross-section holes.
        self.add_input('fuselage_exposed_wetted_area', val=0.585, units='m**2')
        self.add_input(Aircraft.Fuselage.LENGTH, val=2.0, units='m')
        self.add_input(Aircraft.Fuselage.MAX_WIDTH, val=0.15, units='m')

        # ── Array outputs (nc = 3) ────────────────────────────────────────────
        self.add_output('wetted_area_arr', val=np.ones(3), units='m**2')
        self.add_output('char_length_arr', val=np.ones(3), units='m')
        self.add_output('tc_arr', val=np.ones(3) * 0.12, units='unitless')
        self.add_output('sweep_arr', val=np.zeros(3), units='deg')
        self.add_output('fineness_arr', val=np.ones(3) * 6.0, units='unitless')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        def _f(x):
            return float(x.flat[0])

        wing_chord = _f(inputs[Aircraft.Wing.AREA]) / max(_f(inputs[Aircraft.Wing.SPAN]), 1e-6)
        vtp_chord = _f(inputs['vtp_area']) / max(_f(inputs[Aircraft.VerticalTail.SPAN]), 1e-6)
        fus_f = _f(inputs[Aircraft.Fuselage.LENGTH]) / max(_f(inputs[Aircraft.Fuselage.MAX_WIDTH]), 1e-6)

        outputs['wetted_area_arr'] = np.array([
            _f(inputs[Aircraft.Wing.WETTED_AREA]),
            _f(inputs['vtp_wetted_area']),
            _f(inputs['fuselage_exposed_wetted_area']),
        ])
        outputs['char_length_arr'] = np.array([
            wing_chord,
            vtp_chord,
            _f(inputs[Aircraft.Fuselage.LENGTH]),
        ])
        outputs['tc_arr'] = np.array([
            _f(inputs[Aircraft.Wing.THICKNESS_TO_CHORD]),
            _f(inputs[Aircraft.VerticalTail.THICKNESS_TO_CHORD]),
            0.0,   # fuselage: body form factor uses fineness, not t/c
        ])
        outputs['sweep_arr'] = np.array([
            _f(inputs[Aircraft.Wing.SWEEP]),
            _f(inputs[Aircraft.VerticalTail.SWEEP]),
            0.0,   # fuselage: not used by body form factor
        ])
        outputs['fineness_arr'] = np.array([
            0.0,   # wing: lifting-surface form factor uses t/c, not fineness
            0.0,   # VTP: same
            fus_f,
        ])


class RoskamMissionAeroGroup(om.Group):
    """Roskam/DATCOM parasite drag build-up for Aviary mission phases.

    Replaces FLOPS ``ComputedAeroGroup`` entirely.  Uses:

    - ``_LiftEqualsWeightLocal`` — CL and lift force
    - ``RoskamParasiteDragBuildUp`` — CD0 for wing + VTP + fuselage
    - ``_RoskamMissionInducedDrag`` — wing CDI using the model-scope
                                      ``aircraft:wing:span_efficiency_factor``
                                      set by ``span_eff_correction``
    - ``_FuselageMissionLiftDrag`` — fuselage lift-induced drag
    - ExecComp total drag        — DRAG = (CD0 + CDI + CDI_fus) * q * S_ref

    The geometry arrays required by ``RoskamParasiteDragBuildUp`` are
    assembled from individual ``aircraft:*`` trajectory parameters inside
    ``_GeomArrayAssembler``.
    """

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int)
        self.options.declare(
            'h_wing_interference_factor',
            default=1.0,
            desc='R_h: H-wing junction interference factor applied to all '
                 'lifting_surface and vtp components. Set to 1.04 for the '
                 'SpaJeti H-tail layout.',
        )

    def setup(self):
        nn = self.options['num_nodes']
        r_h = self.options['h_wing_interference_factor']

        # ── 1. Dynamic pressure ───────────────────────────────────────────────
        self.add_subsystem(
            'DynamicPressure',
            DynamicPressure(num_nodes=nn),
            promotes_inputs=[
                Dynamic.Atmosphere.MACH,
                Dynamic.Atmosphere.STATIC_PRESSURE,
            ],
            promotes_outputs=[Dynamic.Atmosphere.DYNAMIC_PRESSURE],
        )

        # ── 2. Lift = Weight  →  cl, Dynamic.Vehicle.LIFT ────────────────────
        self.add_subsystem(
            Dynamic.Vehicle.LIFT,
            _LiftEqualsWeightLocal(num_nodes=nn),
            promotes_inputs=[
                Aircraft.Wing.AREA,
                Dynamic.Vehicle.MASS,
                Dynamic.Atmosphere.DYNAMIC_PRESSURE,
            ],
            promotes_outputs=['cl', Dynamic.Vehicle.LIFT],
        )

        # ── 3. Geometry assembler  →  nc=3 arrays ────────────────────────────
        self.add_subsystem(
            'GeomAssembler',
            _GeomArrayAssembler(),
            promotes_inputs=['aircraft:*', 'vtp_area', 'vtp_wetted_area', 'fuselage_exposed_wetted_area'],
            promotes_outputs=[
                'wetted_area_arr',
                'char_length_arr',
                'tc_arr',
                'sweep_arr',
                'fineness_arr',
            ],
        )

        # ── 4. Roskam/DATCOM parasite drag build-up ───────────────────────────
        pd = RoskamParasiteDragBuildUp(
            num_nodes=nn,
            component_names=_COMPONENT_NAMES,
            component_kinds=_COMPONENT_KINDS,
        )
        self.add_subsystem(
            'RoskamCD0',
            pd,
            promotes_inputs=[
                Dynamic.Atmosphere.MACH,
                Dynamic.Atmosphere.STATIC_PRESSURE,
                Dynamic.Atmosphere.TEMPERATURE,
                ('reference_area', Aircraft.Wing.AREA),
                ('wetted_area', 'wetted_area_arr'),
                ('characteristic_length', 'char_length_arr'),
                ('thickness_to_chord', 'tc_arr'),
                ('sweep_at_max_thickness', 'sweep_arr'),
                ('fineness_ratio', 'fineness_arr'),
                ('fuselage_length', Aircraft.Fuselage.LENGTH),
                # fixed geometry: promote with unique names so group defaults apply
                ('h_wing_interference_factor', 'roskam_h_wing_intf'),
                ('max_thickness_location_over_chord', 'roskam_xc_max_t'),
            ],
            promotes_outputs=['CD0'],
        )
        # Set fixed geometry constants at group scope
        self.set_input_defaults('roskam_h_wing_intf', val=r_h, units='unitless')
        self.set_input_defaults(
            'roskam_xc_max_t',
            val=np.array([0.30, 0.30, 0.30]),
            units='unitless',
        )
        # roughness, laminar_fraction, leakage_protuberance_factor, interference_factor:
        # use the RoskamParasiteDragBuildUp defaults (0.0 / 1.0)

        # ── 5. Induced drag (FLOPS InducedDrag, uses span_efficiency_factor) ──
        self.add_subsystem(
            'WingInducedDrag',
            _RoskamMissionInducedDrag(num_nodes=nn),
            promotes_inputs=[
                'cl',
                Aircraft.Wing.ASPECT_RATIO,
                Aircraft.Wing.SPAN_EFFICIENCY_FACTOR,
            ],
            promotes_outputs=['CDI'],
        )

        # ── 6. CD = CD0 + CDI ─────────────────────────────────────────────────
        self.add_subsystem(
            'FuselageLiftDrag',
            _FuselageMissionLiftDrag(num_nodes=nn),
            promotes_inputs=[
                'cl',
                'wing_CL_alpha',
                Dynamic.Atmosphere.MACH,
                'fuselage_base_area',
                'fuselage_planform_area',
                Aircraft.Wing.AREA,
                'fuselage_fineness_ratio',
            ],
            promotes_outputs=['CDI_fus', 'mission_alpha'],
        )

        self.add_subsystem(
            'CDComp',
            om.ExecComp(
                'CD = CD0 + CDI + CDI_fus',
                CD0={'val': np.ones(nn), 'units': 'unitless'},
                CDI={'val': np.ones(nn), 'units': 'unitless'},
                CDI_fus={'val': np.zeros(nn), 'units': 'unitless'},
                CD={'val': np.ones(nn), 'units': 'unitless'},
            ),
            promotes_inputs=['CD0', 'CDI', 'CDI_fus'],
            promotes_outputs=['CD'],
        )

        # ── 7. DRAG = CD × q × S_ref ──────────────────────────────────────────
        self.add_subsystem(
            'TotalDrag',
            om.ExecComp(
                'drag = CD * q * S',
                CD={'val': np.ones(nn), 'units': 'unitless'},
                q={'val': np.ones(nn), 'units': 'Pa'},
                S={'val': 1.0, 'units': 'm**2'},
                drag={'val': np.ones(nn), 'units': 'N'},
            ),
            promotes_inputs=[
                'CD',
                ('q', Dynamic.Atmosphere.DYNAMIC_PRESSURE),
                ('S', Aircraft.Wing.AREA),
            ],
            promotes_outputs=[('drag', Dynamic.Vehicle.DRAG)],
        )
