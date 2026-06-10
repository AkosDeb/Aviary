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
from aviary.subsystems.aerodynamics.flops_based.induced_drag import InducedDrag
from aviary.subsystems.aerodynamics.flops_based.lift import LiftEqualsWeight
from aviary.subsystems.aerodynamics.flops_based.parasite_drag import RoskamParasiteDragBuildUp
from aviary.variable_info.variables import Aircraft, Dynamic


_COMPONENT_NAMES = ('wing', 'vtp', 'fuselage')
_COMPONENT_KINDS = ('lifting_surface', 'vtp', 'fuselage')


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
        self.add_input(Aircraft.VerticalTail.WETTED_AREA, val=0.1, units='m**2')
        self.add_input(Aircraft.VerticalTail.AREA, val=0.05, units='m**2')
        self.add_input(Aircraft.VerticalTail.SPAN, val=0.31, units='m')
        self.add_input(Aircraft.VerticalTail.THICKNESS_TO_CHORD, val=0.12, units='unitless')
        self.add_input(Aircraft.VerticalTail.SWEEP, val=0.0, units='deg')

        # ── Fuselage ─────────────────────────────────────────────────────────
        self.add_input(Aircraft.Fuselage.WETTED_AREA, val=2.0, units='m**2')
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
        wing_chord = float(inputs[Aircraft.Wing.AREA]) / max(float(inputs[Aircraft.Wing.SPAN]), 1e-6)
        vtp_chord = float(inputs[Aircraft.VerticalTail.AREA]) / max(float(inputs[Aircraft.VerticalTail.SPAN]), 1e-6)
        fus_f = float(inputs[Aircraft.Fuselage.LENGTH]) / max(float(inputs[Aircraft.Fuselage.MAX_WIDTH]), 1e-6)

        outputs['wetted_area_arr'] = np.array([
            float(inputs[Aircraft.Wing.WETTED_AREA]),
            float(inputs[Aircraft.VerticalTail.WETTED_AREA]),
            float(inputs[Aircraft.Fuselage.WETTED_AREA]),
        ])
        outputs['char_length_arr'] = np.array([
            wing_chord,
            vtp_chord,
            float(inputs[Aircraft.Fuselage.LENGTH]),
        ])
        outputs['tc_arr'] = np.array([
            float(inputs[Aircraft.Wing.THICKNESS_TO_CHORD]),
            float(inputs[Aircraft.VerticalTail.THICKNESS_TO_CHORD]),
            0.0,   # fuselage: body form factor uses fineness, not t/c
        ])
        outputs['sweep_arr'] = np.array([
            float(inputs[Aircraft.Wing.SWEEP]),
            float(inputs[Aircraft.VerticalTail.SWEEP]),
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

    - ``LiftEqualsWeight``       — CL and lift force (same as FLOPS)
    - ``RoskamParasiteDragBuildUp`` — CD0 for wing + VTP + fuselage
    - Aviary ``InducedDrag``     — CDI using the model-scope
                                   ``aircraft:wing:span_efficiency_factor``
                                   (set by ``span_eff_correction`` ExecComp
                                   in ``add_load_factor_subsystems``)
    - ExecComp total drag        — DRAG = (CD0 + CDI) × q × S_ref

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
            LiftEqualsWeight(num_nodes=nn),
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
            promotes_inputs=['aircraft:*'],
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
            'InducedDrag',
            InducedDrag(num_nodes=nn),
            promotes_inputs=[
                Dynamic.Atmosphere.MACH,
                Dynamic.Vehicle.LIFT,
                Dynamic.Atmosphere.STATIC_PRESSURE,
                Aircraft.Wing.AREA,
                Aircraft.Wing.ASPECT_RATIO,
                Aircraft.Wing.SPAN_EFFICIENCY_FACTOR,
                Aircraft.Wing.SWEEP,
                Aircraft.Wing.TAPER_RATIO,
            ],
            promotes_outputs=[('induced_drag_coeff', 'CDI')],
        )

        # ── 6. CD = CD0 + CDI ─────────────────────────────────────────────────
        self.add_subsystem(
            'CDComp',
            om.ExecComp(
                'CD = CD0 + CDI',
                CD0={'val': np.ones(nn), 'units': 'unitless'},
                CDI={'val': np.ones(nn), 'units': 'unitless'},
                CD={'val': np.ones(nn), 'units': 'unitless'},
            ),
            promotes_inputs=['CD0', 'CDI'],
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
