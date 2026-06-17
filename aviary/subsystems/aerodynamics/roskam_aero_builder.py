"""
Aviary SubsystemBuilder for Roskam/DATCOM mission aerodynamics.

Usage
-----
In the run script, after ``prob.load_inputs(...)`` and before
``prob.check_and_preprocess_inputs()``:

    from aviary.subsystems.aerodynamics.roskam_aero_builder import RoskamAeroBuilder
    prob.load_external_subsystems([RoskamAeroBuilder(h_wing_interference_factor=1.04)])

In ``phase_info``, disable the built-in FLOPS aero for every mission phase:

    phase_info['climb']['subsystem_options']['aerodynamics'] = {'method': 'external'}
    phase_info['cruise']['subsystem_options']['aerodynamics'] = {'method': 'external'}
    ...
"""

from aviary.subsystems.aerodynamics.SpaJeti_based.roskam_aero_group import RoskamMissionAeroGroup
from aviary.subsystems.subsystem_builder import SubsystemBuilder
from aviary.variable_info.variables import Aircraft, Dynamic


class RoskamAeroBuilder(SubsystemBuilder):
    """Replace FLOPS ComputedAeroGroup with Roskam/DATCOM drag build-up.

    Parameters
    ----------
    h_wing_interference_factor : float
        R_h — H-wing junction interference factor applied to all lifting
        surfaces and VTPs.  Use 1.04 for the SpaJeti H-tail layout
        (Roskam/Raymer clean joint estimate).  1.0 = conventional surfaces.
    name : str, optional
        Override the default subsystem name (default: 'roskam_aero').
    """

    _default_name = 'roskam_aero'

    def __init__(self, h_wing_interference_factor=1.04, name=None):
        super().__init__(name=name or self._default_name)
        self.h_wing_interference_factor = h_wing_interference_factor

    def build_mission(self, num_nodes, aviary_inputs, user_options, subsystem_options):
        return RoskamMissionAeroGroup(
            num_nodes=num_nodes,
            h_wing_interference_factor=self.h_wing_interference_factor,
        )

    def mission_inputs(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        return [
            'aircraft:*',
            Dynamic.Atmosphere.MACH,
            Dynamic.Atmosphere.STATIC_PRESSURE,
            Dynamic.Atmosphere.TEMPERATURE,
            Dynamic.Vehicle.MASS,
            'wing_CL_alpha',
            'fuselage_base_area',
            'fuselage_planform_area',
            'fuselage_fineness_ratio',
            # Custom geometry variables not matched by aircraft:* wildcard:
            # VTP (from HTailGeometry) and fuselage exposed Swet (from
            # FuselageExposedWettedAreaComp -- superellipse gross minus cutouts).
            'vtp_area',
            'vtp_wetted_area',
            'fuselage_exposed_wetted_area',
        ]

    def mission_outputs(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        return [
            Dynamic.Vehicle.DRAG,
            Dynamic.Vehicle.LIFT,
        ]

    def get_parameters(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        # All static (non-time-varying) inputs to RoskamMissionAeroGroup must be
        # declared here so Aviary/Dymos creates trajectory parameters and promotes
        # them to model scope, where they auto-connect to pre_mission geometry
        # outputs and optimizer design variables.
        return {
            # ── Wing scalars ──────────────────────────────────────────────────
            Aircraft.Wing.AREA: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft**2',
            },
            Aircraft.Wing.ASPECT_RATIO: {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
            Aircraft.Wing.SPAN: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft',
            },
            Aircraft.Wing.SPAN_EFFICIENCY_FACTOR: {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
            Aircraft.Wing.SWEEP: {
                'shape': (1,),
                'static_target': True,
                'units': 'deg',
            },
            Aircraft.Wing.THICKNESS_TO_CHORD: {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
            Aircraft.Wing.WETTED_AREA: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft**2',
            },
            # ── Vertical tail scalars ─────────────────────────────────────────
            # vtp_area and vtp_wetted_area use custom names (not aircraft:* Aviary
            # standard) so they connect to HTailGeometry's live outputs at model
            # scope rather than the FLOPS pre-mission static IndepVarComp values.
            # This ensures VTP span DV changes propagate to mission parasite drag.
            'vtp_area': {
                'shape': (1,),
                'static_target': True,
                'units': 'm**2',
            },
            'vtp_wetted_area': {
                'shape': (1,),
                'static_target': True,
                'units': 'm**2',
            },
            Aircraft.VerticalTail.SPAN: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft',
            },
            Aircraft.VerticalTail.SWEEP: {
                'shape': (1,),
                'static_target': True,
                'units': 'deg',
            },
            Aircraft.VerticalTail.THICKNESS_TO_CHORD: {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
            # ── Fuselage scalars ──────────────────────────────────────────────
            Aircraft.Fuselage.LENGTH: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft',
            },
            Aircraft.Fuselage.MAX_WIDTH: {
                'shape': (1,),
                'static_target': True,
                'units': 'ft',
            },
            # fuselage_exposed_wetted_area replaces the FLOPS/CSV static
            # aircraft:fuselage:wetted_area.  It is computed by
            # FuselageExposedWettedAreaComp (superellipse gross minus wing
            # airfoil cross-section cutouts) and is live w.r.t. wing DVs.
            'fuselage_exposed_wetted_area': {
                'shape': (1,),
                'static_target': True,
                'units': 'm**2',
            },
            # ── Custom (non-aircraft:*) parameters ────────────────────────────
            # Computed by add_load_factor_subsystems at model scope and promoted
            # by name, so they auto-connect via the trajectory parameter promote.
            'wing_CL_alpha': {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
            'fuselage_base_area': {
                'shape': (1,),
                'static_target': True,
                'units': 'm**2',
            },
            'fuselage_planform_area': {
                'shape': (1,),
                'static_target': True,
                'units': 'm**2',
            },
            'fuselage_fineness_ratio': {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
        }

    def needs_mission_solver(self, aviary_inputs, subsystem_options):
        return False
