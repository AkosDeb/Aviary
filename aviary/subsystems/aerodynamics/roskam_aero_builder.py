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
        ]

    def mission_outputs(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        return [
            Dynamic.Vehicle.DRAG,
            Dynamic.Vehicle.LIFT,
        ]

    def get_parameters(self, aviary_inputs=None, user_options=None, subsystem_options=None):
        # Aircraft:* variables are already Aviary trajectory parameters.
        # Only declare Wing.AREA explicitly to follow the SimpleAeroBuilder
        # pattern — Aviary needs the shape / units hint for this one.
        return {
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
            Aircraft.Wing.TAPER_RATIO: {
                'shape': (1,),
                'static_target': True,
                'units': 'unitless',
            },
        }

    def needs_mission_solver(self, aviary_inputs, subsystem_options):
        return False
