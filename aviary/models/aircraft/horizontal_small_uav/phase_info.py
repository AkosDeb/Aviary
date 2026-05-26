"""Dymos phase setup for the horizontal-tail small UAV example."""

from aviary.variable_info.enums import Transcription


MAX_TAKEOFF_MASS_KG = 15.0


phase_info = {
    'pre_mission': {
        'include_takeoff': False,
        'optimize_mass': False,
    },
    'climb': {
        'subsystem_options': {
            'aerodynamics': {'method': 'computed'},
        },
        'user_options': {
            'num_segments': 4,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.12, 'unitless'),
            'mach_final': (0.25, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (50.0, 'm'),
            'altitude_final': (5000.0, 'm'),
            'mass_ref': (MAX_TAKEOFF_MASS_KG, 'kg'),
            'mass_bounds': ((6.0, MAX_TAKEOFF_MASS_KG), 'kg'),
            'throttle_enforcement': 'path_constraint',
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((1.0, 30.0), 'min'),
            'no_descent': True,
            'constraints': {
                'altitude_rate': {
                    'type': 'path',
                    'lower': 0.0,
                    'upper': 12.0,
                    'units': 'm/s',
                    'ref': 10.0,
                },
            },
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 20.0], 'min'),
            'altitude': ([50.0, 5000.0], 'm'),
            'mach': ([0.12, 0.25], 'unitless'),
            'mass': ([14.5, 13.5], 'kg'),
            'distance': ([0.0, 60.0], 'km'),
        },
    },
    'cruise': {
        'subsystem_options': {
            'aerodynamics': {'method': 'computed'},
        },
        'user_options': {
            'num_segments': 3,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.25, 'unitless'),
            'mach_final': (0.25, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (5000.0, 'm'),
            'altitude_final': (5000.0, 'm'),
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((30.0, 400.0), 'min'),
            'distance_initial': (0.0, 'km'),
            'mass_bounds': ((6.0, MAX_TAKEOFF_MASS_KG), 'kg'),
            'mass_ref': (MAX_TAKEOFF_MASS_KG, 'kg'),
            'distance_bounds': ((0.0, 700.0), 'km'),
            'throttle_enforcement': 'path_constraint',
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 60.0], 'min'),
            'altitude': ([5000.0, 5000.0], 'm'),
            'mach': ([0.25, 0.25], 'unitless'),
            'mass': ([13.5, 9.0], 'kg'),
            'distance': ([0.0, 200.0], 'km'),
        },
    },
    'post_mission': {
        'include_landing': False,
        'constrain_range': False,
    },
}
