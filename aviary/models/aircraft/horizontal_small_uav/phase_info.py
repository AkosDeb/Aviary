"""Dymos phase setup for the horizontal-tail small UAV example."""

from aviary.variable_info.enums import Transcription


MAX_TAKEOFF_MASS_KG = 15.0
DASH_ALTITUDE_M = 5000.0
DASH_MACH = 0.52


phase_info = {
    'pre_mission': {
        'include_takeoff': False,
        'optimize_mass': False,
    },
    'climb': {
        'subsystem_options': {
            'aerodynamics': {'method': 'external'},
        },
        'user_options': {
            'num_segments': 4,
            'order': 3,
            'mach_optimize': True,
            'mach_bounds': ((0.12, 0.40), 'unitless'),
            'mach_ref': (1.0, 'unitless'),
            'mach_initial': (0.12, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (50.0, 'm'),
            'altitude_final': (DASH_ALTITUDE_M, 'm'),
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
                    'upper': 40,
                    'units': 'm/s',
                    'ref': 10.0,
                },
            },
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 20.0], 'min'),
            'altitude': ([50.0, DASH_ALTITUDE_M], 'm'),
            'mach': ([0.12, 0.25], 'unitless'),
            'mass': ([14.5, 13.5], 'kg'),
            'distance': ([0.0, 60.0], 'km'),
        },
    },
    'cruise': {
        'subsystem_options': {
            'aerodynamics': {'method': 'external'},
        },
        'user_options': {
            'num_segments': 3,
            'order': 3,
            'mach_optimize': True,
            'mach_bounds': ((0.18, DASH_MACH), 'unitless'),
            'mach_ref': (1.0, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (DASH_ALTITUDE_M, 'm'),
            'altitude_final': (DASH_ALTITUDE_M, 'm'),
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((5.0, 400.0), 'min'),
            'distance_initial': (0.0, 'km'),
            'mass_bounds': ((6.0, MAX_TAKEOFF_MASS_KG), 'kg'),
            'mass_ref': (MAX_TAKEOFF_MASS_KG, 'kg'),
            'distance_bounds': ((0.0, 700.0), 'km'),
            'throttle_enforcement': 'path_constraint',
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 60.0], 'min'),
            'altitude': ([DASH_ALTITUDE_M, DASH_ALTITUDE_M], 'm'),
            'mach': ([0.25, 0.35], 'unitless'),
            'mass': ([13.5, 9.0], 'kg'),
            'distance': ([0.0, 200.0], 'km'),
        },
    },
    'accel_to_dash': {
        'subsystem_options': {
            'aerodynamics': {'method': 'external'},
        },
        'user_options': {
            'num_segments': 2,
            'order': 3,
            'mach_optimize': True,
            'mach_bounds': ((0.18, DASH_MACH), 'unitless'),
            'mach_ref': (1.0, 'unitless'),
            'mach_final': (DASH_MACH, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (DASH_ALTITUDE_M, 'm'),
            'altitude_final': (DASH_ALTITUDE_M, 'm'),
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((0.1, 3.0), 'min'),
            'distance_initial': (0.0, 'km'),
            'distance_bounds': ((0.0, 700.0), 'km'),
            'mass_bounds': ((6.0, MAX_TAKEOFF_MASS_KG), 'kg'),
            'mass_ref': (MAX_TAKEOFF_MASS_KG, 'kg'),
            'throttle_enforcement': 'path_constraint',
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 2.0], 'min'),
            'altitude': ([DASH_ALTITUDE_M, DASH_ALTITUDE_M], 'm'),
            'mach': ([0.35, DASH_MACH], 'unitless'),
            'mass': ([9.0, 8.8], 'kg'),
            'distance': ([0.0, 20.0], 'km'),
        },
    },
    'dash': {
        'subsystem_options': {
            'aerodynamics': {'method': 'external'},
        },
        'user_options': {
            'num_segments': 2,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (DASH_MACH, 'unitless'),
            'mach_final': (DASH_MACH, 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (DASH_ALTITUDE_M, 'm'),
            'altitude_final': (DASH_ALTITUDE_M, 'm'),
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((3.0, 4.0), 'min'),
            'distance_initial': (0.0, 'km'),
            'distance_bounds': ((0.0, 700.0), 'km'),
            'mass_bounds': ((6.0, MAX_TAKEOFF_MASS_KG), 'kg'),
            'mass_ref': (MAX_TAKEOFF_MASS_KG, 'kg'),
            'throttle_enforcement': 'path_constraint',
            'transcription': Transcription.COLLOCATION,
        },
        'initial_guesses': {
            'time': ([0.0, 3.5], 'min'),
            'altitude': ([DASH_ALTITUDE_M, DASH_ALTITUDE_M], 'm'),
            'mach': ([DASH_MACH, DASH_MACH], 'unitless'),
            'mass': ([8.8, 8.6], 'kg'),
            'distance': ([0.0, 35.0], 'km'),
        },
    },
    'post_mission': {
        'include_landing': False,
        'constrain_range': False,
    },
}
