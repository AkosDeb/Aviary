"""
Roskam/Raymer style parasite drag build-up.

This module outputs dimensionless drag coefficients only. Convert to drag force
(``D = q*Sref*CD``) in a separate force component when needed.
"""

import warnings

import numpy as np
import openmdao.api as om

from aviary.subsystems.aerodynamics.aero_utils import (
    apply_roughness_cutoff,
    flat_plate_skin_friction_coeff,
    form_factor_datcom_body,
    form_factor_lifting_surface,
    form_factor_raymer_fuselage,
    lifting_surface_correction_factor,
    reynolds_number_from_mach,
    wing_fuselage_interference_factor,
)


SURFACE_KINDS = ('lifting_surface', 'vtp', 'body', 'fuselage', 'raymer_fuselage')


class RoskamParasiteDragBuildUp(om.ExplicitComponent):
    """Compute clean zero-lift parasite drag coefficient from component arrays.

    Roskam Part VI Sections 4.3.1.1-4.3.1.3 build-up:

    Lifting surfaces — fuselage-attached (wing, HTP -- component_kind = 'lifting_surface'):
        CD0 = Cf * FF * R_LS * R_wf * (1 + K_LP) * Swet / Sref

    Tail/VTP panels (component_kind = 'vtp'):
        CD0 = Cf * FF * R_LS * R_tail * (1 + K_LP) * Swet / Sref

    Fuselage (component_kind = 'fuselage' or 'raymer_fuselage'):
        CD0 = Cf_fus * FF_fus * R_wf * (1 + K_LP) * Swet_fus / Sref
        (base-pressure drag CD0_base excluded: engine exhaust fills the base,
         so no separated-wake base drag applies -- Roskam Eq. 4.30)

    External bodies / nacelles (component_kind = 'body'):
        CD0 = Cf * FF * Q * (1 + K_LP) * Swet / Sref

    where:
        Cf    -- flat-plate skin-friction coefficient (mixed lam/turb, Re per component)
        FF    -- DATCOM form factor: 1 + L'*(t/c) + 100*(t/c)^4  (surfaces)
                                  or 1 + 60/f^3 + f/400          (bodies, f = l/d)
        R_LS  -- lifting-surface compressibility correction (DATCOM table, Mach x cos(sweep))
        R_wf  -- wing-fuselage interference factor (DATCOM table, Mach x Re_fus);
                 applied to fuselage-attached lifting surfaces and fuselage
                 (Roskam Eq. 4.30)
        R_tail -- tail junction/layout interference factor; applied to 'vtp'
                  components without fuselage R_wf
        Q     -- user-supplied interference factor; only for external bodies / nacelles
        K_LP  -- leakage and protuberance fraction
        Swet  -- exposed component wetted area
        Sref  -- aircraft reference area
    """

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int)
        self.options.declare(
            'component_names',
            default=('wing', 'horizontal_tail', 'vertical_tail', 'fuselage', 'nacelle'),
            types=tuple,
        )
        self.options.declare(
            'component_kinds',
            default=('lifting_surface', 'lifting_surface', 'lifting_surface', 'fuselage', 'body'),
            types=tuple,
        )

    def setup(self):
        nn = self.options['num_nodes']
        names = self.options['component_names']
        kinds = self.options['component_kinds']
        if len(names) != len(kinds):
            raise ValueError('component_names and component_kinds must have the same length.')
        bad = sorted(set(kinds) - set(SURFACE_KINDS))
        if bad:
            raise ValueError(f'Unsupported component_kinds: {bad}')

        nc = len(names)
        self.nc = nc

        self.add_input('mach', val=np.ones(nn) * 0.3, units='unitless')
        self.add_input('static_pressure', val=np.ones(nn) * 101325.0, units='Pa')
        self.add_input('temperature', val=np.ones(nn) * 288.15, units='K')
        self.add_input('reference_area', val=1.0, units='m**2')

        self.add_input(
            'wetted_area', val=np.ones(nc), units='m**2',
            desc=(
                'Exposed component wetted area [m^2]. For lifting surfaces this is '
                '(planform_area - fuselage_intersection_area) * 2, accounting for '
                'both the upper and lower surfaces of the exposed panel. '
                'Bodies and fuselages use the full outer surface area.'
            ),
        )
        self.add_input('characteristic_length', val=np.ones(nc), units='m')
        self.add_input('roughness', val=np.zeros(nc), units='m')
        self.add_input('laminar_fraction', val=np.zeros(nc), units='unitless')
        self.add_input('thickness_to_chord', val=np.ones(nc) * 0.12, units='unitless')
        self.add_input(
            'max_thickness_location_over_chord',
            val=np.ones(nc) * 0.4,
            units='unitless',
            desc='Airfoil maximum-thickness location `(x/c)_m` for lifting-surface form factor.',
        )
        self.add_input('sweep_at_max_thickness', val=np.zeros(nc), units='deg')
        self.add_input('fineness_ratio', val=np.ones(nc) * 6.0, units='unitless')
        self.add_input(
            'fuselage_length', val=2.0, units='m',
            desc='Fuselage length L_fus [m]. Used to compute the fuselage Reynolds number '
                 'Re_fus = rho*V*L_fus/mu, which drives the R_wf DATCOM table lookup '
                 'applied to all lifting-surface components.',
        )
        self.add_input(
            'interference_factor', val=np.ones(nc), units='unitless',
            desc='Interference factor Q for external body/nacelle components only '
                 '(component_kind="body"). Lifting surfaces and fuselage both use the '
                 'DATCOM R_wf table (Roskam Eq. 4.30); this input is ignored for them.',
        )
        self.add_input(
            'tail_interference_factor', val=1.0, units='unitless',
            desc='Tail junction/interference factor R_tail. Applied to vtp components '
                 'without the fuselage interference factor R_wf.',
        )
        self.add_input('leakage_protuberance_factor', val=np.zeros(nc), units='unitless')

        self.add_output('reynolds_number', val=np.ones((nn, nc)), units='unitless')
        self.add_output('skin_friction_coefficient', val=np.ones((nn, nc)), units='unitless')
        self.add_output('form_factor', val=np.ones((nn, nc)), units='unitless')
        self.add_output(
            'lifting_surface_correction_factor',
            val=np.ones((nn, nc)),
            units='unitless',
            desc='R_LS for lifting surfaces; 1.0 for body/fuselage components.',
        )
        self.add_output(
            'R_wf',
            val=np.ones(nn),
            units='unitless',
            desc='Wing-fuselage interference factor from DATCOM R_wf table '
                 '(function of Mach and Re_fus). Applied to all lifting-surface components.',
        )
        self.add_output('CD0_component', val=np.zeros((nn, nc)), units='unitless')
        self.add_output(
            'CD0',
            val=np.zeros(nn),
            units='unitless',
            desc='Total zero-lift parasite drag coefficient, referenced to reference_area.',
        )

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        mach = inputs['mach']
        static_pressure = inputs['static_pressure']
        temperature = inputs['temperature']
        s_ref = inputs['reference_area']

        if np.any(s_ref <= 0.0):
            raise ValueError(
                f"RoskamParasiteDragBuildUp: reference_area must be > 0; "
                f"got {s_ref.ravel()[0]} m^2."
            )
        if np.any(mach < 0.0):
            raise ValueError(
                f"RoskamParasiteDragBuildUp: Mach number must be >= 0; got {mach}."
            )
        if np.any(mach >= 1.0):
            warnings.warn(
                "RoskamParasiteDragBuildUp: one or more Mach values >= 1.0. "
                "Skin-friction and form-factor correlations are calibrated for "
                "subsonic flight. Use with caution above M = 1.",
                RuntimeWarning,
                stacklevel=2,
            )

        wetted_area = inputs['wetted_area']

        if np.any(wetted_area < 0.0):
            warnings.warn(
                "RoskamParasiteDragBuildUp: one or more wetted_area values < 0. "
                "Wetted area for lifting surfaces must be the EXPOSED area: "
                "(planform - fuselage_intersection) * 2. "
                "Negative values indicate an input error.",
                RuntimeWarning,
                stacklevel=2,
            )
        char_length = inputs['characteristic_length']
        roughness = inputs['roughness']
        laminar_fraction = inputs['laminar_fraction']
        tc = inputs['thickness_to_chord']
        x_c_m = inputs['max_thickness_location_over_chord']
        sweep_rad = np.deg2rad(inputs['sweep_at_max_thickness'])
        fineness = inputs['fineness_ratio']
        fuselage_length = float(inputs['fuselage_length'].ravel()[0])
        q_body = inputs['interference_factor']
        tail_interference = float(inputs['tail_interference_factor'].ravel()[0])
        leakage = inputs['leakage_protuberance_factor']

        # Per-component Reynolds number (characteristic length = MAC or body length)
        re = reynolds_number_from_mach(
            mach[:, np.newaxis],
            static_pressure[:, np.newaxis],
            temperature[:, np.newaxis],
            char_length[np.newaxis, :],
        )
        re = apply_roughness_cutoff(re, char_length[np.newaxis, :], roughness[np.newaxis, :])
        cf = flat_plate_skin_friction_coeff(
            re,
            mach[:, np.newaxis],
            laminar_fraction[np.newaxis, :],
        )

        # Fuselage Reynolds number (based on full fuselage length) → R_wf
        re_fus = reynolds_number_from_mach(mach, static_pressure, temperature, fuselage_length)
        r_wf = wing_fuselage_interference_factor(re_fus, mach)  # shape (nn,)

        ff = np.ones_like(cf)
        r_ls = np.ones_like(cf)
        for idx, kind in enumerate(self.options['component_kinds']):
            if kind in ('lifting_surface', 'vtp'):
                ff[:, idx] = form_factor_lifting_surface(
                    tc[idx],
                    max_thickness_location_over_chord=x_c_m[idx],
                )
                r_ls[:, idx] = lifting_surface_correction_factor(
                    mach,
                    np.cos(sweep_rad[idx]),
                )
            elif kind == 'raymer_fuselage':
                ff[:, idx] = form_factor_raymer_fuselage(fineness[idx])
            else:
                ff[:, idx] = form_factor_datcom_body(fineness[idx])

        # Effective interference factor (Roskam Part VI, Sections 4.3.1.1-4.3.1.3):
        #   lifting_surface  -> R_wf (fuselage)
        #   vtp              -> R_tail (tail junction/layout; no fuselage R_wf)
        #   fuselage         -> R_wf from DATCOM table
        #   body             -> user-supplied Q
        q_eff = np.empty((len(mach), self.nc))
        for idx, kind in enumerate(self.options['component_kinds']):
            if kind == 'lifting_surface':
                q_eff[:, idx] = r_wf
            elif kind == 'vtp':
                q_eff[:, idx] = tail_interference
            elif kind in ('fuselage', 'raymer_fuselage'):
                q_eff[:, idx] = r_wf
            else:
                q_eff[:, idx] = q_body[idx]   # user Q for external bodies / nacelles

        cd0_component = (
            cf
            * ff
            * r_ls
            * q_eff
            * (1.0 + leakage[np.newaxis, :])
            * wetted_area[np.newaxis, :]
            / s_ref
        )

        outputs['reynolds_number'] = re
        outputs['skin_friction_coefficient'] = cf
        outputs['form_factor'] = ff
        outputs['lifting_surface_correction_factor'] = r_ls
        outputs['R_wf'] = r_wf
        outputs['CD0_component'] = cd0_component
        outputs['CD0'] = np.sum(cd0_component, axis=1)
