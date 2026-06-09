"""
Component-level centre-of-gravity estimator.

CGEstimatorGroup builds a weighted-average CG from a list of mass items.  Each
item has a mass (fixed float or string referencing a model-scope variable) and a
3-D position in the aircraft body frame.

Aircraft reference frame (same as surface_geometry.py)
-------------------------------------------------------
    x  --  positive AFT from nose (fuselage station) [m]
    y  --  positive STARBOARD                        [m]
    z  --  positive DOWN                             [m]

Outputs promoted to model scope
--------------------------------
    x_cg       [m]   CG x-station from nose
    y_cg       [m]   CG lateral offset (0 for a symmetric aircraft)
    z_cg       [m]   CG z-station from nose datum
    total_mass [kg]  sum of all registered component masses

Bypass mode
-----------
Set bypass=True to skip the estimator entirely.  The group then exposes four
manual constants under the same output names so all downstream consumers are
unaffected.  Switching is a one-line Python change at the top of the run file.

Typical usage
-------------
    cg_est = CGEstimatorGroup(
        bypass=CG_BYPASS,
        x_cg_manual=CG_X_MANUAL_M,
        y_cg_manual=CG_Y_MANUAL_M,
        z_cg_manual=CG_Z_MANUAL_M,
        total_mass_manual=CG_MASS_MANUAL_KG,
    )

    # Fixed mass + fixed position (known hardware)
    cg_est.add_component('camera', mass=0.15, x=0.30, y=0.05, z=0.08)

    # Variable mass (OpenMDAO variable name) + fixed position
    cg_est.add_component('engine', mass='cg_engine_mass', x=1.55, y=0.0, z=0.0)

    model.add_subsystem(
        'cg_est', cg_est,
        promotes_outputs=['x_cg', 'y_cg', 'z_cg', 'total_mass'],
    )

    # Wire variable masses that live outside model scope via explicit connects
    prob.model.connect(engine_path, 'cg_est.cg_engine_mass')

Variable-mass wiring note
-------------------------
When mass is a string S, CGEstimatorGroup promotes the internal input
``{name}_mass`` to group scope under the name S.  Two connection patterns:

  1. S is a model-scope Aviary variable (e.g. av.Mission.TOTAL_FUEL):
     add S to the call-site promotes_inputs list so OpenMDAO matches it at
     model scope automatically.

  2. S is a custom local name (e.g. 'cg_engine_mass'):
     use prob.model.connect(source_path, 'cg_est.cg_engine_mass') after
     add_subsystem.
"""

import numpy as np
import openmdao.api as om


class CGComputeComp(om.ExplicitComponent):
    """Weighted-average CG from a static list of mass items.

    Inputs (auto-generated from registered components)
    ---------------------------------------------------
    {name}_mass  [kg]  component mass -- fixed default or connected variable
    {name}_x     [m]   x-station of component CG (positive aft from nose)
    {name}_y     [m]   lateral offset of component CG (positive starboard)
    {name}_z     [m]   z-station of component CG (positive down)

    Outputs
    -------
    x_cg, y_cg, z_cg [m]    aircraft CG position
    total_mass       [kg]   sum of all component masses
    """

    def __init__(self, components, **kwargs):
        """
        Parameters
        ----------
        components : list of (name, mass_spec, x, y, z)
            Populated by CGEstimatorGroup; do not construct directly.
        """
        super().__init__(**kwargs)
        self._components = list(components)

    def setup(self):
        for name, mass_spec, x, y, z in self._components:
            mass_default = float(mass_spec) if isinstance(mass_spec, (int, float)) else 1.0
            self.add_input(f'{name}_mass', val=mass_default, units='kg',
                           desc=f'Mass of "{name}" [kg]')
            self.add_input(f'{name}_x',    val=float(x), units='m',
                           desc=f'x-station of "{name}" CG from nose (positive aft)')
            self.add_input(f'{name}_y',    val=float(y), units='m',
                           desc=f'y-station of "{name}" CG (positive starboard)')
            self.add_input(f'{name}_z',    val=float(z), units='m',
                           desc=f'z-station of "{name}" CG (positive down)')

        self.add_output('x_cg',       val=0.90,  units='m',
                        desc='Aircraft CG x-station from nose (positive aft)')
        self.add_output('y_cg',       val=0.0,   units='m',
                        desc='Aircraft CG lateral offset (positive starboard; 0 for symmetric)')
        self.add_output('z_cg',       val=0.05,  units='m',
                        desc='Aircraft CG z-station from nose datum (positive down)')
        self.add_output('total_mass', val=15.0,  units='kg',
                        desc='Sum of all registered component masses')

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        total = np.zeros(1)
        mx    = np.zeros(1)
        my    = np.zeros(1)
        mz    = np.zeros(1)

        for name, _, _, _, _ in self._components:
            m = inputs[f'{name}_mass']
            total += m
            mx    += m * inputs[f'{name}_x']
            my    += m * inputs[f'{name}_y']
            mz    += m * inputs[f'{name}_z']

        outputs['total_mass'] = total
        outputs['x_cg'] = mx / total
        outputs['y_cg'] = my / total
        outputs['z_cg'] = mz / total


class CGEstimatorGroup(om.Group):
    """Builder-pattern Group for component-level CG estimation.

    Call add_component() once per mass item before add_subsystem(), then add
    the group to the model.  When bypass=True the group contains only an
    IndepVarComp with the four manual constants -- identical output names,
    no estimation logic.

    Attributes (read-only, for use in print / post-processing)
    -----------------------------------------------------------
    components : list of (name, mass_spec, x, y, z)
    bypass     : bool
    """

    def __init__(self, bypass=False, x_cg_manual=0.90, y_cg_manual=0.0,
                 z_cg_manual=0.05, total_mass_manual=15.0, **kwargs):
        super().__init__(**kwargs)
        self._bypass            = bool(bypass)
        self._x_cg_manual       = float(x_cg_manual)
        self._y_cg_manual       = float(y_cg_manual)
        self._z_cg_manual       = float(z_cg_manual)
        self._total_mass_manual = float(total_mass_manual)
        self._components        = []

    # ── public API ────────────────────────────────────────────────────────────

    def add_component(self, name, mass, x, y=0.0, z=0.0):
        """Register a mass item.

        Parameters
        ----------
        name : str
            Unique label (used for N2 input names and print tables).
        mass : float or str
            Component mass [kg].
            float -> fixed constant (set as input default; overridable via
                     prob.set_val('cg_est.cg_compute.{name}_mass', ...)).
            str   -> name of an OpenMDAO variable promoted or connected at
                     model scope.  The input is promoted to group scope under
                     that name; wire it at the call site (see module docstring).
        x : float  CG x-station from nose [m] (positive aft).
        y : float  Lateral CG offset [m] (positive starboard; default 0).
        z : float  CG z-station [m] (positive down; default 0).
        """
        if self._bypass:
            return  # component list is irrelevant in bypass mode
        if any(c[0] == name for c in self._components):
            raise ValueError(f'CGEstimatorGroup: duplicate component name "{name}"')
        self._components.append((str(name), mass, float(x), float(y), float(z)))

    @property
    def components(self):
        return list(self._components)

    @property
    def bypass(self):
        return self._bypass

    # ── OpenMDAO ──────────────────────────────────────────────────────────────

    def setup(self):
        if self._bypass:
            ivc = om.IndepVarComp()
            ivc.add_output('x_cg',       val=self._x_cg_manual,       units='m',
                           desc='Manual CG x-station override (bypass mode)')
            ivc.add_output('y_cg',       val=self._y_cg_manual,       units='m',
                           desc='Manual CG y-station override (bypass mode)')
            ivc.add_output('z_cg',       val=self._z_cg_manual,       units='m',
                           desc='Manual CG z-station override (bypass mode)')
            ivc.add_output('total_mass', val=self._total_mass_manual,  units='kg',
                           desc='Manual total mass override (bypass mode)')
            self.add_subsystem('manual_cg', ivc, promotes_outputs=['*'])
            return

        if not self._components:
            raise ValueError(
                'CGEstimatorGroup: no components registered.  '
                'Call add_component() at least once before setup(), '
                'or construct with bypass=True to use manual values.'
            )

        # Promote variable-mass inputs to group scope under their variable name
        promotes_in = [
            (f'{name}_mass', mass_spec)
            for name, mass_spec, _, _, _ in self._components
            if isinstance(mass_spec, str)
        ]

        self.add_subsystem(
            'cg_compute',
            CGComputeComp(self._components),
            promotes_inputs=promotes_in,
            promotes_outputs=['x_cg', 'y_cg', 'z_cg', 'total_mass'],
        )
