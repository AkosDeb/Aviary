import numpy as np
import openmdao.api as om

_GRAV = 9.80665  # m/s² standard gravity


class LateralLoadFactor(om.ExplicitComponent):
    """Side load factor Ny at a fixed design-point flight condition.

    Passive tail side-force (CY_beta_tail at sideslip beta) converted to Ny.
    Wing and fuselage contributions are excluded pending full integration:

        CY_beta_total = CY_beta_tail                         [per rad]
        CY            = CY_beta_total * beta                 [-]
        side_force    = CY * q * S_ref                       [N]
        Ny            = |side_force| / (mass * g)            [-]  (positive magnitude)

    Ny is positive by convention so the optimisation constraint reads:
        prob.model.add_constraint('Ny', lower=5.0)

    All angle inputs are in degrees and converted to radians internally.

    Future integration into a Dymos flight phase
    --------------------------------------------
    Replace the two plain inputs with Aviary dynamic variables:
        dynamic_pressure  →  Dynamic.Atmosphere.DYNAMIC_PRESSURE  ('dynamic_pressure', Pa)
        aircraft_mass     →  Dynamic.Vehicle.MASS                  ('mass', kg)

    and add num_nodes support.  Ny can then be recorded as a Dymos timeseries
    variable and constrained at the required phase or boundary.
    """

    def setup(self):
        self.add_input(
            'CY_beta_tail',
            val=0.0,
            units='unitless',
            desc='Tail side-force slope from TailCantRotation (per radian sideslip)',
        )

        # Test condition
        self.add_input(
            'beta_deg',
            val=15.0,
            units='deg',
            desc='Maximum sideslip angle at the test condition',
        )
        self.add_input(
            'dynamic_pressure',
            val=1000.0,
            units='Pa',
            desc='Dynamic pressure q = 0.5*rho*V² at the test flight condition. '
                 'Replace with Dynamic.Atmosphere.DYNAMIC_PRESSURE when mission-integrated.',
        )
        self.add_input(
            'wing_ref_area',
            val=0.45,
            units='m**2',
            desc='Wing reference area (from tail geometry)',
        )
        self.add_input(
            'aircraft_mass',
            val=15.0,
            units='kg',
            desc='Aircraft mass at the test flight condition. '
                 'Replace with Dynamic.Vehicle.MASS when mission-integrated.',
        )

        # Intermediate and final outputs
        self.add_output(
            'CY_beta_total',
            val=0.0,
            units='unitless',
            desc='Total passive side-force slope (tail) per radian of sideslip',
        )
        self.add_output(
            'CY',
            val=0.0,
            units='unitless',
            desc='Total side-force coefficient (passive + rudder) at the test condition',
        )
        self.add_output(
            'side_force',
            val=0.0,
            units='N',
            desc='Side force at the test condition (negative = stabilising direction)',
        )
        self.add_output(
            'Ny',
            val=0.0,
            units='unitless',
            desc='Lateral load factor magnitude: |side_force| / (mass * g)',
        )

    def setup_partials(self):
        cy_inputs = ['CY_beta_tail']
        cy_beta_inputs = cy_inputs + ['beta_deg']
        sf_inputs = cy_beta_inputs + ['dynamic_pressure', 'wing_ref_area']
        ny_inputs = sf_inputs + ['aircraft_mass']

        self.declare_partials('CY_beta_total', cy_inputs,      method='cs')
        self.declare_partials('CY',            cy_beta_inputs, method='cs')
        self.declare_partials('side_force',    sf_inputs,      method='cs')
        self.declare_partials('Ny',            ny_inputs,      method='cs')

    def compute(self, inputs, outputs):
        cy_tail  = inputs['CY_beta_tail']
        beta_rad = inputs['beta_deg'] * (np.pi / 180.0)
        q        = inputs['dynamic_pressure']
        s_ref    = inputs['wing_ref_area']
        mass     = inputs['aircraft_mass']

        cy_beta_total = cy_tail
        cy = cy_beta_total * beta_rad
        side_force = cy * q * s_ref
        ny = -side_force / (mass * _GRAV)   # negate → positive Ny magnitude

        outputs['CY_beta_total'] = cy_beta_total
        outputs['CY'] = cy
        outputs['side_force'] = side_force
        outputs['Ny'] = ny


class LongitudinalLoadFactor(om.ExplicitComponent):
    """Vertical load factor Nz at a fixed design-point flight condition.

    Computes the maximum normal force the wing produces at a given angle of attack
    and converts to a load factor:

        CL    = CL_alpha * alpha_max                         [-]
        lift  = CL * q * S_ref                               [N]
        Nz    = lift / (mass * g)                            [-]  (positive upward)

    CL_alpha should come from LiftCurveSlopePolhamus with aspect ratio corrected
    for VTP endplate effect by EndplateARCorrection (AR_eff path).

    Nz is positive by convention so the optimisation constraint reads:
        prob.model.add_constraint('Nz', lower=<required_load_factor>)

    All angle inputs are in degrees and converted to radians internally.

    Future integration into a Dymos flight phase
    --------------------------------------------
    Replace the two plain inputs with Aviary dynamic variables:
        dynamic_pressure  →  Dynamic.Atmosphere.DYNAMIC_PRESSURE  ('dynamic_pressure', Pa)
        aircraft_mass     →  Dynamic.Vehicle.MASS                  ('mass', kg)

    and add num_nodes support.
    """

    def setup(self):
        self.add_input(
            'CL_alpha',
            val=4.0,
            units='unitless',
            desc='Wing 3-D lift curve slope [per rad]. '
                 'Connect from LiftCurveSlopePolhamus with EndplateARCorrection AR_eff.',
        )
        self.add_input(
            'alpha_max_deg',
            val=12.0,
            units='deg',
            desc='Maximum angle of attack for the load-factor check [deg]',
        )
        self.add_input(
            'dynamic_pressure',
            val=1000.0,
            units='Pa',
            desc='Dynamic pressure q = 0.5*rho*V² at the test flight condition. '
                 'Replace with Dynamic.Atmosphere.DYNAMIC_PRESSURE when mission-integrated.',
        )
        self.add_input(
            'wing_ref_area',
            val=0.45,
            units='m**2',
            desc='Wing reference area S_ref',
        )
        self.add_input(
            'aircraft_mass',
            val=15.0,
            units='kg',
            desc='Aircraft mass at the test flight condition. '
                 'Replace with Dynamic.Vehicle.MASS when mission-integrated.',
        )

        self.add_output(
            'CL',
            val=0.0,
            units='unitless',
            desc='Wing lift coefficient at alpha_max: CL = CL_alpha * alpha_max',
        )
        self.add_output(
            'lift',
            val=0.0,
            units='N',
            desc='Wing lift force at alpha_max: lift = CL * q * S_ref',
        )
        self.add_output(
            'Nz',
            val=0.0,
            units='unitless',
            desc='Vertical load factor: lift / (mass * g)',
        )

    def setup_partials(self):
        cl_inputs   = ['CL_alpha', 'alpha_max_deg']
        lift_inputs = cl_inputs + ['dynamic_pressure', 'wing_ref_area']
        nz_inputs   = lift_inputs + ['aircraft_mass']

        self.declare_partials('CL',   cl_inputs,   method='cs')
        self.declare_partials('lift', lift_inputs, method='cs')
        self.declare_partials('Nz',   nz_inputs,   method='cs')

    def compute(self, inputs, outputs):
        cl_alpha  = inputs['CL_alpha']
        alpha_rad = inputs['alpha_max_deg'] * (np.pi / 180.0)
        q         = inputs['dynamic_pressure']
        s_ref     = inputs['wing_ref_area']
        mass      = inputs['aircraft_mass']

        cl   = cl_alpha * alpha_rad
        lift = cl * q * s_ref
        nz   = lift / (mass * _GRAV)

        outputs['CL']   = cl
        outputs['lift'] = lift
        outputs['Nz']   = nz
