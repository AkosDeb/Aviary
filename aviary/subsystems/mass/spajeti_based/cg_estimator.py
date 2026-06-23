import numpy as np
import openmdao.api as om


class SpaJetiCGEstimator(om.ExplicitComponent):
    """Aircraft-level CG and total mass from structural components + point masses.

    Body-fixed frame: x positive forward, nose at origin (x=0), z positive down.
    All structural component positions are negative (aft of nose).

    Point masses (engine, fuel) are provided as scalar inputs.  The CG is the
    mass-weighted average of all contributions:

        x_cg = (Σ mᵢ · xᵢ) / Σmᵢ
        z_cg = (Σ mᵢ · zᵢ) / Σmᵢ

    Outputs:
        aircraft_x_cg          longitudinal CG at MTOW (with fuel) [m]
        aircraft_z_cg          vertical CG at MTOW [m]
        aircraft_empty_mass     structural mass only (no fuel) [kg]
        aircraft_total_mass     structural + fuel mass [kg]
        aircraft_zfw_x_cg      longitudinal CG at zero fuel weight [m]
    """

    def setup(self):
        # Wing
        self.add_input('wing_structural_mass', val=3.0, units='kg')
        self.add_input('wing_x_cg', val=-0.75, units='m')
        self.add_input('wing_z_cg', val=0.0, units='m')

        # Fuselage
        self.add_input('fuselage_structural_mass', val=1.5, units='kg')
        self.add_input('fuselage_x_cg', val=-1.0, units='m')
        self.add_input('fuselage_z_cg', val=0.0, units='m')

        # VTP (geometry-based; from VTPStructuralMass)
        self.add_input('vtp_structural_mass', val=0.25, units='kg',
                       desc='Total VTP mass for all panels on the aircraft')
        self.add_input('vtp_x_cg', val=-0.90, units='m',
                       desc='VTP CG x-position in body frame')
        self.add_input('vtp_z_cg', val=0.0, units='m',
                       desc='VTP CG z-position in body frame (0: up+down panels cancel)')

        # Engine (point mass; from PropulsionLocationComp)
        self.add_input('engine_mass', val=0.8, units='kg',
                       desc='Total propulsion system mass (engine + mount)')
        self.add_input('engine_x', val=-1.70, units='m',
                       desc='Engine CG x-position in body frame')
        self.add_input('engine_z', val=0.0, units='m',
                       desc='Engine CG z-position in body frame')

        # Fuel (point mass; from PropulsionLocationComp)
        self.add_input('fuel_mass', val=1.0, units='kg',
                       desc='Total fuel mass at design point')
        self.add_input('fuel_x', val=-1.0, units='m',
                       desc='Fuel CG x-position in body frame (fuselage centre tank)')
        self.add_input('fuel_z', val=0.0, units='m',
                       desc='Fuel CG z-position in body frame')

        # Avionics / payload (fixed point mass)
        self.add_input('avionics_mass', val=0.5, units='kg')
        self.add_input('avionics_x', val=-0.3, units='m')
        self.add_input('avionics_z', val=0.0, units='m')

        self.add_output('aircraft_x_cg', val=-0.8, units='m')
        self.add_output('aircraft_z_cg', val=0.0, units='m')
        self.add_output('aircraft_empty_mass', val=6.0, units='kg')
        self.add_output('aircraft_total_mass', val=7.0, units='kg')
        self.add_output('aircraft_zfw_x_cg', val=-0.8, units='m',
                        desc='CG at zero fuel weight (fuel_mass = 0)')

        mass_inputs = [
            'wing_structural_mass',
            'fuselage_structural_mass',
            'vtp_structural_mass',
            'engine_mass',
            'fuel_mass',
            'avionics_mass',
        ]
        x_inputs = ['wing_x_cg', 'fuselage_x_cg', 'vtp_x_cg', 'engine_x', 'fuel_x', 'avionics_x']
        z_inputs = ['wing_z_cg', 'fuselage_z_cg', 'vtp_z_cg', 'engine_z', 'fuel_z', 'avionics_z']

        non_fuel_mass_inputs = [
            'wing_structural_mass', 'fuselage_structural_mass', 'vtp_structural_mass',
            'engine_mass', 'avionics_mass',
        ]
        non_fuel_x_inputs = ['wing_x_cg', 'fuselage_x_cg', 'vtp_x_cg', 'engine_x', 'avionics_x']

        self.declare_partials('aircraft_x_cg', mass_inputs + x_inputs, method='cs')
        self.declare_partials('aircraft_z_cg', mass_inputs + z_inputs, method='cs')
        self.declare_partials('aircraft_empty_mass', non_fuel_mass_inputs, method='cs')
        self.declare_partials('aircraft_total_mass', mass_inputs, method='cs')
        self.declare_partials(
            'aircraft_zfw_x_cg', non_fuel_mass_inputs + non_fuel_x_inputs, method='cs',
        )

    def compute(self, inputs, outputs):
        def _f(key):
            return np.asarray(inputs[key]).item()

        masses = np.array([
            _f('wing_structural_mass'),
            _f('fuselage_structural_mass'),
            _f('vtp_structural_mass'),
            _f('engine_mass'),
            _f('fuel_mass'),
            _f('avionics_mass'),
        ])
        x_positions = np.array([
            _f('wing_x_cg'),
            _f('fuselage_x_cg'),
            _f('vtp_x_cg'),
            _f('engine_x'),
            _f('fuel_x'),
            _f('avionics_x'),
        ])
        z_positions = np.array([
            _f('wing_z_cg'),
            _f('fuselage_z_cg'),
            _f('vtp_z_cg'),
            _f('engine_z'),
            _f('fuel_z'),
            _f('avionics_z'),
        ])

        empty_masses = masses.copy()
        empty_masses[4] = 0.0  # exclude fuel from empty mass (index 4 after adding VTP)

        mass_sum = np.sum(masses)
        empty_sum = np.sum(empty_masses)
        m_total = mass_sum if abs(np.real(mass_sum)) > 1.0e-12 else 1.0e-12
        m_empty = empty_sum if abs(np.real(empty_sum)) > 1.0e-12 else 1.0e-12

        outputs['aircraft_x_cg'] = np.dot(masses, x_positions) / m_total
        outputs['aircraft_z_cg'] = np.dot(masses, z_positions) / m_total
        outputs['aircraft_empty_mass'] = m_empty
        outputs['aircraft_total_mass'] = m_total
        # ZFW CG: empty_masses[4]=0 already zeros out fuel, so the dot product
        # naturally excludes the fuel*fuel_x term without any extra zeroing.
        outputs['aircraft_zfw_x_cg'] = np.dot(empty_masses, x_positions) / m_empty
