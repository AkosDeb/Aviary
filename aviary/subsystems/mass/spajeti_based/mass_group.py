import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.subsystems.mass.spajeti_based.wing_structural_mass import WingStructuralMass
from aviary.subsystems.mass.spajeti_based.fuselage_structural_mass import FuselageStructuralMass
from aviary.subsystems.mass.spajeti_based.vtp_structural_mass import VTPStructuralMass
from aviary.subsystems.mass.spajeti_based.propulsion_location import PropulsionLocationComp
from aviary.subsystems.mass.spajeti_based.fuel_tank import FuelTankComp
from aviary.subsystems.mass.spajeti_based.cg_estimator import SpaJetiCGEstimator
from aviary.subsystems.mass.spajeti_based.fuel_budget import FuelBudgetComp
from aviary.subsystems.subsystem_builder import SubsystemBuilder
from aviary.variable_info.variables import Aircraft


class SpaJetiMassGroup(om.Group):
    """Pre-mission mass, CG, and fuel budget for the SpaJeti H-wing UAV.

    Subsystems (in order):
      wing_mass      — structural mass and CG of the wing box
      fuselage_mass  — structural mass and CG of the fuselage shell
      tail_mass      — HTP and VTP areal-density mass
      vtp_mass       — geometry-based VTP tip-panel mass and CG
      propulsion_loc — engine CG station (aft pusher; engine_x, engine_z)
      fuel_tank      — two-tank fuel CG (front/rear stations + distribution; fuel_x)
      cg             — weighted aircraft CG and total mass (SpaJetiCGEstimator)
      struct_sum     — structural_empty_mass = wing + fus + htp + vtp structural masses
      fuel_budget    — available_fuel and fuel_budget_margin (constraint >= 0)

    Body-fixed frame: x positive forward, nose at origin (x=0), z positive down.

    Group-scope inputs connected from outside:
      engine_mass  ← pre_mission propulsion engine mass (optimizer DV)
      fuel_mass    ← mission:total_fuel (mission phase output)
      aircraft:design:gross_mass, aircraft:crew_payload:total_payload_mass  ← aviary_inputs
    """

    def initialize(self):
        self.options.declare('num_wing_stations', default=21, types=int)
        self.options.declare(
            'fuselage_shape',
            default=None,
            allow_none=True,
            desc='Dict of fuselage shape options: nose_frac, tail_frac, base_w_frac, base_h_frac',
        )

    def setup(self):
        n = self.options['num_wing_stations']
        fus_shape = self.options['fuselage_shape'] or {}

        # Resolve default ambiguities: multiple subsystems promote the same input
        # with different component-level defaults.  The group sets the authoritative
        # default here; the run script overrides via connect() at model scope.
        self.set_input_defaults('wing_x_apex', val=-0.80, units='m')
        # cg (SpaJetiCGEstimator) defaults engine_mass=0.8; fuel_budget defaults 3.0.
        self.set_input_defaults('engine_mass', val=3.0, units='kg')
        self.set_input_defaults('tail_physical_structural_mass', val=0.35, units='kg')
        self.set_input_defaults('tail_physical_x_cg', val=-0.90, units='m')
        self.set_input_defaults('tail_physical_z_cg', val=0.0, units='m')

        self.add_subsystem(
            'wing_mass',
            WingStructuralMass(num_stations=n),
            promotes_inputs=[
                AE.SPANWISE_STATIONS,
                AE.SPANWISE_CHORD,
                AE.SPANWISE_MASS_PER_UNIT_SPAN,
                AE.FRONT_SPAR_FRACTION,
                AE.REAR_SPAR_FRACTION,
                Aircraft.Wing.TAPER_RATIO,
                Aircraft.Wing.SWEEP,
                'wing_x_apex',
                'wing_z_apex',
                'non_structural_mass_fraction',
            ],
            promotes_outputs=[
                'wing_structural_mass',
                'wing_x_cg',
                'wing_z_cg',
            ],
        )

        self.add_subsystem(
            'fuselage_mass',
            FuselageStructuralMass(**fus_shape),
            promotes_inputs=[
                Aircraft.Fuselage.LENGTH,
                Aircraft.Fuselage.MAX_WIDTH,
                Aircraft.Fuselage.MAX_HEIGHT,
                'fuselage_wetted_area',
                'fuselage_centroid_x',
                'fuselage_areal_density',
            ],
            promotes_outputs=[
                'fuselage_structural_mass',
                'fuselage_x_cg',
                'fuselage_z_cg',
            ],
        )

        self.add_subsystem(
            'tail_mass',
            om.ExecComp(
                [
                    'htp_structural_mass = 0.0 * tail_physical_structural_mass',
                    'vtp_area_mass = tail_physical_structural_mass',
                    'tail_structural_mass = tail_physical_structural_mass',
                ],
                tail_physical_structural_mass={'val': 0.35, 'units': 'kg'},
                htp_structural_mass={'val': 0.0, 'units': 'kg'},
                vtp_area_mass={'val': 0.35, 'units': 'kg'},
                tail_structural_mass={'val': 0.35, 'units': 'kg'},
            ),
            promotes_inputs=[
                'tail_physical_structural_mass',
            ],
            promotes_outputs=[
                'htp_structural_mass',
                'vtp_area_mass',
                'tail_structural_mass',
            ],
        )

        self.add_subsystem(
            'vtp_mass',
            VTPStructuralMass(),
            promotes_inputs=[
                'tail_physical_structural_mass',
                'tail_physical_x_cg',
                'tail_physical_z_cg',
            ],
            promotes_outputs=[
                'vtp_structural_mass',
                'vtp_x_cg',
                'vtp_z_cg',
            ],
        )

        self.add_subsystem(
            'propulsion_loc',
            PropulsionLocationComp(),
            promotes_inputs=[
                Aircraft.Fuselage.LENGTH,
                'engine_station_fraction',
            ],
            promotes_outputs=[
                'engine_x',
                'engine_z',
            ],
        )

        self.add_subsystem(
            'fuel_tank',
            FuelTankComp(),
            promotes_inputs=['fuel_distribution', 'front_tank_x', 'rear_tank_x'],
            promotes_outputs=['fuel_x', 'fuel_z'],
        )

        self.add_subsystem(
            'cg',
            SpaJetiCGEstimator(),
            promotes_inputs=[
                'wing_structural_mass', 'wing_x_cg', 'wing_z_cg',
                'fuselage_structural_mass', 'fuselage_x_cg', 'fuselage_z_cg',
                'vtp_structural_mass', 'vtp_x_cg', 'vtp_z_cg',
                'engine_mass', 'engine_x', 'engine_z',
                'fuel_mass', 'fuel_x', 'fuel_z',
                'avionics_mass', 'avionics_x', 'avionics_z',
            ],
            promotes_outputs=[
                'aircraft_x_cg',
                'aircraft_z_cg',
                'aircraft_empty_mass',
                'aircraft_total_mass',
                'aircraft_zfw_x_cg',
            ],
        )

        self.add_subsystem(
            'struct_sum',
            om.ExecComp(
                'structural_empty_mass = (wing_structural_mass + fuselage_structural_mass'
                ' + htp_structural_mass + vtp_structural_mass)',
                wing_structural_mass={'val': 1.8, 'units': 'kg'},
                fuselage_structural_mass={'val': 2.2, 'units': 'kg'},
                htp_structural_mass={'val': 0.36, 'units': 'kg'},
                vtp_structural_mass={'val': 0.25, 'units': 'kg'},
                structural_empty_mass={'val': 4.6, 'units': 'kg'},
            ),
            promotes=['*'],
        )

        self.add_subsystem(
            'fuel_budget',
            FuelBudgetComp(),
            promotes_inputs=[
                Aircraft.Design.GROSS_MASS,
                'structural_empty_mass',
                Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
                'engine_mass',
                'fuel_mass',
            ],
            promotes_outputs=[
                'available_fuel',
                'fuel_budget_margin',
            ],
        )


class SpaJetiMassBuilder(SubsystemBuilder):
    """Builder for the SpaJeti physics-based mass and CG estimation subsystem."""

    _default_name = 'spajeti_mass'

    def __init__(self, name=None, meta_data=None, num_wing_stations=21,
                 fuselage_shape=None):
        self.num_wing_stations = num_wing_stations
        self.fuselage_shape = fuselage_shape
        super().__init__(name, meta_data)

    def build_pre_mission(self, aviary_inputs, subsystem_options=None):
        n = self.num_wing_stations
        fus_shape = self.fuselage_shape
        if subsystem_options:
            n = subsystem_options.get('num_wing_stations', n)
            fus_shape = subsystem_options.get('fuselage_shape', fus_shape)
        return SpaJetiMassGroup(num_wing_stations=n, fuselage_shape=fus_shape)
