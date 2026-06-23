import openmdao.api as om

from aviary.variable_info.variables import Aircraft, Mission


class FuelBudgetComp(om.ExplicitComponent):
    """Mass-balance fuel budget for the SpaJeti H-wing UAV.

    available_fuel    = gross_mass - structural_empty_mass - payload_mass - engine_mass
    fuel_budget_margin = available_fuel - fuel_mass   (mission total fuel burned)

    Constraint target: fuel_budget_margin >= 0.

    All partials are analytic (the equations are linear in every input).
    """

    def setup(self):
        self.add_input(Aircraft.Design.GROSS_MASS, val=15.0, units='kg')
        self.add_input('structural_empty_mass', val=5.3, units='kg')
        self.add_input(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, val=2.5, units='kg')
        self.add_input('engine_mass', val=3.0, units='kg')
        self.add_input('fuel_mass', val=1.0, units='kg')
        self.add_output('available_fuel', val=2.5, units='kg')
        self.add_output('fuel_budget_margin', val=1.5, units='kg')

    def setup_partials(self):
        avail_inputs = [
            Aircraft.Design.GROSS_MASS,
            'structural_empty_mass',
            Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS,
            'engine_mass',
        ]
        self.declare_partials('available_fuel', avail_inputs)
        self.declare_partials('fuel_budget_margin', avail_inputs + ['fuel_mass'])

    def compute(self, inputs, outputs):
        available_fuel = (
            inputs[Aircraft.Design.GROSS_MASS]
            - inputs['structural_empty_mass']
            - inputs[Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS]
            - inputs['engine_mass']
        )
        outputs['available_fuel'] = available_fuel
        outputs['fuel_budget_margin'] = available_fuel - inputs['fuel_mass']

    def compute_partials(self, inputs, partials):
        partials['available_fuel', Aircraft.Design.GROSS_MASS] = 1.0
        partials['available_fuel', 'structural_empty_mass'] = -1.0
        partials['available_fuel', Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS] = -1.0
        partials['available_fuel', 'engine_mass'] = -1.0

        partials['fuel_budget_margin', Aircraft.Design.GROSS_MASS] = 1.0
        partials['fuel_budget_margin', 'structural_empty_mass'] = -1.0
        partials['fuel_budget_margin', Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS] = -1.0
        partials['fuel_budget_margin', 'engine_mass'] = -1.0
        partials['fuel_budget_margin', 'fuel_mass'] = -1.0
