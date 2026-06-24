import openmdao.api as om


class VTPStructuralMass(om.ExplicitComponent):
    """Map layout-owned tail physical properties to legacy VTP mass names.

    Tail layout geometry now owns the physical mass and CG.  This component keeps
    the current mass/CG API stable while downstream code is migrated from VTP
    terminology to tail terminology.
    """

    def setup(self):
        self.add_input(
            'tail_physical_structural_mass',
            val=0.25,
            units='kg',
            desc='Physical tail structural mass from tail geometry.',
        )
        self.add_input(
            'tail_physical_x_cg',
            val=-0.90,
            units='m',
            desc='Physical tail CG x in the x-positive-forward mass frame.',
        )
        self.add_input(
            'tail_physical_z_cg',
            val=0.0,
            units='m',
            desc='Physical tail CG z in the body frame, positive down.',
        )

        self.add_output(
            'vtp_structural_mass',
            val=0.25,
            units='kg',
            desc='Compatibility mass output sourced from physical tail mass.',
        )
        self.add_output(
            'vtp_x_cg',
            val=-0.90,
            units='m',
            desc='Compatibility CG x output sourced from physical tail CG.',
        )
        self.add_output(
            'vtp_z_cg',
            val=0.0,
            units='m',
            desc='Compatibility CG z output sourced from physical tail CG.',
        )

        self.declare_partials(
            'vtp_structural_mass',
            'tail_physical_structural_mass',
            val=1.0,
        )
        self.declare_partials('vtp_x_cg', 'tail_physical_x_cg', val=1.0)
        self.declare_partials('vtp_z_cg', 'tail_physical_z_cg', val=1.0)

    def compute(self, inputs, outputs):
        outputs['vtp_structural_mass'] = inputs['tail_physical_structural_mass']
        outputs['vtp_x_cg'] = inputs['tail_physical_x_cg']
        outputs['vtp_z_cg'] = inputs['tail_physical_z_cg']
