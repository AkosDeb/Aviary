import openmdao.api as om

from aviary.variable_info.variables import Aircraft


class FuselageStructuralMass(om.ExplicitComponent):
    """Fuselage shell mass and CG from canonical fuselage geometry.

    The fuselage shape is defined upstream by ``SuperellipseFuselageGeometry``.
    This component consumes its global outputs instead of rebuilding a separate
    nose/tail model:

        fuselage_structural_mass = fuselage_wetted_area * fuselage_areal_density
        fuselage_x_cg = -fuselage_centroid_x

    ``fuselage_centroid_x`` is measured aft from the nose.  The SpaJeti mass
    frame uses x positive forward with the nose at x=0, hence the sign flip.
    """

    def setup(self):
        # Kept as context inputs for reports/backward-compatible wiring.  The
        # mass and CG are driven by the canonical geometry outputs below.
        self.add_input(Aircraft.Fuselage.LENGTH, val=2.0, units='m')
        self.add_input(Aircraft.Fuselage.MAX_WIDTH, val=0.30, units='m')
        self.add_input(Aircraft.Fuselage.MAX_HEIGHT, val=0.30, units='m')
        self.add_input(
            'fuselage_wetted_area',
            val=1.43,
            units='m**2',
            desc='Gross fuselage wetted area from SuperellipseFuselageGeometry',
        )
        self.add_input(
            'fuselage_centroid_x',
            val=0.90,
            units='m',
            desc='Fuselage volume centroid station aft from nose',
        )
        self.add_input(
            'fuselage_areal_density',
            val=1.95,
            units='kg/m**2',
            desc='Composite skin areal density including stringers and secondary structure',
        )

        self.add_output('fuselage_structural_mass', val=2.0, units='kg')
        self.add_output(
            'fuselage_x_cg',
            val=-0.90,
            units='m',
            desc='Fuselage CG x-position in body frame (negative = aft of nose)',
        )
        self.add_output(
            'fuselage_z_cg',
            val=0.0,
            units='m',
            desc='Fuselage CG z-position (0 = fuselage centreline)',
        )

        self.declare_partials(
            'fuselage_structural_mass',
            ['fuselage_wetted_area', 'fuselage_areal_density'],
            method='cs',
        )
        self.declare_partials('fuselage_x_cg', 'fuselage_centroid_x', val=-1.0)

    def compute(self, inputs, outputs):
        outputs['fuselage_structural_mass'] = (
            inputs['fuselage_wetted_area'] * inputs['fuselage_areal_density']
        )
        outputs['fuselage_x_cg'] = -inputs['fuselage_centroid_x']
        outputs['fuselage_z_cg'] = 0.0
