import numpy as np  # noqa: F401 — needed for np.asarray in compute()
import openmdao.api as om

from aviary.variable_info.variables import Aircraft


class TailStructuralMass(om.ExplicitComponent):
    """Tail (HTP + VTP) mass from planform area × composite areal density.

    For the H-wing UAV the VTP panels are the H-tail endplates; their planform
    area is Aircraft.VerticalTail.AREA (total for all panels). Both surfaces
    use the same areal density by default but accept separate inputs for tuning.

        HTP_mass = Aircraft.HorizontalTail.AREA * htp_areal_density
        VTP_mass = Aircraft.VerticalTail.AREA  * vtp_areal_density
        TAIL_STRUCTURAL_MASS = HTP_mass + VTP_mass

    areal_density defaults to 1.2 kg/m² (typical composite sandwich panel).
    """

    def setup(self):
        self.add_input(Aircraft.HorizontalTail.AREA, val=0.3, units='m**2')
        self.add_input(Aircraft.VerticalTail.AREA, val=0.15, units='m**2')
        self.add_input('htp_areal_density', val=1.2, units='kg/m**2')
        self.add_input('vtp_areal_density', val=1.2, units='kg/m**2')

        self.add_output('htp_structural_mass', val=0.5, units='kg')
        self.add_output('vtp_structural_mass', val=0.2, units='kg')
        self.add_output('tail_structural_mass', val=0.7, units='kg')

        self.declare_partials(
            'htp_structural_mass',
            [Aircraft.HorizontalTail.AREA, 'htp_areal_density'],
            method='cs',
        )
        self.declare_partials(
            'vtp_structural_mass',
            [Aircraft.VerticalTail.AREA, 'vtp_areal_density'],
            method='cs',
        )
        self.declare_partials(
            'tail_structural_mass',
            [
                Aircraft.HorizontalTail.AREA,
                Aircraft.VerticalTail.AREA,
                'htp_areal_density',
                'vtp_areal_density',
            ],
            method='cs',
        )

    def compute(self, inputs, outputs):
        htp_area = np.asarray(inputs[Aircraft.HorizontalTail.AREA]).item()
        vtp_area = np.asarray(inputs[Aircraft.VerticalTail.AREA]).item()
        htp_rho = np.asarray(inputs['htp_areal_density']).item()
        vtp_rho = np.asarray(inputs['vtp_areal_density']).item()

        htp_mass = htp_area * htp_rho
        vtp_mass = vtp_area * vtp_rho
        outputs['htp_structural_mass'] = htp_mass
        outputs['vtp_structural_mass'] = vtp_mass
        outputs['tail_structural_mass'] = htp_mass + vtp_mass
