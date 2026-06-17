import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class SpanwiseEquivalentProperties(om.ExplicitComponent):
    """Reduce spanwise beam arrays to scalar properties for legacy checks.

    The current static divergence and 3-DOF flutter screens consume scalar
    stiffness and mass properties.  This reducer lets those active constraints
    use the Step-3 spanwise beam data before the Step-4 modal beam model exists.

    Equivalent bending stiffness is based on matching tip deflection under a
    unit tip force. Equivalent torsional rigidity is based on matching tip twist
    under unit torque. Mass and pitch inertia are span averages over the half
    wing, including added equipment masses from ``SpanwiseMassDistribution``.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(AE.SPANWISE_STATIONS, val=np.linspace(0.0, 1.0, n), units='m')
        self.add_input(AE.SPANWISE_BENDING_STIFFNESS, val=np.ones(n), units='N*m**2')
        self.add_input(AE.SPANWISE_TORSIONAL_RIGIDITY, val=np.ones(n), units='N*m**2')
        self.add_input(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_input(
            AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
            val=np.ones(n),
            units='kg*m',
        )

        self.add_output(AE.BENDING_STIFFNESS, val=1.0e5, units='N*m**2')
        self.add_output(AE.TORSIONAL_RIGIDITY, val=1.0e5, units='N*m**2')
        self.add_output(AE.TORSIONAL_STIFFNESS, val=1.0e5, units='N*m/rad')
        self.add_output(AE.BENDING_STIFFNESS_PLUNGE, val=1.0e5, units='N/m')
        self.add_output(AE.MASS_PER_UNIT_SPAN, val=1.0, units='kg/m')
        self.add_output(AE.PITCH_INERTIA_PER_UNIT_SPAN, val=0.01, units='kg*m')

        self.declare_partials('*', '*', method='fd')

    @staticmethod
    def _trapz(y, value):
        return np.sum(0.5 * (value[:-1] + value[1:]) * (y[1:] - y[:-1]))

    def compute(self, inputs, outputs):
        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        EI = np.maximum(np.asarray(inputs[AE.SPANWISE_BENDING_STIFFNESS]), 1.0e-30)
        GJ = np.maximum(np.asarray(inputs[AE.SPANWISE_TORSIONAL_RIGIDITY]), 1.0e-30)
        mass = np.asarray(inputs[AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN])
        pitch_inertia = np.asarray(inputs[AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN])

        semispan = np.maximum(y[-1] - y[0], 1.0e-12)
        arm = y[-1] - y

        unit_tip_deflection_flexibility = self._trapz(y, arm**2 / EI)
        unit_twist_flexibility = self._trapz(y, 1.0 / GJ)

        EI_equiv = semispan**3 / (3.0 * np.maximum(unit_tip_deflection_flexibility, 1.0e-30))
        GJ_equiv = semispan / np.maximum(unit_twist_flexibility, 1.0e-30)

        outputs[AE.BENDING_STIFFNESS] = EI_equiv
        outputs[AE.TORSIONAL_RIGIDITY] = GJ_equiv
        outputs[AE.TORSIONAL_STIFFNESS] = GJ_equiv / semispan
        outputs[AE.BENDING_STIFFNESS_PLUNGE] = 3.0 * EI_equiv / semispan**3
        outputs[AE.MASS_PER_UNIT_SPAN] = self._trapz(y, mass) / semispan
        outputs[AE.PITCH_INERTIA_PER_UNIT_SPAN] = self._trapz(y, pitch_inertia) / semispan
