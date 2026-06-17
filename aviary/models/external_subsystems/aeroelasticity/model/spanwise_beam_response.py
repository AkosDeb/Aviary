import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class SpanwiseBeamResponse(om.ExplicitComponent):
    """Small-deflection half-wing beam response from distributed loads.

    The component integrates Schrenk bending moment and torque against local
    station stiffnesses:

    ``curvature = M(y) / EI(y)`` and ``twist_rate = T(y) / GJ(y)``.

    Root boundary conditions are clamped: zero slope, deflection, and twist at
    the aircraft centerline.  The outputs are analysis/reporting quantities for
    Step 3 and do not replace the current scalar flutter constraints yet.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(AE.SPANWISE_STATIONS, val=np.linspace(0.0, 1.0, n), units='m')
        self.add_input(AE.SCHRENK_BENDING_MOMENT, val=np.ones(n), units='N*m')
        self.add_input(AE.SCHRENK_TORQUE, val=np.ones(n), units='N*m')
        self.add_input(AE.SPANWISE_BENDING_STIFFNESS, val=np.ones(n), units='N*m**2')
        self.add_input(AE.SPANWISE_TORSIONAL_RIGIDITY, val=np.ones(n), units='N*m**2')

        self.add_output(AE.SPANWISE_SLOPE, val=np.zeros(n), units='rad')
        self.add_output(AE.SPANWISE_DEFLECTION, val=np.zeros(n), units='m')
        self.add_output(AE.SPANWISE_TWIST, val=np.zeros(n), units='rad')
        self.add_output(AE.SPANWISE_TWIST_DEG, val=np.zeros(n), units='deg')
        self.add_output(AE.SCHRENK_TIP_DEFLECTION, val=0.0, units='m')
        self.add_output(AE.SCHRENK_TIP_TWIST, val=0.0, units='rad')
        self.add_output(AE.SCHRENK_TIP_TWIST_DEG, val=0.0, units='deg')

        self.declare_partials('*', '*', method='fd')

    @staticmethod
    def _cumtrapz_from_root(y, value):
        """Return cumulative trapezoidal integral from root to each station."""
        integral = np.zeros_like(value)
        for idx in range(1, len(value)):
            dy = y[idx] - y[idx - 1]
            integral[idx] = integral[idx - 1] + 0.5 * (value[idx - 1] + value[idx]) * dy
        return integral

    def compute(self, inputs, outputs):
        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        bending = np.asarray(inputs[AE.SCHRENK_BENDING_MOMENT])
        torque = np.asarray(inputs[AE.SCHRENK_TORQUE])
        EI = np.maximum(np.asarray(inputs[AE.SPANWISE_BENDING_STIFFNESS]), 1.0e-30)
        GJ = np.maximum(np.asarray(inputs[AE.SPANWISE_TORSIONAL_RIGIDITY]), 1.0e-30)

        curvature = bending / EI
        slope = self._cumtrapz_from_root(y, curvature)
        deflection = self._cumtrapz_from_root(y, slope)

        twist_rate = torque / GJ
        twist = self._cumtrapz_from_root(y, twist_rate)
        twist_deg = twist * (180.0 / np.pi)

        outputs[AE.SPANWISE_SLOPE] = slope
        outputs[AE.SPANWISE_DEFLECTION] = deflection
        outputs[AE.SPANWISE_TWIST] = twist
        outputs[AE.SPANWISE_TWIST_DEG] = twist_deg
        outputs[AE.SCHRENK_TIP_DEFLECTION] = deflection[-1]
        outputs[AE.SCHRENK_TIP_TWIST] = twist[-1]
        outputs[AE.SCHRENK_TIP_TWIST_DEG] = twist_deg[-1]
