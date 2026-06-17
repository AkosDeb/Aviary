import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


class SpanwiseMassDistribution(om.ExplicitComponent):
    """Combine structural and equipment masses into spanwise beam properties.

    This component produces half-wing mass and pitch-inertia arrays for the
    future modal beam model.  It keeps the bookkeeping explicit:

    - structural wingbox mass comes from ``SpanwiseWingboxProperties``
    - VTP/engine/servo masses are represented as station point masses
    - fuel and elevon masses are distributed over configurable span fractions

    The arrays are reporting-only for now; the existing scalar flutter checks
    still consume the older equivalent mass properties.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(AE.SPANWISE_STATIONS, val=np.linspace(0.0, 1.0, n), units='m')
        self.add_input(AE.SPANWISE_CHORD, val=np.ones(n), units='m')
        self.add_input(AE.SPANWISE_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_input(
            AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN,
            val=np.ones(n),
            units='kg*m',
        )
        self.add_input(AE.ELASTIC_AXIS_FRACTION, val=0.375, units='unitless')

        self.add_input(AE.VTP_TIP_MASS, val=0.075, units='kg')
        self.add_input(AE.VTP_TIP_PITCH_INERTIA, val=0.0, units='kg*m**2')
        self.add_input(AE.ENGINE_MASS, val=0.0, units='kg')
        self.add_input(AE.ENGINE_SPAN_FRACTION, val=0.0, units='unitless')
        self.add_input(AE.ENGINE_X_OFFSET_TO_EA, val=0.0, units='m')
        self.add_input(AE.FUEL_MASS, val=0.0, units='kg')
        self.add_input(AE.FUEL_SPAN_FRACTION, val=0.35, units='unitless')
        self.add_input(AE.FUEL_X_OFFSET_TO_EA, val=0.0, units='m')
        self.add_input(AE.SERVO_MASS, val=0.0, units='kg')
        self.add_input(AE.SERVO_SPAN_FRACTION, val=0.85, units='unitless')
        self.add_input(AE.SERVO_X_OFFSET_TO_EA, val=0.0, units='m')
        self.add_input(AE.ELEVON_MASS, val=0.0, units='kg')
        self.add_input(AE.ELEVON_SPAN_START_FRACTION, val=0.45, units='unitless')
        self.add_input(AE.ELEVON_SPAN_END_FRACTION, val=0.95, units='unitless')
        self.add_input(AE.ELEVON_CG_FRACTION, val=0.80, units='unitless')

        self.add_output(AE.SPANWISE_ADDED_MASS_PER_UNIT_SPAN, val=np.zeros(n), units='kg/m')
        self.add_output(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_output(
            AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
            val=np.ones(n),
            units='kg*m',
        )
        self.add_output(AE.SPANWISE_TOTAL_HALF_WING_MASS, val=1.0, units='kg')

        self.declare_partials('*', '*', method='fd')

    @staticmethod
    def _control_widths(y):
        widths = np.zeros_like(y)
        if len(y) == 1:
            widths[0] = 1.0
            return widths

        widths[0] = 0.5 * (y[1] - y[0])
        widths[-1] = 0.5 * (y[-1] - y[-2])
        widths[1:-1] = 0.5 * (y[2:] - y[:-2])
        return np.maximum(widths, 1.0e-12)

    @staticmethod
    def _trapz(y, value):
        return np.sum(0.5 * (value[:-1] + value[1:]) * (y[1:] - y[:-1]))

    @staticmethod
    def _scalar(value):
        return float(np.asarray(value).reshape(-1)[0])

    def _add_point_mass(self, y, widths, eta, mass, dx, added_mass, added_inertia):
        semispan = np.maximum(y[-1], 1.0e-12)
        y_target = np.clip(self._scalar(eta), 0.0, 1.0) * semispan
        idx = int(np.argmin(np.abs(y - y_target)))
        density = self._scalar(mass) / widths[idx]
        added_mass[idx] += density
        added_inertia[idx] += density * self._scalar(dx) ** 2

    def _add_distributed_mass(
        self,
        y,
        eta_start,
        eta_end,
        mass,
        dx_array,
        added_mass,
        added_inertia,
    ):
        semispan = np.maximum(y[-1], 1.0e-12)
        eta_start = self._scalar(eta_start)
        eta_end = self._scalar(eta_end)
        mass = self._scalar(mass)
        start = np.clip(np.minimum(eta_start, eta_end), 0.0, 1.0) * semispan
        end = np.clip(np.maximum(eta_start, eta_end), 0.0, 1.0) * semispan
        length = np.maximum(end - start, 1.0e-12)
        mask = (y >= start) & (y <= end)
        if not np.any(mask):
            mask[int(np.argmin(np.abs(y - 0.5 * (start + end))))] = True
        density = mass / length
        added_mass[mask] += density
        added_inertia[mask] += density * dx_array[mask] ** 2

    def compute(self, inputs, outputs):
        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        chord = np.asarray(inputs[AE.SPANWISE_CHORD])
        structure_mass = np.asarray(inputs[AE.SPANWISE_MASS_PER_UNIT_SPAN])
        structure_inertia = np.asarray(inputs[AE.SPANWISE_PITCH_INERTIA_PER_UNIT_SPAN])
        elastic_axis = self._scalar(inputs[AE.ELASTIC_AXIS_FRACTION])

        added_mass = np.zeros_like(y)
        added_inertia = np.zeros_like(y)
        widths = self._control_widths(y)

        self._add_point_mass(
            y,
            widths,
            1.0,
            inputs[AE.VTP_TIP_MASS],
            0.0,
            added_mass,
            added_inertia,
        )
        added_inertia[-1] += self._scalar(inputs[AE.VTP_TIP_PITCH_INERTIA]) / widths[-1]

        self._add_point_mass(
            y,
            widths,
            inputs[AE.ENGINE_SPAN_FRACTION],
            inputs[AE.ENGINE_MASS],
            inputs[AE.ENGINE_X_OFFSET_TO_EA],
            added_mass,
            added_inertia,
        )
        self._add_point_mass(
            y,
            widths,
            inputs[AE.SERVO_SPAN_FRACTION],
            inputs[AE.SERVO_MASS],
            inputs[AE.SERVO_X_OFFSET_TO_EA],
            added_mass,
            added_inertia,
        )

        self._add_distributed_mass(
            y,
            0.0,
            inputs[AE.FUEL_SPAN_FRACTION],
            inputs[AE.FUEL_MASS],
            inputs[AE.FUEL_X_OFFSET_TO_EA] * np.ones_like(y),
            added_mass,
            added_inertia,
        )
        elevon_dx = (inputs[AE.ELEVON_CG_FRACTION] - elastic_axis) * chord
        self._add_distributed_mass(
            y,
            inputs[AE.ELEVON_SPAN_START_FRACTION],
            inputs[AE.ELEVON_SPAN_END_FRACTION],
            inputs[AE.ELEVON_MASS],
            elevon_dx,
            added_mass,
            added_inertia,
        )

        total_mass = structure_mass + added_mass
        total_inertia = structure_inertia + added_inertia

        outputs[AE.SPANWISE_ADDED_MASS_PER_UNIT_SPAN] = added_mass
        outputs[AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN] = total_mass
        outputs[AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN] = total_inertia
        outputs[AE.SPANWISE_TOTAL_HALF_WING_MASS] = self._trapz(y, total_mass)
