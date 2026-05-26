import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Model: MultiPoly(deg=2) – thrust_max_N → diameter_mm
# Standardization: z(x) = (x - mu) / sigma
# ---------------------------------------------------------------------------
_MU_T  = 394.046052632;   _SIG_T  = 356.632257316   # thrust_max_N
_MU_T2 = 282458.858553;   _SIG_T2 = 537407.575006   # thrust_max_N^2

_INTERCEPT =  140.096
_C_T       =   76.6005   # z(thrust_max_N)
_C_T2      =  -31.872    # z(thrust_max_N^2)

# Categorical indicators are all ~0 (machine-epsilon) for the turbojet /
# centrifugal-compressor / generic-manufacturer baseline, so they are omitted.


class MaxDiameter(om.ExplicitComponent):
    """
    Predicts engine outer diameter from max SLS thrust using a degree-2
    polynomial regression with standardized features (MultiPoly deg=2).

    Input is in SI (N); output is in SI (m). The model was trained in mm so
    the conversion is applied internally.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline / centrifugal turbojet).
    """

    def setup(self):
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST, val=394.0, units='N')
        self.add_output(SmallTurbojetVariables.DIAMETER, val=0.14, units='m')
        self.declare_partials(
            SmallTurbojetVariables.DIAMETER, Aircraft.Engine.SCALED_SLS_THRUST
        )

    def compute(self, inputs, outputs):
        t = inputs[Aircraft.Engine.SCALED_SLS_THRUST]

        z_t  = (t       - _MU_T)  / _SIG_T
        z_t2 = (t ** 2  - _MU_T2) / _SIG_T2

        diameter_mm = _INTERCEPT + _C_T * z_t + _C_T2 * z_t2
        outputs[SmallTurbojetVariables.DIAMETER] = diameter_mm / 1000.0

    def compute_partials(self, inputs, partials):
        t = inputs[Aircraft.Engine.SCALED_SLS_THRUST]

        # d(diameter_mm)/d(thrust) / 1000 for the mm→m conversion
        dd_dt = (_C_T / _SIG_T + _C_T2 * 2.0 * t / _SIG_T2) / 1000.0
        partials[SmallTurbojetVariables.DIAMETER, Aircraft.Engine.SCALED_SLS_THRUST] = dd_dt
