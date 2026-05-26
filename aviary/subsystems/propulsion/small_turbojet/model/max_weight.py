import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Model: MultiRidge(alpha=1.0) – thrust_max_N + diameter_mm + rpm_max → weight_kg
# Standardization: z(x) = (x - mu) / sigma
# ---------------------------------------------------------------------------
_MU_T  = 394.046052632;   _SIG_T  = 356.632257316    # thrust_max_N
_MU_D  = 138.640131579;   _SIG_D  = 50.2073044721    # diameter_mm
_MU_R  = 111038.157895;   _SIG_R  = 32630.6159397    # rpm_max

_INTERCEPT = 4.74004
_C_T       = 3.8524     # z(thrust_max_N)
_C_D       = 1.7622     # z(diameter_mm)
_C_R       = 0.914894   # z(rpm_max)

# Categorical indicators are all 0 for the turbojet / centrifugal-compressor /
# generic-manufacturer baseline, so they are omitted.


class MaxWeight(om.ExplicitComponent):
    """
    Predicts engine mass (kg) from max SLS thrust, diameter, and max_rpm using
    a ridge linear regression with standardized inputs (MultiRidge alpha=1.0).

    Inputs are in SI (N for thrust, m for diameter, rpm); diameter is converted
    to mm internally to match training units.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline).
    """

    def setup(self):
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST, val=394.0,    units='N')
        self.add_input(SmallTurbojetVariables.DIAMETER,   val=0.14,     units='m')
        self.add_input(SmallTurbojetVariables.MAX_RPM,    val=112444.0, units='rpm')

        self.add_output(SmallTurbojetVariables.MASS, val=4.74, units='kg')

        self.declare_partials(SmallTurbojetVariables.MASS, Aircraft.Engine.SCALED_SLS_THRUST)
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.MAX_RPM)

    def compute(self, inputs, outputs):
        t    = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        r    = inputs[SmallTurbojetVariables.MAX_RPM]

        z_T = (t    - _MU_T) / _SIG_T
        z_D = (d_mm - _MU_D) / _SIG_D
        z_R = (r    - _MU_R) / _SIG_R

        outputs[SmallTurbojetVariables.MASS] = _INTERCEPT + _C_T * z_T + _C_D * z_D + _C_R * z_R

    def compute_partials(self, inputs, partials):
        partials[SmallTurbojetVariables.MASS, Aircraft.Engine.SCALED_SLS_THRUST] = _C_T / _SIG_T
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER]   = _C_D * 1000.0 / _SIG_D
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.MAX_RPM]    = _C_R / _SIG_R
