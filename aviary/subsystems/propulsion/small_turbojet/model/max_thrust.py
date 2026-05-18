import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Scaler statistics from model training (MultiLinear, then StandardScaler).
# Replace placeholder values with actual mu/sigma from your scaler.
# ---------------------------------------------------------------------------
_MU_W  = 4.52688157895;    _SIG_W  = 5.11277056154    # weight_kg
_MU_D  = 138.640131579;    _SIG_D  = 50.2073044721    # diameter_mm
_MU_L  = 340.222368421;    _SIG_L  = 123.516115078    # length_mm
_MU_R  = 111038.157895;    _SIG_R  = 32630.6159397    # rpm_max


# Regression coefficients. Categorical terms default to 0 (unknown/other baseline).
_INTERCEPT = 376.727
_C_W  =  282.125   # z(weight_kg)
_C_D  =  139.501   # z(diameter_mm)
_C_L  =  -70.2145  # z(length_mm)
_C_R  =  -26.3036  # z(rpm_max)


class MaxThrust(om.ExplicitComponent):
    """
    Predicts max SLS thrust (N) from engine mass, diameter, length, and max_rpm
    using a linear regression with standardized inputs (MultiLinear).

    Inputs are in SI (m for geometry, rpm, kg for mass); mm conversion applied
    internally for diameter and length to match training units.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline). Fill in _MU_* / _SIG_* constants
    before first use.
    """

    def setup(self):
        self.add_input(SmallTurbojetVariables.MASS,    val=4.68,     units='kg')
        self.add_input(SmallTurbojetVariables.DIAMETER, val=0.15,    units='m')
        self.add_input(SmallTurbojetVariables.LENGTH,   val=0.45,    units='m')
        self.add_input(SmallTurbojetVariables.MAX_RPM,  val=112831.0, units='rpm')

        self.add_output(Aircraft.Engine.SCALED_SLS_THRUST, val=376.727, units='N')

        self.declare_partials(Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.MASS)
        self.declare_partials(Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.LENGTH)
        self.declare_partials(Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.MAX_RPM)

    def compute(self, inputs, outputs):
        w    = inputs[SmallTurbojetVariables.MASS]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        l_mm = inputs[SmallTurbojetVariables.LENGTH]   * 1000.0
        r    = inputs[SmallTurbojetVariables.MAX_RPM]

        z_w = (w    - _MU_W) / _SIG_W
        z_d = (d_mm - _MU_D) / _SIG_D
        z_l = (l_mm - _MU_L) / _SIG_L
        z_r = (r    - _MU_R) / _SIG_R

        outputs[Aircraft.Engine.SCALED_SLS_THRUST] = (
            _INTERCEPT
            + _C_W * z_w
            + _C_D * z_d
            + _C_L * z_l
            + _C_R * z_r
        )

    def compute_partials(self, inputs, partials):
        partials[Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.MASS]     = _C_W / _SIG_W
        partials[Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.DIAMETER] = _C_D * 1000.0 / _SIG_D
        partials[Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.LENGTH]   = _C_L * 1000.0 / _SIG_L
        partials[Aircraft.Engine.SCALED_SLS_THRUST, SmallTurbojetVariables.MAX_RPM]  = _C_R / _SIG_R
