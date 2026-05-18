import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Scaler statistics from model training (MultiRidge alpha=1.0, StandardScaler).
# Replace placeholder values with actual mu/sigma from your scaler.
# ---------------------------------------------------------------------------
_MU_T  = 373.079710145;   _SIG_T  = 351.579990931   # thrust_max_N
_MU_R  = 112534.782609;   _SIG_R  = 33259.2182976   # rpm_max
_MU_D  = 134.483333333;   _SIG_D  = 48.8054275512   # diameter_mm
_MU_L  = 335.917391304;   _SIG_L  = 122.605367824   # length_mm
_MU_W  = 4.00062318841;   _SIG_W  = 4.8043609812    # weight_kg

# Regression coefficients. Categorical terms default to 0 (unknown/other baseline).
_INTERCEPT =  0.157449
_C_R       =  0.0111733   # z(rpm_max)
_C_D       = -0.00527282  # z(diameter_mm)
_C_W       =  0.00213555  # z(weight_kg)
_C_T       = -0.00110495  # z(thrust_max_N)
_C_L       = -0.0006475   # z(length_mm)

# Training target unit is kg/(N*h); convert to kg/(N*s) for output.
_H_TO_S = 1.0 / 3600.0


class SFC(om.ExplicitComponent):
    """
    Predicts specific fuel consumption from rpm, diameter, mass, thrust, and length
    using a ridge linear regression with standardized inputs (MultiRidge alpha=1.0).

    The model was trained with sfc in kg/(N*h); output is converted to kg/(N*s)
    to match the rest of the propulsion model.

    Inputs are in SI (m for geometry, N for thrust, kg for mass, rpm);
    mm conversion applied internally for diameter and length.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline).
    """

    def setup(self):
        self.add_input(SmallTurbojetVariables.MAX_RPM,           val=112831.0, units='rpm')
        self.add_input(SmallTurbojetVariables.DIAMETER,          val=0.15,     units='m')
        self.add_input(SmallTurbojetVariables.MASS,              val=4.68,     units='kg')
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST,        val=376.727,  units='N')
        self.add_input(SmallTurbojetVariables.LENGTH,            val=0.45,     units='m')

        self.add_output(SmallTurbojetVariables.SFC, val=3.0e-5, units='kg/(N*s)')

        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.MAX_RPM)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.MASS)
        self.declare_partials(SmallTurbojetVariables.SFC, Aircraft.Engine.SCALED_SLS_THRUST)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.LENGTH)

    def compute(self, inputs, outputs):
        r    = inputs[SmallTurbojetVariables.MAX_RPM]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        w    = inputs[SmallTurbojetVariables.MASS]
        t    = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        l_mm = inputs[SmallTurbojetVariables.LENGTH]   * 1000.0

        z_r = (r    - _MU_R) / _SIG_R
        z_d = (d_mm - _MU_D) / _SIG_D
        z_w = (w    - _MU_W) / _SIG_W
        z_t = (t    - _MU_T) / _SIG_T
        z_l = (l_mm - _MU_L) / _SIG_L

        sfc_per_hour = (
            _INTERCEPT
            + _C_R * z_r
            + _C_D * z_d
            + _C_W * z_w
            + _C_T * z_t
            + _C_L * z_l
        )

        outputs[SmallTurbojetVariables.SFC] = sfc_per_hour * _H_TO_S

    def compute_partials(self, inputs, partials):
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.MAX_RPM]          = _C_R * _H_TO_S / _SIG_R
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER]         = _C_D * _H_TO_S * 1000.0 / _SIG_D
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.MASS]             = _C_W * _H_TO_S / _SIG_W
        partials[SmallTurbojetVariables.SFC, Aircraft.Engine.SCALED_SLS_THRUST]       = _C_T * _H_TO_S / _SIG_T
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.LENGTH]           = _C_L * _H_TO_S * 1000.0 / _SIG_L
