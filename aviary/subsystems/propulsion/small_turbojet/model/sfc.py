import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Model: MultiRidge(alpha=1.0) – thrust_max_N + rpm_max + diameter_mm + weight_kg → sfc_kg_Nh
# Standardization: z(x) = (x - mu) / sigma
# ---------------------------------------------------------------------------
_MU_T  = 373.079710145;   _SIG_T  = 351.579990931   # thrust_max_N
_MU_R  = 112534.782609;   _SIG_R  = 33259.2182976   # rpm_max
_MU_D  = 134.483333333;   _SIG_D  = 48.8054275512   # diameter_mm
_MU_W  = 4.00062318841;   _SIG_W  = 4.8043609812    # weight_kg

_INTERCEPT =  0.157423
_C_R       =  0.0113815   # z(rpm_max)
_C_D       = -0.00536706  # z(diameter_mm)
_C_W       =  0.00185695  # z(weight_kg)
_C_T       = -0.00111153  # z(thrust_max_N)

# Training target unit is kg/(N*h); convert to kg/(N*s) for output.
_H_TO_S = 1.0 / 3600.0

# Categorical indicators are all 0 for the turbojet / centrifugal-compressor /
# generic-manufacturer baseline, so they are omitted.


class SFC(om.ExplicitComponent):
    """
    Predicts specific fuel consumption from max SLS thrust, max_rpm, diameter, and
    engine mass using a ridge linear regression with standardized inputs
    (MultiRidge alpha=1.0).

    The model was trained with sfc in kg/(N*h); output is converted to kg/(N*s)
    to match the rest of the propulsion model.

    Inputs are in SI (N for thrust, rpm, m for diameter, kg for mass);
    mm conversion applied internally for diameter.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline).
    """

    def setup(self):
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST, val=394.0,    units='N')
        self.add_input(SmallTurbojetVariables.MAX_RPM,    val=112444.0, units='rpm')
        self.add_input(SmallTurbojetVariables.DIAMETER,   val=0.14,     units='m')
        self.add_input(SmallTurbojetVariables.MASS,       val=4.74,     units='kg')

        self.add_output(SmallTurbojetVariables.SFC, val=3.0e-5, units='kg/(N*s)')

        self.declare_partials(SmallTurbojetVariables.SFC, Aircraft.Engine.SCALED_SLS_THRUST)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.MAX_RPM)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.SFC, SmallTurbojetVariables.MASS)

    def compute(self, inputs, outputs):
        t    = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        r    = inputs[SmallTurbojetVariables.MAX_RPM]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        w    = inputs[SmallTurbojetVariables.MASS]

        z_T = (t    - _MU_T) / _SIG_T
        z_R = (r    - _MU_R) / _SIG_R
        z_D = (d_mm - _MU_D) / _SIG_D
        z_W = (w    - _MU_W) / _SIG_W

        sfc_per_hour = _INTERCEPT + _C_R * z_R + _C_D * z_D + _C_W * z_W + _C_T * z_T
        outputs[SmallTurbojetVariables.SFC] = sfc_per_hour * _H_TO_S

    def compute_partials(self, inputs, partials):
        partials[SmallTurbojetVariables.SFC, Aircraft.Engine.SCALED_SLS_THRUST] = _C_T * _H_TO_S / _SIG_T
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.MAX_RPM]    = _C_R * _H_TO_S / _SIG_R
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.DIAMETER]   = _C_D * _H_TO_S * 1000.0 / _SIG_D
        partials[SmallTurbojetVariables.SFC, SmallTurbojetVariables.MASS]       = _C_W * _H_TO_S / _SIG_W
