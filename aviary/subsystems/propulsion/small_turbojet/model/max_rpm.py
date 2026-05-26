import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Model: MultiPoly(deg=2) – thrust_max_N + diameter_mm → rpm_max
# Standardization: z(x) = (x - mu) / sigma
# ---------------------------------------------------------------------------
_MU_T   = 390.895833333;   _SIG_T   = 360.249074113    # thrust_max_N
_MU_D   = 137.51875;       _SIG_D   = 50.4461425113     # diameter_mm
_MU_T2  = 282578.947917;   _SIG_T2  = 547506.598405     # thrust_max_N^2
_MU_TD  = 71366.5472222;   _SIG_TD  = 98406.5626158     # thrust_max_N * diameter_mm
_MU_D2  = 21456.2198958;   _SIG_D2  = 17069.300497      # diameter_mm^2

_INTERCEPT =  112444.0
_C_TD      = -313125.0   # z(thrust_max_N * diameter_mm)
_C_D2      =  298952.0   # z(diameter_mm^2)
_C_D       = -242297.0   # z(diameter_mm)
_C_T       =  135013.0   # z(thrust_max_N)
_C_T2      =   86464.9   # z(thrust_max_N^2)

# Categorical indicators are all ~0 (machine-epsilon) for the turbojet /
# centrifugal-compressor / generic-manufacturer baseline, so they are omitted.


class MaxRPM(om.ExplicitComponent):
    """
    Predicts max_rpm from max SLS thrust and engine diameter using a degree-2
    polynomial regression with standardized features (MultiPoly deg=2).

    Inputs are in SI (N for thrust, m for diameter); diameter is converted to
    mm internally to match training units.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline).
    """

    def setup(self):
        self.add_input(Aircraft.Engine.SCALED_SLS_THRUST, val=394.0,   units='N')
        self.add_input(SmallTurbojetVariables.DIAMETER,   val=0.14,    units='m')

        self.add_output(SmallTurbojetVariables.MAX_RPM, val=112444.0, units='rpm')

        self.declare_partials(SmallTurbojetVariables.MAX_RPM, Aircraft.Engine.SCALED_SLS_THRUST)
        self.declare_partials(SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.DIAMETER)

    def compute(self, inputs, outputs):
        t    = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0

        z_T  = (t           - _MU_T)  / _SIG_T
        z_D  = (d_mm        - _MU_D)  / _SIG_D
        z_T2 = (t ** 2      - _MU_T2) / _SIG_T2
        z_TD = (t * d_mm    - _MU_TD) / _SIG_TD
        z_D2 = (d_mm ** 2   - _MU_D2) / _SIG_D2

        outputs[SmallTurbojetVariables.MAX_RPM] = (
            _INTERCEPT
            + _C_TD * z_TD
            + _C_D2 * z_D2
            + _C_D  * z_D
            + _C_T  * z_T
            + _C_T2 * z_T2
        )

    def compute_partials(self, inputs, partials):
        t    = inputs[Aircraft.Engine.SCALED_SLS_THRUST]
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0

        # ∂rpm / ∂thrust
        drpm_dt = (
            _C_T  * (1.0          / _SIG_T)
            + _C_T2 * (2.0 * t    / _SIG_T2)
            + _C_TD * (d_mm       / _SIG_TD)
        )

        # ∂rpm / ∂diameter(m) — chain rule includes 1000 for mm→m conversion
        drpm_dd = (
            _C_D  * (1000.0           / _SIG_D)
            + _C_D2 * (2.0 * d_mm * 1000.0 / _SIG_D2)
            + _C_TD * (t         * 1000.0 / _SIG_TD)
        )

        partials[SmallTurbojetVariables.MAX_RPM, Aircraft.Engine.SCALED_SLS_THRUST] = drpm_dt
        partials[SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.DIAMETER]   = drpm_dd
