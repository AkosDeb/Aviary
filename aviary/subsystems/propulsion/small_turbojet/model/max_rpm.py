import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables

# ---------------------------------------------------------------------------
# Scaler statistics from model training (MultiPoly deg=2, then StandardScaler).
# ---------------------------------------------------------------------------
_MU_D   = 137.51875;      _SIG_D   = 50.4461425113
_MU_D2  = 21456.2198958;  _SIG_D2  = 17069.300497
_MU_DL  = 53176.2965278;  _SIG_DL  = 41552.482902
_MU_L2  = 133061.907361;  _SIG_L2  = 104380.394763
_MU_L   = 342.3375;       _SIG_L   = 125.964056202

# Regression coefficients (intercept + polynomial terms).
# Categorical one-hot terms (manufacturer, engine_type, compressor_arch) are
# excluded here; all categorical indicators default to 0 (unknown/other baseline).
_INTERCEPT = 112831.0
_C_D   = -92778.9   # z(diameter_mm)
_C_D2  =  47734.7   # z(diameter_mm^2)
_C_DL  =  44667.5   # z(diameter_mm * length_mm)
_C_L2  = -19947.2   # z(length_mm^2)
_C_L   = -15760.8   # z(length_mm)


class MaxRPM(om.ExplicitComponent):
    """
    Predicts max_rpm from engine diameter and length using a degree-2
    polynomial regression with standardized features (MultiPoly deg=2).

    Inputs are in SI (m); the model was trained in mm so conversion is
    applied internally.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    are not optimizer-controlled and default to 0 (the implicit baseline).
    Fill in the _MU_* / _SIG_* module constants before first use.
    """

    def setup(self):
        self.add_input(SmallTurbojetVariables.DIAMETER, val=0.15, units='m')
        self.add_input(SmallTurbojetVariables.LENGTH, val=0.45, units='m')

        self.add_output(SmallTurbojetVariables.MAX_RPM, val=112831.0, units='rpm')

        self.declare_partials(SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.LENGTH)

    def compute(self, inputs, outputs):
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        l_mm = inputs[SmallTurbojetVariables.LENGTH] * 1000.0

        z_d  = (d_mm          - _MU_D)  / _SIG_D
        z_d2 = (d_mm**2       - _MU_D2) / _SIG_D2
        z_dl = (d_mm * l_mm   - _MU_DL) / _SIG_DL
        z_l2 = (l_mm**2       - _MU_L2) / _SIG_L2
        z_l  = (l_mm          - _MU_L)  / _SIG_L

        outputs[SmallTurbojetVariables.MAX_RPM] = (
            _INTERCEPT
            + _C_D  * z_d
            + _C_D2 * z_d2
            + _C_DL * z_dl
            + _C_L2 * z_l2
            + _C_L  * z_l
        )

    def compute_partials(self, inputs, partials):
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        l_mm = inputs[SmallTurbojetVariables.LENGTH] * 1000.0

        # Chain rule through the mm unit conversion (factor 1000 each).
        drpm_dd = (
            _C_D  * (1000.0          / _SIG_D)
            + _C_D2 * (2.0 * d_mm * 1000.0 / _SIG_D2)
            + _C_DL * (l_mm         * 1000.0 / _SIG_DL)
        )
        drpm_dl = (
            _C_DL * (d_mm           * 1000.0 / _SIG_DL)
            + _C_L2 * (2.0 * l_mm * 1000.0 / _SIG_L2)
            + _C_L  * (1000.0          / _SIG_L)
        )

        partials[SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.DIAMETER] = drpm_dd
        partials[SmallTurbojetVariables.MAX_RPM, SmallTurbojetVariables.LENGTH]   = drpm_dl
