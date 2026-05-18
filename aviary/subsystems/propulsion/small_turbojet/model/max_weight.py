import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables

# ---------------------------------------------------------------------------
# Scaler statistics from model training (MultiPoly deg=2, then StandardScaler).
# ---------------------------------------------------------------------------
_MU_L2  = 131007.490658;    _SIG_L2  = 102217.058419    # length_mm^2
_MU_DL  = 53010.8859211;    _SIG_DL  = 40724.6216575    # diameter_mm * length_mm
_MU_L   = 340.222368421;    _SIG_L   = 123.516115078    # length_mm
_MU_D   = 138.640131579;    _SIG_D   = 50.2073044721    # diameter_mm
_MU_D2  = 21741.8595066;    _SIG_D2  = 16940.3867194    # diameter_mm^2
_MU_R   = 111038.157895;    _SIG_R   = 32630.6159397    # rpm_max
_MU_R2  = 13394229605.3;    _SIG_R2  = 7682861232.93    # rpm_max^2
_MU_DR  = 13928521.0526;    _SIG_DR  = 1610198.16787    # diameter_mm * rpm_max
_MU_LR  = 34129615.7895;    _SIG_LR  = 3681743.03232    # length_mm * rpm_max


# Regression coefficients. Categorical one-hot terms (manufacturer, engine_type,
# compressor_arch) default to 0 (unknown/other baseline).
_INTERCEPT = 4.67846
_C_L2  =  14.8589
_C_DL  = -11.2356
_C_L   =  -9.17922
_C_D   =   8.2556
_C_D2  =   3.71187
_C_R   =   3.14158
_C_R2  =  -1.9198
_C_DR  =  -0.78167
_C_LR  =   0.69979


class MaxWeight(om.ExplicitComponent):
    """
    Predicts engine mass (kg) from diameter, length, and max_rpm using a
    degree-2 polynomial regression with standardized features (MultiPoly deg=2).

    Inputs are in SI (m for geometry, rpm for speed); the model was trained
    in mm and rpm so the mm conversion is applied internally.

    Categorical predictors (manufacturer, engine_type, compressor_arch)
    default to 0 (the implicit baseline). Fill in _MU_* / _SIG_* constants
    before first use.
    """

    def setup(self):
        self.add_input(SmallTurbojetVariables.DIAMETER, val=0.15, units='m')
        self.add_input(SmallTurbojetVariables.LENGTH,   val=0.45, units='m')
        self.add_input(SmallTurbojetVariables.MAX_RPM,  val=112831.0, units='rpm')

        self.add_output(SmallTurbojetVariables.MASS, val=4.68, units='kg')

        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER)
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.LENGTH)
        self.declare_partials(SmallTurbojetVariables.MASS, SmallTurbojetVariables.MAX_RPM)

    def compute(self, inputs, outputs):
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        l_mm = inputs[SmallTurbojetVariables.LENGTH]   * 1000.0
        r    = inputs[SmallTurbojetVariables.MAX_RPM]

        z_l2 = (l_mm**2        - _MU_L2) / _SIG_L2
        z_dl = (d_mm * l_mm    - _MU_DL) / _SIG_DL
        z_l  = (l_mm           - _MU_L)  / _SIG_L
        z_d  = (d_mm           - _MU_D)  / _SIG_D
        z_d2 = (d_mm**2        - _MU_D2) / _SIG_D2
        z_r  = (r              - _MU_R)  / _SIG_R
        z_r2 = (r**2           - _MU_R2) / _SIG_R2
        z_dr = (d_mm * r       - _MU_DR) / _SIG_DR
        z_lr = (l_mm * r       - _MU_LR) / _SIG_LR

        outputs[SmallTurbojetVariables.MASS] = (
            _INTERCEPT
            + _C_L2 * z_l2
            + _C_DL * z_dl
            + _C_L  * z_l
            + _C_D  * z_d
            + _C_D2 * z_d2
            + _C_R  * z_r
            + _C_R2 * z_r2
            + _C_DR * z_dr
            + _C_LR * z_lr
        )

    def compute_partials(self, inputs, partials):
        d_mm = inputs[SmallTurbojetVariables.DIAMETER] * 1000.0
        l_mm = inputs[SmallTurbojetVariables.LENGTH]   * 1000.0
        r    = inputs[SmallTurbojetVariables.MAX_RPM]

        dm_dd = (
            _C_DL * (l_mm         * 1000.0 / _SIG_DL)
            + _C_D  * (1000.0             / _SIG_D)
            + _C_D2 * (2.0 * d_mm * 1000.0 / _SIG_D2)
            + _C_DR * (r          * 1000.0 / _SIG_DR)
        )
        dm_dl = (
            _C_L2 * (2.0 * l_mm * 1000.0 / _SIG_L2)
            + _C_DL * (d_mm       * 1000.0 / _SIG_DL)
            + _C_L  * (1000.0             / _SIG_L)
            + _C_LR * (r          * 1000.0 / _SIG_LR)
        )
        dm_dr = (
            _C_R  * (1.0          / _SIG_R)
            + _C_R2 * (2.0 * r    / _SIG_R2)
            + _C_DR * (d_mm       / _SIG_DR)
            + _C_LR * (l_mm       / _SIG_LR)
        )

        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.DIAMETER] = dm_dd
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.LENGTH]   = dm_dl
        partials[SmallTurbojetVariables.MASS, SmallTurbojetVariables.MAX_RPM]  = dm_dr
