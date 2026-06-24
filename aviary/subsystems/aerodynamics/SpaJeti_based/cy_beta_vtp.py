import numpy as np
import openmdao.api as om

from aviary.variable_info.functions import add_aviary_input
from aviary.variable_info.variables import Aircraft, Dynamic


def _piecewise_linear(x, xp, fp):
    """Piecewise linear interpolation compatible with OpenMDAO complex-step derivatives.

    Handles both scalar and 1-element array inputs (OpenMDAO stores scalars as shape-(1,)
    arrays). Uses the real part of x for the index lookup so complex perturbations
    propagate correctly through the slope.
    """
    x0 = x.flat[0] if hasattr(x, 'flat') else x
    i = max(0, min(int(np.searchsorted(xp, float(np.real(x0)))) - 1, len(xp) - 2))
    t = (x0 - xp[i]) / (xp[i + 1] - xp[i])
    return fp[i] + t * (fp[i + 1] - fp[i])


# --------------------------------------------------------------------------
# Figure 10.19 — Effective VTP Aspect Ratio correction for H-tail geometry
# Source: Roskam, Figure 10.19
#
# X-axis: b'_v / b_v
#   For a symmetric VTP on an H-tail: b'_v = half-panel span = b_v_geometric/2
#   and b_v here = full span from fuselage centreline = b_h/2 + b_v_geometric/2
#   → b'_v/b_v = b_v_geometric / (b_h + b_v_geometric)
#
# Y-axis: A_v_eff / A_v  (U-shaped; minimum ≈ 1.0 at b'_v/b_v ≈ 0.4–0.5)
# Calibrate against the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_FIG1019_BPV_BV_GRID  = np.array([0.0,  0.1,  0.2,  0.3,  0.4,  0.5,
                                   0.6,  0.7,  0.8,  0.9,  1.0])
_FIG1019_AV_EFF_RATIO = np.array([1.50, 1.34, 1.2, 1.1, 1.03, 1.00,
                                   1.03, 1.1, 1.2, 1.34, 1.50])

# --------------------------------------------------------------------------
# Figure 10.18 — Effective side-force slope CY_beta_v_eff vs A_v_eff
# Source: Roskam VI, Figure 10.18  (replaces _AR_TABLE / _CY_EFF_TABLE in CyBetaVtp)
#
# X-axis: A_v_eff (effective aspect ratio, 0 → 5)
# Family: phi_LE — VTP leading-edge sweep (0° and 20° curves shown)
# Units: CY_beta_v_eff in per radian
# Calibrate against the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_FIG1018_AV_GRID    = np.array([0.50, 1.00, 1.50, 2.00, 2.5 ,3.00, 4.00, 5.0])
_FIG1018_SWEEP_GRID = np.array([0.0, 20.0])   # leading-edge sweep, deg

# Rows = sweep (0° → 20°),  Columns = A_v_eff (0.0 → 5.0)
_FIG1018_CY_EFF_2D = np.array([
    [1, 1.75, 2.3, 2.75, 3.1 ,3.45 , 3.8, 4.1],  # phi=0°
    [0.9, 1.7, 2.2, 2.6, 2.98, 3.3, 3.6, 3.8 ],  # phi=20°
])

# --------------------------------------------------------------------------
# Table 2: k_v = CY_beta_v(wfh) / CY_beta_v_eff
#          Wing-Fuselage-Horizontal-tail interference factor
#          Source: Rokam VI, Figure 10.17 (Ref 9) — 2D lookup
#
# k_v < 1: fuselage+HTP interference REDUCES isolated panel effectiveness.
# X-axis: 2r_i / b_v  (fuselage depth / VTP full span)
# Family: b_h / l_f   (HTP span / fuselage length)
# Calibrate these values to the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_KV_2RI_BV_GRID = np.array([0.0,  0.2,  0.4,  0.6,  0.8,  1.0])
_KV_BH_LF_GRID  = np.array([0.2,  0.4,  0.6,  0.8,  1.0])

# Rows = b_h/l_f (0.2 → 1.0),  Columns = 2r_i/b_v (0.0 → 1.0)
# All curves start at 1.0 at 2r_i/b_v = 0 (no fuselage at root → no interference).
_KV_TABLE_2D = np.array([
    [1.00, 0.81, 0.69, 0.6, 0.55, 0.51],   # b_h/l_f = 0.2  (most interference)
    [1.00, 0.88, 0.79, 0.72, 0.7, 0.69],   # b_h/l_f = 0.4
    [1.00, 0.91, 0.83, 0.82, 0.81, 0.8],   # b_h/l_f = 0.6
    [1.00, 0.94, 0.9, 0.895, 0.89, 0.89],   # b_h/l_f = 0.8
    [1.00, 0.97, 0.93, 0.925, 0.92, 0.92],   # b_h/l_f = 1.0  (least interference)
])


# --------------------------------------------------------------------------
# Figure 8.14 — Theoretical lift effectiveness of a plain flap
# Source: Roskam Part VI, Figure 8.14
#
# X-axis: c_f/c  (rudder chord / VTP chord)
# Family: t/c    (VTP airfoil thickness-to-chord ratio)
# Units: (cl_delta)_theory  [per radian]
# Calibrate against the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_FIG814_CF_C_GRID = np.array([0.05, 0.1, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50])
_FIG814_TC_GRID   = np.array([0.00, 0.05, 0.08, 0.10, 0.12, 0.15])

# Rows = t/c (0.00 → 0.15),  Columns = c_f/c (0.10 → 0.50)
_FIG814_CL_DELTA_2D = np.array([
    [1.9, 2.50, 3.00, 3.45, 3.80, 4.15, 4.45, 4.70, 4.95, 5.15],  # t/c = 0.00
    [1.9, 2.52, 3.04, 3.51, 3.88, 4.24, 4.60, 4.83, 4.04, 5.30],  # t/c = 0.05
    [1.9, 2.54, 3.08, 3.57, 3.96, 4.33, 4.70, 4.97, 5.21, 5.45],  # t/c = 0.08
    [1.9, 2.56, 3.12, 3.63, 4.04, 4.42, 4.80, 5.10, 5.34, 5.60],  # t/c = 0.10
    [1.9, 2.58, 3.16, 3.69, 4.12, 4.51, 4.90, 5.23, 5.47, 5.75],  # t/c = 0.12
    [1.9, 2.60, 3.20, 3.75, 4.20, 4.60, 5.00, 5.35, 5.60, 5.95],  # t/c = 0.15
])

# --------------------------------------------------------------------------
# Figure 8.15 — Empirical correction cl_delta / (cl_delta)_theory
# Source: Roskam Part VI, Figure 8.15
#
# X-axis: c_f/c
# Family: c_l_alpha / (c_l_alpha)_theory   where (c_l_alpha)_theory = 2π
#   → supply actual 2D airfoil lift slope as input; ratio is computed internally
# Calibrate against the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_FIG815_CF_C_GRID      = np.array([0.05, 0.10, 0.15, 0.50])
_FIG815_CLA_RATIO_GRID = np.array([0.70, 0.80, 0.90, 0.94, 1.00])

# Rows = c_l_alpha/(c_l_alpha)_theory (0.70 → 1.00),  Columns = c_f/c (0.10 → 0.50)
_FIG815_CL_RATIO_2D = np.array([
    [0.46, 0.490, 0.505,  0.54],  # ratio = 0.70
    [0.58, 0.610, 0.630,  0.72],  # ratio = 0.80
    [0.79, 0.805, 0.820,  0.87],  # ratio = 0.90
    [0.87, 0.890, 0.900,  0.92],  # ratio = 0.94
    [1.00, 1.000, 1.000,  1.00],  # ratio = 1.00
])

# --------------------------------------------------------------------------
# Figure 8.13 — Nonlinear flap correction factor κ'
# Source: Roskam Part VI, Figure 8.13
#
# X-axis: rudder deflection δ_f  [deg]
# Family: c_f/c
# κ' = 1.0 at small δ_f; decreases as separation grows at large deflections.
# Calibrate against the project's specific aerodynamic reference.
# --------------------------------------------------------------------------
_FIG813_DELTA_GRID = np.array([0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 60.0])
_FIG813_CF_C_GRID  = np.array([0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50])

# Rows = c_f/c (0.10 → 0.50),  Columns = δ_f (0 → 80 deg)
_FIG813_KAPPA_2D = np.array([
    [1.00, 1.00, 0.98, 0.90, 0.80, 0.71, 0.67, 0.65, 0.57 ],  # c_f/c=0.10
    [1.00, 1.00, 0.98, 0.87, 0.77, 0.69, 0.65, 0.62, 0.53 ],  # c_f/c=0.15
    [1.00, 1.00, 0.98, 0.84, 0.75, 0.68, 0.62, 0.59, 0.50 ],  # c_f/c=0.20
    [1.00, 1.00, 0.98, 0.81, 0.70, 0.64, 0.60, 0.57, 0.49 ],  # c_f/c=0.25
    [1.00, 1.00, 0.98, 0.78, 0.66, 0.61, 0.57, 0.54, 0.46 ],  # c_f/c=0.30
    [1.00, 1.00, 0.95, 0.75, 0.62, 0.58, 0.53, 0.51, 0.45 ],  # c_f/c=0.40
    [1.00, 0.99, 0.90, 0.70, 0.60, 0.55, 0.51, 0.49, 0.43 ],  # c_f/c=0.50
])


def _bilinear(x, y, x_grid, y_grid, z_table):
    """Bilinear interpolation over a 2-D table, compatible with complex-step.

    z_table shape: (len(y_grid), len(x_grid)).
    Index lookup uses real parts; slope computation is fully algebraic so complex
    perturbations propagate correctly through both partial derivatives.
    """
    x0 = x.flat[0] if hasattr(x, 'flat') else x
    y0 = y.flat[0] if hasattr(y, 'flat') else y
    ix = max(0, min(int(np.searchsorted(x_grid, float(np.real(x0)))) - 1, len(x_grid) - 2))
    iy = max(0, min(int(np.searchsorted(y_grid, float(np.real(y0)))) - 1, len(y_grid) - 2))
    tx = (x0 - x_grid[ix]) / (x_grid[ix + 1] - x_grid[ix])
    ty = (y0 - y_grid[iy]) / (y_grid[iy + 1] - y_grid[iy])
    z00 = z_table[iy,     ix    ]
    z01 = z_table[iy,     ix + 1]
    z10 = z_table[iy + 1, ix    ]
    z11 = z_table[iy + 1, ix + 1]
    return z00*(1-tx)*(1-ty) + z01*tx*(1-ty) + z10*(1-tx)*ty + z11*tx*ty


def _kb_cumulative(eta, lam):
    """Chord-weighted cumulative span fraction from root (η=0) to η.

    Derived analytically from Figure 8.52 (Roskam / Ref 9): for a trapezoidal
    panel with taper ratio λ, the chord distribution is c(η) = c_root*(1−η*(1−λ)).
    K_b_curve(η, λ) is the fraction of total panel lift generated inboard of η
    when lift per unit span is proportional to local chord.

        K_b_curve = [η − η²·(1−λ)/2] / [1 − (1−λ)/2]

    To get K_b for a partial-span control surface (η_root to η_tip):
        K_b = K_b_curve(η_tip, λ) − K_b_curve(η_root, λ)

    Special cases:
        λ=1 (rectangular): K_b_curve = η  (linear, as expected)
        λ=0 (triangular):  K_b_curve = 2η − η²  (concave, root-heavy)
        Full span (η_root=0, η_tip=1): K_b = 1.0 for any λ
    """
    denom = 1.0 - 0.5 * (1.0 - lam)   # = (1 + λ) / 2 = mean-chord fraction
    return (eta - 0.5 * eta ** 2 * (1.0 - lam)) / denom


class CyBetaVtp(om.ExplicitComponent):
    """Side force coefficient derivative CY_beta for H-tail vertical tail panels.

    Implements the DATCOM / Roskam empirical formula for a twin-VTP (H-tail):

        CY_beta_vtp = -2 * k_v * CY_beta_v_eff * (S_v / S_ref)

    Three-step look-up chain (all from Ref 9):

    Step 1 — Figure 10.19: effective AR ratio from H-tail geometry
        b'_v / b_v = b_v_geom / (b_h + b_v_geom)   (symmetric VTP on H-tail)
        A_v_eff / A_v = f(b'_v/b_v)  [U-curve; ≈1 when VTP is centred on HTP]

    Step 2 — Figure 10.18: isolated panel side-force slope
        CY_beta_v_eff = f(A_v_eff, phi_LE)  [per radian; 2D look-up vs LE sweep]

    Step 3 — Figure 10.17: wing-fuselage-HTP interference correction
        k_v = f(2*r_i/b_v,  b_h/l_f)  [k_v < 1; 2D look-up]

    All geometry (b_h, b_v_geom, b'_v/b_v) is derived from existing Aviary
    inputs — no additional CSV entries are required.
    """

    def setup(self):
        self.add_input(
            'fuselage_vtp_span_ratio',
            val=0.5,
            units='unitless',
            desc='2*r_i/b_v: fuselage depth over VTP full span (from tail geometry)',
        )
        # vtp_ar and vtp_area come from tail geometry (plain names, not Aviary
        # variables) so they track the VTP span design variable without
        # conflicting with the FLOPS pre-mission IndepVarComp.
        self.add_input('vtp_ar',   val=1.2,  units='unitless', desc='VTP aspect ratio from tail geometry')
        self.add_input('vtp_area', val=0.08, units='m**2',     desc='VTP panel area from tail geometry')
        add_aviary_input(self, Aircraft.VerticalTail.SWEEP, units='deg')
        self.add_input(
            'wing_ref_area',
            val=0.45,
            units='m**2',
            desc='Wing reference area (from tail geometry)',
        )
        # HTP and fuselage inputs for Figures 10.17 and 10.19
        add_aviary_input(self, Aircraft.HorizontalTail.AREA, units='m**2')
        add_aviary_input(self, Aircraft.HorizontalTail.ASPECT_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.Fuselage.LENGTH, units='m')

        self.add_output(
            'CY_beta_vtp',
            val=-1.0,
            units='unitless',
            desc='dCY/dbeta for both H-tail VTP panels (per radian, negative = stabilising)',
        )

    def setup_partials(self):
        wrt = [
            'fuselage_vtp_span_ratio',
            'vtp_ar',
            'vtp_area',
            Aircraft.VerticalTail.SWEEP,
            'wing_ref_area',
            Aircraft.HorizontalTail.AREA,
            Aircraft.HorizontalTail.ASPECT_RATIO,
            Aircraft.Fuselage.LENGTH,
        ]
        self.declare_partials('CY_beta_vtp', wrt, method='cs')

    def compute(self, inputs, outputs):
        ratio      = inputs['fuselage_vtp_span_ratio']
        vtp_ar     = inputs['vtp_ar']
        vtp_area   = inputs['vtp_area']
        sweep_deg  = inputs[Aircraft.VerticalTail.SWEEP]
        s_ref      = inputs['wing_ref_area']
        htp_area   = inputs[Aircraft.HorizontalTail.AREA]
        htp_ar     = inputs[Aircraft.HorizontalTail.ASPECT_RATIO]
        fus_len    = inputs[Aircraft.Fuselage.LENGTH]

        b_h = np.sqrt(htp_area * htp_ar)     # HTP full span
        b_v = np.sqrt(vtp_area * vtp_ar)      # VTP geometric span

        # --- Step 1: Figure 10.19 — effective AR ratio ---
        # Symmetric VTP: b'_v = half-panel span, b_v_fig = fuselage-centre to tip
        bpv_bv = b_v / (b_h + b_v)
        av_eff_ratio = _piecewise_linear(
            bpv_bv, _FIG1019_BPV_BV_GRID, _FIG1019_AV_EFF_RATIO
        )
        av_eff = vtp_ar * av_eff_ratio

        # --- Step 2: Figure 10.18 — CY_beta_v_eff from A_v_eff and LE sweep ---
        cy_beta_eff = _bilinear(
            av_eff, sweep_deg, _FIG1018_AV_GRID, _FIG1018_SWEEP_GRID, _FIG1018_CY_EFF_2D
        )

        # --- Step 3: Figure 10.17 — k_v interference correction ---
        b_h_over_l_f = b_h / fus_len
        k_v = _bilinear(ratio, b_h_over_l_f,
                        _KV_2RI_BV_GRID, _KV_BH_LF_GRID, _KV_TABLE_2D)

        outputs['CY_beta_vtp'] = -2.0 * k_v * cy_beta_eff * (vtp_area / s_ref)


class TailCantRotation(om.ExplicitComponent):
    """Rotate physical-panel lift slope into body-axis stability derivatives.

    For N symmetric panels at cant angle φ from horizontal:

        CY_beta_tail  = -N × sin²(φ) × CL_alpha_panel × S_panel / S_ref
        CL_alpha_tail =  N × cos²(φ) × CL_alpha_panel × S_panel / S_ref

    Limiting cases:
        φ = 90°  (pure VTP):  CY_beta_tail full, CL_alpha_tail = 0
        φ = 0°   (pure HTP):  CY_beta_tail = 0,  CL_alpha_tail full
        φ = 45°  (X-tail):    equal lateral and longitudinal authority

    For asymmetric tails, replace tail_cant_angle with a per-panel array and
    sum sin²/cos² over the array — this component stays unchanged.
    """

    def setup(self):
        self.add_input('CL_alpha_panel', val=2.5, units='unitless',
                       desc='Physical panel lift slope from Polhamus [per rad]')
        self.add_input('tail_physical_panel_area', val=0.08, units='m**2',
                       desc='True single-panel planform area')
        self.add_input('tail_cant_angle', val=45.0, units='deg',
                       desc='Panel cant angle from horizontal plane')
        self.add_input('tail_panel_count', val=4.0, units='unitless',
                       desc='Number of panels')
        self.add_input('wing_ref_area', val=0.45, units='m**2',
                       desc='Wing reference area S_ref')

        self.add_output('CY_beta_tail', val=-1.0, units='unitless',
                        desc='dCY/dbeta from tail, body axes [/rad], negative = stabilising')
        self.add_output('CL_alpha_tail', val=1.0, units='unitless',
                        desc='dCL/dalpha from tail, body axes [/rad]')

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        phi   = inputs['tail_cant_angle'] * (np.pi / 180.0)
        N     = inputs['tail_panel_count']
        CL_a  = inputs['CL_alpha_panel']
        S_p   = inputs['tail_physical_panel_area']
        S_ref = inputs['wing_ref_area']

        authority = N * CL_a * S_p / S_ref

        outputs['CY_beta_tail']  = -authority * np.sin(phi) ** 2
        outputs['CL_alpha_tail'] =  authority * np.cos(phi) ** 2


class CyBetaWing(om.ExplicitComponent):
    """Wing contribution to the side-force coefficient derivative CY_beta.

    Implements the DATCOM 5.1.1.1-c empirical formula for the wing CY_beta due to
    quarter-chord sweep and dihedral, with a Prandtl-Glauert Mach correction:

        CY_beta_w = (CY_beta_w_CL + CY_beta_w_dihedral) * Mach_correction

    where:
        CY_beta_w_CL       = 6·tan(Λ)·sin(Λ) / [AR·π·(AR + 4·cos(Λ))] · CL²
        CY_beta_w_dihedral = -0.00573 · Γ  [Γ in radians]
        Mach_correction    = (AR + 4·cos(Λ)) / (AR·β + 4·cos(Λ))
        β (Prandtl-Glauert) = √(1 − M²·cos²(Λ))
        Λ = quarter-chord sweep angle

    Notes
    -----
    CL is accepted as a plain input so the component is self-contained.  When wiring
    into a full mission model, connect it from the flight-dynamics CL output.

    For zero sweep and zero dihedral (e.g. simple flat-plate UAV wing) CY_beta_wing
    evaluates to zero, which is physically correct.
    """

    def initialize(self):
        self.options.declare('num_nodes', default=1, types=int,
                             desc='Number of nodes along mission segment')

    def setup(self):
        nn = self.options['num_nodes']

        # Dynamic inputs — one value per mission node
        add_aviary_input(self, Dynamic.Atmosphere.MACH, shape=nn, units='unitless')
        self.add_input(
            'CL',
            shape=nn,
            val=0.5,
            units='unitless',
            desc='Lift coefficient at each mission node',
        )

        # Geometric / design inputs — scalar
        add_aviary_input(self, Aircraft.Wing.ASPECT_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.Wing.SWEEP, units='deg')
        add_aviary_input(self, Aircraft.Wing.DIHEDRAL, units='deg')

        self.add_output(
            'CY_beta_wing',
            shape=nn,
            val=0.0,
            units='unitless',
            desc='Wing dCY/d_beta (per radian sideslip)',
        )

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        mach = inputs[Dynamic.Atmosphere.MACH]
        CL = inputs['CL']
        AR_w = inputs[Aircraft.Wing.ASPECT_RATIO]
        sweep_deg = inputs[Aircraft.Wing.SWEEP]
        dihedral_deg = inputs[Aircraft.Wing.DIHEDRAL]

        sweep_rad = sweep_deg * (np.pi / 180.0)
        dihedral_rad = dihedral_deg * (np.pi / 180.0)

        cos_lam = np.cos(sweep_rad)
        sin_lam = np.sin(sweep_rad)
        tan_lam = np.tan(sweep_rad)

        CY_beta_w_CL = (
            (6.0 * tan_lam * sin_lam)
            / (AR_w * np.pi * (AR_w + 4.0 * cos_lam))
            * CL ** 2
        )
        CY_beta_w_dihedral = -0.00573 * dihedral_rad

        # Prandtl-Glauert Mach correction (DATCOM 5.1.1.1-c)
        beta_M = np.sqrt(1.0 - mach ** 2 * cos_lam ** 2)
        mach_factor = (AR_w + 4.0 * cos_lam) / (AR_w * beta_M + 4.0 * cos_lam)

        outputs['CY_beta_wing'] = (CY_beta_w_CL + CY_beta_w_dihedral) * mach_factor


class CyBetaFuselage(om.ExplicitComponent):
    """Fuselage contribution to the side-force coefficient derivative CY_beta.

    Implements the DATCOM 5.2.1.1-a slender-body formula with wing-fuselage
    interference correction from Roskam Part VI, Figure 10.8:

        CY_beta_fuselage = -2 * Ki * (S_0 / S_ref)

    where:
        S_0  = fuselage cross-sectional area at station x_0              [m²]
               Currently approximated as constant (circular, max diameter).
               TODO: derive S_0 at x_0 when variable cross-section data is available.
        Ki   = wing-fuselage interference factor (Roskam Fig 10.8 linear fit)
             = 1.0 + 0.5 * z_w   (z_w=0: mid-wing; z_w=1: low-wing)

    The -2/rad constant is the DATCOM slender-body value valid for conventional
    fuselages in subsonic flow.

    Parameters
    ----------
    z_w : float
        Normalised vertical wing position relative to fuselage centreline.
        z_w = 0 → mid-mounted wing (minimum interference, Ki = 1.0).
        z_w = 1 → low-mounted wing (maximum interference, Ki = 1.5).
        Only low-to-mid (z_w ∈ [0, 1]) is supported; high wings are not modelled.
    """

    def setup(self):
        add_aviary_input(self, Aircraft.Fuselage.MAX_WIDTH, units='m')
        self.add_input(
            'wing_ref_area', val=0.45, units='m**2',
            desc='Wing reference area S_ref (from tail geometry)',
        )
        self.add_input(
            'z_w', val=0.0, units='unitless',
            desc='Normalised vertical wing position: 0=mid-wing (min interference), '
                 '1=low-wing (max interference). Roskam Part VI, Fig 10.8.',
        )

        self.add_output(
            'CY_beta_fuselage', val=-1.0, units='unitless',
            desc='Fuselage dCY/d_beta (per radian sideslip, negative = destabilising)',
        )

    def setup_partials(self):
        self.declare_partials(
            'CY_beta_fuselage',
            [Aircraft.Fuselage.MAX_WIDTH, 'wing_ref_area', 'z_w'],
            method='cs',
        )

    def compute(self, inputs, outputs):
        d_f   = inputs[Aircraft.Fuselage.MAX_WIDTH]
        s_ref = inputs['wing_ref_area']
        z_w   = inputs['z_w']

        ki  = 1.0 + 0.5 * z_w                  # Roskam Fig 10.8 linear fit
        s_0 = np.pi * (d_f / 2.0) ** 2         # circular cross-section at x_0

        outputs['CY_beta_fuselage'] = -2.0 * ki * (s_0 / s_ref)


class CyDeltaRudder(om.ExplicitComponent):
    """Rudder side-force effectiveness CY_delta_r for H-tail VTP panels.

    Implements the Roskam empirical formula for a twin-panel H-tail
    (Roskam Part VI, Eq. 10.12):

        CY_delta_r = -2 * CL_alpha_v * k' * K_b * cl_delta * (S_v / S_ref)

    cl_delta is computed via three Roskam figures:

        cl_delta = cl_delta_theory(cf/c, t/c)                  [Figure 8.14]
                 * cl_ratio(cf/c, CL_alpha_v / (2π))           [Figure 8.15]
                 * kappa_prime(delta_r, cf/c)                   [Figure 8.13]

    where:
        cl_delta_theory  = theoretical flap lift slope          [Fig 8.14, f(cf/c, t/c)]
        cl_ratio         = viscous correction factor            [Fig 8.15, f(cf/c, CL_alpha_v/2π)]
        kappa_prime (k') = nonlinear plain-flap correction      [Fig 8.13, f(delta_r, cf/c)]
        CL_alpha_v       = VTP 3-D lift curve slope [/rad]
        K_b              = chord-weighted rudder span fraction  [Fig 8.52, analytical]

    CL_alpha_v is an explicit input — connect from LiftCurveSlopePolhamus
    (Roskam Part VI, Eq. 8.22) for Mach- and sweep-corrected values.
    """

    def setup(self):
        # VTP geometry -- vtp_area uses plain name (from tail geometry) to avoid
        # conflict with FLOPS pre-mission IndepVarComp for aircraft:vertical_tail:area.
        self.add_input('vtp_area', val=0.08, units='m**2', desc='VTP panel area from tail geometry')
        add_aviary_input(self, Aircraft.VerticalTail.TAPER_RATIO, units='unitless')
        add_aviary_input(self, Aircraft.VerticalTail.THICKNESS_TO_CHORD, units='unitless')
        self.add_input(
            'wing_ref_area', val=0.45, units='m**2',
            desc='Wing reference area (from tail geometry)',
        )

        # VTP 3-D lift slope — wire from LiftCurveSlopePolhamus (Roskam Eq. 8.22)
        self.add_input(
            'CL_alpha_v', val=2.5, units='unitless',
            desc='VTP 3-D lift curve slope C_L_alpha [per rad]. '
                 'Connect from LiftCurveSlopePolhamus for Mach- and sweep-corrected values.',
        )

        # Rudder geometry
        self.add_input(
            'rudder_cf_c', val=0.25, units='unitless',
            desc='Rudder chord / VTP chord  (cf/c)',
        )
        self.add_input(
            'rudder_eta_root', val=0.0, units='unitless',
            desc='Inboard rudder edge as fraction of VTP span (0=root, 1=tip)',
        )
        self.add_input(
            'rudder_eta_tip', val=1.0, units='unitless',
            desc='Outboard rudder edge as fraction of VTP span (0=root, 1=tip)',
        )

        # Rudder deflection — same promoted name as in LateralLoadFactor
        self.add_input(
            'delta_r_deg', val=0.0, units='deg',
            desc='Rudder deflection angle [deg]; drives Figure 8.13 nonlinear correction',
        )

        self.add_output(
            'CY_delta_r', val=0.0, units='unitless',
            desc='Rudder dCY/d_delta_r at the current deflection (includes κ\' correction), '
                 'both H-tail panels, per radian of deflection',
        )

    def setup_partials(self):
        wrt = [
            'vtp_area',
            Aircraft.VerticalTail.TAPER_RATIO,
            Aircraft.VerticalTail.THICKNESS_TO_CHORD,
            'wing_ref_area',
            'CL_alpha_v',
            'rudder_cf_c',
            'rudder_eta_root',
            'rudder_eta_tip',
            'delta_r_deg',
        ]
        self.declare_partials('CY_delta_r', wrt, method='cs')

    def compute(self, inputs, outputs):
        s_v        = inputs['vtp_area']
        lam        = inputs[Aircraft.VerticalTail.TAPER_RATIO]
        t_c        = inputs[Aircraft.VerticalTail.THICKNESS_TO_CHORD]
        s_ref      = inputs['wing_ref_area']
        cl_alpha_v = inputs['CL_alpha_v']
        cf_c       = inputs['rudder_cf_c']
        eta_root   = inputs['rudder_eta_root']
        eta_tip    = inputs['rudder_eta_tip']
        delta_r    = inputs['delta_r_deg']

        # K_b: chord-weighted rudder span fraction (Figure 8.52, analytical)
        k_b = _kb_cumulative(eta_tip, lam) - _kb_cumulative(eta_root, lam)

        # Figure 8.14: cl_delta_theory = f(cf/c, t/c)
        cl_delta_theory = _bilinear(
            cf_c, t_c, _FIG814_CF_C_GRID, _FIG814_TC_GRID, _FIG814_CL_DELTA_2D
        )

        # Figure 8.15: viscous correction = f(cf/c, CL_alpha_v / (cl_alpha)_theory)
        cl_alpha_ratio = cl_alpha_v / (2.0 * np.pi)
        cl_ratio = _bilinear(
            cf_c, cl_alpha_ratio,
            _FIG815_CF_C_GRID, _FIG815_CLA_RATIO_GRID, _FIG815_CL_RATIO_2D
        )

        # Figure 8.13: k' nonlinear plain-flap correction = f(delta_r, cf/c)
        kappa_prime = _bilinear(
            delta_r, cf_c, _FIG813_DELTA_GRID, _FIG813_CF_C_GRID, _FIG813_KAPPA_2D
        )

        # Negative: DATCOM sign convention (stabilising force opposes sideslip).
        # Factor 2: one rudder per VTP panel, both deflecting in the same direction.
        outputs['CY_delta_r'] = (
            -2.0 * cl_alpha_v * kappa_prime * k_b
            * cl_delta_theory * cl_ratio
            * (s_v / s_ref)
        )
