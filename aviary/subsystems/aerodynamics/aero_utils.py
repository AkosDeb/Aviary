"""
Reusable aerodynamic utility functions.

These helpers are intentionally small and framework-agnostic so they can be used
from drag build-up, stability, aeroelasticity, and reporting code without pulling
in an OpenMDAO component.

Safety convention
-----------------
Unrecoverable physics violations (M >= 1 in Polhamus, AR <= 0, S_ref <= 0) raise
``ValueError`` in the component that detects them.
Degraded-but-computable conditions (Re <= 0, M above table edge, fineness ratio < 1,
t/c out of expected range, out-of-range interpolation) emit ``RuntimeWarning`` via
``warnings.warn`` and continue with edge-held or clamped values.
"""

import warnings

import numpy as np


GAMMA_AIR = 1.4
R_AIR_SI = 287.05287
SUTHERLAND_REF_TEMP_K = 273.15
SUTHERLAND_REF_MU = 1.716e-5
SUTHERLAND_CONSTANT_K = 110.4
ROUGHNESS_HEIGHTS_M = {
    'aluminum': 10.0e-6,
    'flat_coating': 6.3e-6,
    'unpolished_sheet': 4.1e-6,
    'polished_sheet': 1.5e-6,
    'flat_cfk': 0.5e-6,
}

# R_LS: rows = Mach [0.25, 0.60, 0.80, 0.90], cols = cos(Λ_t/c) [0.50, 0.80, 0.90, 1.00]
# Digitized from Roskam Part VI, Figure 4.2.
# Extrapolation below M=0.25 uses the M=0.25 row silently (low-speed limit is stable).
# Extrapolation above M=0.90 uses the M=0.90 row with a RuntimeWarning (transonic regime).
_R_LS_MACH = np.array([0.25, 0.6, 0.8, 0.9])
_R_LS_COS_SWEEP = np.array([0.5, 0.8, 0.9, 1.0])
_R_LS_TABLE = np.array([
    [0.81, 1.03, 1.07, 1.08],
    [0.89, 1.01, 1.14, 1.15],
    [1.00, 1.21, 1.25, 1.26],
    [1.10, 1.30, 1.35, 1.36],
])

_R_WF_MACH = np.array([0.25, 0.4, 0.6, 0.7,0.8, 0.85, 0.9])
# Fuselage Reynolds number breakpoints (raw values; log10 taken internally before lookup).
# Rows of _R_WF_TABLE correspond to the Mach axis above; columns to these Re values.
_R_WF_RE_FUS = np.array([3e6, 7e6, 1e7, 1.5e7, 2e7, 3e7,4e7,5e7,6e7,7e7,1e8,7e8])
_R_WF_TABLE = np.array([
    [ 1.620, 1.720, 1.790, 1.710, 1.680, 1.520, 0.980, 0.950, 0.940, 0.934, 0.930, 0.925],
    [ 1.200, 1.300, 1.390, 1.500, 1.600, 1.600, 1.190, 1.000, 0.991, 0.989, 0.980, 0.975],
    [ 0.980, 0.990, 0.999, 1.100, 1.200, 1.490, 1.310, 1.220, 1.200, 1.130, 1.130, 1.130],
    [ 0.955, 0.965, 0.972, 0.984, 0.997, 1.130, 1.130, 1.130, 1.130, 1.130, 1.130, 1.130],
    [ 0.925, 0.935, 0.940, 0.955, 0.965, 0.990, 1.020, 1.100, 1.130, 1.130, 1.130, 1.130],
    [ 0.902, 0.910, 0.920, 0.930, 0.941, 0.970, 0.980, 1.020, 1.100, 1.130, 1.130, 1.130],
    [ 0.858, 0.878, 0.884, 0.895, 0.910, 0.940, 0.972, 0.995, 1.090, 1.130, 1.130, 1.130],
])

# Leading-edge suction parameter R, digitized approximately from Roskam Part VI,
# Figure 4.7 supplied in the project notes. Both the main chart and high-x inset
# are first-pass placeholders and intentionally warn until properly checked. The
# main chart uses
# x = Re_LER * cot(Lambda_LE) * sqrt(1 - M^2*cos(Lambda_LE)^2), with families
# keyed by A*lambda/cos(Lambda_LE). Figure 4.7 is marked M < 0.8 only.
_LE_SUCTION_PARAM_AXIS = np.array([0.0, 1.0, 2.0, 4.0, 10.0])
_LE_SUCTION_X_AXIS = np.array([2.0e3, 3.0e3, 4.0e3, 6.0e3, 8.0e3, 1.0e4, 2.0e4, 4.0e4, 8.0e4, 1.3e5])
_LE_SUCTION_TABLE = np.array([
    [0.16, 0.28, 0.39, 0.50, 0.58, 0.64, 0.74, 0.82, 0.88, 0.90],
    [0.16, 0.28, 0.39, 0.52, 0.61, 0.68, 0.78, 0.86, 0.91, 0.93],
    [0.16, 0.28, 0.39, 0.54, 0.64, 0.72, 0.82, 0.89, 0.93, 0.95],
    [0.16, 0.28, 0.39, 0.56, 0.67, 0.75, 0.85, 0.92, 0.95, 0.97],
    [0.16, 0.28, 0.39, 0.58, 0.70, 0.78, 0.88, 0.95, 0.98, 0.99],
])
_LE_SUCTION_HIGH_X_LIMIT = 1.3e5
_LE_SUCTION_INSET_PARAM_AXIS = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0])
_LE_SUCTION_INSET_R = np.array([0.86, 0.918, 0.938, 0.95, 0.958, 0.965, 0.970, 0.972])


def _interp_table_2d(x, y, x_grid, y_grid, values, table_name=None):
    """Bilinearly interpolate a table with rows keyed by x and columns by y.

    Issues a ``RuntimeWarning`` when any input falls outside the digitized table
    bounds.  Edge-held extrapolation (constant, zero derivative at boundary) is
    then applied.  Set *table_name* to include the table identifier in warnings.
    """
    x_arr, y_arr = np.broadcast_arrays(np.asarray(x), np.asarray(y))
    if table_name is not None:
        x_lo, x_hi = float(x_grid[0]), float(x_grid[-1])
        y_lo, y_hi = float(y_grid[0]), float(y_grid[-1])
        n_x = int(np.sum((x_arr < x_lo) | (x_arr > x_hi)))
        n_y = int(np.sum((y_arr < y_lo) | (y_arr > y_hi)))
        if n_x:
            warnings.warn(
                f"{table_name}: {n_x} Mach value(s) outside digitized range "
                f"[{x_lo}, {x_hi}]. Edge-held extrapolation applied.",
                RuntimeWarning,
                stacklevel=3,
            )
        if n_y:
            warnings.warn(
                f"{table_name}: {n_y} secondary-axis value(s) outside digitized range "
                f"[{y_lo}, {y_hi}]. Edge-held extrapolation applied.",
                RuntimeWarning,
                stacklevel=3,
            )
    flat_x = np.clip(x_arr.ravel(), x_grid[0], x_grid[-1])
    flat_y = np.clip(y_arr.ravel(), y_grid[0], y_grid[-1])
    out = np.empty_like(flat_x, dtype=float)

    for idx, (x_val, y_val) in enumerate(zip(flat_x, flat_y)):
        row_vals = np.array([np.interp(y_val, y_grid, row) for row in values])
        out[idx] = np.interp(x_val, x_grid, row_vals)

    return out.reshape(x_arr.shape)


def dynamic_viscosity_sutherland(temperature_K):
    """Return air dynamic viscosity [kg/(m*s)] using Sutherland's law."""
    temperature_K = np.asarray(temperature_K)
    return (
        SUTHERLAND_REF_MU
        * (temperature_K / SUTHERLAND_REF_TEMP_K) ** 1.5
        * (SUTHERLAND_REF_TEMP_K + SUTHERLAND_CONSTANT_K)
        / (temperature_K + SUTHERLAND_CONSTANT_K)
    )


def speed_of_sound(temperature_K, gamma=GAMMA_AIR, gas_constant=R_AIR_SI):
    """Return speed of sound [m/s] for a calorically perfect gas."""
    return np.sqrt(gamma * gas_constant * np.asarray(temperature_K))


def air_density_ideal_gas(static_pressure_Pa, temperature_K, gas_constant=R_AIR_SI):
    """Return air density [kg/m**3] from static pressure and temperature."""
    return np.asarray(static_pressure_Pa) / (gas_constant * np.asarray(temperature_K))


def reynolds_number(rho, velocity, characteristic_length, dynamic_viscosity):
    """Return Reynolds number from density, velocity, length, and viscosity."""
    return (
        np.asarray(rho)
        * np.asarray(velocity)
        * np.asarray(characteristic_length)
        / np.asarray(dynamic_viscosity)
    )


def reynolds_number_from_mach(
    mach,
    static_pressure_Pa,
    temperature_K,
    characteristic_length_m,
    gamma=GAMMA_AIR,
    gas_constant=R_AIR_SI,
):
    """Return Reynolds number from Mach, static pressure, temperature, and length."""
    rho = air_density_ideal_gas(static_pressure_Pa, temperature_K, gas_constant)
    velocity = np.asarray(mach) * speed_of_sound(temperature_K, gamma, gas_constant)
    mu = dynamic_viscosity_sutherland(temperature_K)
    return reynolds_number(rho, velocity, characteristic_length_m, mu)


def leading_edge_radius_ratio_naca_4_digit(thickness_to_chord):
    """Return NACA 4-digit leading-edge radius ratio ``r_LE/c``.

    Abbott and von Doenhoff give the NACA 4-digit nose radius as:

        r_LE / c = 1.1019 * (t/c)^2

    Use measured airfoil data when available; this helper is a reasonable
    fallback for the current NACA 4-digit catalog.
    """
    tc = np.asarray(thickness_to_chord, dtype=float)
    if np.any(tc < 0.0):
        raise ValueError(
            f"leading_edge_radius_ratio_naca_4_digit: t/c must be >= 0; "
            f"got min={float(np.min(tc)):.6g}."
        )
    return 1.1019 * tc**2


def leading_edge_reynolds_number_from_mach(
    mach,
    static_pressure_Pa,
    temperature_K,
    leading_edge_radius_m,
    gamma=GAMMA_AIR,
    gas_constant=R_AIR_SI,
):
    """Return leading-edge Reynolds number ``Re_LER``.

    Roskam Figure 4.7 uses:

        Re_LER = rho * U * l_LER / mu

    where ``l_LER`` is the airfoil leading-edge radius, not chord. If airfoil
    data stores ``r_LE/c``, multiply by local chord before calling this helper.
    """
    radius = np.asarray(leading_edge_radius_m, dtype=float)
    if np.any(radius <= 0.0):
        raise ValueError(
            f"leading_edge_reynolds_number_from_mach: leading_edge_radius_m "
            f"must be > 0; got min={float(np.min(radius)):.6g}."
        )
    return reynolds_number_from_mach(
        mach,
        static_pressure_Pa,
        temperature_K,
        radius,
        gamma=gamma,
        gas_constant=gas_constant,
    )


def roughness_height(surface):
    """Return representative roughness height [m] for a named surface finish."""
    return ROUGHNESS_HEIGHTS_M[surface]


def exposed_wetted_area_lifting_surface(planform_area, buried_planform_area=0.0, sides=2.0):
    """Return exposed wetted area for an airfoil-based lifting surface.

    The Roskam/DATCOM parasite-drag build-up uses the exposed wetted area, not
    the full reference/planform area hidden inside the fuselage or another body:

        S_wet_exposed = (S_planform - S_buried) * sides

    For a normal wing or tail, ``sides = 2`` accounts for top and bottom.  For
    paired panels, pass the total exposed planform area of all panels.
    """
    planform = np.asarray(planform_area, dtype=float)
    buried = np.asarray(buried_planform_area, dtype=float)
    sides_arr = np.asarray(sides, dtype=float)

    if np.any(planform <= 0.0):
        raise ValueError(
            f"exposed_wetted_area_lifting_surface: planform_area must be > 0; "
            f"got min={float(np.min(planform)):.6g}."
        )
    if np.any(buried < 0.0):
        raise ValueError(
            f"exposed_wetted_area_lifting_surface: buried_planform_area must be >= 0; "
            f"got min={float(np.min(buried)):.6g}."
        )
    if np.any(buried > planform):
        raise ValueError(
            "exposed_wetted_area_lifting_surface: buried_planform_area cannot exceed "
            "planform_area. Check fuselage intersection geometry."
        )
    if np.any(sides_arr <= 0.0):
        raise ValueError(
            f"exposed_wetted_area_lifting_surface: sides must be > 0; "
            f"got min={float(np.min(sides_arr)):.6g}."
        )

    return (planform - buried) * sides_arr


def roughness_cutoff_reynolds(characteristic_length_m, roughness_m, mach=0.0):
    """Return Raymer roughness-limited cutoff Reynolds number.

    Subsonic:
        Re_co = 38 * (l/k)**1.053

    Transonic/supersonic:
        Re_co = 44 * (l/k)**1.053 * M**1.16

    Reference: Raymer, Aircraft Design: A Conceptual Approach, 6th ed.,
    Section 12.5.3, Eq. 12.28 and 12.29.
    """
    length = np.asarray(characteristic_length_m)
    roughness = np.asarray(roughness_m)
    mach = np.asarray(mach)
    ratio = length / roughness
    subsonic = 38.0 * ratio**1.053
    transonic_supersonic = 44.0 * ratio**1.053 * np.maximum(mach, 1.0e-12) ** 1.16
    return np.where(mach < 1.0, subsonic, transonic_supersonic)


def apply_roughness_cutoff(reynolds, characteristic_length_m, roughness_m, mach=0.0):
    """Limit Reynolds number by the roughness cutoff when roughness is positive."""
    reynolds = np.asarray(reynolds)
    roughness_m = np.asarray(roughness_m)
    cutoff = np.where(
        roughness_m > 0.0,
        roughness_cutoff_reynolds(
            characteristic_length_m,
            np.maximum(roughness_m, 1.0e-30),
            mach=mach,
        ),
        reynolds,
    )
    return np.minimum(reynolds, cutoff)


def flat_plate_skin_friction_coeff(reynolds, mach=0.0, laminar_fraction=0.0):
    """Return mixed laminar/turbulent flat-plate skin-friction coefficient.

    Issues a ``RuntimeWarning`` when any Reynolds number is <= 0 (zero velocity /
    M = 0 case).  The Reynolds number is clamped to a near-zero value and the
    returned Cf is unreliably large; results at those nodes are not physically
    meaningful.
    """
    reynolds = np.asarray(reynolds)
    mach = np.asarray(mach)
    laminar_fraction = np.clip(np.asarray(laminar_fraction), 0.0, 1.0)

    n_bad = int(np.sum(reynolds <= 0.0))
    if n_bad:
        warnings.warn(
            f"flat_plate_skin_friction_coeff: {n_bad} Reynolds number(s) <= 0 "
            "(zero velocity / M=0 case). Cf clamped to near-zero-Re value; "
            "result is not physically meaningful at those nodes.",
            RuntimeWarning,
            stacklevel=2,
        )

    re_safe = np.maximum(reynolds, 1.0 + 1.0e-12)
    cf_laminar = 1.328 / np.sqrt(re_safe)
    cf_turbulent = 0.455 / (
        np.log10(re_safe) ** 2.58 * (1.0 + 0.144 * mach**2) ** 0.65
    )
    return laminar_fraction * cf_laminar + (1.0 - laminar_fraction) * cf_turbulent


def airfoil_thickness_location_parameter(max_thickness_location_over_chord):
    """Return DATCOM/Roskam airfoil thickness-location parameter ``L'``.

    Per the requested convention for this implementation:
    `L' = 1.2` when `(x/c)_m <= 0.30`, otherwise `L' = 2.0`.
    """
    x_c_m = np.asarray(max_thickness_location_over_chord)
    return np.where(x_c_m <= 0.30, 1.2, 2.0)


def lifting_surface_correction_factor(mach, cos_sweep_at_max_thickness):
    """Return DATCOM/Roskam lifting-surface correction factor ``R_LS``.

    Digitized table from Roskam Part VI, Figure 4.2.
    Valid range: Mach in [0.25, 0.90], cos(Λ_t/c) in [0.50, 1.00].

    Extrapolation behavior:
    - Below M = 0.25: edge-held (low-speed limit is stable; no warning).
    - Above M = 0.90: edge-held **and** a ``RuntimeWarning`` is issued — this
      regime is transonic/supersonic and the DATCOM correlation is no longer
      reliable.
    - cos(sweep) outside [0.50, 1.00]: edge-held and a ``RuntimeWarning`` is
      issued.
    """
    mach_arr = np.asarray(mach)
    cos_arr = np.asarray(cos_sweep_at_max_thickness)

    n_hi_mach = int(np.sum(mach_arr > _R_LS_MACH[-1]))
    if n_hi_mach:
        warnings.warn(
            f"lifting_surface_correction_factor (R_LS): {n_hi_mach} Mach value(s) "
            f"> {_R_LS_MACH[-1]} (table upper bound). The DATCOM R_LS digitization "
            "is not reliable in the transonic/supersonic regime. "
            "Edge-held extrapolation applied.",
            RuntimeWarning,
            stacklevel=2,
        )

    n_cos = int(
        np.sum((cos_arr < _R_LS_COS_SWEEP[0]) | (cos_arr > _R_LS_COS_SWEEP[-1]))
    )
    if n_cos:
        warnings.warn(
            f"lifting_surface_correction_factor (R_LS): {n_cos} cos(sweep) value(s) "
            f"outside digitized range [{_R_LS_COS_SWEEP[0]}, {_R_LS_COS_SWEEP[-1]}]. "
            "Edge-held extrapolation applied.",
            RuntimeWarning,
            stacklevel=2,
        )

    return _interp_table_2d(
        mach_arr,
        cos_arr,
        _R_LS_MACH,
        _R_LS_COS_SWEEP,
        _R_LS_TABLE,
    )


def wing_fuselage_interference_factor(fuselage_reynolds_number, mach):
    """Return DATCOM/Roskam wing-fuselage interference factor ``R_wf``.

    Interpolation is bilinear in Mach and log10(Re_fus); log10 is taken
    internally so the caller passes raw Reynolds numbers.

    Boundary behaviour
    ------------------
    - Re <= 0            : ``ValueError`` — physically impossible.
    - Re outside table   : ``ValueError`` — no reliable extrapolation.
    - Mach > 0.90        : ``ValueError`` — table ends at M = 0.90.
    - Mach < 0.25        : silent edge-hold at M = 0.25 (low-speed limit stable).
    """
    re_arr   = np.asarray(fuselage_reynolds_number)
    mach_arr = np.asarray(mach)

    # Re <= 0: hard stop (unphysical)
    if np.any(re_arr <= 0.0):
        raise ValueError(
            f"wing_fuselage_interference_factor: fuselage Reynolds number must be > 0; "
            f"got min(Re) = {float(np.min(re_arr)):.3e}."
        )

    # Re outside table range: hard stop (no valid extrapolation)
    re_lo, re_hi = float(_R_WF_RE_FUS[0]), float(_R_WF_RE_FUS[-1])
    if np.any(re_arr < re_lo) or np.any(re_arr > re_hi):
        bad = re_arr[(re_arr < re_lo) | (re_arr > re_hi)]
        raise ValueError(
            f"wing_fuselage_interference_factor: fuselage Re value(s) {bad} outside "
            f"table range [{re_lo:.3g}, {re_hi:.3g}]. "
            "No extrapolation is performed — adjust the Reynolds number or extend the table."
        )

    # Mach > 0.90: hard stop
    if np.any(mach_arr > _R_WF_MACH[-1]):
        raise ValueError(
            f"wing_fuselage_interference_factor: Mach = {float(np.max(mach_arr)):.4f} "
            f"> {_R_WF_MACH[-1]} (table upper bound). "
            "The R_wf table ends at M = 0.90; no extrapolation is performed."
        )

    # Mach < 0.25: silent edge-hold at M = 0.25 (no warning — low-speed limit is stable)

    log_re_input = np.log10(re_arr)
    log_re_axis  = np.log10(_R_WF_RE_FUS)
    return _interp_table_2d(
        mach_arr, log_re_input,
        _R_WF_MACH, log_re_axis, _R_WF_TABLE,
    )


def form_factor_lifting_surface(
    thickness_to_chord,
    max_thickness_location_over_chord=0.4,
):
    """Return DATCOM/Roskam lifting-surface thickness form factor.

    ``max_thickness_location_over_chord`` is `(x/c)_m`, the chordwise location
    of maximum airfoil thickness. For example, many NACA 6-series sections use
    `(x/c)_m ~= 0.4`; NACA 4-digit sections are closer to 0.3.

    Issues a ``RuntimeWarning`` for t/c < 0 (unphysical) or t/c > 0.50
    (well beyond practical airfoil geometry; DATCOM calibrated for t/c <= 0.30).
    """
    tc = np.asarray(thickness_to_chord, dtype=float)
    if np.any(tc < 0.0):
        warnings.warn(
            f"form_factor_lifting_surface: t/c = {float(np.min(tc)):.4f} < 0. "
            "Thickness-to-chord ratio must be >= 0.",
            RuntimeWarning,
            stacklevel=2,
        )
    if np.any(tc > 0.50):
        warnings.warn(
            f"form_factor_lifting_surface: t/c = {float(np.max(tc)):.3f} > 0.50. "
            "DATCOM calibrated for t/c <= 0.30; results above 0.50 are heavily "
            "extrapolated.",
            RuntimeWarning,
            stacklevel=2,
        )
    l_prime = airfoil_thickness_location_parameter(max_thickness_location_over_chord)
    return 1.0 + l_prime * tc + 100.0 * tc**4


def form_factor_datcom_body(fineness_ratio):
    """Return DATCOM streamlined-body form factor.

    This is the default fuselage/body form factor for the local Roskam/DATCOM
    parasite-drag build-up.

    Issues a ``RuntimeWarning`` when fineness ratio < 1. The term 60/f^3
    diverges rapidly below f = 1; verify that characteristic_length /
    max_diameter is correct.
    """
    f = np.asarray(fineness_ratio, dtype=float)
    if np.any(f < 1.0):
        warnings.warn(
            f"form_factor_datcom_body: fineness ratio {float(np.min(f)):.3f} < 1.0. "
            "The DATCOM body form factor (1 + 60/f^3 + f/400) diverges for f < 1. "
            "Verify characteristic_length / max_diameter.",
            RuntimeWarning,
            stacklevel=2,
        )
    f_safe = np.maximum(f, 1.0e-6)
    return 1.0 + 60.0 / f_safe**3 + f_safe / 400.0


def form_factor_raymer_fuselage(fineness_ratio):
    """Return Raymer fuselage form factor.

    Implemented for comparison/reference, but not used by the default fuselage
    path in ``RoskamParasiteDragBuildUp``.

    Issues a ``RuntimeWarning`` when fineness ratio < 1.  The term 5/f^1.5
    diverges rapidly below f = 1.
    """
    f = np.asarray(fineness_ratio, dtype=float)
    if np.any(f < 1.0):
        warnings.warn(
            f"form_factor_raymer_fuselage: fineness ratio {float(np.min(f)):.3f} < 1.0. "
            "The Raymer fuselage form factor (0.9 + 5/f^1.5 + f/400) diverges for f < 1. "
            "Verify characteristic_length / max_diameter.",
            RuntimeWarning,
            stacklevel=2,
        )
    f_safe = np.maximum(f, 1.0e-6)
    return 0.9 + 5.0 / f_safe**1.5 + f_safe / 400.0


# ---------------------------------------------------------------------------
# Induced drag helpers (Roskam Part VI, Section 4.2.1.2)
# ---------------------------------------------------------------------------


def leading_edge_suction_parameter_roskam(
    leading_edge_reynolds_number,
    mach,
    leading_edge_sweep_rad,
    aspect_ratio,
    taper_ratio,
):
    """Return Roskam Figure 4.7 leading-edge suction parameter ``R``.

    Parameters
    ----------
    leading_edge_reynolds_number : float or array
        ``Re_LER = rho * U * r_LE / mu``, where ``r_LE`` is leading-edge radius.
    mach : float or array
        Mach number. Figure 4.7 is marked valid for ``M < 0.8`` only.
    leading_edge_sweep_rad : float or array
        Leading-edge sweep angle [rad].
    aspect_ratio : float or array
        Wing aspect ratio ``A``. For the SpaJeti H-tail/endplate wing, pass the
        effective aspect ratio ``AR_eff`` rather than the geometric AR.
    taper_ratio : float or array
        Wing taper ratio ``lambda``.

    Notes
    -----
    First-pass linearized digitization from the supplied Figure 4.7 image:

        x = Re_LER * cot(Lambda_LE) * sqrt(1 - M^2*cos(Lambda_LE)^2)
        p = A * lambda / cos(Lambda_LE)

    For ``x >= 1.3e5``, the inset curve ``R(p)`` is used. For lower x values,
    the rough main-chart table is used down to the lower chart bound. Values
    below the chart lower bound raise ``NotImplementedError``.
    """
    re_ler, mach_arr, sweep, ar, taper = np.broadcast_arrays(
        np.asarray(leading_edge_reynolds_number, dtype=float),
        np.asarray(mach, dtype=float),
        np.asarray(leading_edge_sweep_rad, dtype=float),
        np.asarray(aspect_ratio, dtype=float),
        np.asarray(taper_ratio, dtype=float),
    )

    if np.any(re_ler <= 0.0):
        raise ValueError(
            f"leading_edge_suction_parameter_roskam: Re_LER must be > 0; "
            f"got min={float(np.min(re_ler)):.6g}."
        )
    if np.any(mach_arr < 0.0):
        raise ValueError(
            f"leading_edge_suction_parameter_roskam: Mach must be >= 0; "
            f"got min={float(np.min(mach_arr)):.6g}."
        )
    if np.any(mach_arr >= 0.8):
        warnings.warn(
            "leading_edge_suction_parameter_roskam: Figure 4.7 is marked valid "
            "for M < 0.8 only. Values at M >= 0.8 are extrapolated by the same "
            "digitized chart and should be treated with caution.",
            RuntimeWarning,
            stacklevel=2,
        )
    if np.any(ar <= 0.0):
        raise ValueError(
            f"leading_edge_suction_parameter_roskam: aspect_ratio must be > 0; "
            f"got min={float(np.min(ar)):.6g}."
        )
    if np.any(taper < 0.0):
        raise ValueError(
            f"leading_edge_suction_parameter_roskam: taper_ratio must be >= 0; "
            f"got min={float(np.min(taper)):.6g}."
        )

    cos_le = np.cos(sweep)
    if np.any(cos_le <= 0.0):
        raise ValueError(
            "leading_edge_suction_parameter_roskam: leading-edge sweep must have "
            "cos(Lambda_LE) > 0."
        )

    tan_le = np.tan(sweep)
    with np.errstate(divide='ignore', invalid='ignore'):
        cot_le = 1.0 / tan_le
    compressibility_term = np.maximum(1.0 - mach_arr**2 * cos_le**2, 0.0)
    x_param = re_ler * cot_le * np.sqrt(compressibility_term)
    family_param = ar * taper / cos_le

    # Unswept or very-low-sweep wings drive cot(Lambda_LE) -> infinity; Figure
    # 4.7 then falls onto the high-x inset curve.
    use_inset = (~np.isfinite(x_param)) | (x_param >= _LE_SUCTION_HIGH_X_LIMIT)
    use_main = ~use_inset
    if np.any(use_main & (x_param < _LE_SUCTION_X_AXIS[0])):
        bad_x = x_param[use_main & (x_param < _LE_SUCTION_X_AXIS[0])]
        raise NotImplementedError(
            "leading_edge_suction_parameter_roskam: Figure 4.7 main-chart "
            f"interpolation is not implemented for x < {_LE_SUCTION_X_AXIS[0]:.3g}. "
            f"Got x value(s) {bad_x}. The rough table currently in aero_utils.py is "
            "only a placeholder from the supplied image and has not been properly "
            "checked against a clean digitization."
        )

    warnings.warn(
        "leading_edge_suction_parameter_roskam: using first-pass digitized Roskam "
        "Figure 4.7 data. This table is a placeholder from the supplied "
        "image and has not been independently checked as a perfect digitization "
        "of the original chart.",
        RuntimeWarning,
        stacklevel=2,
    )
    inset_r = np.interp(
        np.clip(family_param, _LE_SUCTION_INSET_PARAM_AXIS[0], _LE_SUCTION_INSET_PARAM_AXIS[-1]),
        _LE_SUCTION_INSET_PARAM_AXIS,
        _LE_SUCTION_INSET_R,
    )
    main_r = _interp_table_2d(
        family_param,
        x_param,
        _LE_SUCTION_PARAM_AXIS,
        _LE_SUCTION_X_AXIS,
        _LE_SUCTION_TABLE,
    )
    return np.where(use_inset, inset_r, main_r)


def oswald_efficiency_roskam(cl_alpha_w, ar_eff, leading_edge_suction_parameter=0.98):
    """Roskam Part VI Eq. 4.12: Oswald span efficiency factor.

    e = 1.1 * (CL_alpha_w / AR_eff)
        / (R * (CL_alpha_w / AR_eff)  +  (1 - R) * pi)

    Parameters
    ----------
    cl_alpha_w : float or array
        3-D wing lift-curve slope [1/rad].  Must come from Polhamus (or
        equivalent) using AR_eff — not geometric AR.
    ar_eff : float or array
        Effective aspect ratio from the Scholz endplate correction.
    leading_edge_suction_parameter : float
        Roskam leading-edge suction parameter ``R`` from Part VI Figure 4.7.
        Default 0.98 is a clean high-Re preliminary value.

    Returns
    -------
    float or array
        Oswald span efficiency e.  Values slightly above 1.0 are physically
        valid for winglet configurations where the endplate loading distribution
        beats a bare-elliptic span loading.
    """
    cl_alpha_w = np.asarray(cl_alpha_w, dtype=float)
    ar_eff = np.asarray(ar_eff, dtype=float)
    r_suction = np.asarray(leading_edge_suction_parameter, dtype=float)
    ratio = cl_alpha_w / ar_eff
    denom = r_suction * ratio + (1.0 - r_suction) * np.pi
    return 1.1 * ratio / denom


def wing_induced_drag_roskam(
    cl,
    cl_alpha_w,
    ar_eff,
    leading_edge_suction_parameter=0.98,
):
    """Roskam Part VI Eq. 4.8 wing induced drag coefficient (no-twist, first term only).

    CDi = CL^2 / (pi * AR_eff * e)

    where  e = oswald_efficiency_roskam(cl_alpha_w, ar_eff, leading_edge_suction_parameter)

    The Roskam Eq. 4.11 trim factor (C_Lw = 1.05 * CL) is intentionally not applied
    here — CL is used directly.

    The twist correction terms from the full Eq. 4.8 are excluded:
        + 2*pi * CL * eps_t * V    (twist-CL cross term)
        + 4*pi^2 * eps_t^2 * w    (pure twist term)
    These are zero for an untwisted wing.  Implement when geometric twist is added;
    V and w are integrals over the spanwise lift distribution (Roskam Part VI App. B).

    Parameters
    ----------
    cl : float or array
        Aircraft total lift coefficient.
    cl_alpha_w : float or array
        3-D wing lift-curve slope [1/rad] (must use AR_eff).
    ar_eff : float or array
        Effective aspect ratio from Scholz endplate correction.
    leading_edge_suction_parameter : float
        Roskam leading-edge suction parameter ``R``. Default 0.98.

    Returns
    -------
    CDi : float or array
    e_oswald : float or array
    """
    cl_arr = np.asarray(cl, dtype=float)
    e = oswald_efficiency_roskam(cl_alpha_w, ar_eff, leading_edge_suction_parameter)
    cdi = cl_arr**2 / (np.pi * ar_eff * e)
    return cdi, e
