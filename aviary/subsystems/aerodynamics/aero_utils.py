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


def roughness_height(surface):
    """Return representative roughness height [m] for a named surface finish."""
    return ROUGHNESS_HEIGHTS_M[surface]


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

    Issues a ``RuntimeWarning`` when fineness ratio < 1. The term 5/f^1.5
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
