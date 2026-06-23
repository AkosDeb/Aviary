#!/usr/bin/env python3
"""Quick 3D aircraft visualizer: reads an Aviary CSV config and shows a matplotlib figure.

Usage
-----
    python -m aviary.visualization.plot_aircraft path/to/aircraft.csv

    # or from Python
    from aviary.visualization.plot_aircraft import visualize
    visualize('aviary/models/aircraft/small_uav/small_uav.csv')
"""

import argparse
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from openmdao.utils.units import convert_units
from aviary.utils.aviary_values import AviaryValues
from aviary.utils.process_input_decks import parse_inputs


# ── colours ───────────────────────────────────────────────────────────────────
_C_FUSE    = '#c8c8c8'
_C_WING    = '#90b4e0'
_C_TAIL    = '#6890b0'
_C_ENGINE  = '#707070'
_ALPHA     = 0.80
_ALPHA_ENG = 0.90


# ── helpers ───────────────────────────────────────────────────────────────────

def _get(aviary_vals, key, unit=None, default=None):
    """Return a scalar from AviaryValues, converting to *unit* if needed."""
    val, src_unit = aviary_vals.get_item(key)   # returns (None, None) if missing
    if val is None:
        return default
    v = float(np.atleast_1d(val)[0])
    if unit and src_unit and src_unit not in ('unitless', unit):
        try:
            v = convert_units(v, src_unit, unit)
        except Exception:
            pass
    return v


def _get_array(aviary_vals, key, unit=None):
    """Return an ndarray from AviaryValues (for multi-valued variables like wing_locations)."""
    val, src_unit = aviary_vals.get_item(key)
    if val is None:
        return None
    arr = np.atleast_1d(np.asarray(val, dtype=float))
    if unit and src_unit and src_unit not in ('unitless', unit):
        try:
            arr = convert_units(arr, src_unit, unit)
        except Exception:
            pass
    return arr


# ── geometry builders ─────────────────────────────────────────────────────────

def _wing_half(x_le_root, y0, z_root, half_span, chord_root, taper, sweep_qc_deg, dih_deg):
    """
    Four (x,y,z) vertices for one wing half (y runs from y0 outward).

    Convention: x=forward, y=right/outboard, z=up.
    Sweep angle is measured at the quarter-chord line.
    """
    chord_tip = chord_root * taper
    sweep_tan = math.tan(math.radians(sweep_qc_deg))
    dih_tan   = math.tan(math.radians(dih_deg))

    # Tip LE x: follow qc-line, then shift forward by c_tip/4
    x_le_tip = x_le_root + chord_root / 4 + half_span * sweep_tan - chord_tip / 4
    z_tip    = z_root + half_span * dih_tan

    return np.array([
        [x_le_root,              y0,              z_root],   # root LE
        [x_le_root + chord_root, y0,              z_root],   # root TE
        [x_le_tip  + chord_tip,  y0 + half_span,  z_tip ],   # tip  TE
        [x_le_tip,               y0 + half_span,  z_tip ],   # tip  LE
    ])


def _sym_lifting_surface(x_le, z, span, chord_root, taper, sweep, dih=0.0):
    """Return (left_patch, right_patch) for a symmetric lifting surface."""
    right = _wing_half(x_le, 0.0, z, span / 2, chord_root, taper, sweep, dih)
    left  = right.copy()
    left[:, 1] *= -1          # mirror in Y
    return left, right


def _vtail_quad(x_le_root, y, z_root, height, chord_root, taper, sweep_qc_deg):
    """Four vertices for a vertical tail (span is in Z direction)."""
    chord_tip = chord_root * taper
    sweep_tan = math.tan(math.radians(sweep_qc_deg))
    x_le_tip  = x_le_root + chord_root / 4 + height * sweep_tan - chord_tip / 4
    return np.array([
        [x_le_root,              y, z_root],
        [x_le_root + chord_root, y, z_root],
        [x_le_tip  + chord_tip,  y, z_root + height],
        [x_le_tip,               y, z_root + height],
    ])


def _fuselage_mesh(
    length, width, height,
    nose_frac=0.20, tail_frac=0.35,
    base_w_frac=0.20, base_h_frac=0.20,
    exponent=4.0,
    nose_power_exponent=0.5,
    nose_type='power_law',
    nose_aspect_ratio=2.0,
    exponent_blend_frac=0.10,
    n_x=32, n_t=40,
):
    """
    Surface arrays (X, Y, Z) using the SuperellipseFuselageGeometry shape model.

    Longitudinal profile and cross-section exponent match the actual component
    so the visualizer shows the same body that feeds parasite drag and CDi_fus.
    """
    from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
        _fuselage_x_distribution,
        superellipse_width_height_distribution,
        superellipse_exponent_distribution,
        _superellipse_surface_points,
    )
    x_norm = _fuselage_x_distribution(
        n_x,
        nose_type,
        nose_aspect_ratio,
        width,
        length,
        blend_fraction=exponent_blend_frac,
    )
    n_x = len(x_norm)
    widths, heights = superellipse_width_height_distribution(
        x_norm, width, height, nose_frac, tail_frac, base_w_frac, base_h_frac,
        nose_power_exponent=nose_power_exponent,
        nose_type=nose_type,
        nose_aspect_ratio=nose_aspect_ratio,
        fuselage_length=length,
    )
    exponents = superellipse_exponent_distribution(
        x_norm,
        nose_frac,
        tail_frac,
        exponent,
        nose_type=nose_type,
        nose_aspect_ratio=nose_aspect_ratio,
        max_width=width,
        fuselage_length=length,
        blend_fraction=exponent_blend_frac,
    )
    theta = np.linspace(0.0, 2.0 * np.pi, n_t, endpoint=False)

    X = np.zeros((n_x, n_t))
    Y = np.zeros((n_x, n_t))
    Z = np.zeros((n_x, n_t))
    for i in range(n_x):
        X[i, :] = x_norm[i] * length
        Y[i, :], Z[i, :] = _superellipse_surface_points(
            widths[i], heights[i], exponents[i], theta
        )

    return X, Y, Z


def _resolve_aft_base_fractions(
    base_w_frac,
    base_h_frac,
    aft_base_diameter,
    aft_engine_clearance,
    engine_diameter,
    fuselage_width,
    fuselage_height,
):
    """Return aft base fractions, optionally derived from engine/nozzle diameter."""
    if aft_base_diameter is None and aft_engine_clearance is not None:
        if engine_diameter is None:
            raise ValueError(
                '--aft-engine-clearance requires an engine/nacelle diameter in the CSV '
                'or an explicit --aft-base-diameter.'
            )
        aft_base_diameter = engine_diameter + aft_engine_clearance

    if aft_base_diameter is None:
        return base_w_frac, base_h_frac

    if fuselage_width <= 0.0 or fuselage_height <= 0.0:
        raise ValueError('Fuselage width and height must be positive to derive aft base fractions.')

    return aft_base_diameter / fuselage_width, aft_base_diameter / fuselage_height


def _nacelle_mesh(x_fwd, x_aft, y_center, z_center, radius, n_t=20):
    """Surface arrays for a cylindrical engine nacelle."""
    theta = np.linspace(0.0, 2 * np.pi, n_t)
    X = np.array([[x_fwd] * n_t, [x_aft] * n_t])   # shape (2, n_t)
    Y = y_center + radius * np.cos(theta)
    Z = z_center + radius * np.sin(theta)
    Y = np.vstack([Y, Y])
    Z = np.vstack([Z, Z])
    return X, Y, Z


# ── main visualizer ───────────────────────────────────────────────────────────

def visualize(
    csv_path: str | Path,
    nose_frac: float = 0.20,
    tail_frac: float = 0.35,
    base_w_frac: float = 0.20,
    base_h_frac: float = 0.20,
    exponent: float = 4.0,
    nose_power_exponent: float = 0.5,
    nose_type: str = 'power_law',
    nose_aspect_ratio: float = 2.0,
    exponent_blend_frac: float = 0.10,
    aft_base_diameter: float | None = None,
    aft_engine_clearance: float | None = None,
) -> None:
    """
    Read an Aviary CSV aircraft config and display a 3-D matplotlib figure.

    Parameters
    ----------
    csv_path : str or Path
        Path to the Aviary aircraft CSV configuration file.
    nose_frac : float
        Fraction of fuselage length used by the smooth nose growth (default 0.20).
        Match the ``nose_length_fraction`` input of ``SuperellipseFuselageGeometry``.
    tail_frac : float
        Fraction used by the aft taper (default 0.35).
    base_w_frac : float
        Aft-end width as fraction of max width (default 0.20).
    base_h_frac : float
        Aft-end height as fraction of max height (default 0.20).
    exponent : float
        Superellipse cross-section exponent (default 4.0).
        2 = ellipse, 4 = rounded-rectangle.
    nose_type : {'power_law', 'ellipsoid'}
        Nose profile model. ``ellipsoid`` uses circular nose sections and an
        ellipsoidal radial profile.
    nose_aspect_ratio : float
        Ellipsoid nose length divided by radius. Used only for ``ellipsoid``.
    exponent_blend_frac : float
        Fraction of fuselage length used to blend ellipsoid nose sections
        from circular (n=2) to the body superellipse exponent.
    aft_base_diameter : float, optional
        Absolute aft base diameter in metres. When supplied, overrides
        ``base_w_frac`` and ``base_h_frac``.
    aft_engine_clearance : float, optional
        Clearance added to the CSV engine/nacelle diameter to derive the aft
        base diameter.
    """
    if nose_type not in ('power_law', 'ellipsoid'):
        raise ValueError("nose_type must be 'power_law' or 'ellipsoid'.")

    csv_path = Path(csv_path)
    aviary_vals = AviaryValues()
    parse_inputs(csv_path, aircraft_values=aviary_vals)

    # ── fuselage ──────────────────────────────────────────────────────────────
    fus_len = _get(aviary_vals, 'aircraft:fuselage:length', 'm', 10.0)
    fus_w   = (_get(aviary_vals, 'aircraft:fuselage:max_width', 'm')
               or _get(aviary_vals, 'aircraft:fuselage:avg_diameter', 'm', fus_len / 8))
    fus_h   = _get(aviary_vals, 'aircraft:fuselage:max_height', 'm', fus_w)

    # ── main wing ─────────────────────────────────────────────────────────────
    w_area  = _get(aviary_vals, 'aircraft:wing:area',         'm**2')
    w_span  = _get(aviary_vals, 'aircraft:wing:span',         'm')
    w_ar    = _get(aviary_vals, 'aircraft:wing:aspect_ratio',  None)   # unitless
    w_taper = _get(aviary_vals, 'aircraft:wing:taper_ratio',   None, 0.5)
    w_sweep = _get(aviary_vals, 'aircraft:wing:sweep',        'deg', 0.0)
    w_dih   = _get(aviary_vals, 'aircraft:wing:dihedral',     'deg', 0.0)

    if w_span is None and w_ar and w_area:
        w_span = math.sqrt(w_ar * w_area)
    if w_span is None:
        w_span = fus_len * 0.9
    if w_area is None:
        w_area = w_span ** 2 / (w_ar or 7.0)

    w_chord_r = 2.0 * w_area / (w_span * (1.0 + w_taper))

    # Wing LE positioned at 35% of fuselage from nose; low-mid wing by default
    wing_x_le = fus_len * 0.35
    wing_z    = -fus_h * 0.20

    # ── horizontal tail ───────────────────────────────────────────────────────
    ht_area  = _get(aviary_vals, 'aircraft:horizontal_tail:area',         'm**2')
    ht_ar    = _get(aviary_vals, 'aircraft:horizontal_tail:aspect_ratio',  None, 4.0)
    ht_taper = _get(aviary_vals, 'aircraft:horizontal_tail:taper_ratio',   None, 0.5)
    ht_sweep = _get(aviary_vals, 'aircraft:horizontal_tail:sweep',        'deg', 10.0)

    if ht_area:
        ht_span    = math.sqrt(ht_ar * ht_area)
        ht_chord_r = 2.0 * ht_area / (ht_span * (1.0 + ht_taper))
    else:
        ht_span = ht_chord_r = None

    # ── vertical tail ─────────────────────────────────────────────────────────
    vt_area  = _get(aviary_vals, 'aircraft:vertical_tail:area',         'm**2')
    vt_ar    = _get(aviary_vals, 'aircraft:vertical_tail:aspect_ratio',  None, 1.5)
    vt_taper = _get(aviary_vals, 'aircraft:vertical_tail:taper_ratio',   None, 0.5)
    vt_sweep = _get(aviary_vals, 'aircraft:vertical_tail:sweep',        'deg', 25.0)

    if vt_area:
        vt_span    = math.sqrt(vt_ar * vt_area)
        vt_chord_r = 2.0 * vt_area / (vt_span * (1.0 + vt_taper))
    else:
        vt_span = vt_chord_r = None

    # Tail TE target: 90% of fuselage length
    tail_te_x = fus_len * 0.90
    ht_x_le   = tail_te_x - (ht_chord_r or 0.0)
    vt_x_le   = tail_te_x - (vt_chord_r or 0.0)

    # ── engines ───────────────────────────────────────────────────────────────
    eng_d       = _get(aviary_vals, 'aircraft:nacelle:avg_diameter', 'm')
    eng_l       = _get(aviary_vals, 'aircraft:nacelle:avg_length',   'm')
    eng_locs    = _get_array(aviary_vals, 'aircraft:engine:wing_locations')   # fraction of semi-span
    n_fus_eng   = int(_get(aviary_vals, 'aircraft:engine:num_fuselage_engines', None, 0) or 0)
    n_wing_eng  = int(_get(aviary_vals, 'aircraft:engine:num_wing_engines',    None, 0) or 0)

    base_w_frac, base_h_frac = _resolve_aft_base_fractions(
        base_w_frac,
        base_h_frac,
        aft_base_diameter,
        aft_engine_clearance,
        eng_d,
        fus_w,
        fus_h,
    )

    if eng_locs is None and n_wing_eng > 0:
        eng_locs = np.array([0.35])     # sensible default: one engine at 35% semi-span

    # ── plot ──────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 7))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#f0f4f8')
    fig.patch.set_facecolor('#f0f4f8')

    # fuselage
    Xf, Yf, Zf = _fuselage_mesh(fus_len, fus_w, fus_h,
                                 nose_frac=nose_frac, tail_frac=tail_frac,
                                 base_w_frac=base_w_frac, base_h_frac=base_h_frac,
                                 exponent=exponent,
                                 nose_power_exponent=nose_power_exponent,
                                 nose_type=nose_type,
                                 nose_aspect_ratio=nose_aspect_ratio,
                                 exponent_blend_frac=exponent_blend_frac)
    ax.plot_surface(Xf, Yf, Zf, color=_C_FUSE, alpha=_ALPHA, linewidth=0, zorder=1)

    # main wing
    lw, rw = _sym_lifting_surface(wing_x_le, wing_z, w_span, w_chord_r, w_taper, w_sweep, w_dih)
    ax.add_collection3d(
        Poly3DCollection([lw, rw], color=_C_WING, alpha=_ALPHA,
                         edgecolor='#336699', linewidth=0.6)
    )

    # horizontal tail
    if ht_span:
        lh, rh = _sym_lifting_surface(ht_x_le, 0.0, ht_span, ht_chord_r, ht_taper, ht_sweep)
        ax.add_collection3d(
            Poly3DCollection([lh, rh], color=_C_TAIL, alpha=_ALPHA,
                             edgecolor='#224466', linewidth=0.6)
        )

    # vertical tail
    if vt_span:
        vt = _vtail_quad(vt_x_le, 0.0, fus_h / 2, vt_span, vt_chord_r, vt_taper, vt_sweep)
        ax.add_collection3d(
            Poly3DCollection([vt], color=_C_TAIL, alpha=_ALPHA,
                             edgecolor='#224466', linewidth=0.6)
        )

    # engines
    if eng_d and eng_l:
        sweep_tan = math.tan(math.radians(w_sweep))
        eng_z     = wing_z - eng_d * 0.6   # hang below wing

        if n_wing_eng > 0 and eng_locs is not None:
            for loc in eng_locs:
                y_e = loc * w_span / 2
                # Engine center x follows the wing quarter-chord line
                x_e = wing_x_le + w_chord_r / 4 + y_e * sweep_tan
                for sign in (1, -1):
                    Xe, Ye, Ze = _nacelle_mesh(x_e - eng_l / 2, x_e + eng_l / 2,
                                               sign * y_e, eng_z, eng_d / 2)
                    ax.plot_surface(Xe, Ye, Ze, color=_C_ENGINE, alpha=_ALPHA_ENG, linewidth=0)

        if n_fus_eng > 0:
            # Centre-line fuselage engine (e.g. aft fuselage)
            x_e = fus_len * 0.78
            Xe, Ye, Ze = _nacelle_mesh(x_e - eng_l / 2, x_e + eng_l / 2, 0.0, 0.0, eng_d / 2)
            ax.plot_surface(Xe, Ye, Ze, color=_C_ENGINE, alpha=_ALPHA_ENG, linewidth=0)

    # ── axes / framing ────────────────────────────────────────────────────────
    ax.set_xlabel('X — forward (m)', labelpad=6)
    ax.set_ylabel('Y — right (m)',   labelpad=6)
    ax.set_zlabel('Z — up (m)',      labelpad=6)
    ax.set_title(csv_path.stem.replace('_', ' '), fontsize=13, pad=10)
    ax.view_init(elev=22, azim=-55)

    # equal-aspect bounding box
    max_half = max(fus_len, w_span) / 2
    cx       = fus_len / 2
    z_top    = fus_h / 2 + (vt_span or 0.0)
    ax.set_xlim(cx - max_half, cx + max_half)
    ax.set_ylim(-max_half, max_half)
    ax.set_zlim(-max_half * 0.4, max(z_top, max_half * 0.4))

    plt.tight_layout()
    plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='3-D aircraft visualizer from an Aviary CSV config file'
    )
    parser.add_argument('csv', help='Path to an Aviary aircraft CSV config file')
    parser.add_argument('--nose-frac',   type=float, default=0.20, metavar='F',
                        help='Fuselage nose growth fraction (default 0.20)')
    parser.add_argument('--tail-frac',   type=float, default=0.35, metavar='F',
                        help='Fuselage aft taper fraction (default 0.35)')
    parser.add_argument('--base-w-frac', type=float, default=0.20, metavar='F',
                        help='Aft-end width / max_width (default 0.20)')
    parser.add_argument('--base-h-frac', type=float, default=0.20, metavar='F',
                        help='Aft-end height / max_height (default 0.20)')
    parser.add_argument('--exponent',    type=float, default=4.0,  metavar='N',
                        help='Superellipse cross-section exponent (default 4.0)')
    parser.add_argument('--nose-power-exponent', type=float, default=0.5, metavar='P',
                        help='Power-law nose exponent for nose_type=power_law (default 0.5)')
    parser.add_argument('--nose-type', choices=('power_law', 'ellipsoid'), default='power_law',
                        help='Fuselage nose profile model (default power_law)')
    parser.add_argument('--nose-aspect-ratio', type=float, default=2.0, metavar='A',
                        help='Ellipsoid nose length / radius for nose_type=ellipsoid (default 2.0)')
    parser.add_argument('--exponent-blend-frac', type=float, default=0.10, metavar='F',
                        help='Ellipsoid nose n=2 to body exponent blend length fraction (default 0.10)')
    parser.add_argument('--aft-base-diameter', type=float, default=None, metavar='M',
                        help='Absolute aft base diameter in metres; overrides base fractions')
    parser.add_argument('--aft-engine-clearance', type=float, default=None, metavar='M',
                        help='Clearance added to CSV nacelle diameter to derive aft base diameter')
    args = parser.parse_args()
    visualize(
        args.csv,
        nose_frac=args.nose_frac,
        tail_frac=args.tail_frac,
        base_w_frac=args.base_w_frac,
        base_h_frac=args.base_h_frac,
        exponent=args.exponent,
        nose_power_exponent=args.nose_power_exponent,
        nose_type=args.nose_type,
        nose_aspect_ratio=args.nose_aspect_ratio,
        exponent_blend_frac=args.exponent_blend_frac,
        aft_base_diameter=args.aft_base_diameter,
        aft_engine_clearance=args.aft_engine_clearance,
    )


if __name__ == '__main__':
    main()
