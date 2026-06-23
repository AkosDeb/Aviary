#!/usr/bin/env python3
"""Standalone fuselage nose-profile comparison.

Renders three nose power-law exponents (cone, parabolic ogive, sphere-cap)
as side-by-side 3-D matplotlib subplots.  No OpenMDAO, no dashboard needed.

Usage
-----
    python -m aviary.visualization.visualize_fuselage_nose
    # or with custom exponents:
    python aviary/visualization/visualize_fuselage_nose.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers 3D projection

from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    superellipse_width_height_distribution,
    _superellipse_surface_points,
)

# ── UAV baseline parameters (matches horizontal_small_uav_config.py) ──────────
LENGTH = 2.0          # fuselage length [m]
MAX_WIDTH = 0.30      # max body width  [m]
MAX_HEIGHT = 0.30     # max body height [m]
NOSE_FRAC = 0.20      # nose section fraction
TAIL_FRAC = 0.35      # tail section fraction
BASE_W_FRAC = 0.20    # aft-end width  / max width
BASE_H_FRAC = 0.20    # aft-end height / max height
EXPONENT = 4.0        # superellipse cross-section exponent

# Nose profiles to compare
PROFILES = [
    (1.0,   'p = 1.0\nLinear cone',        '#e05d44'),
    (0.5,   'p = 0.5\nParabolic ogive',    '#3b82f6'),
    (0.333, 'p = 0.333\nSphere cap',       '#22c55e'),
]

N_X = 60     # longitudinal stations
N_T = 48     # circumferential stations


def _build_mesh(nose_power_exponent, n_x=N_X, n_t=N_T):
    """Return (X, Y, Z) surface arrays for the full fuselage."""
    x_norm = np.linspace(0.0, 1.0, n_x)
    widths, heights = superellipse_width_height_distribution(
        x_norm, MAX_WIDTH, MAX_HEIGHT,
        NOSE_FRAC, TAIL_FRAC, BASE_W_FRAC, BASE_H_FRAC,
        nose_power_exponent=nose_power_exponent,
    )
    theta = np.linspace(0.0, 2.0 * np.pi, n_t, endpoint=False)

    X = np.zeros((n_x, n_t))
    Y = np.zeros((n_x, n_t))
    Z = np.zeros((n_x, n_t))
    for i in range(n_x):
        X[i, :] = x_norm[i] * LENGTH
        Y[i, :], Z[i, :] = _superellipse_surface_points(
            widths[i], heights[i], EXPONENT, theta
        )
    return X, Y, Z


def _nose_profile_xy(nose_power_exponent, n=200):
    """Return (x, half-width) for the nose section only — for the inset 2D plot."""
    t = np.linspace(0.0, 1.0, n)
    s = t ** float(nose_power_exponent)
    x = t * NOSE_FRAC * LENGTH
    hw = s * MAX_WIDTH / 2.0
    return x, hw


def main():
    fig = plt.figure(figsize=(16, 6))
    fig.patch.set_facecolor('#1a1a2e')

    n_profiles = len(PROFILES)
    axes_3d = []

    for col, (p, label, color) in enumerate(PROFILES):
        # ── 3-D fuselage view ──────────────────────────────────────────────────
        ax = fig.add_subplot(1, n_profiles, col + 1, projection='3d')
        ax.set_facecolor('#1a1a2e')
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor('#333355')

        X, Y, Z = _build_mesh(p)
        ax.plot_surface(
            X, Y, Z,
            color=color, alpha=0.65, linewidth=0, antialiased=True,
            rcount=N_X, ccount=N_T,
        )

        # nose profile outline (top view silhouette)
        xp, hw = _nose_profile_xy(p)
        ax.plot(xp,  hw, np.zeros_like(xp), color='white', lw=1.2, zorder=5)
        ax.plot(xp, -hw, np.zeros_like(xp), color='white', lw=1.2, zorder=5)

        # axis labels and limits
        ax.set_xlim(0, LENGTH)
        ax.set_ylim(-MAX_WIDTH * 0.8, MAX_WIDTH * 0.8)
        ax.set_zlim(-MAX_HEIGHT * 0.8, MAX_HEIGHT * 0.8)
        ax.set_xlabel('x [m]', color='#aaaacc', fontsize=8)
        ax.set_ylabel('y [m]', color='#aaaacc', fontsize=8)
        ax.set_zlabel('z [m]', color='#aaaacc', fontsize=8)
        ax.tick_params(colors='#888899', labelsize=7)
        ax.set_title(label, color=color, fontsize=10, fontweight='bold', pad=8)
        ax.view_init(elev=22, azim=-60)
        axes_3d.append(ax)

    # ── inset: nose half-width profiles overlaid ───────────────────────────────
    ax2 = fig.add_axes([0.35, 0.07, 0.30, 0.22], facecolor='#0f0f1e')
    ax2.spines['bottom'].set_color('#444466')
    ax2.spines['left'].set_color('#444466')
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)
    ax2.tick_params(colors='#aaaacc', labelsize=7)
    ax2.set_xlabel('x [m]', color='#aaaacc', fontsize=8)
    ax2.set_ylabel('half-width [m]', color='#aaaacc', fontsize=8)
    ax2.set_title('Nose half-width profile', color='#ccccee', fontsize=8)

    for p, label, color in PROFILES:
        xp, hw = _nose_profile_xy(p)
        ax2.plot(xp, hw, color=color, lw=1.5, label=f'p={p}')
    ax2.legend(fontsize=7, facecolor='#1a1a2e', edgecolor='#444466',
               labelcolor='#ccccee')

    fig.suptitle(
        f'Fuselage nose power-law profiles  —  L={LENGTH} m  '
        f'W×H={MAX_WIDTH}×{MAX_HEIGHT} m  nose_frac={NOSE_FRAC}',
        color='#ccccee', fontsize=11, y=1.01,
    )
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
