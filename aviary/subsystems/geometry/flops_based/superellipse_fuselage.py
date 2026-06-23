"""
Parametric superellipse fuselage geometry.

This component is intentionally standalone.  It provides the geometric scalars
needed by the local Roskam/DATCOM drag build-up before those scalars are wired
into parasite drag, fuselage lift-induced drag, CG, or stability components.
"""

import math
import warnings

import numpy as np
import openmdao.api as om


def _smoothstep(t):
    """Cubic smoothstep, with zero slope at both endpoints."""
    return t * t * (3.0 - 2.0 * t)


def superellipse_area(width, height, exponent):
    """Return the area of a centered superellipse cross-section.

    The cross-section is:

        |y/a|**n + |z/b|**n = 1

    with full width ``2a`` and full height ``2b``.
    """
    a = np.asarray(width, dtype=float) / 2.0
    b = np.asarray(height, dtype=float) / 2.0
    n = np.asarray(exponent, dtype=float)
    if n.ndim == 0:
        n_float = float(n)
        coeff = 4.0 * math.gamma(1.0 + 1.0 / n_float) ** 2 / math.gamma(1.0 + 2.0 / n_float)
    else:
        coeff = np.vectorize(
            lambda n_i: 4.0
            * math.gamma(1.0 + 1.0 / n_i) ** 2
            / math.gamma(1.0 + 2.0 / n_i)
        )(n)
    return coeff * a * b


def superellipse_width_height_distribution(
    x_over_length,
    max_width,
    max_height,
    nose_fraction,
    tail_fraction,
    base_width_fraction,
    base_height_fraction,
    nose_power_exponent=0.5,
    nose_type='power_law',
    nose_aspect_ratio=2.0,
    fuselage_length=1.0,
):
    """Return full width and height along the normalized fuselage length.

    The nose contour follows a power-law profile:

        s(t) = t ** nose_power_exponent,   t = x / L_nose ∈ [0, 1]

    nose_power_exponent controls the nose openness:
      1.0  → linear (conical)
      0.5  → parabolic ogive  (default; approximates a hemisphere profile)
      0.333→ cubic root        (closely matches a spherical cap)

    The tail uses a cubic smoothstep (zero slope at both ends, unchanged).
    """
    x = np.asarray(x_over_length, dtype=float)
    nose_type = str(nose_type)
    if nose_type == 'ellipsoid':
        nose = float(nose_aspect_ratio) * float(max_width) / (2.0 * float(fuselage_length))
    else:
        nose = float(nose_fraction)
    tail = float(tail_fraction)
    body_start = nose
    body_end = 1.0 - tail

    width = np.empty_like(x, dtype=float)
    height = np.empty_like(x, dtype=float)

    nose_mask = x < body_start
    body_mask = (x >= body_start) & (x <= body_end)
    tail_mask = x > body_end

    if nose > 0.0:
        t = np.clip(x[nose_mask] / nose, 0.0, 1.0)
        if nose_type == 'ellipsoid':
            nose_shape = np.sqrt(np.maximum(1.0 - (1.0 - t) ** 2, 0.0))
        else:
            # Power-law nose: t^p opens quickly (p<1) or slowly (p>1) from the tip.
            # numpy correctly returns 0.0 for 0.0**p when p > 0, so no special casing needed.
            nose_shape = t ** float(nose_power_exponent)
    else:
        nose_shape = np.ones(np.count_nonzero(nose_mask))
    width[nose_mask] = max_width * nose_shape
    if nose_type == 'ellipsoid':
        height[nose_mask] = max_width * nose_shape
    else:
        height[nose_mask] = max_height * nose_shape

    width[body_mask] = max_width
    height[body_mask] = max_height

    if tail > 0.0:
        t = np.clip((x[tail_mask] - body_end) / tail, 0.0, 1.0)
        tail_shape = 1.0 + (base_width_fraction - 1.0) * _smoothstep(t)
        tail_height_shape = 1.0 + (base_height_fraction - 1.0) * _smoothstep(t)
    else:
        tail_shape = np.ones(np.count_nonzero(tail_mask)) * base_width_fraction
        tail_height_shape = np.ones(np.count_nonzero(tail_mask)) * base_height_fraction
    width[tail_mask] = max_width * tail_shape
    height[tail_mask] = max_height * tail_height_shape

    return width, height


def superellipse_exponent_distribution(
    x_over_length,
    nose_fraction,
    tail_fraction,
    superellipse_exponent,
    nose_type='power_law',
    nose_aspect_ratio=2.0,
    max_width=1.0,
    fuselage_length=1.0,
    blend_fraction=0.10,
):
    """Return the cross-section exponent along the normalized fuselage length."""
    x = np.asarray(x_over_length, dtype=float)
    exponent = float(superellipse_exponent)

    if str(nose_type) != 'ellipsoid':
        return np.full_like(x, exponent, dtype=float)

    nose = float(nose_aspect_ratio) * float(max_width) / (2.0 * float(fuselage_length))
    blend = max(float(blend_fraction), 0.0)

    exponents = np.full_like(x, exponent, dtype=float)
    exponents[x <= nose] = 2.0

    if blend > 0.0:
        blend_mask = (x > nose) & (x < nose + blend)
        t = np.clip((x[blend_mask] - nose) / blend, 0.0, 1.0)
        exponents[blend_mask] = 2.0 + (exponent - 2.0) * _smoothstep(t)

    return exponents


def _fuselage_x_distribution(
    num_x,
    nose_type,
    nose_aspect_ratio,
    max_width,
    fuselage_length,
    blend_fraction=0.10,
):
    """Return normalized fuselage stations with extra samples in the nose blend."""
    x_norm = np.linspace(0.0, 1.0, num_x)

    if str(nose_type) != 'ellipsoid':
        return x_norm

    nose = float(nose_aspect_ratio) * float(max_width) / (2.0 * float(fuselage_length))
    blend_start = nose
    blend_end = min(nose + float(blend_fraction), 1.0)
    if blend_end <= blend_start:
        return x_norm

    base_in_blend = np.count_nonzero((x_norm > blend_start) & (x_norm < blend_end))
    if base_in_blend <= 0:
        return x_norm

    extra = np.linspace(blend_start, blend_end, base_in_blend + 2)[1:-1]
    return np.unique(np.concatenate((x_norm, extra)))


def _superellipse_surface_points(width, height, exponent, theta):
    """Return y/z points around one superellipse cross-section."""
    a = width / 2.0
    b = height / 2.0
    c = np.cos(theta)
    s = np.sin(theta)
    power = 2.0 / exponent
    y = a * np.sign(c) * np.abs(c) ** power
    z = b * np.sign(s) * np.abs(s) ** power
    return y, z


def _mesh_surface_area(x, widths, heights, exponent, num_theta=96):
    """Return numerical superellipse loft surface area."""
    theta = np.linspace(0.0, 2.0 * np.pi, num_theta, endpoint=False)
    points = np.empty((len(x), num_theta, 3))
    exponents = np.broadcast_to(np.asarray(exponent, dtype=float), widths.shape)

    for i, (x_i, width, height, exp_i) in enumerate(zip(x, widths, heights, exponents)):
        y, z = _superellipse_surface_points(width, height, exp_i, theta)
        points[i, :, 0] = x_i
        points[i, :, 1] = y
        points[i, :, 2] = z

    area = 0.0
    for i in range(len(x) - 1):
        p00 = points[i]
        p10 = points[i + 1]
        p01 = np.roll(points[i], -1, axis=0)
        p11 = np.roll(points[i + 1], -1, axis=0)

        tri1 = 0.5 * np.linalg.norm(np.cross(p10 - p00, p11 - p00), axis=1)
        tri2 = 0.5 * np.linalg.norm(np.cross(p11 - p00, p01 - p00), axis=1)
        area += float(np.sum(tri1 + tri2))

    return area


class SuperellipseFuselageGeometry(om.ExplicitComponent):
    """Smooth parametric fuselage geometry based on superellipse sections.

    Inputs
    ------
    fuselage_length : m
        Overall fuselage length.
    max_width : m
        Maximum body width in top view.
    max_height : m
        Maximum body height in side view.
    nose_length_fraction : unitless
        Fraction of length used by the smooth nose growth.
    tail_length_fraction : unitless
        Fraction of length used by the smooth aft taper.
    base_width_fraction, base_height_fraction : unitless
        Aft-end width/height as fractions of the maximum dimensions.
    superellipse_exponent : unitless
        Cross-section exponent. n=2 is an ellipse; n=4 is rounded-rectangle-like.
    nose_power_exponent : unitless
        Power-law exponent for the nose contour profile. Controls how quickly
        the cross-section opens from the tip:

            s(t) = t ** p,   t = x / L_nose

        p = 1.0  → linear / conical (sharp cone)
        p = 0.5  → parabolic ogive  (default; approximates hemisphere profile)
        p = 0.333→ cubic-root       (closely matches a spherical cap)

    Outputs
    -------
    fuselage_planform_area : m**2
        Top-view planform area, integral of width(x) dx. This is the Roskam
        ``S_plf_fus`` candidate.
    fuselage_base_area : m**2
        Aft-end cross-section area, the Roskam ``S_b_fus`` candidate.
    fuselage_base_diameter : m
        Diameter of a circle with the same aft-end cross-section area.
    fuselage_wetted_area : m**2
        Numerical loft wetted area, excluding base cap area.
    fuselage_equivalent_diameter : m
        Diameter of a circle with the same maximum cross-section area.
    fuselage_fineness_ratio : unitless
        length / equivalent_diameter.
    fuselage_max_cross_section_area : m**2
        Maximum superellipse cross-section area.
    fuselage_volume : m**3
        Integrated body volume.
    fuselage_centroid_x : m
        Geometric volume centroid from the nose.
    """

    def initialize(self):
        self.options.declare('num_x', default=81, types=int)
        self.options.declare('num_theta', default=96, types=int)
        self.options.declare('nose_type', default='power_law', values=('power_law', 'ellipsoid'))

    def setup(self):
        self.add_input('fuselage_length', val=2.0, units='m')
        self.add_input('max_width', val=0.15, units='m')
        self.add_input('max_height', val=0.15, units='m')
        self.add_input('nose_length_fraction', val=0.20, units='unitless')
        self.add_input('tail_length_fraction', val=0.35, units='unitless')
        self.add_input('base_width_fraction', val=0.20, units='unitless')
        self.add_input('base_height_fraction', val=0.20, units='unitless')
        self.add_input('superellipse_exponent', val=4.0, units='unitless')
        self.add_input(
            'nose_power_exponent', val=0.5, units='unitless',
            desc='Power-law exponent for nose profile: s=t^p. '
                 'p=0.5 (default) = parabolic ogive; p=1.0 = cone.',
        )
        self.add_input(
            'nose_aspect_ratio', val=2.0, units='unitless',
            desc='Ellipsoid nose length divided by nose radius. 1.0=hemisphere; '
                 '2.0=prolate ellipsoid; used when nose_type="ellipsoid".',
        )

        self.add_output('fuselage_planform_area', val=0.24, units='m**2')
        self.add_output('fuselage_base_area', val=0.004, units='m**2')
        self.add_output('fuselage_base_diameter', val=0.07, units='m')
        self.add_output('fuselage_wetted_area', val=0.9, units='m**2')
        self.add_output('fuselage_equivalent_diameter', val=0.17, units='m')
        self.add_output('fuselage_fineness_ratio', val=12.0, units='unitless')
        self.add_output('fuselage_max_cross_section_area', val=0.02, units='m**2')
        self.add_output('fuselage_volume', val=0.03, units='m**3')
        self.add_output('fuselage_centroid_x', val=1.0, units='m')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        length = float(inputs['fuselage_length'].ravel()[0])
        max_width = float(inputs['max_width'].ravel()[0])
        max_height = float(inputs['max_height'].ravel()[0])
        nose_frac = float(inputs['nose_length_fraction'].ravel()[0])
        tail_frac = float(inputs['tail_length_fraction'].ravel()[0])
        base_w_frac = float(inputs['base_width_fraction'].ravel()[0])
        base_h_frac = float(inputs['base_height_fraction'].ravel()[0])
        exponent = float(inputs['superellipse_exponent'].ravel()[0])
        nose_power = float(inputs['nose_power_exponent'].ravel()[0])
        nose_aspect_ratio = float(inputs['nose_aspect_ratio'].ravel()[0])
        nose_type = self.options['nose_type']

        if length <= 0.0:
            raise ValueError(f'SuperellipseFuselageGeometry: fuselage_length must be > 0; got {length}.')
        if max_width <= 0.0:
            raise ValueError(f'SuperellipseFuselageGeometry: max_width must be > 0; got {max_width}.')
        if max_height <= 0.0:
            raise ValueError(f'SuperellipseFuselageGeometry: max_height must be > 0; got {max_height}.')
        if nose_frac < 0.0 or tail_frac < 0.0:
            raise ValueError(
                'SuperellipseFuselageGeometry: nose_length_fraction and '
                f'tail_length_fraction must be >= 0; got {nose_frac}, {tail_frac}.'
            )
        if nose_type != 'ellipsoid' and nose_frac + tail_frac > 1.0:
            raise ValueError(
                'SuperellipseFuselageGeometry: nose_length_fraction + '
                f'tail_length_fraction must be <= 1; got {nose_frac + tail_frac}.'
            )
        if nose_type == 'ellipsoid':
            if nose_aspect_ratio < 1.0:
                raise ValueError(
                    'SuperellipseFuselageGeometry: nose_aspect_ratio must be >= 1 '
                    f'for ellipsoid nose; got {nose_aspect_ratio}.'
                )
            ellipsoid_nose_frac = nose_aspect_ratio * max_width / (2.0 * length)
            if ellipsoid_nose_frac + tail_frac > 1.0:
                raise ValueError(
                    'SuperellipseFuselageGeometry: ellipsoid nose length + tail length '
                    f'must be <= fuselage length; got fraction {ellipsoid_nose_frac + tail_frac}.'
                )
            if not math.isclose(max_width, max_height, rel_tol=1.0e-9, abs_tol=1.0e-12):
                warnings.warn(
                    'SuperellipseFuselageGeometry: ellipsoid nose uses circular sections '
                    'based on max_width; max_height differs, so the nose/body junction '
                    'height will step unless the body dimensions are made equal.',
                    RuntimeWarning,
                    stacklevel=2,
                )
        if base_w_frac < 0.0 or base_h_frac < 0.0:
            raise ValueError(
                'SuperellipseFuselageGeometry: base fractions must be >= 0; '
                f'got {base_w_frac}, {base_h_frac}.'
            )
        if exponent < 1.0:
            raise ValueError(
                f'SuperellipseFuselageGeometry: superellipse_exponent must be >= 1; got {exponent}.'
            )
        if nose_power <= 0.0:
            raise ValueError(
                f'SuperellipseFuselageGeometry: nose_power_exponent must be > 0; got {nose_power}.'
            )

        num_x = self.options['num_x']
        x_norm = _fuselage_x_distribution(
            num_x,
            nose_type,
            nose_aspect_ratio,
            max_width,
            length,
        )
        x = x_norm * length
        widths, heights = superellipse_width_height_distribution(
            x_norm,
            max_width,
            max_height,
            nose_frac,
            tail_frac,
            base_w_frac,
            base_h_frac,
            nose_power_exponent=nose_power,
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
            max_width=max_width,
            fuselage_length=length,
        )
        areas = superellipse_area(widths, heights, exponents)
        max_area = float(np.max(areas))
        base_area = superellipse_area(
            max_width * base_w_frac,
            max_height * base_h_frac,
            exponent,
        )

        planform_area = np.trapezoid(widths, x)
        volume = np.trapezoid(areas, x)
        if volume <= 0.0:
            raise ValueError('SuperellipseFuselageGeometry: computed volume is nonpositive.')
        centroid_x = np.trapezoid(areas * x, x) / volume
        wetted_area = _mesh_surface_area(
            x,
            widths,
            heights,
            exponents,
            num_theta=self.options['num_theta'],
        )
        equivalent_diameter = np.sqrt(4.0 * max_area / np.pi)
        base_diameter = np.sqrt(4.0 * base_area / np.pi)

        outputs['fuselage_planform_area'] = planform_area
        outputs['fuselage_base_area'] = base_area
        outputs['fuselage_base_diameter'] = base_diameter
        outputs['fuselage_wetted_area'] = wetted_area
        outputs['fuselage_equivalent_diameter'] = equivalent_diameter
        outputs['fuselage_fineness_ratio'] = length / equivalent_diameter
        outputs['fuselage_max_cross_section_area'] = max_area
        outputs['fuselage_volume'] = volume
        outputs['fuselage_centroid_x'] = centroid_x
