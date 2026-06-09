"""
2D section aerodynamic data for lifting surface airfoils.

Convention
----------
``cl_alpha_per_rad`` is the INCOMPRESSIBLE (M ~ 0) 2-D section lift slope.
Do NOT apply a Prandtl-Glauert correction before passing to
``LiftCurveSlopePolhamus`` -- the Polhamus formula handles 3-D compressibility
internally via beta = sqrt(1 - M^2).

Reference Reynolds numbers are representative of the SpaJeti UAV operating point:
  - Wing: chord ~0.33 m, V ~153 m/s, 5000 m ISA  ->  Re ~2e6
  - VTP:  chord ~0.25 m, V ~153 m/s, 5000 m ISA  ->  Re ~1.5e6

Sources: XFOIL panel code; Abbott & von Doenhoff (1959) "Theory of Wing Sections".
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AirfoilData:
    """Immutable 2-D section aerodynamic properties at the low-speed reference condition.

    cl_alpha_per_rad : float
        Incompressible 2-D lift curve slope [/rad].
        Pass directly to ``LiftCurveSlopePolhamus.section_lift_slope``.
        Polhamus handles 3-D compressibility via beta = sqrt(1 - M^2).
    cl_max : float
        Maximum 2-D lift coefficient at ``re_ref``.
    cd_min : float
        Minimum profile drag coefficient (clean, near design CL).
    cm_ac : float
        Pitching moment about the aerodynamic centre (quarter-chord).
        Negative for positively cambered sections.
    tc_ratio : float
        Thickness-to-chord ratio t/c.
    camber_ratio : float
        Maximum camber-to-chord ratio.
    re_ref : float
        Reference Reynolds number at which these values were obtained.
    """

    name:             str
    cl_alpha_per_rad: float
    cl_max:           float
    cd_min:           float
    cm_ac:            float
    tc_ratio:         float
    camber_ratio:     float
    re_ref:           float


# ---------------------------------------------------------------------------
# Catalog -- NACA 4-digit series at Re ~ 1-3e6 (UAV range)
# ---------------------------------------------------------------------------

NACA_0009 = AirfoilData(
    name='NACA 0009',
    cl_alpha_per_rad=5.95,
    cl_max=1.10,
    cd_min=0.0055,
    cm_ac=0.000,
    tc_ratio=0.09,
    camber_ratio=0.00,
    re_ref=1.0e6,
)

NACA_0012 = AirfoilData(
    name='NACA 0012',
    cl_alpha_per_rad=5.73,
    cl_max=1.30,
    cd_min=0.0070,
    cm_ac=0.000,
    tc_ratio=0.12,
    camber_ratio=0.00,
    re_ref=2.0e6,
)

NACA_2412 = AirfoilData(
    name='NACA 2412',
    cl_alpha_per_rad=5.93,
    cl_max=1.45,
    cd_min=0.0062,
    cm_ac=-0.047,
    tc_ratio=0.12,
    camber_ratio=0.02,
    re_ref=2.0e6,
)

NACA_4412 = AirfoilData(
    name='NACA 4412',
    cl_alpha_per_rad=6.10,
    cl_max=1.50,
    cd_min=0.0060,
    cm_ac=-0.099,
    tc_ratio=0.12,
    camber_ratio=0.04,
    re_ref=2.0e6,
)

NACA_4415 = AirfoilData(
    name='NACA 4415',
    cl_alpha_per_rad=6.00,   # Abbott & von Doenhoff (1959), Re=2e6; slightly lower than
                              # 4412 due to thicker boundary layer displacement effect
    cl_max=1.60,             # higher than 4412 -- thicker LE delays flow separation
    cd_min=0.0076,           # slightly higher profile drag from increased wetted thickness
    cm_ac=-0.100,            # same 4% camber as 4412 -> similar AC pitching moment
    tc_ratio=0.15,           # 15% chord -- primary reason for choosing this section:
                              # better structural depth for spar at the cost of lower M_crit
    camber_ratio=0.04,
    re_ref=2.0e6,
)
