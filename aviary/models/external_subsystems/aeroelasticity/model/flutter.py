import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


def _scalar(value):
    # .item() preserves complex dtype needed for complex-step differentiation.
    return np.asarray(value).item()


def max_real_eigenvalue(
    velocity,
    rho,
    wing_area,
    chord,
    mass_per_span,
    pitch_inertia,
    control_inertia,
    pitch_static_unbalance,
    control_static_unbalance,
    k_h,
    k_alpha,
    k_delta,
    zeta,
    cl_alpha,
    cl_delta,
    cm_alpha,
    cm_delta,
    ch_alpha,
    ch_delta,
):
    """Return the eigenvalue with the largest real part for the 3-DOF quasi-steady model.

    Returns a complex number so that complex-step differentiation is preserved through
    the call chain. Callers that need a real scalar for comparisons should use np.real().
    """

    velocity = _scalar(velocity)
    rho = _scalar(rho)
    wing_area = _scalar(wing_area)
    chord = _scalar(chord)
    mass_per_span = _scalar(mass_per_span)
    pitch_inertia = _scalar(pitch_inertia)
    control_inertia = _scalar(control_inertia)
    pitch_static_unbalance = _scalar(pitch_static_unbalance)
    control_static_unbalance = _scalar(control_static_unbalance)
    k_h = _scalar(k_h)
    k_alpha = _scalar(k_alpha)
    k_delta = _scalar(k_delta)
    zeta = _scalar(zeta)
    cl_alpha = _scalar(cl_alpha)
    cl_delta = _scalar(cl_delta)
    cm_alpha = _scalar(cm_alpha)
    cm_delta = _scalar(cm_delta)
    ch_alpha = _scalar(ch_alpha)
    ch_delta = _scalar(ch_delta)

    # Guard: zero or negative mass would make the system degenerate.
    mass_per_span_safe = mass_per_span if np.real(mass_per_span) > 0.0 else 1e-12

    # M[0,2] = M[2,0] = Sδ (control static unbalance about hinge).
    # Sδ > 0: control CG aft of hinge (destabilising); Sδ = 0: perfectly mass-balanced.
    mass_matrix = np.array(
        [
            [mass_per_span_safe, pitch_static_unbalance, control_static_unbalance],
            [pitch_static_unbalance, pitch_inertia + control_inertia, control_inertia],
            [control_static_unbalance, control_inertia, control_inertia],
        ]
    )
    stiffness_matrix = np.diag([k_h, k_alpha, k_delta])

    damping_matrix = np.diag(
        [
            2.0 * zeta * np.sqrt(mass_per_span_safe * k_h),
            2.0 * zeta * np.sqrt((pitch_inertia + control_inertia) * k_alpha),
            2.0 * zeta * np.sqrt(control_inertia * k_delta),
        ]
    )

    aero_matrix = np.array(
        [
            [0.0, wing_area * cl_alpha, wing_area * cl_delta],
            [0.0, wing_area * chord * cm_alpha, wing_area * chord * cm_delta],
            [0.0, wing_area * chord * ch_alpha, wing_area * chord * ch_delta],
        ]
    )

    q_dyn = 0.5 * rho * velocity**2
    effective_stiffness = stiffness_matrix - q_dyn * aero_matrix

    try:
        inv_mass = np.linalg.inv(mass_matrix)
    except np.linalg.LinAlgError:
        # Singular mass matrix (degenerate geometry). Return large positive value
        # so the caller treats the condition as unstable rather than crashing.
        return 1e6

    # Quasi-steady p-k: plunge rate changes apparent AoA. With the h-positive
    # convention used by this lumped model, the resulting force opposes plunge
    # rate, so it adds to the effective damping matrix.
    aero_vel = np.array([
        [wing_area * cl_alpha, 0.0, 0.0],
        [wing_area * chord * cm_alpha, 0.0, 0.0],
        [wing_area * chord * ch_alpha, 0.0, 0.0],
    ])
    effective_damping = damping_matrix + 0.5 * rho * velocity * aero_vel
    top = np.hstack((np.zeros((3, 3)), np.eye(3)))
    bottom = np.hstack((-inv_mass @ effective_stiffness, -inv_mass @ effective_damping))
    state_matrix = np.vstack((top, bottom))
    eigvals = np.linalg.eigvals(state_matrix)

    # Return the full complex eigenvalue so the imaginary part (CS derivative info)
    # is preserved. Callers must use np.real() for threshold comparisons.
    return eigvals[np.argmax(eigvals.real)]


class QuasiSteadyFlutterScreen(om.ExplicitComponent):
    """3-DOF quasi-steady flutter screen with constant generalized lift distribution."""

    def initialize(self):
        self.options.declare('num_speed_samples', default=80, types=int)
        self.options.declare('bisection_iterations', default=32, types=int)

    def setup(self):
        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')
        self.add_input(AE.DESIGN_SPEED, val=150.0, units='m/s')
        self.add_input(AE.REQUIRED_SPEED, val=1.15 * 550.0 / 3.6, units='m/s')
        self.add_input(AE.FLUTTER_MAX_SPEED, val=350.0, units='m/s')

        self.add_input(AE.MASS_PER_UNIT_SPAN, val=1.0, units='kg/m')
        self.add_input(AE.PITCH_INERTIA_PER_UNIT_SPAN, val=0.01, units='kg*m')
        self.add_input(AE.CONTROL_INERTIA_PER_UNIT_SPAN, val=0.001, units='kg*m')
        self.add_input(AE.PITCH_STATIC_UNBALANCE, val=0.0, units='kg')
        self.add_input(AE.CONTROL_STATIC_UNBALANCE, val=0.0, units='kg')
        self.add_input(AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN, val=0.0, units='kg/m')
        self.add_input(AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN, val=0.0, units='kg*m')
        self.add_input(AE.VTP_PITCH_STATIC_UNBALANCE, val=0.0, units='kg')
        self.add_input(AE.BENDING_STIFFNESS_PLUNGE, val=1.0e5, units='N/m')
        self.add_input(AE.TORSIONAL_STIFFNESS, val=1.0e4, units='N*m/rad')
        self.add_input(AE.CONTROL_STIFFNESS, val=5.0e3, units='N*m/rad')
        self.add_input(AE.STRUCTURAL_DAMPING_RATIO, val=0.02)

        self.add_input(AE.LIFT_CURVE_SLOPE, val=2.0 * np.pi, units='unitless')
        self.add_input(AE.CONTROL_LIFT_DERIVATIVE, val=2.5, units='1/rad')
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125, units='unitless')
        # Cm/alpha about the AC (thin-airfoil default ≈ 0). EA-referenced value is
        # derived internally: cm_alpha_EA = cl_alpha * e_frac + cm_alpha_AC.
        self.add_input(AE.CONTROL_MOMENT_ALPHA_DERIVATIVE, val=-0.05, units='1/rad')
        self.add_input(AE.CONTROL_MOMENT_DERIVATIVE, val=-0.60, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_ALPHA_DERIVATIVE, val=-0.02, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_CONTROL_DERIVATIVE, val=-0.30, units='1/rad')

        self.add_output(AE.FLUTTER_SPEED, val=350.0, units='m/s')
        self.add_output(AE.FLUTTER_DYNAMIC_PRESSURE, val=1.0e5, units='Pa')
        self.add_output(AE.FLUTTER_MARGIN, val=1.0)
        self.add_output(AE.FLUTTER_SPEED_MARGIN, val=100.0, units='m/s')
        self.add_output(AE.MAX_REAL_EIGENVALUE_AT_DESIGN, val=-1.0, units='1/s')

        # CS gives machine-precision derivatives for MAX_REAL_EIGENVALUE_AT_DESIGN (smooth).
        # FLUTTER_SPEED/FLUTTER_MARGIN are found via bisection so their CS derivatives
        # w.r.t. structural inputs are zero — use FD if those gradients are needed for
        # optimisation. MAX_REAL_EIGENVALUE_AT_DESIGN < 0 is the preferred constraint.
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        area = inputs[Aircraft.Wing.AREA]
        span = inputs[Aircraft.Wing.SPAN]
        taper = inputs[Aircraft.Wing.TAPER_RATIO]
        rho = inputs[AE.AIR_DENSITY]
        design_speed = inputs[AE.DESIGN_SPEED]
        required_speed = inputs[AE.REQUIRED_SPEED]
        flutter_max_speed = inputs[AE.FLUTTER_MAX_SPEED]

        root_chord = 2.0 * area / (span * (1.0 + taper))
        chord = (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / (1.0 + taper)

        effective_mass_per_span = (
            inputs[AE.MASS_PER_UNIT_SPAN] + inputs[AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN]
        )
        effective_pitch_inertia = (
            inputs[AE.PITCH_INERTIA_PER_UNIT_SPAN] + inputs[AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN]
        )
        effective_static_unbalance = (
            inputs[AE.PITCH_STATIC_UNBALANCE] + inputs[AE.VTP_PITCH_STATIC_UNBALANCE]
        )

        cm_alpha_ea = (
            inputs[AE.LIFT_CURVE_SLOPE] * inputs[AE.AERO_CENTER_TO_EA_FRACTION]
            + inputs[AE.CONTROL_MOMENT_ALPHA_DERIVATIVE]
        )
        args = (
            rho,
            area,
            chord,
            effective_mass_per_span,
            effective_pitch_inertia,
            inputs[AE.CONTROL_INERTIA_PER_UNIT_SPAN],
            effective_static_unbalance,
            inputs[AE.CONTROL_STATIC_UNBALANCE],
            inputs[AE.BENDING_STIFFNESS_PLUNGE],
            inputs[AE.TORSIONAL_STIFFNESS],
            inputs[AE.CONTROL_STIFFNESS],
            inputs[AE.STRUCTURAL_DAMPING_RATIO],
            inputs[AE.LIFT_CURVE_SLOPE],
            inputs[AE.CONTROL_LIFT_DERIVATIVE],
            cm_alpha_ea,
            inputs[AE.CONTROL_MOMENT_DERIVATIVE],
            inputs[AE.HINGE_MOMENT_ALPHA_DERIVATIVE],
            inputs[AE.HINGE_MOMENT_CONTROL_DERIVATIVE],
        )

        # Design-point eigenvalue is a smooth function → CS derivative is exact.
        max_real_design = max_real_eigenvalue(design_speed, *args)

        # Bisection operates in real arithmetic: np.real() extracts the stability
        # threshold while preserving complex arithmetic elsewhere for CS.
        speeds = np.linspace(0.0, flutter_max_speed, self.options['num_speed_samples'])
        eigs = np.array([np.real(max_real_eigenvalue(speed, *args)) for speed in speeds])

        unstable = np.flatnonzero(eigs > 0.0)
        if len(unstable) == 0:
            flutter_speed = flutter_max_speed
        else:
            hi_idx = unstable[0]
            lo = speeds[max(hi_idx - 1, 0)]
            hi = speeds[hi_idx]
            for _ in range(self.options['bisection_iterations']):
                mid = 0.5 * (lo + hi)
                if np.real(max_real_eigenvalue(mid, *args)) > 0.0:
                    hi = mid
                else:
                    lo = mid
            flutter_speed = 0.5 * (lo + hi)

        outputs[AE.FLUTTER_SPEED] = flutter_speed
        outputs[AE.FLUTTER_DYNAMIC_PRESSURE] = 0.5 * rho * flutter_speed**2
        outputs[AE.FLUTTER_MARGIN] = flutter_speed / design_speed - 1.0
        outputs[AE.FLUTTER_SPEED_MARGIN] = flutter_speed - required_speed
        outputs[AE.MAX_REAL_EIGENVALUE_AT_DESIGN] = max_real_design
