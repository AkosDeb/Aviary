import numpy as np
from scipy.special import hankel2
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft


def _as_float(value):
    """Return an OpenMDAO scalar input as a plain float for real-valued P-K loops."""

    return float(np.asarray(value).item())


def equivalent_chord(area, span, taper):
    root_chord = 2.0 * area / (span * (1.0 + taper))
    return (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / (1.0 + taper)


def build_structural_matrices(
    mass_per_span,
    pitch_inertia,
    control_inertia,
    pitch_static_unbalance,
    control_static_unbalance,
    k_h,
    k_alpha,
    k_delta,
    zeta,
):
    """Build M, C, K for plunge, pitch, and full-span control rotation."""

    mass_per_span = max(mass_per_span, 1.0e-12)
    control_inertia = max(control_inertia, 1.0e-12)
    pitch_inertia = max(pitch_inertia, 1.0e-12)

    mass_matrix = np.array(
        [
            [mass_per_span, pitch_static_unbalance, control_static_unbalance],
            [pitch_static_unbalance, pitch_inertia + control_inertia, control_inertia],
            [control_static_unbalance, control_inertia, control_inertia],
        ]
    )
    stiffness_matrix = np.diag([k_h, k_alpha, k_delta])
    damping_matrix = np.diag(
        [
            2.0 * zeta * np.sqrt(mass_per_span * k_h),
            2.0 * zeta * np.sqrt((pitch_inertia + control_inertia) * k_alpha),
            2.0 * zeta * np.sqrt(control_inertia * k_delta),
        ]
    )
    return mass_matrix, damping_matrix, stiffness_matrix


def placeholder_pk_aero_matrices(
    reduced_frequency,
    wing_area,
    chord,
    cl_alpha,
    cl_delta,
    cm_alpha,
    cm_delta,
    ch_alpha,
    ch_delta,
):
    """Return placeholder reduced-frequency generalized aero matrices.

    This is the adapter point for future DLM/AVL/strip-theory generalized
    aerodynamic matrices.  For now we preserve the current quasi-steady
    derivative matrix and add a simple lag with reduced frequency:

        lag = 1 / (1 + k^2)
        phase = k / (1 + k^2)

    The real part acts like aerodynamic stiffness.  The imaginary part acts like
    aerodynamic damping in the P-K state matrix after scaling by dynamic pressure
    and reduced frequency.  This is not a substitute for real unsteady aero; it
    is a numerically stable P-K workflow placeholder.
    """

    k = max(float(reduced_frequency), 0.0)
    base = np.array(
        [
            [0.0, wing_area * cl_alpha, wing_area * cl_delta],
            [0.0, wing_area * chord * cm_alpha, wing_area * chord * cm_delta],
            [0.0, wing_area * chord * ch_alpha, wing_area * chord * ch_delta],
        ]
    )
    lag = 1.0 / (1.0 + k**2)
    phase = k / (1.0 + k**2)
    q_real = lag * base
    q_imag = phase * base
    return q_real, q_imag


def theodorsen_C(k):
    """Theodorsen circulation function C(k) = F(k) + iG(k).

    Quasi-steady limit k→0: C = 1.  High-frequency limit k→∞: C → 0.5.
    """
    if k < 1.0e-8:
        return complex(1.0, 0.0)
    h0 = hankel2(0, k)
    h1 = hankel2(1, k)
    return h1 / (h0 + h1)


def theodorsen_strip_gaf(
    reduced_frequency,
    wing_area,
    chord,
    cl_alpha,
    cl_delta,
    cm_alpha_ea,
    cm_delta,
    ch_alpha,
    ch_delta,
    elastic_axis_fraction,
):
    """Theodorsen thin-airfoil strip-theory 3×3 GAF matrix at reduced frequency k.

    Circulatory (C(k)-corrected) unsteady aerodynamics for [h, α]; quasi-steady
    for control surface δ (Theodorsen-Garrick T-functions required for full
    treatment — use this as the swap point for DLM/AVL matrices).

    At k=0 the real part equals the quasi-steady aero matrix and the imaginary
    part is zero, so this is a compatible drop-in for placeholder_pk_aero_matrices.

    Parameters
    ----------
    reduced_frequency : float
        k = ω·b/V where b = c/2.  Pass the current P-K iteration k.
    wing_area, chord : float
        Reference wing area [m²] and mean aerodynamic chord [m].
    cl_alpha, cl_delta : float  [1/rad]
    cm_alpha_ea : float  [1/rad]  Cm/alpha about the elastic axis.
    cm_delta, ch_alpha, ch_delta : float  [1/rad]
    elastic_axis_fraction : float
        Elastic axis as fraction of chord from LE (e.g. 0.375).

    Returns
    -------
    q_real, q_imag : (3, 3) ndarray
        Rows: [lift L, pitch moment about EA, hinge moment].
        Columns: [plunge h, pitch α, control δ].
    """
    k = max(float(reduced_frequency), 0.0)
    b = max(0.5 * chord, 1.0e-12)
    S = wing_area
    # a = 0 at midchord, a = -1 at LE, a = +1 at TE
    a = 2.0 * float(elastic_axis_fraction) - 1.0

    C = theodorsen_C(k)
    F_th = C.real
    G_th = C.imag

    # Quasi-steady derivative vectors [lift; pitch moment; hinge moment] per unit q_dyn:
    d_alpha = np.array([
        S * cl_alpha,
        S * chord * cm_alpha_ea,
        S * chord * ch_alpha,
    ])
    d_delta = np.array([
        S * cl_delta,
        S * chord * cm_delta,
        S * chord * ch_delta,
    ])

    # --- Column 0: plunge h ---
    # Harmonic plunge → apparent AoA = iω·h/V = ik·h/b.
    # C(k)·(ik/b) = (F+iG)(ik/b) = (−Gk/b) + i(Fk/b)
    Q_R_h = (-G_th * k / b) * d_alpha
    Q_I_h = ( F_th * k / b) * d_alpha

    # --- Column 1: pitch α ---
    # 3/4-chord apparent velocity = V·α·[1 + ik·(1/2−a)]  (η = 1/2 − a)
    # C(k)·[1 + ik·η] = [F − G·k·η] + i[G + F·k·η]
    eta = 0.5 - a
    Q_R_alpha = (F_th - G_th * k * eta) * d_alpha
    Q_I_alpha = (G_th + F_th * k * eta) * d_alpha

    # --- Column 2: control δ ---
    # Quasi-steady; Theodorsen-Garrick T-functions needed for full treatment.
    Q_R_delta = d_delta.copy()
    Q_I_delta = np.zeros(3)

    q_real = np.column_stack([Q_R_h, Q_R_alpha, Q_R_delta])
    q_imag = np.column_stack([Q_I_h, Q_I_alpha, Q_I_delta])
    return q_real, q_imag


def state_matrix_for_pk(
    speed,
    rho,
    chord,
    mass_matrix,
    damping_matrix,
    stiffness_matrix,
    q_real,
    q_imag,
    k=1.0,
):
    """Build the real state matrix for one P-K iteration.

    Parameters
    ----------
    k : float
        Current reduced-frequency estimate for this mode.
        Used to convert Q_imag to aerodynamic damping:
          C_aero = q_dyn · Q_imag / ω,  ω = k·V/b.
        Default 1.0 gives ω = V/b, matching the original placeholder behaviour.
    """
    q_dyn = 0.5 * rho * speed**2
    b = max(0.5 * chord, 1.0e-12)
    omega = max(1.0e-6, abs(k) * max(speed, 1.0e-6) / b)
    inv_mass = np.linalg.inv(mass_matrix)
    effective_stiffness = stiffness_matrix - q_dyn * q_real
    aero_damping = q_dyn * q_imag / omega
    effective_damping = damping_matrix + aero_damping

    top = np.hstack((np.zeros((3, 3)), np.eye(3)))
    bottom = np.hstack((-inv_mass @ effective_stiffness, -inv_mass @ effective_damping))
    return np.vstack((top, bottom))


def modal_frequency_and_damping(eigval):
    omega = abs(eigval.imag)
    denom = np.sqrt(eigval.real**2 + eigval.imag**2)
    damping_ratio = -eigval.real / denom if denom > 0.0 else 1.0
    return omega, damping_ratio


def pk_modes_at_speed(
    speed,
    rho,
    chord,
    mass_matrix,
    damping_matrix,
    stiffness_matrix,
    aero_args,
    num_modes=3,
    max_iterations=30,
    tolerance=1.0e-4,
    gaf_function=None,
):
    """Iterate reduced frequency for each structural mode at one airspeed.

    Parameters
    ----------
    aero_args : tuple
        Arguments forwarded to gaf_function after the reduced-frequency k.
    gaf_function : callable or None
        f(k, *aero_args) → (q_real, q_imag).
        Defaults to placeholder_pk_aero_matrices when None.
    """
    if gaf_function is None:
        gaf_function = placeholder_pk_aero_matrices

    # Start from dry structural frequencies so each mode has a sensible k seed.
    dry_a = state_matrix_for_pk(
        max(speed, 1.0e-6),
        rho,
        chord,
        mass_matrix,
        damping_matrix,
        stiffness_matrix,
        np.zeros((3, 3)),
        np.zeros((3, 3)),
    )
    dry_eigs = np.linalg.eigvals(dry_a)
    dry_positive = sorted(
        [eig for eig in dry_eigs if eig.imag >= 0.0],
        key=lambda eig: abs(eig.imag),
    )[:num_modes]

    frequencies = np.zeros(num_modes)
    damping = np.ones(num_modes)
    converged = np.ones(num_modes)

    for mode_idx, seed_eig in enumerate(dry_positive):
        omega = max(abs(seed_eig.imag), 1.0e-6)
        k = omega * chord / (2.0 * max(speed, 1.0e-6))
        mode_converged = False

        for _ in range(max_iterations):
            q_real, q_imag = gaf_function(k, *aero_args)
            system = state_matrix_for_pk(
                max(speed, 1.0e-6),
                rho,
                chord,
                mass_matrix,
                damping_matrix,
                stiffness_matrix,
                q_real,
                q_imag,
                k=k,
            )
            eigs = [candidate for candidate in np.linalg.eigvals(system) if candidate.imag >= 0.0]
            eig = min(eigs, key=lambda candidate: abs(abs(candidate.imag) - omega))
            omega_new, damping_ratio = modal_frequency_and_damping(eig)
            k_new = omega_new * chord / (2.0 * max(speed, 1.0e-6))

            if abs(k_new - k) <= tolerance * max(1.0, abs(k)):
                mode_converged = True
                omega = omega_new
                damping[mode_idx] = damping_ratio
                break

            omega = max(omega_new, 1.0e-6)
            k = k_new
            damping[mode_idx] = damping_ratio

        frequencies[mode_idx] = omega / (2.0 * np.pi)
        converged[mode_idx] = 1.0 if mode_converged else 0.0

    return frequencies, damping, converged


class PKFlutterAnalysis(om.ExplicitComponent):
    """Reduced-frequency P-K flutter analysis with Theodorsen strip-theory GAF."""

    def initialize(self):
        self.options.declare('num_speed_samples', default=50, types=int)
        self.options.declare('num_modes', default=3, types=int)
        self.options.declare('pk_iterations', default=30, types=int)
        self.options.declare('pk_tolerance', default=1.0e-4, types=float)

    def setup(self):
        nm = self.options['num_modes']

        self.add_input(Aircraft.Wing.AREA, val=1.0, units='m**2')
        self.add_input(Aircraft.Wing.SPAN, val=2.0, units='m')
        self.add_input(Aircraft.Wing.TAPER_RATIO, val=1.0)
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')
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
        self.add_input(AE.ELASTIC_AXIS_FRACTION, val=0.375)
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125, units='unitless')
        self.add_input(AE.CONTROL_MOMENT_ALPHA_DERIVATIVE, val=-0.05, units='1/rad')
        self.add_input(AE.CONTROL_MOMENT_DERIVATIVE, val=-0.60, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_ALPHA_DERIVATIVE, val=-0.02, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_CONTROL_DERIVATIVE, val=-0.30, units='1/rad')

        self.add_output(AE.PK_FLUTTER_SPEED, val=350.0, units='m/s')
        self.add_output(AE.PK_FLUTTER_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.PK_FLUTTER_DYNAMIC_PRESSURE, val=1.0e5, units='Pa')
        self.add_output(AE.PK_FLUTTER_SPEED_MARGIN, val=100.0, units='m/s')
        self.add_output(AE.PK_CRITICAL_MODE, val=0.0)
        self.add_output(AE.PK_CONVERGED, val=1.0)
        self.add_output(AE.PK_MODE_DAMPING, val=np.zeros(nm))
        self.add_output(AE.PK_MODE_FREQUENCY, val=np.zeros(nm), units='Hz')

        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        nm = self.options['num_modes']
        area = _as_float(inputs[Aircraft.Wing.AREA])
        span = _as_float(inputs[Aircraft.Wing.SPAN])
        taper = _as_float(inputs[Aircraft.Wing.TAPER_RATIO])
        rho = _as_float(inputs[AE.AIR_DENSITY])
        required_speed = _as_float(inputs[AE.REQUIRED_SPEED])
        flutter_max_speed = _as_float(inputs[AE.FLUTTER_MAX_SPEED])
        chord = equivalent_chord(area, span, taper)

        mass_per_span = (
            _as_float(inputs[AE.MASS_PER_UNIT_SPAN])
            + _as_float(inputs[AE.VTP_EQUIVALENT_MASS_PER_UNIT_SPAN])
        )
        pitch_inertia = (
            _as_float(inputs[AE.PITCH_INERTIA_PER_UNIT_SPAN])
            + _as_float(inputs[AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN])
        )
        pitch_static_unbalance = (
            _as_float(inputs[AE.PITCH_STATIC_UNBALANCE])
            + _as_float(inputs[AE.VTP_PITCH_STATIC_UNBALANCE])
        )
        cm_alpha_ea = (
            _as_float(inputs[AE.LIFT_CURVE_SLOPE])
            * _as_float(inputs[AE.AERO_CENTER_TO_EA_FRACTION])
            + _as_float(inputs[AE.CONTROL_MOMENT_ALPHA_DERIVATIVE])
        )
        ea_frac = _as_float(inputs[AE.ELASTIC_AXIS_FRACTION])

        mass_matrix, damping_matrix, stiffness_matrix = build_structural_matrices(
            mass_per_span,
            pitch_inertia,
            _as_float(inputs[AE.CONTROL_INERTIA_PER_UNIT_SPAN]),
            pitch_static_unbalance,
            _as_float(inputs[AE.CONTROL_STATIC_UNBALANCE]),
            _as_float(inputs[AE.BENDING_STIFFNESS_PLUNGE]),
            _as_float(inputs[AE.TORSIONAL_STIFFNESS]),
            _as_float(inputs[AE.CONTROL_STIFFNESS]),
            _as_float(inputs[AE.STRUCTURAL_DAMPING_RATIO]),
        )
        aero_args = (
            area,
            chord,
            _as_float(inputs[AE.LIFT_CURVE_SLOPE]),
            _as_float(inputs[AE.CONTROL_LIFT_DERIVATIVE]),
            cm_alpha_ea,
            _as_float(inputs[AE.CONTROL_MOMENT_DERIVATIVE]),
            _as_float(inputs[AE.HINGE_MOMENT_ALPHA_DERIVATIVE]),
            _as_float(inputs[AE.HINGE_MOMENT_CONTROL_DERIVATIVE]),
            ea_frac,
        )

        speeds = np.linspace(1.0, flutter_max_speed, self.options['num_speed_samples'])
        damping_history = []
        frequency_history = []
        convergence_history = []
        for speed in speeds:
            freq, damp, conv = pk_modes_at_speed(
                speed,
                rho,
                chord,
                mass_matrix,
                damping_matrix,
                stiffness_matrix,
                aero_args,
                gaf_function=theodorsen_strip_gaf,
                num_modes=nm,
                max_iterations=self.options['pk_iterations'],
                tolerance=self.options['pk_tolerance'],
            )
            frequency_history.append(freq)
            damping_history.append(damp)
            convergence_history.append(conv)

        damping_history = np.asarray(damping_history)
        frequency_history = np.asarray(frequency_history)
        convergence_history = np.asarray(convergence_history)

        flutter_speed = flutter_max_speed
        flutter_frequency = frequency_history[-1, 0]
        critical_mode = 0
        found = False
        for mode_idx in range(nm):
            mode_damping = damping_history[:, mode_idx]
            crossing = np.flatnonzero(mode_damping <= 0.0)
            if len(crossing) == 0:
                continue

            hi_idx = crossing[0]
            if hi_idx == 0:
                candidate_speed = speeds[0]
                candidate_freq = frequency_history[0, mode_idx]
            else:
                lo_idx = hi_idx - 1
                d0 = mode_damping[lo_idx]
                d1 = mode_damping[hi_idx]
                frac = d0 / (d0 - d1) if abs(d0 - d1) > 1.0e-12 else 0.0
                candidate_speed = speeds[lo_idx] + frac * (speeds[hi_idx] - speeds[lo_idx])
                candidate_freq = frequency_history[lo_idx, mode_idx] + frac * (
                    frequency_history[hi_idx, mode_idx] - frequency_history[lo_idx, mode_idx]
                )

            if (not found) or candidate_speed < flutter_speed:
                flutter_speed = candidate_speed
                flutter_frequency = candidate_freq
                critical_mode = mode_idx
                found = True

        outputs[AE.PK_FLUTTER_SPEED] = flutter_speed
        outputs[AE.PK_FLUTTER_FREQUENCY] = flutter_frequency
        outputs[AE.PK_FLUTTER_DYNAMIC_PRESSURE] = 0.5 * rho * flutter_speed**2
        outputs[AE.PK_FLUTTER_SPEED_MARGIN] = flutter_speed - required_speed
        outputs[AE.PK_CRITICAL_MODE] = float(critical_mode)
        outputs[AE.PK_CONVERGED] = float(np.all(convergence_history[-1, :] > 0.5))
        outputs[AE.PK_MODE_DAMPING] = damping_history[-1, :]
        outputs[AE.PK_MODE_FREQUENCY] = frequency_history[-1, :]
