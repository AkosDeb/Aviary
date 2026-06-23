import numpy as np
import openmdao.api as om
from scipy.linalg import eigh

from aviary.models.external_subsystems.aeroelasticity.model.pk_flutter import (
    theodorsen_C,
)
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE


def _trapz(y, value):
    return np.sum(0.5 * (value[:-1] + value[1:]) * (y[1:] - y[:-1]))


def _gradient(y, value):
    return np.gradient(value, y, edge_order=2)


def _safe_first_mode(k_matrix, m_matrix):
    eigvals, eigvecs = eigh(k_matrix, m_matrix)
    eigvals = np.maximum(eigvals, 1.0e-12)
    idx = int(np.argmin(eigvals))
    return eigvals[idx], eigvecs[:, idx]


def _bending_mode_fe(y, EI, mass):
    """Return first Euler-Bernoulli bending mode from a clamped-root beam FE model."""
    n = len(y)
    ndof = 2 * n
    k_global = np.zeros((ndof, ndof))
    m_global = np.zeros((ndof, ndof))

    for idx in range(n - 1):
        length = max(y[idx + 1] - y[idx], 1.0e-12)
        ei = 0.5 * (EI[idx] + EI[idx + 1])
        m = 0.5 * (mass[idx] + mass[idx + 1])
        le = length

        k_ele = ei / le**3 * np.array(
            [
                [12.0, 6.0 * le, -12.0, 6.0 * le],
                [6.0 * le, 4.0 * le**2, -6.0 * le, 2.0 * le**2],
                [-12.0, -6.0 * le, 12.0, -6.0 * le],
                [6.0 * le, 2.0 * le**2, -6.0 * le, 4.0 * le**2],
            ]
        )
        m_ele = m * le / 420.0 * np.array(
            [
                [156.0, 22.0 * le, 54.0, -13.0 * le],
                [22.0 * le, 4.0 * le**2, 13.0 * le, -3.0 * le**2],
                [54.0, 13.0 * le, 156.0, -22.0 * le],
                [-13.0 * le, -3.0 * le**2, -22.0 * le, 4.0 * le**2],
            ]
        )
        dofs = np.array([2 * idx, 2 * idx + 1, 2 * idx + 2, 2 * idx + 3])
        k_global[np.ix_(dofs, dofs)] += k_ele
        m_global[np.ix_(dofs, dofs)] += m_ele

    free = np.arange(2, ndof)
    eigval, mode_free = _safe_first_mode(
        k_global[np.ix_(free, free)],
        m_global[np.ix_(free, free)] + np.eye(len(free)) * 1.0e-12,
    )
    mode = np.zeros(ndof)
    mode[free] = mode_free
    bending = mode[0::2]
    tip = bending[-1] if abs(bending[-1]) > 1.0e-12 else np.max(np.abs(bending))
    bending = bending / tip
    return eigval, bending


def _torsion_mode_fe(y, GJ, pitch_inertia):
    """Return first torsion mode from a clamped-root torsion-rod FE model."""
    n = len(y)
    k_global = np.zeros((n, n))
    m_global = np.zeros((n, n))

    for idx in range(n - 1):
        length = max(y[idx + 1] - y[idx], 1.0e-12)
        gj = 0.5 * (GJ[idx] + GJ[idx + 1])
        ip = 0.5 * (pitch_inertia[idx] + pitch_inertia[idx + 1])
        k_ele = gj / length * np.array([[1.0, -1.0], [-1.0, 1.0]])
        m_ele = ip * length / 6.0 * np.array([[2.0, 1.0], [1.0, 2.0]])
        dofs = np.array([idx, idx + 1])
        k_global[np.ix_(dofs, dofs)] += k_ele
        m_global[np.ix_(dofs, dofs)] += m_ele

    free = np.arange(1, n)
    eigval, mode_free = _safe_first_mode(
        k_global[np.ix_(free, free)],
        m_global[np.ix_(free, free)] + np.eye(len(free)) * 1.0e-12,
    )
    torsion = np.zeros(n)
    torsion[free] = mode_free
    tip = torsion[-1] if abs(torsion[-1]) > 1.0e-12 else np.max(np.abs(torsion))
    torsion = torsion / tip
    return eigval, torsion


def _garrick_T10_T11(hinge_fraction_from_LE):
    """Theodorsen-Garrick T10 and T11 for the unsteady control-surface δ column.

    T10 and T11 determine the effective 3/4-chord downwash velocity due to
    oscillating control deflection. The circulatory lift scales as C(k)·(T10/π + ik·T11/(2π)),
    which at k=0 recovers the quasi-steady cl_delta and at k>0 applies the same
    F−Gkη / G+Fkη phase correction as the pitch column, with η_δ = T11/(2·T10).

    c = 2·hinge_fraction − 1  maps LE→−1, midchord→0, TE→+1.
    T10 = √(1−c²) + arccos(c)
    T11 = ½(1−2c)·arccos(c) + √(1−c²)
    """
    c = float(np.clip(2.0 * hinge_fraction_from_LE - 1.0, -1.0 + 1.0e-9, 1.0 - 1.0e-9))
    sc = np.sqrt(max(1.0 - c**2, 0.0))
    ac = np.arccos(c)
    T10 = sc + ac
    T11 = 0.5 * (1.0 - 2.0 * c) * ac + sc
    return T10, T11


def _control_mode_shape(y, span_start_frac, span_end_frac):
    """Rigid control surface rotation mode: 1 over elevon span, 0 elsewhere."""
    semispan = max(y[-1], 1.0e-12)
    y_start = float(span_start_frac) * semispan
    y_end = float(span_end_frac) * semispan
    return np.where((y >= y_start) & (y <= y_end), 1.0, 0.0)


def _modal_matrices_3dof(
    y, EI, GJ, mass, pitch_inertia,
    phi_delta, control_inertia_per_span, control_static_unbalance_per_span,
    k_hinge,
):
    """3×3 modal mass and stiffness for [bending, torsion, control rotation]."""
    _, bending = _bending_mode_fe(y, EI, mass)
    _, torsion = _torsion_mode_fe(y, GJ, pitch_inertia)
    bending_curvature = _gradient(y, _gradient(y, bending))
    torsion_rate = _gradient(y, torsion)

    M_hh = max(_trapz(y, mass * bending**2), 1.0e-12)
    M_aa = max(_trapz(y, pitch_inertia * torsion**2), 1.0e-12)
    M_dd = max(_trapz(y, control_inertia_per_span * phi_delta**2), 1.0e-12)
    M_hd = _trapz(y, control_static_unbalance_per_span * bending * phi_delta)
    M_ad = _trapz(y, control_inertia_per_span * torsion * phi_delta)

    modal_mass = np.array([
        [M_hh, 0.0,  M_hd],
        [0.0,  M_aa, M_ad],
        [M_hd, M_ad, M_dd],
    ])
    K_hh = max(_trapz(y, EI * bending_curvature**2), 1.0e-12)
    K_aa = max(_trapz(y, GJ * torsion_rate**2), 1.0e-12)
    K_dd = max(float(k_hinge), 1.0e-12)

    modal_stiffness = np.diag([K_hh, K_aa, K_dd])
    return modal_mass, modal_stiffness, bending, torsion


def beam_modal_theodorsen_gaf_3dof(
    reduced_frequency,
    y,
    chord,
    bending,
    torsion,
    phi_delta,
    cl_alpha,
    cm_alpha_ea,
    cl_delta,
    cm_delta,
    ch_alpha,
    ch_delta,
    elastic_axis_fraction,
    control_hinge_fraction,
):
    """3×3 generalized aero matrix for [bending, torsion, control] modes.

    All three columns use Theodorsen C(k)-based circulatory unsteady aerodynamics.
    The [h, α] block follows the standard Theodorsen formulation. The δ column
    applies the Theodorsen-Garrick T10/T11 correction: the effective 3/4-chord
    downwash due to oscillating δ is proportional to (T10/π + ik·T11/(2π))·δ,
    giving a pitch-rate arm η_δ = T11/(2·T10) that depends only on hinge position.
    At k=0 this recovers the quasi-steady cl_delta exactly.

    Non-circulatory apparent-mass terms (T3, T4, T5) are deferred; they are O(k²)
    and small for the moderate reduced frequencies typical of this UAV.
    """
    k = max(float(reduced_frequency), 0.0)
    ref_chord = max(_trapz(y, chord) / max(y[-1] - y[0], 1.0e-12), 1.0e-12)
    b_ref = 0.5 * ref_chord
    axis_a = 2.0 * float(elastic_axis_fraction) - 1.0
    eta = 0.5 - axis_a

    C = theodorsen_C(k)
    F, G = C.real, C.imag

    h_real = -G * k / b_ref
    h_imag =  F * k / b_ref
    a_real = F - G * k * eta
    a_imag = G + F * k * eta

    # Garrick rate arm for δ: η_δ = T11 / (2·T10), analogous to η = 0.5−a for α
    T10, T11 = _garrick_T10_T11(control_hinge_fraction)
    eta_delta = T11 / (2.0 * max(T10, 1.0e-12))
    d_real = F - G * k * eta_delta
    d_imag = G + F * k * eta_delta

    # Weighted shape-function integrals
    def I(a, b):
        return _trapz(y, a * b)

    # Rows: [bending gen. force, torsion gen. moment, control gen. hinge moment]
    # Columns: [h (plunge), α (torsion), δ (control)]

    # Column 0 — plunge h (Theodorsen)
    Q_R_h = np.array([
        h_real * I(bending * chord * cl_alpha,    bending),
        h_real * I(torsion * chord**2 * cm_alpha_ea, bending),
        h_real * I(phi_delta * chord**2 * ch_alpha,  bending),
    ])
    Q_I_h = np.array([
        h_imag * I(bending * chord * cl_alpha,    bending),
        h_imag * I(torsion * chord**2 * cm_alpha_ea, bending),
        h_imag * I(phi_delta * chord**2 * ch_alpha,  bending),
    ])

    # Column 1 — pitch α (Theodorsen)
    Q_R_a = np.array([
        a_real * I(bending * chord * cl_alpha,    torsion),
        a_real * I(torsion * chord**2 * cm_alpha_ea, torsion),
        a_real * I(phi_delta * chord**2 * ch_alpha,  torsion),
    ])
    Q_I_a = np.array([
        a_imag * I(bending * chord * cl_alpha,    torsion),
        a_imag * I(torsion * chord**2 * cm_alpha_ea, torsion),
        a_imag * I(phi_delta * chord**2 * ch_alpha,  torsion),
    ])

    # Column 2 — control δ (Theodorsen-Garrick circulatory)
    Q_R_d = np.array([
        d_real * I(bending * chord * cl_delta,       phi_delta),
        d_real * I(torsion * chord**2 * cm_delta,    phi_delta),
        d_real * I(phi_delta * chord**2 * ch_delta,  phi_delta),
    ])
    Q_I_d = np.array([
        d_imag * I(bending * chord * cl_delta,       phi_delta),
        d_imag * I(torsion * chord**2 * cm_delta,    phi_delta),
        d_imag * I(phi_delta * chord**2 * ch_delta,  phi_delta),
    ])

    q_real = np.column_stack([Q_R_h, Q_R_a, Q_R_d])
    q_imag = np.column_stack([Q_I_h, Q_I_a, Q_I_d])
    return q_real, q_imag, ref_chord


def _modal_matrices(y, EI, GJ, mass, pitch_inertia):
    _, bending = _bending_mode_fe(y, EI, mass)
    _, torsion = _torsion_mode_fe(y, GJ, pitch_inertia)
    bending_curvature = _gradient(y, _gradient(y, bending))
    torsion_rate = _gradient(y, torsion)

    modal_mass = np.array([
        [_trapz(y, mass * bending**2), 0.0],
        [0.0, _trapz(y, pitch_inertia * torsion**2)],
    ])
    modal_stiffness = np.array([
        [_trapz(y, EI * bending_curvature**2), 0.0],
        [0.0, _trapz(y, GJ * torsion_rate**2)],
    ])
    return modal_mass, modal_stiffness, bending, torsion


def beam_modal_state_matrix(
    speed,
    rho,
    y,
    chord,
    EI,
    GJ,
    mass,
    pitch_inertia,
    structural_damping,
    cl_alpha,
    cm_alpha_ea,
):
    """Return a 2-mode bending/torsion quasi-steady aeroelastic state matrix."""
    modal_mass, modal_stiffness, bending, torsion = _modal_matrices(
        y,
        EI,
        GJ,
        mass,
        pitch_inertia,
    )

    modal_mass[0, 0] = max(modal_mass[0, 0], 1.0e-12)
    modal_mass[1, 1] = max(modal_mass[1, 1], 1.0e-12)
    modal_stiffness[0, 0] = max(modal_stiffness[0, 0], 1.0e-12)
    modal_stiffness[1, 1] = max(modal_stiffness[1, 1], 1.0e-12)

    omega = np.sqrt(np.diag(modal_stiffness) / np.diag(modal_mass))
    structural_damping_matrix = np.diag(
        2.0 * structural_damping * np.diag(modal_mass) * omega
    )

    q_dyn = 0.5 * rho * speed**2
    speed_safe = max(speed, 1.0e-6)

    # Lift from local twist and plunge rate:
    #   alpha_eff = theta_mode * psi_t(y) + h_dot * psi_b(y) / V
    # Generalized bending force uses the bending displacement shape; generalized
    # torsion moment uses the torsion shape and EA-referenced moment slope.
    aero_stiffness = q_dyn * np.array(
        [
            [0.0, _trapz(y, bending * chord * cl_alpha * torsion)],
            [0.0, _trapz(y, torsion * chord**2 * cm_alpha_ea * torsion)],
        ]
    )
    aero_damping = q_dyn / speed_safe * np.array(
        [
            [_trapz(y, bending * chord * cl_alpha * bending), 0.0],
            [_trapz(y, torsion * chord**2 * cm_alpha_ea * bending), 0.0],
        ]
    )

    effective_stiffness = modal_stiffness - aero_stiffness
    effective_damping = structural_damping_matrix + aero_damping

    inv_mass = np.linalg.inv(modal_mass)
    top = np.hstack((np.zeros((2, 2)), np.eye(2)))
    bottom = np.hstack((-inv_mass @ effective_stiffness, -inv_mass @ effective_damping))
    return np.vstack((top, bottom)), modal_mass, modal_stiffness


def beam_modal_theodorsen_gaf(
    reduced_frequency,
    y,
    chord,
    bending,
    torsion,
    cl_alpha,
    cm_alpha_ea,
    elastic_axis_fraction,
):
    """Return 2-mode Theodorsen generalized aero matrices.

    Rows and columns are [bending, torsion]. The real matrix acts as
    aerodynamic stiffness. The imaginary matrix is converted to aerodynamic
    damping in the P-K state matrix with q_dyn * Q_imag / omega.
    """
    k = max(float(reduced_frequency), 0.0)
    ref_chord = max(_trapz(y, chord) / max(y[-1] - y[0], 1.0e-12), 1.0e-12)
    b_ref = 0.5 * ref_chord
    axis_a = 2.0 * elastic_axis_fraction - 1.0
    pitch_rate_arm = 0.5 - axis_a

    c_theodorsen = theodorsen_C(k)
    f_th = c_theodorsen.real
    g_th = c_theodorsen.imag

    lift_bending = bending * chord * cl_alpha
    moment_bending = torsion * chord**2 * cm_alpha_ea
    lift_torsion = bending * chord * cl_alpha
    moment_torsion = torsion * chord**2 * cm_alpha_ea

    h_real_scale = -g_th * k / b_ref
    h_imag_scale = f_th * k / b_ref
    theta_real_scale = f_th - g_th * k * pitch_rate_arm
    theta_imag_scale = g_th + f_th * k * pitch_rate_arm

    q_real = np.array(
        [
            [
                h_real_scale * _trapz(y, lift_bending * bending),
                theta_real_scale * _trapz(y, lift_torsion * torsion),
            ],
            [
                h_real_scale * _trapz(y, moment_bending * bending),
                theta_real_scale * _trapz(y, moment_torsion * torsion),
            ],
        ]
    )
    q_imag = np.array(
        [
            [
                h_imag_scale * _trapz(y, lift_bending * bending),
                theta_imag_scale * _trapz(y, lift_torsion * torsion),
            ],
            [
                h_imag_scale * _trapz(y, moment_bending * bending),
                theta_imag_scale * _trapz(y, moment_torsion * torsion),
            ],
        ]
    )
    return q_real, q_imag, ref_chord


def beam_modal_pk_state_matrix(
    speed,
    rho,
    ref_chord,
    modal_mass,
    modal_stiffness,
    structural_damping_matrix,
    q_real,
    q_imag,
    reduced_frequency,
):
    """Build a real 2-mode P-K state matrix for one reduced-frequency iterate."""
    q_dyn = 0.5 * rho * speed**2
    b_ref = max(0.5 * ref_chord, 1.0e-12)
    omega = max(abs(reduced_frequency) * max(speed, 1.0e-6) / b_ref, 1.0e-8)
    effective_stiffness = modal_stiffness - q_dyn * q_real
    effective_damping = structural_damping_matrix + q_dyn * q_imag / omega

    inv_mass = np.linalg.inv(modal_mass)
    top = np.hstack((np.zeros((2, 2)), np.eye(2)))
    bottom = np.hstack((-inv_mass @ effective_stiffness, -inv_mass @ effective_damping))
    return np.vstack((top, bottom))


def _modal_frequency_and_damping(eigval):
    omega = abs(eigval.imag)
    denom = np.sqrt(eigval.real**2 + eigval.imag**2)
    damping_ratio = -eigval.real / denom if denom > 0.0 else 1.0
    return omega / (2.0 * np.pi), damping_ratio


def beam_modal_pk_modes_at_speed(
    speed,
    rho,
    y,
    chord,
    modal_mass,
    modal_stiffness,
    structural_damping_matrix,
    bending,
    torsion,
    cl_alpha,
    cm_alpha_ea,
    elastic_axis_fraction,
    max_iterations=30,
    tolerance=1.0e-4,
):
    """Run a two-mode P-K iteration at one speed."""
    dry_omega = np.sqrt(np.diag(modal_stiffness) / np.diag(modal_mass))
    ref_chord = max(_trapz(y, chord) / max(y[-1] - y[0], 1.0e-12), 1.0e-12)
    b_ref = 0.5 * ref_chord
    speed_safe = max(speed, 1.0e-6)
    reduced_frequency = np.maximum(dry_omega * b_ref / speed_safe, 1.0e-6)

    mode_eigs = np.zeros(2, dtype=complex)
    damping = np.zeros(2)
    frequency = np.zeros(2)
    converged = True

    for mode_idx in range(2):
        k = reduced_frequency[mode_idx]
        eigval = complex(-1.0, dry_omega[mode_idx])
        for _ in range(max_iterations):
            q_real, q_imag, ref_chord = beam_modal_theodorsen_gaf(
                k,
                y,
                chord,
                bending,
                torsion,
                cl_alpha,
                cm_alpha_ea,
                elastic_axis_fraction,
            )
            state = beam_modal_pk_state_matrix(
                speed_safe,
                rho,
                ref_chord,
                modal_mass,
                modal_stiffness,
                structural_damping_matrix,
                q_real,
                q_imag,
                k,
            )
            eigvals = np.linalg.eigvals(state)
            positive = eigvals[eigvals.imag >= 0.0]
            if len(positive) == 0:
                positive = eigvals
            target = dry_omega[mode_idx]
            eigval = positive[np.argmin(np.abs(np.abs(positive.imag) - target))]
            new_k = max(abs(eigval.imag) * b_ref / speed_safe, 1.0e-6)
            if abs(new_k - k) < tolerance:
                k = new_k
                break
            k = 0.5 * (k + new_k)
        else:
            converged = False

        mode_eigs[mode_idx] = eigval
        frequency[mode_idx], damping[mode_idx] = _modal_frequency_and_damping(eigval)

    return mode_eigs, damping, frequency, converged


def beam_modal_pk_modes_at_speed_ndof(
    speed,
    rho,
    ref_chord,
    modal_mass,
    modal_stiffness,
    structural_damping_matrix,
    gaf_fn,
    gaf_args,
    max_iterations=30,
    tolerance=1.0e-4,
):
    """Run an N-mode P-K iteration at one speed using a caller-supplied GAF function.

    Parameters
    ----------
    gaf_fn : callable
        f(k, *gaf_args) -> (q_real, q_imag, ref_chord)
    gaf_args : tuple
        Forwarded to gaf_fn after the reduced-frequency k.
    """
    n = modal_mass.shape[0]
    dry_omega = np.sqrt(np.maximum(np.diag(modal_stiffness) / np.diag(modal_mass), 1.0e-12))
    b_ref = 0.5 * ref_chord
    speed_safe = max(speed, 1.0e-6)
    reduced_frequency = np.maximum(dry_omega * b_ref / speed_safe, 1.0e-6)

    damping = np.zeros(n)
    frequency = np.zeros(n)
    converged = True

    for mode_idx in range(n):
        k = reduced_frequency[mode_idx]
        for _ in range(max_iterations):
            q_real, q_imag, _ = gaf_fn(k, *gaf_args)
            q_dyn = 0.5 * rho * speed_safe**2
            omega = max(abs(k) * speed_safe / b_ref, 1.0e-8)
            effective_stiffness = modal_stiffness - q_dyn * q_real
            effective_damping = structural_damping_matrix + q_dyn * q_imag / omega
            inv_m = np.linalg.inv(modal_mass)
            top = np.hstack((np.zeros((n, n)), np.eye(n)))
            bottom = np.hstack((-inv_m @ effective_stiffness, -inv_m @ effective_damping))
            state = np.vstack((top, bottom))
            eigvals = np.linalg.eigvals(state)
            positive = eigvals[eigvals.imag >= 0.0]
            if len(positive) == 0:
                positive = eigvals
            target = dry_omega[mode_idx]
            eigval = positive[np.argmin(np.abs(np.abs(positive.imag) - target))]
            new_k = max(abs(eigval.imag) * b_ref / speed_safe, 1.0e-6)
            if abs(new_k - k) < tolerance:
                k = new_k
                break
            k = 0.5 * (k + new_k)
        else:
            converged = False

        omega_hz = abs(eigval.imag) / (2.0 * np.pi)
        denom = np.sqrt(eigval.real**2 + eigval.imag**2)
        damp = -eigval.real / denom if denom > 0.0 else 1.0
        frequency[mode_idx] = omega_hz
        damping[mode_idx] = damp

    return damping, frequency, converged


class BeamModalFlutter(om.ExplicitComponent):
    """Two-mode spanwise beam-modal aeroelastic flutter screen.

    This is the first Step-4 model. It computes one finite-element bending mode
    and one finite-element torsion mode from the Step-3 spanwise beam arrays,
    then solves a small aeroelastic state-space system over a speed sweep. The
    aerodynamic model is quasi-steady strip theory; this is the bridge toward a
    later P-K/modal GAF implementation.
    """

    def initialize(self):
        self.options.declare('num_stations', default=21, types=int)
        self.options.declare('num_speed_samples', default=80, types=int)
        self.options.declare('bisection_iterations', default=32, types=int)
        self.options.declare('pk_iterations', default=30, types=int)
        self.options.declare('pk_tolerance', default=1.0e-4, types=float)

    def setup(self):
        n = self.options['num_stations']

        self.add_input(AE.SPANWISE_STATIONS, val=np.linspace(0.0, 1.0, n), units='m')
        self.add_input(AE.SPANWISE_CHORD, val=np.ones(n), units='m')
        self.add_input(AE.SPANWISE_BENDING_STIFFNESS, val=np.ones(n), units='N*m**2')
        self.add_input(AE.SPANWISE_TORSIONAL_RIGIDITY, val=np.ones(n), units='N*m**2')
        self.add_input(AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN, val=np.ones(n), units='kg/m')
        self.add_input(
            AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN,
            val=np.ones(n),
            units='kg*m',
        )
        self.add_input(AE.AIR_DENSITY, val=1.225, units='kg/m**3')
        self.add_input(AE.DESIGN_SPEED, val=150.0, units='m/s')
        self.add_input(AE.REQUIRED_SPEED, val=1.15 * 550.0 / 3.6, units='m/s')
        self.add_input(AE.FLUTTER_MAX_SPEED, val=350.0, units='m/s')
        self.add_input(AE.STRUCTURAL_DAMPING_RATIO, val=0.02)
        self.add_input(AE.LIFT_CURVE_SLOPE, val=2.0 * np.pi, units='unitless')
        self.add_input(AE.AERO_CENTER_TO_EA_FRACTION, val=0.125, units='unitless')
        self.add_input(AE.CONTROL_MOMENT_ALPHA_DERIVATIVE, val=-0.05, units='1/rad')
        self.add_input(AE.ELEVON_SPAN_START_FRACTION, val=0.45, units='unitless')
        self.add_input(AE.ELEVON_SPAN_END_FRACTION, val=0.95, units='unitless')
        self.add_input(AE.CONTROL_INERTIA_PER_UNIT_SPAN, val=0.001, units='kg*m')
        self.add_input(AE.CONTROL_STATIC_UNBALANCE, val=0.0, units='kg')
        self.add_input(AE.CONTROL_STIFFNESS, val=5.0e3, units='N*m/rad')
        self.add_input(AE.CONTROL_LIFT_DERIVATIVE, val=2.5, units='1/rad')
        self.add_input(AE.CONTROL_MOMENT_DERIVATIVE, val=-0.60, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_ALPHA_DERIVATIVE, val=-0.02, units='1/rad')
        self.add_input(AE.HINGE_MOMENT_CONTROL_DERIVATIVE, val=-0.30, units='1/rad')
        self.add_input(AE.CONTROL_HINGE_FRACTION, val=0.75, units='unitless')

        self.add_output(AE.BEAM_MODAL_BENDING_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.BEAM_MODAL_TORSION_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.BEAM_MODAL_FLUTTER_SPEED, val=350.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_FLUTTER_SPEED_MARGIN, val=100.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_MAX_REAL_EIGENVALUE_AT_DESIGN, val=-1.0, units='1/s')
        self.add_output(AE.BEAM_MODAL_CRITICAL_MODE, val=0.0)
        self.add_output(AE.BEAM_MODAL_PK_FLUTTER_SPEED, val=350.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_PK_FLUTTER_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.BEAM_MODAL_PK_FLUTTER_SPEED_MARGIN, val=100.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_PK_MODE_DAMPING, val=np.ones(2))
        self.add_output(AE.BEAM_MODAL_PK_MODE_FREQUENCY, val=np.zeros(2), units='Hz')
        self.add_output(AE.BEAM_MODAL_PK_CONVERGED, val=1.0)
        self.add_output(AE.BEAM_MODAL_CONTROL_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED, val=350.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_3DOF_PK_FLUTTER_FREQUENCY, val=0.0, units='Hz')
        self.add_output(AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, val=100.0, units='m/s')
        self.add_output(AE.BEAM_MODAL_3DOF_PK_CONVERGED, val=1.0)
        self.add_output(AE.BEAM_MODAL_3DOF_PK_MODE_DAMPING, val=np.ones(3))
        self.add_output(AE.BEAM_MODAL_3DOF_PK_MODE_FREQUENCY, val=np.zeros(3), units='Hz')

        self.declare_partials('*', '*', method='fd')

    def _max_real_at_speed(self, speed, inputs):
        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        chord = np.asarray(inputs[AE.SPANWISE_CHORD])
        EI = np.asarray(inputs[AE.SPANWISE_BENDING_STIFFNESS])
        GJ = np.asarray(inputs[AE.SPANWISE_TORSIONAL_RIGIDITY])
        mass = np.asarray(inputs[AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN])
        pitch_inertia = np.asarray(inputs[AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN])
        cm_alpha_ea = (
            float(np.asarray(inputs[AE.LIFT_CURVE_SLOPE]).item())
            * float(np.asarray(inputs[AE.AERO_CENTER_TO_EA_FRACTION]).item())
            + float(np.asarray(inputs[AE.CONTROL_MOMENT_ALPHA_DERIVATIVE]).item())
        )

        state, _, _ = beam_modal_state_matrix(
            float(speed),
            float(np.asarray(inputs[AE.AIR_DENSITY]).item()),
            y,
            chord,
            EI,
            GJ,
            mass,
            pitch_inertia,
            float(np.asarray(inputs[AE.STRUCTURAL_DAMPING_RATIO]).item()),
            float(np.asarray(inputs[AE.LIFT_CURVE_SLOPE]).item()),
            cm_alpha_ea,
        )
        eigvals = np.linalg.eigvals(state)
        idx = int(np.argmax(eigvals.real))
        return eigvals[idx], idx

    def compute(self, inputs, outputs):
        y = np.asarray(inputs[AE.SPANWISE_STATIONS])
        chord = np.asarray(inputs[AE.SPANWISE_CHORD])
        EI = np.asarray(inputs[AE.SPANWISE_BENDING_STIFFNESS])
        GJ = np.asarray(inputs[AE.SPANWISE_TORSIONAL_RIGIDITY])
        mass = np.asarray(inputs[AE.SPANWISE_TOTAL_MASS_PER_UNIT_SPAN])
        pitch_inertia = np.asarray(inputs[AE.SPANWISE_TOTAL_PITCH_INERTIA_PER_UNIT_SPAN])
        rho = float(np.asarray(inputs[AE.AIR_DENSITY]).item())
        structural_damping = float(np.asarray(inputs[AE.STRUCTURAL_DAMPING_RATIO]).item())
        cl_alpha = float(np.asarray(inputs[AE.LIFT_CURVE_SLOPE]).item())
        aero_center_to_ea = float(np.asarray(inputs[AE.AERO_CENTER_TO_EA_FRACTION]).item())
        cm_alpha_ea = (
            cl_alpha * aero_center_to_ea
            + float(np.asarray(inputs[AE.CONTROL_MOMENT_ALPHA_DERIVATIVE]).item())
        )
        elastic_axis_fraction = 0.25 + aero_center_to_ea

        _, modal_mass, modal_stiffness = beam_modal_state_matrix(
            1.0e-6,
            rho,
            y,
            chord,
            EI,
            GJ,
            mass,
            pitch_inertia,
            structural_damping,
            cl_alpha,
            cm_alpha_ea,
        )
        dry_freq = np.sqrt(np.diag(modal_stiffness) / np.diag(modal_mass)) / (2.0 * np.pi)
        structural_damping_matrix = np.diag(
            2.0
            * structural_damping
            * np.diag(modal_mass)
            * np.sqrt(np.diag(modal_stiffness) / np.diag(modal_mass))
        )
        _, bending = _bending_mode_fe(y, EI, mass)
        _, torsion = _torsion_mode_fe(y, GJ, pitch_inertia)

        design_speed = float(np.asarray(inputs[AE.DESIGN_SPEED]).item())
        required_speed = float(np.asarray(inputs[AE.REQUIRED_SPEED]).item())
        flutter_max_speed = float(np.asarray(inputs[AE.FLUTTER_MAX_SPEED]).item())

        max_real_design, design_idx = self._max_real_at_speed(design_speed, inputs)

        speeds = np.linspace(0.0, flutter_max_speed, self.options['num_speed_samples'])
        eigs = np.array([self._max_real_at_speed(speed, inputs)[0].real for speed in speeds])

        unstable = np.flatnonzero(eigs > 0.0)
        if len(unstable) == 0:
            flutter_speed = flutter_max_speed
            critical_idx = design_idx
        else:
            hi_idx = unstable[0]
            lo = speeds[max(hi_idx - 1, 0)]
            hi = speeds[hi_idx]
            critical_idx = self._max_real_at_speed(hi, inputs)[1]
            for _ in range(self.options['bisection_iterations']):
                mid = 0.5 * (lo + hi)
                if self._max_real_at_speed(mid, inputs)[0].real > 0.0:
                    hi = mid
                else:
                    lo = mid
            flutter_speed = 0.5 * (lo + hi)

        pk_speeds = np.linspace(
            max(flutter_max_speed / max(self.options['num_speed_samples'] - 1, 1), 1.0e-3),
            flutter_max_speed,
            self.options['num_speed_samples'],
        )
        pk_damping_by_speed = []
        pk_frequency_by_speed = []
        pk_converged = True
        for speed in pk_speeds:
            _, damping, frequency, converged = beam_modal_pk_modes_at_speed(
                max(speed, 1.0e-6),
                rho,
                y,
                chord,
                modal_mass,
                modal_stiffness,
                structural_damping_matrix,
                bending,
                torsion,
                cl_alpha,
                cm_alpha_ea,
                elastic_axis_fraction,
                max_iterations=self.options['pk_iterations'],
                tolerance=self.options['pk_tolerance'],
            )
            pk_damping_by_speed.append(damping)
            pk_frequency_by_speed.append(frequency)
            pk_converged = pk_converged and converged

        pk_damping_by_speed = np.asarray(pk_damping_by_speed)
        pk_frequency_by_speed = np.asarray(pk_frequency_by_speed)
        pk_min_damping = np.min(pk_damping_by_speed, axis=1)
        pk_unstable = np.flatnonzero(pk_min_damping < 0.0)
        if len(pk_unstable) == 0:
            pk_flutter_speed = flutter_max_speed
            pk_flutter_frequency = pk_frequency_by_speed[-1, int(np.argmin(pk_damping_by_speed[-1]))]
        else:
            hi_idx = pk_unstable[0]
            lo = pk_speeds[max(hi_idx - 1, 0)]
            hi = pk_speeds[hi_idx]
            pk_flutter_frequency = pk_frequency_by_speed[hi_idx, int(np.argmin(pk_damping_by_speed[hi_idx]))]
            for _ in range(self.options['bisection_iterations']):
                mid = 0.5 * (lo + hi)
                _, damping, frequency, converged = beam_modal_pk_modes_at_speed(
                    max(mid, 1.0e-6),
                    rho,
                    y,
                    chord,
                    modal_mass,
                    modal_stiffness,
                    structural_damping_matrix,
                    bending,
                    torsion,
                    cl_alpha,
                    cm_alpha_ea,
                    elastic_axis_fraction,
                    max_iterations=self.options['pk_iterations'],
                    tolerance=self.options['pk_tolerance'],
                )
                pk_converged = pk_converged and converged
                if np.min(damping) < 0.0:
                    hi = mid
                    pk_flutter_frequency = frequency[int(np.argmin(damping))]
                else:
                    lo = mid
            pk_flutter_speed = 0.5 * (lo + hi)

        outputs[AE.BEAM_MODAL_BENDING_FREQUENCY] = dry_freq[0]
        outputs[AE.BEAM_MODAL_TORSION_FREQUENCY] = dry_freq[1]
        outputs[AE.BEAM_MODAL_FLUTTER_SPEED] = flutter_speed
        outputs[AE.BEAM_MODAL_FLUTTER_SPEED_MARGIN] = flutter_speed - required_speed
        outputs[AE.BEAM_MODAL_MAX_REAL_EIGENVALUE_AT_DESIGN] = max_real_design.real
        outputs[AE.BEAM_MODAL_CRITICAL_MODE] = float(critical_idx)
        outputs[AE.BEAM_MODAL_PK_FLUTTER_SPEED] = pk_flutter_speed
        outputs[AE.BEAM_MODAL_PK_FLUTTER_FREQUENCY] = pk_flutter_frequency
        outputs[AE.BEAM_MODAL_PK_FLUTTER_SPEED_MARGIN] = pk_flutter_speed - required_speed
        outputs[AE.BEAM_MODAL_PK_MODE_DAMPING] = pk_damping_by_speed[-1]
        outputs[AE.BEAM_MODAL_PK_MODE_FREQUENCY] = pk_frequency_by_speed[-1]
        outputs[AE.BEAM_MODAL_PK_CONVERGED] = 1.0 if pk_converged else 0.0

        # ── 3-DOF P-K with control surface DOF ──────────────────────────────
        span_start = float(np.asarray(inputs[AE.ELEVON_SPAN_START_FRACTION]).item())
        span_end = float(np.asarray(inputs[AE.ELEVON_SPAN_END_FRACTION]).item())
        ctrl_I = float(np.asarray(inputs[AE.CONTROL_INERTIA_PER_UNIT_SPAN]).item())
        ctrl_S = float(np.asarray(inputs[AE.CONTROL_STATIC_UNBALANCE]).item())
        k_hinge = float(np.asarray(inputs[AE.CONTROL_STIFFNESS]).item())
        cl_delta = float(np.asarray(inputs[AE.CONTROL_LIFT_DERIVATIVE]).item())
        cm_delta = float(np.asarray(inputs[AE.CONTROL_MOMENT_DERIVATIVE]).item())
        ch_alpha = float(np.asarray(inputs[AE.HINGE_MOMENT_ALPHA_DERIVATIVE]).item())
        ch_delta = float(np.asarray(inputs[AE.HINGE_MOMENT_CONTROL_DERIVATIVE]).item())
        control_hinge_fraction = float(np.asarray(inputs[AE.CONTROL_HINGE_FRACTION]).item())

        phi_delta = _control_mode_shape(y, span_start, span_end)
        ctrl_I_arr = ctrl_I * np.ones_like(y)
        ctrl_S_arr = ctrl_S * np.ones_like(y)

        modal_mass_3, modal_stiffness_3, bending_3, torsion_3 = _modal_matrices_3dof(
            y, EI, GJ, mass, pitch_inertia,
            phi_delta, ctrl_I_arr, ctrl_S_arr, k_hinge,
        )
        dry_freq_3 = np.sqrt(np.maximum(
            np.diag(modal_stiffness_3) / np.diag(modal_mass_3), 1.0e-12
        )) / (2.0 * np.pi)
        struct_damp_3 = np.diag(
            2.0 * structural_damping * np.diag(modal_mass_3)
            * np.sqrt(np.maximum(np.diag(modal_stiffness_3) / np.diag(modal_mass_3), 1.0e-12))
        )
        ref_chord_3 = max(_trapz(y, chord) / max(y[-1] - y[0], 1.0e-12), 1.0e-12)

        gaf_args_3 = (
            y, chord, bending_3, torsion_3, phi_delta,
            cl_alpha, cm_alpha_ea, cl_delta, cm_delta, ch_alpha, ch_delta,
            elastic_axis_fraction, control_hinge_fraction,
        )

        pk3_damping_by_speed = []
        pk3_frequency_by_speed = []
        pk3_converged = True
        for speed in pk_speeds:
            damp3, freq3, conv3 = beam_modal_pk_modes_at_speed_ndof(
                max(speed, 1.0e-6),
                rho,
                ref_chord_3,
                modal_mass_3,
                modal_stiffness_3,
                struct_damp_3,
                beam_modal_theodorsen_gaf_3dof,
                gaf_args_3,
                max_iterations=self.options['pk_iterations'],
                tolerance=self.options['pk_tolerance'],
            )
            pk3_damping_by_speed.append(damp3)
            pk3_frequency_by_speed.append(freq3)
            pk3_converged = pk3_converged and conv3

        pk3_damping_by_speed = np.asarray(pk3_damping_by_speed)
        pk3_frequency_by_speed = np.asarray(pk3_frequency_by_speed)
        pk3_min_damping = np.min(pk3_damping_by_speed, axis=1)
        pk3_unstable = np.flatnonzero(pk3_min_damping < 0.0)
        if len(pk3_unstable) == 0:
            pk3_flutter_speed = flutter_max_speed
            pk3_flutter_frequency = pk3_frequency_by_speed[-1, int(np.argmin(pk3_damping_by_speed[-1]))]
        else:
            hi_idx = pk3_unstable[0]
            lo3 = pk_speeds[max(hi_idx - 1, 0)]
            hi3 = pk_speeds[hi_idx]
            pk3_flutter_frequency = pk3_frequency_by_speed[hi_idx, int(np.argmin(pk3_damping_by_speed[hi_idx]))]
            for _ in range(self.options['bisection_iterations']):
                mid3 = 0.5 * (lo3 + hi3)
                damp_m, freq_m, _ = beam_modal_pk_modes_at_speed_ndof(
                    max(mid3, 1.0e-6),
                    rho,
                    ref_chord_3,
                    modal_mass_3,
                    modal_stiffness_3,
                    struct_damp_3,
                    beam_modal_theodorsen_gaf_3dof,
                    gaf_args_3,
                    max_iterations=self.options['pk_iterations'],
                    tolerance=self.options['pk_tolerance'],
                )
                if np.min(damp_m) < 0.0:
                    hi3 = mid3
                    pk3_flutter_frequency = freq_m[int(np.argmin(damp_m))]
                else:
                    lo3 = mid3
            pk3_flutter_speed = 0.5 * (lo3 + hi3)

        outputs[AE.BEAM_MODAL_CONTROL_FREQUENCY] = dry_freq_3[2]
        outputs[AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED] = pk3_flutter_speed
        outputs[AE.BEAM_MODAL_3DOF_PK_FLUTTER_FREQUENCY] = pk3_flutter_frequency
        outputs[AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN] = pk3_flutter_speed - required_speed
        outputs[AE.BEAM_MODAL_3DOF_PK_CONVERGED] = 1.0 if pk3_converged else 0.0
        outputs[AE.BEAM_MODAL_3DOF_PK_MODE_DAMPING] = pk3_damping_by_speed[-1]
        outputs[AE.BEAM_MODAL_3DOF_PK_MODE_FREQUENCY] = pk3_frequency_by_speed[-1]
