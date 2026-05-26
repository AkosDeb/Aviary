"""
Regression-chain tests for the small turbojet components.

Run with random order:
    pip install pytest-randomly
    pytest test_regression_chain.py -v

Fix the seed for reproducibility:
    pytest test_regression_chain.py -v -p randomly --randomly-seed=1234
"""
import os

import numpy as np
import openmdao.api as om
import pytest
from openmdao.utils.assert_utils import assert_check_partials, assert_near_equal

from aviary.subsystems.propulsion.small_turbojet.model.max_diameter import MaxDiameter
from aviary.subsystems.propulsion.small_turbojet.model.max_rpm import MaxRPM
from aviary.subsystems.propulsion.small_turbojet.model.max_weight import MaxWeight
from aviary.subsystems.propulsion.small_turbojet.model.sfc import SFC
from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Reference point (mean of training data for thrust)
# ---------------------------------------------------------------------------
_T_REF = 394.046   # N  (≈ training mean)
_D_REF = 0.14      # m  (approximate, computed by MaxDiameter at _T_REF)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _tmp_workdir(tmp_path):
    """Run each test in a clean temp directory (OpenMDAO may write output files)."""
    old = os.getcwd()
    os.chdir(tmp_path)
    yield
    os.chdir(old)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run_single(component, inputs: dict) -> om.Problem:
    prob = om.Problem()
    prob.model.add_subsystem('comp', component, promotes=['*'])
    prob.setup(force_alloc_complex=True)
    for name, (val, units) in inputs.items():
        prob.set_val(name, val, units=units)
    prob.run_model()
    return prob


def _build_chain(force_alloc_complex=False) -> om.Problem:
    prob = om.Problem()
    m = prob.model
    m.add_subsystem('max_diameter', MaxDiameter(), promotes=['*'])
    m.add_subsystem('max_rpm',      MaxRPM(),      promotes=['*'])
    m.add_subsystem('max_weight',   MaxWeight(),   promotes=['*'])
    m.add_subsystem('sfc',          SFC(),         promotes=['*'])
    prob.setup(force_alloc_complex=force_alloc_complex)
    return prob


def _run_chain(thrust_N=_T_REF, complex=False) -> om.Problem:
    prob = _build_chain(force_alloc_complex=complex)
    prob.set_val(Aircraft.Engine.SCALED_SLS_THRUST, thrust_N, units='N')
    prob.run_model()
    return prob


def _get_scalar(prob: om.Problem, name: str, units: str) -> float:
    """Return a promoted OpenMDAO variable as a Python scalar."""
    return prob.get_val(name, units=units).item()


# ===========================================================================
# MaxDiameter — standalone
# ===========================================================================

def test_max_diameter_output_finite_and_positive():
    prob = _run_single(MaxDiameter(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
    })
    d = prob.get_val(SmallTurbojetVariables.DIAMETER, units='m')
    assert np.all(np.isfinite(d)), 'diameter contains non-finite values'
    assert np.all(d > 0), 'diameter must be positive'


def test_max_diameter_physical_range():
    """Small turbojets typically have outer diameters of 50–300 mm."""
    prob = _run_single(MaxDiameter(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
    })
    d_mm = _get_scalar(prob, SmallTurbojetVariables.DIAMETER, 'm') * 1000.0
    assert d_mm > 30,  f'diameter={d_mm:.1f} mm below plausible minimum'
    assert d_mm < 400, f'diameter={d_mm:.1f} mm above plausible maximum'


def test_max_diameter_intercept_at_training_mean():
    """At the training-mean thrust the z-terms are nearly zero; output ≈ intercept."""
    mu_T = 394.046052632   # training mean for thrust_max_N
    prob = _run_single(MaxDiameter(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (mu_T, 'N'),
    })
    d_mm = _get_scalar(prob, SmallTurbojetVariables.DIAMETER, 'm') * 1000.0
    # z_T ≈ 0 but z_T2 is not exactly 0 at mu_T (quadratic term shifts it)
    assert 100.0 < d_mm < 200.0, f'diameter at training mean = {d_mm:.1f} mm, expected ~140 mm'


def test_max_diameter_larger_thrust_larger_diameter():
    """More thrust should yield a larger engine."""
    prob_lo = _run_single(MaxDiameter(), {Aircraft.Engine.SCALED_SLS_THRUST: (100.0, 'N')})
    prob_hi = _run_single(MaxDiameter(), {Aircraft.Engine.SCALED_SLS_THRUST: (800.0, 'N')})
    d_lo = _get_scalar(prob_lo, SmallTurbojetVariables.DIAMETER, 'm')
    d_hi = _get_scalar(prob_hi, SmallTurbojetVariables.DIAMETER, 'm')
    assert d_hi > d_lo, f'Expected diameter to increase with thrust; got {d_lo:.4f} m and {d_hi:.4f} m'


def test_max_diameter_partials():
    prob = _run_single(MaxDiameter(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# MaxRPM — standalone
# ===========================================================================

def test_max_rpm_output_finite_and_positive():
    prob = _run_single(MaxRPM(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF, 'm'),
    })
    rpm = prob.get_val(SmallTurbojetVariables.MAX_RPM, units='rpm')
    assert np.all(np.isfinite(rpm)), 'rpm_max contains non-finite values'
    assert np.all(rpm > 0), 'rpm_max must be positive'


def test_max_rpm_physical_range():
    """Small turbojets typically spin 30 000 – 200 000 rpm."""
    prob = _run_single(MaxRPM(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF, 'm'),
    })
    rpm = _get_scalar(prob, SmallTurbojetVariables.MAX_RPM, 'rpm')
    assert rpm > 10_000,  f'rpm_max={rpm:.0f} below plausible minimum'
    assert rpm < 300_000, f'rpm_max={rpm:.0f} above plausible maximum'


def test_max_rpm_partials():
    prob = _run_single(MaxRPM(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF, 'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF, 'm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# MaxWeight — standalone
# ===========================================================================

def test_max_weight_output_finite_and_positive():
    prob = _run_single(MaxWeight(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,     'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,     'm'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0,  'rpm'),
    })
    mass = prob.get_val(SmallTurbojetVariables.MASS, units='kg')
    assert np.all(np.isfinite(mass)), 'mass contains non-finite values'
    assert np.all(mass > 0), 'mass must be positive'


def test_max_weight_physical_range():
    """Small turbojets typically weigh 0.3 – 30 kg."""
    prob = _run_single(MaxWeight(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,    'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0, 'rpm'),
    })
    mass = _get_scalar(prob, SmallTurbojetVariables.MASS, 'kg')
    assert mass > 0.1,  f'mass={mass:.3f} kg below plausible minimum'
    assert mass < 100.0, f'mass={mass:.3f} kg above plausible maximum'


def test_max_weight_intercept_at_training_mean():
    """At the input-space training means all z-terms vanish → output = intercept = 4.74004."""
    prob = _run_single(MaxWeight(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (394.046052632,        'N'),
        SmallTurbojetVariables.DIAMETER:   (138.640131579 / 1e3,  'm'),
        SmallTurbojetVariables.MAX_RPM:    (111038.157895,         'rpm'),
    })
    mass = _get_scalar(prob, SmallTurbojetVariables.MASS, 'kg')
    assert_near_equal(mass, 4.74004, tolerance=1e-8)


def test_max_weight_partials():
    prob = _run_single(MaxWeight(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,    'N'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0, 'rpm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# SFC — standalone
# ===========================================================================

def test_sfc_output_finite_and_positive():
    prob = _run_single(SFC(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,     'N'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0,  'rpm'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,     'm'),
        SmallTurbojetVariables.MASS:       (4.5,        'kg'),
    })
    sfc = prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)')
    assert np.all(np.isfinite(sfc))
    assert np.all(sfc > 0)


def test_sfc_physical_range():
    """Typical small turbojet SFC: 0.03–0.15 kg/(N·h) = ~8e-6–4e-5 kg/(N·s)."""
    prob = _run_single(SFC(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,    'N'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0, 'rpm'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,    'm'),
        SmallTurbojetVariables.MASS:       (4.5,       'kg'),
    })
    sfc = _get_scalar(prob, SmallTurbojetVariables.SFC, 'kg/(N*s)')
    assert sfc > 1e-6, f'SFC={sfc:.2e} below plausible minimum'
    assert sfc < 5e-4, f'SFC={sfc:.2e} above plausible maximum'


def test_sfc_intercept_at_training_mean():
    """Linear model: at training-mean inputs all z-terms vanish → output = intercept.
    0.157423 kg/(N·h) converted to kg/(N·s) = 0.157423 / 3600."""
    prob = _run_single(SFC(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (373.079710145,        'N'),
        SmallTurbojetVariables.MAX_RPM:    (112534.782609,         'rpm'),
        SmallTurbojetVariables.DIAMETER:   (134.483333333 / 1e3,  'm'),
        SmallTurbojetVariables.MASS:       (4.00062318841,         'kg'),
    })
    sfc = _get_scalar(prob, SmallTurbojetVariables.SFC, 'kg/(N*s)')
    assert_near_equal(sfc, 0.157423 / 3600.0, tolerance=1e-8)


def test_sfc_partials():
    prob = _run_single(SFC(), {
        Aircraft.Engine.SCALED_SLS_THRUST: (_T_REF,    'N'),
        SmallTurbojetVariables.MAX_RPM:    (100_000.0, 'rpm'),
        SmallTurbojetVariables.DIAMETER:   (_D_REF,    'm'),
        SmallTurbojetVariables.MASS:       (4.5,       'kg'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# Chained model
# ===========================================================================

def test_chain_outputs_finite_and_positive():
    prob = _run_chain()
    for label, var, units in [
        ('diameter', SmallTurbojetVariables.DIAMETER,          'm'),
        ('rpm_max',  SmallTurbojetVariables.MAX_RPM,           'rpm'),
        ('mass',     SmallTurbojetVariables.MASS,              'kg'),
        ('sfc',      SmallTurbojetVariables.SFC,               'kg/(N*s)'),
    ]:
        val = prob.get_val(var, units=units)
        assert np.all(np.isfinite(val)), f'{label} not finite'
        assert np.all(val > 0),          f'{label} not positive'


def test_chain_outputs_physical_range():
    prob = _run_chain()
    checks = [
        ('diameter (mm)', SmallTurbojetVariables.DIAMETER, 'm',        0.03,   0.40),
        ('rpm_max (rpm)', SmallTurbojetVariables.MAX_RPM,  'rpm',      10_000, 300_000),
        ('mass (kg)',     SmallTurbojetVariables.MASS,     'kg',       0.1,    100.0),
        ('sfc (kg/N/s)', SmallTurbojetVariables.SFC,      'kg/(N*s)', 1e-6,   5e-4),
    ]
    for label, var, units, lo, hi in checks:
        val = _get_scalar(prob, var, units)
        assert val > lo, f'{label}={val:.4g} below lower bound {lo}'
        assert val < hi, f'{label}={val:.4g} above upper bound {hi}'


def test_chain_component_partials():
    """Complex-step partial check on every component through the full chain."""
    prob = _run_chain(complex=True)
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


def test_chain_larger_thrust_larger_diameter():
    """More thrust should produce a larger-diameter engine."""
    prob_lo = _run_chain(thrust_N=100.0)
    prob_hi = _run_chain(thrust_N=800.0)
    d_lo = _get_scalar(prob_lo, SmallTurbojetVariables.DIAMETER, 'm')
    d_hi = _get_scalar(prob_hi, SmallTurbojetVariables.DIAMETER, 'm')
    assert d_hi > d_lo, (
        f'Expected diameter to increase with thrust; got {d_lo:.4f} m (100 N) '
        f'and {d_hi:.4f} m (800 N)'
    )


def test_chain_larger_thrust_larger_mass():
    """More thrust should produce a heavier engine."""
    prob_lo = _run_chain(thrust_N=100.0)
    prob_hi = _run_chain(thrust_N=800.0)
    m_lo = _get_scalar(prob_lo, SmallTurbojetVariables.MASS, 'kg')
    m_hi = _get_scalar(prob_hi, SmallTurbojetVariables.MASS, 'kg')
    assert m_hi > m_lo, (
        f'Expected mass to increase with thrust; got {m_lo:.3f} kg (100 N) '
        f'and {m_hi:.3f} kg (800 N)'
    )


def test_chain_reproducible():
    """Identical inputs must produce bit-for-bit identical outputs on two runs."""
    r1 = _run_chain()
    r2 = _run_chain()
    for var, units in [
        (SmallTurbojetVariables.DIAMETER, 'm'),
        (SmallTurbojetVariables.MAX_RPM,  'rpm'),
        (SmallTurbojetVariables.MASS,     'kg'),
        (SmallTurbojetVariables.SFC,      'kg/(N*s)'),
    ]:
        assert_near_equal(
            r1.get_val(var, units=units),
            r2.get_val(var, units=units),
            tolerance=1e-14,
        )
