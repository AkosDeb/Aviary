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

from aviary.subsystems.propulsion.small_turbojet.model.max_rpm import MaxRPM
from aviary.subsystems.propulsion.small_turbojet.model.max_thrust import MaxThrust
from aviary.subsystems.propulsion.small_turbojet.model.max_weight import MaxWeight
from aviary.subsystems.propulsion.small_turbojet.model.sfc import SFC
from aviary.subsystems.propulsion.small_turbojet.variables import SmallTurbojetVariables
from aviary.variable_info.variables import Aircraft

# ---------------------------------------------------------------------------
# Reference geometry
# ---------------------------------------------------------------------------
_D_REF = 0.15   # m
_L_REF = 0.45   # m


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
    m.add_subsystem('max_rpm',    MaxRPM(),    promotes=['*'])
    m.add_subsystem('max_weight', MaxWeight(), promotes=['*'])
    m.add_subsystem('max_thrust', MaxThrust(), promotes=['*'])
    m.add_subsystem('sfc',        SFC(),       promotes=['*'])
    prob.setup(force_alloc_complex=force_alloc_complex)
    return prob


def _run_chain(d_m=_D_REF, l_m=_L_REF, complex=False) -> om.Problem:
    prob = _build_chain(force_alloc_complex=complex)
    prob.set_val(SmallTurbojetVariables.DIAMETER, d_m, units='m')
    prob.set_val(SmallTurbojetVariables.LENGTH,   l_m, units='m')
    prob.run_model()
    return prob


def _get_scalar(prob: om.Problem, name: str, units: str) -> float:
    """Return a promoted OpenMDAO variable as a Python scalar."""
    return prob.get_val(name, units=units).item()



# ===========================================================================
# MaxRPM — standalone
# ===========================================================================

def test_max_rpm_output_finite_and_positive():
    prob = _run_single(MaxRPM(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF, 'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF, 'm'),
    })
    rpm = prob.get_val(SmallTurbojetVariables.MAX_RPM, units='rpm')
    assert np.all(np.isfinite(rpm)), 'rpm_max contains non-finite values'
    assert np.all(rpm > 0), 'rpm_max must be positive'


def test_max_rpm_physical_range():
    """Small turbojets typically spin 30 000 – 200 000 rpm."""
    prob = _run_single(MaxRPM(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF, 'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF, 'm'),
    })
    rpm = _get_scalar(prob, SmallTurbojetVariables.MAX_RPM, 'rpm')
    assert rpm > 10_000,  f'rpm_max={rpm:.0f} below plausible minimum'
    assert rpm < 300_000, f'rpm_max={rpm:.0f} above plausible maximum'


def test_max_rpm_partials():
    prob = _run_single(MaxRPM(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF, 'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF, 'm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# MaxWeight — standalone
# ===========================================================================

def test_max_weight_output_finite_and_positive():
    prob = _run_single(MaxWeight(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF,     'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,     'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0,  'rpm'),
    })
    mass = prob.get_val(SmallTurbojetVariables.MASS, units='kg')
    assert np.all(np.isfinite(mass)), 'mass contains non-finite values'
    assert np.all(mass > 0), 'mass must be positive'


def test_max_weight_physical_range():
    """Small turbojets typically weigh 0.3 – 30 kg."""
    prob = _run_single(MaxWeight(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF,    'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0, 'rpm'),
    })
    mass = _get_scalar(prob, SmallTurbojetVariables.MASS, 'kg')
    assert mass > 0.1,  f'mass={mass:.3f} kg below plausible minimum'
    assert mass < 100.0, f'mass={mass:.3f} kg above plausible maximum'


def test_max_weight_near_training_mean():
    """At the input-space training means the prediction should be near the dataset mean ~4.68 kg."""
    prob = _run_single(MaxWeight(), {
        SmallTurbojetVariables.DIAMETER: (138.640131579 / 1e3, 'm'),
        SmallTurbojetVariables.LENGTH:   (340.222368421 / 1e3, 'm'),
        SmallTurbojetVariables.MAX_RPM:  (111038.157895,       'rpm'),
    })
    mass = _get_scalar(prob, SmallTurbojetVariables.MASS, 'kg')
    assert 0.5 < mass < 30.0, f'mass={mass:.3f} kg far from expected ~4.68 kg at training means'


def test_max_weight_partials():
    prob = _run_single(MaxWeight(), {
        SmallTurbojetVariables.DIAMETER: (_D_REF,    'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0, 'rpm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# MaxThrust — standalone
# ===========================================================================

def test_max_thrust_intercept_at_training_mean():
    """Linear model: prediction at all-zero standardized inputs equals the intercept exactly."""
    prob = _run_single(MaxThrust(), {
        SmallTurbojetVariables.MASS:     (4.52688157895,        'kg'),
        SmallTurbojetVariables.DIAMETER: (138.640131579 / 1e3,  'm'),
        SmallTurbojetVariables.LENGTH:   (340.222368421 / 1e3,  'm'),
        SmallTurbojetVariables.MAX_RPM:  (111038.157895,        'rpm'),
    })
    thrust = _get_scalar(prob, Aircraft.Engine.SCALED_SLS_THRUST, 'N')
    assert_near_equal(thrust, 376.727, tolerance=1e-8)


def test_max_thrust_output_finite_and_positive():
    prob = _run_single(MaxThrust(), {
        SmallTurbojetVariables.MASS:     (4.5,       'kg'),
        SmallTurbojetVariables.DIAMETER: (_D_REF,    'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0, 'rpm'),
    })
    thrust = prob.get_val(Aircraft.Engine.SCALED_SLS_THRUST, units='N')
    assert np.all(np.isfinite(thrust))
    assert np.all(thrust > 0)


def test_max_thrust_physical_range():
    """Small turbojets typically produce 10 – 2000 N."""
    prob = _run_single(MaxThrust(), {
        SmallTurbojetVariables.MASS:     (4.5,       'kg'),
        SmallTurbojetVariables.DIAMETER: (_D_REF,    'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0, 'rpm'),
    })
    thrust = _get_scalar(prob, Aircraft.Engine.SCALED_SLS_THRUST, 'N')
    assert thrust > 1.0,   f'thrust={thrust:.1f} N below plausible minimum'
    assert thrust < 5000.0, f'thrust={thrust:.1f} N above plausible maximum'


def test_max_thrust_partials():
    prob = _run_single(MaxThrust(), {
        SmallTurbojetVariables.MASS:     (4.5,       'kg'),
        SmallTurbojetVariables.DIAMETER: (_D_REF,    'm'),
        SmallTurbojetVariables.LENGTH:   (_L_REF,    'm'),
        SmallTurbojetVariables.MAX_RPM:  (100_000.0, 'rpm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# SFC — standalone
# ===========================================================================

def test_sfc_scaler_stats_are_populated():
    """SFC physical tests require real training scaler statistics."""
    from aviary.subsystems.propulsion.small_turbojet.model.sfc import (
        _MU_D, _MU_L, _MU_R, _MU_T, _MU_W,
        _SIG_D, _SIG_L, _SIG_R, _SIG_T, _SIG_W,
    )

    means = np.array([_MU_R, _MU_D, _MU_W, _MU_T, _MU_L])
    sigmas = np.array([_SIG_R, _SIG_D, _SIG_W, _SIG_T, _SIG_L])

    assert np.any(means != 0.0), 'SFC scaler means still look like placeholders'
    assert np.all(sigmas > 0.0), 'SFC scaler sigmas must be positive'
    assert np.any(sigmas != 1.0), 'SFC scaler sigmas still look like placeholders'


def test_sfc_output_finite_and_positive():
    prob = _run_single(SFC(), {
        SmallTurbojetVariables.MAX_RPM:        (100_000.0, 'rpm'),
        SmallTurbojetVariables.DIAMETER:       (_D_REF,    'm'),
        SmallTurbojetVariables.MASS:           (4.5,       'kg'),
        Aircraft.Engine.SCALED_SLS_THRUST:     (300.0,     'N'),
        SmallTurbojetVariables.LENGTH:         (_L_REF,    'm'),
    })
    sfc = prob.get_val(SmallTurbojetVariables.SFC, units='kg/(N*s)')
    assert np.all(np.isfinite(sfc))
    assert np.all(sfc > 0)


def test_sfc_physical_range():
    """Typical small turbojet SFC: 0.03–0.15 kg/(N·h) = ~8e-6–4e-5 kg/(N·s)."""
    prob = _run_single(SFC(), {
        SmallTurbojetVariables.MAX_RPM:        (100_000.0, 'rpm'),
        SmallTurbojetVariables.DIAMETER:       (_D_REF,    'm'),
        SmallTurbojetVariables.MASS:           (4.5,       'kg'),
        Aircraft.Engine.SCALED_SLS_THRUST:     (300.0,     'N'),
        SmallTurbojetVariables.LENGTH:         (_L_REF,    'm'),
    })
    sfc = _get_scalar(prob, SmallTurbojetVariables.SFC, 'kg/(N*s)')
    assert sfc > 1e-6, f'SFC={sfc:.2e} below plausible minimum'
    assert sfc < 5e-4, f'SFC={sfc:.2e} above plausible maximum'


def test_sfc_intercept_at_training_mean():
    """Linear model: at training-mean inputs all z-terms vanish -> output = intercept.
    0.157449 kg/(N·h) converted to kg/(N·s) = 0.157449 / 3600."""
    prob = _run_single(SFC(), {
        SmallTurbojetVariables.MAX_RPM:        (112534.782609,        'rpm'),
        SmallTurbojetVariables.DIAMETER:       (134.483333333 / 1e3,  'm'),
        SmallTurbojetVariables.MASS:           (4.00062318841,        'kg'),
        Aircraft.Engine.SCALED_SLS_THRUST:     (373.079710145,        'N'),
        SmallTurbojetVariables.LENGTH:         (335.917391304 / 1e3,  'm'),
    })
    sfc = _get_scalar(prob, SmallTurbojetVariables.SFC, 'kg/(N*s)')
    assert_near_equal(sfc, 0.157449 / 3600.0, tolerance=1e-8)


def test_sfc_partials():
    prob = _run_single(SFC(), {
        SmallTurbojetVariables.MAX_RPM:        (100_000.0, 'rpm'),
        SmallTurbojetVariables.DIAMETER:       (_D_REF,    'm'),
        SmallTurbojetVariables.MASS:           (4.5,       'kg'),
        Aircraft.Engine.SCALED_SLS_THRUST:     (300.0,     'N'),
        SmallTurbojetVariables.LENGTH:         (_L_REF,    'm'),
    })
    data = prob.check_partials(out_stream=None, method='cs')
    assert_check_partials(data, atol=1e-8, rtol=1e-8)


# ===========================================================================
# Chained model
# ===========================================================================

def test_chain_outputs_finite_and_positive():
    prob = _run_chain()
    for label, var, units in [
        ('rpm_max', SmallTurbojetVariables.MAX_RPM,          'rpm'),
        ('mass',    SmallTurbojetVariables.MASS,             'kg'),
        ('thrust',  Aircraft.Engine.SCALED_SLS_THRUST,       'N'),
        ('sfc',     SmallTurbojetVariables.SFC,              'kg/(N*s)'),
    ]:
        val = prob.get_val(var, units=units)
        assert np.all(np.isfinite(val)), f'{label} not finite'
        assert np.all(val > 0),          f'{label} not positive'


def test_chain_outputs_physical_range():
    prob = _run_chain()
    checks = [
        ('rpm_max (rpm)',  SmallTurbojetVariables.MAX_RPM,          'rpm',      10_000,  300_000),
        ('mass (kg)',      SmallTurbojetVariables.MASS,             'kg',       0.1,     100.0),
        ('thrust (N)',     Aircraft.Engine.SCALED_SLS_THRUST,       'N',        1.0,     5000.0),
        ('sfc (kg/N/s)',   SmallTurbojetVariables.SFC,              'kg/(N*s)', 1e-6,    5e-4),
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


def test_chain_larger_diameter_increases_mass():
    """Larger diameter (at fixed length) should produce a heavier engine."""
    prob_sm = _run_chain(d_m=0.10, l_m=_L_REF)
    prob_lg = _run_chain(d_m=0.20, l_m=_L_REF)
    mass_sm = _get_scalar(prob_sm, SmallTurbojetVariables.MASS, 'kg')
    mass_lg = _get_scalar(prob_lg, SmallTurbojetVariables.MASS, 'kg')
    assert mass_lg > mass_sm, (
        f'Expected mass to increase with diameter; got {mass_sm:.3f} kg (d=0.10) '
        f'and {mass_lg:.3f} kg (d=0.20)'
    )


def test_chain_reproducible():
    """Identical inputs must produce bit-for-bit identical outputs on two runs."""
    r1 = _run_chain()
    r2 = _run_chain()
    for var, units in [
        (SmallTurbojetVariables.MAX_RPM,          'rpm'),
        (SmallTurbojetVariables.MASS,             'kg'),
        (Aircraft.Engine.SCALED_SLS_THRUST,       'N'),
        (SmallTurbojetVariables.SFC,              'kg/(N*s)'),
    ]:
        assert_near_equal(
            r1.get_val(var, units=units),
            r2.get_val(var, units=units),
            tolerance=1e-14,
        )
