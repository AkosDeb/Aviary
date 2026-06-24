"""Derivative robustness checks for the SpaJeti optimization model.

Implements TOOD.md Gradient robustness:
  1. check_totals() at baseline — compares analytic vs CS total derivatives for
     the active optimizer constraints and objective after a single run_model()
     solve.  Relative errors > 1e-4 flag a gradient path that IPOPT cannot trust.
  2. Bounds sensitivity — repeats check_totals() with all DVs pushed to their
     lower and upper bounds.  A relative-error jump > 10x vs. the interior
     baseline indicates a near-discontinuity in some compute() method that will
     confuse IPOPT near the bound.
  3. Component timing profiler — times each ExplicitComponent's compute() to
     identify which subsystem dominates the optimization coloring startup cost.

Usage (standalone)
------------------
    # interior baseline only:
    python -m aviary.models.aircraft.horizontal_small_uav.gradient_checks

    # lower-bound point only:
    python -m aviary.models.aircraft.horizontal_small_uav.gradient_checks --lower

    # upper-bound point only:
    python -m aviary.models.aircraft.horizontal_small_uav.gradient_checks --upper

    # all three points (baseline + lower + upper):
    python -m aviary.models.aircraft.horizontal_small_uav.gradient_checks --all

    # component timing profile (identify coloring bottleneck):
    python -m aviary.models.aircraft.horizontal_small_uav.gradient_checks --profile

The script reuses build_problem() from run_horizontal_small_uav, then calls
run_model() once per DV point instead of run_aviary_problem(), so no optimizer
license is required.

Focused derivatives (per TOOD.md)
----------------------------------
  of: BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN, structural_empty_mass,
      vtp_structural_mass, fuel_budget_margin, Nz, mach_crit_margin,
      divergence_speed_margin
  wrt: Aircraft.Wing.SPAN, Aircraft.VerticalTail.SPAN, wing_section_tc,
       Aircraft.Wing.AREA
"""

import argparse
import sys
import time
import warnings
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import aviary.api as av

try:
    from . import horizontal_small_uav_config as cfg
    from .optimization_setup import OPTIMIZER
except ImportError:
    import horizontal_small_uav_config as cfg
    from optimization_setup import OPTIMIZER

from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE

# ── Derivative scopes ─────────────────────────────────────────────────────────

# "of" — outputs whose total derivatives we verify.
# Includes all variables mentioned in TOOD.md plus the full active constraint set.
_FOCUSED_OF = [
    AE.BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN,
    AE.DIVERGENCE_SPEED_MARGIN,
    'structural_empty_mass',
    'vtp_structural_mass',
    cfg.FUEL_BUDGET_MARGIN,
    'Nz',
    'Ny',
    'mach_crit_margin',
]

# "wrt" — design variables whose sensitivity we probe.
_FOCUSED_WRT = [
    av.Aircraft.Wing.SPAN,
    av.Aircraft.Wing.AREA,
    av.Aircraft.VerticalTail.SPAN,
    'wing_section_tc',
]

# DV bounds matching optimization_setup.py (used for bounds sensitivity).
_DV_BOUNDS = {
    av.Aircraft.Wing.SPAN:         {'lower': 1.34,                  'upper': 2.00,                  'units': 'm'},
    av.Aircraft.Wing.AREA:         {'lower': 0.40,                  'upper': 0.75,                  'units': 'm**2'},
    av.Aircraft.VerticalTail.SPAN: {'lower': cfg.VTP_SPAN_LOWER_M,  'upper': cfg.VTP_SPAN_UPPER_M,  'units': 'm'},
    'wing_section_tc':             {'lower': cfg.WING_TC_LOWER,      'upper': cfg.WING_TC_UPPER,     'units': None},
}

# Threshold above which a relative error is flagged as a failure.
_REL_ERR_FAIL  = 1.0e-4
# Ratio of relative errors (bound / interior) above which a bound-sensitivity
# spike is flagged as evidence of a near-discontinuity.
_SPIKE_RATIO   = 10.0

_SEP = '=' * 72


def _set_dvs(prob, point):
    """Set all DVs to the requested point: 'lower', 'upper', or 'interior'."""
    for name, bounds in _DV_BOUNDS.items():
        if point == 'lower':
            val = bounds['lower']
        elif point == 'upper':
            val = bounds['upper']
        else:
            val = 0.5 * (bounds['lower'] + bounds['upper'])
        units = bounds['units']
        try:
            if units:
                prob.set_val(name, val, units=units)
            else:
                prob.set_val(name, val)
        except Exception as exc:
            print(f'  [WARN] could not set {name}={val}: {exc}')


def _extract_rel_errors(data):
    """Return {(of, wrt): rel_err_forward} from check_totals() output dict."""
    result = {}
    for key, entry in data.items():
        rel = entry.get('rel error')
        if rel is None:
            continue
        try:
            result[key] = float(rel.forward)
        except Exception:
            result[key] = float('nan')
    return result


def _print_totals_report(data, point_name):
    """Print a formatted check_totals() summary with PASS/WARN/FAIL flags."""
    rel_errors = _extract_rel_errors(data)
    n_fail = sum(1 for v in rel_errors.values() if v > _REL_ERR_FAIL)
    print(f'\n{_SEP}')
    print(f'check_totals()  —  point: {point_name}')
    print(f'  rel_err threshold: {_REL_ERR_FAIL:.0e}   failures: {n_fail}/{len(rel_errors)}')
    print(_SEP)
    for (of_var, wrt_var), rel_err in sorted(rel_errors.items()):
        of_short  = of_var.split(':')[-1]  if ':' in of_var  else of_var
        wrt_short = wrt_var.split(':')[-1] if ':' in wrt_var else wrt_var
        if rel_err > _REL_ERR_FAIL:
            flag = '  *** FAIL'
        elif rel_err > _REL_ERR_FAIL * 0.1:
            flag = '  WARN'
        else:
            flag = '  ok'
        print(f'  d({of_short:40s}) / d({wrt_short:20s})  rel_err={rel_err:.2e}{flag}')
    print(_SEP)
    return rel_errors


def _print_bounds_sensitivity(interior_errs, lower_errs, upper_errs):
    """Compare relative errors across the three DV points and flag spikes."""
    all_keys = set(interior_errs) | set(lower_errs) | set(upper_errs)
    spike_found = False

    print(f'\n{_SEP}')
    print('Bounds sensitivity  —  ratio = bound_err / interior_err')
    print(f'  A ratio > {_SPIKE_RATIO:.0f}x flags a near-discontinuity at the bound.')
    print(_SEP)

    rows = []
    for key in sorted(all_keys):
        base = interior_errs.get(key, float('nan'))
        lo   = lower_errs.get(key, float('nan'))
        hi   = upper_errs.get(key, float('nan'))

        ratio_lo = lo / base if (base > 0 and not np.isnan(lo)) else float('nan')
        ratio_hi = hi / base if (base > 0 and not np.isnan(hi)) else float('nan')

        spike_lo = (not np.isnan(ratio_lo)) and ratio_lo > _SPIKE_RATIO
        spike_hi = (not np.isnan(ratio_hi)) and ratio_hi > _SPIKE_RATIO
        if spike_lo or spike_hi:
            spike_found = True

        of_var, wrt_var = key
        of_s  = of_var.split(':')[-1]  if ':' in of_var  else of_var
        wrt_s = wrt_var.split(':')[-1] if ':' in wrt_var else wrt_var

        lo_str = f'{ratio_lo:6.1f}x' if not np.isnan(ratio_lo) else '    n/a'
        hi_str = f'{ratio_hi:6.1f}x' if not np.isnan(ratio_hi) else '    n/a'
        lo_flag = ' ***' if spike_lo else ''
        hi_flag = ' ***' if spike_hi else ''

        rows.append(
            f'  d({of_s:38s})/d({wrt_s:18s})  '
            f'lower:{lo_str}{lo_flag:<4}  upper:{hi_str}{hi_flag}'
        )

    for row in rows:
        print(row)

    print(_SEP)
    if spike_found:
        print(
            'ACTION: *** spikes detected. Inspect compute() methods at DV bounds\n'
            '        for abs(), max(), min(), or np.where() with zero crossing.'
        )
    else:
        print('Bounds sensitivity PASSED — no spikes > 10x detected.')
    print(_SEP)


def run_check_totals(prob, point, method='cs'):
    """Set DVs to *point*, run the model, and return check_totals() data."""
    _set_dvs(prob, point)
    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore')
        prob.run_model()

    # Filter the focused "of" list to only those that exist in the model.
    model_outputs = {
        meta['prom_name']
        for _, meta in prob.model.list_outputs(val=False, prom_name=True, out_stream=None)
    }
    of_live  = [v for v in _FOCUSED_OF  if v in model_outputs]
    wrt_live = [v for v in _FOCUSED_WRT if v in model_outputs]

    missing_of  = [v for v in _FOCUSED_OF  if v not in model_outputs]
    missing_wrt = [v for v in _FOCUSED_WRT if v not in model_outputs]
    if missing_of:
        print(f'  [WARN] "of" not found in model, skipping: {missing_of}')
    if missing_wrt:
        print(f'  [WARN] "wrt" not found in model, skipping: {missing_wrt}')

    if not of_live or not wrt_live:
        print('  [ERROR] nothing to check — returning empty dict.')
        return {}

    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore')
        data = prob.check_totals(
            of=of_live,
            wrt=wrt_live,
            method=method,
            compact_print=True,
            out_stream=None,
        )
    return data


def profile_component_timing(prob, n_evals=3, top_n=15, out_stream=None):
    """Time each ExplicitComponent's compute() to identify optimization bottlenecks.

    Runs n_evals full run_model() calls and attributes the wall time to individual
    components by timing their compute() directly.  Reports the top_n slowest
    components and estimates the coloring startup cost.

    Parameters
    ----------
    prob : AviaryProblem
        A fully set-up (prob.setup() already called) problem.
    n_evals : int
        Number of compute() calls per component for averaging.
    top_n : int
        Number of slowest components to print.
    out_stream : file-like, optional
        Output stream (defaults to sys.stdout).
    """
    import sys as _sys
    stream = out_stream if out_stream is not None else _sys.stdout

    # First: time a single run_model to get the total baseline.
    t0 = time.perf_counter()
    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore')
        prob.run_model()
    total_run_model_s = time.perf_counter() - t0

    # Collect all ExplicitComponents.
    from openmdao.core.explicitcomp import ExplicitComponent
    components = [
        sys for sys in prob.model.system_iter(include_self=False, recurse=True)
        if isinstance(sys, ExplicitComponent)
    ]

    # Time each component's compute() independently.
    timings = []
    for comp in components:
        inputs  = comp._inputs
        outputs = comp._outputs
        # Warm-up once (not counted).
        try:
            with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
                warnings.simplefilter('ignore')
                comp.compute(inputs, outputs)
        except Exception:
            timings.append((comp.pathname, float('nan')))
            continue
        t_start = time.perf_counter()
        for _ in range(n_evals):
            try:
                with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
                    warnings.simplefilter('ignore')
                    comp.compute(inputs, outputs)
            except Exception:
                break
        elapsed = (time.perf_counter() - t_start) / n_evals
        timings.append((comp.pathname, elapsed))

    timings.sort(key=lambda x: x[1] if not np.isnan(x[1]) else -1.0, reverse=True)

    sep = '=' * 72
    print(f'\n{sep}', file=stream)
    print(f'Component timing profile   (n_evals={n_evals} per component)', file=stream)
    print(f'  Total run_model() wall time: {total_run_model_s:.2f} s', file=stream)
    print(f'  Top {top_n} slowest components:', file=stream)
    print(sep, file=stream)
    for name, t in timings[:top_n]:
        if np.isnan(t):
            label = '  ERROR  '
        else:
            label = f'{t * 1000.0:8.1f} ms'
        short = name.split('.')[-1] if '.' in name else name
        print(f'  {label}   {name}', file=stream)

    total_component_s = sum(t for _, t in timings if not np.isnan(t))
    print(sep, file=stream)
    print(f'  Sum of timed compute() calls: {total_component_s:.2f} s', file=stream)

    # Estimate coloring cost: coloring does O(n_colors) run_model() calls.
    # n_colors ≈ max number of non-overlapping columns in the total Jacobian.
    # For a dense Jacobian from BeamModalFlutter (6 inputs × 1 output group),
    # n_colors ≈ n_design_vars × rows ≈ 10–100.  601 s / run_model ~ x evaluations.
    if total_run_model_s > 0:
        estimated_coloring_calls = 601.0 / total_run_model_s
        print(f'\n  [Estimate] v1.33 coloring took ~601 s @ this run_model cost = '
              f'{estimated_coloring_calls:.0f} evaluations', file=stream)
        print(f'  ACTION: set AEROELASTIC_SPEED_SAMPLES=20, AEROELASTIC_PK_ITERATIONS=15', file=stream)
        print(f'          Expected speedup: ~8x → coloring ~{601/8:.0f} s', file=stream)
        print(f'          With coloring cache (USE_COLORING_CACHE=True): ~0 s on repeat runs', file=stream)
    print(sep, file=stream)
    return timings


def main(points=('interior',), method='cs'):
    """Build the problem once and run check_totals at each requested point."""
    # Lazy import to avoid circular dependency: run_horizontal_small_uav imports
    # gradient_checks.main, so we cannot import build_problem at module level.
    try:
        from .run_horizontal_small_uav import build_problem
    except ImportError:
        from run_horizontal_small_uav import build_problem

    print(f'\n{_SEP}')
    print(f'SpaJeti gradient checks   method={method}   points={points}')
    print(f'optimizer configured: {OPTIMIZER}')
    print(_SEP)

    prob, _ = build_problem()

    # Warm-up: run model once at the initial guesses before any DV override.
    with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
        warnings.simplefilter('ignore')
        prob.run_model()

    results = {}
    for point in points:
        print(f'\nRunning at DV point: {point} …')
        data = run_check_totals(prob, point, method=method)
        results[point] = _print_totals_report(data, point)

    if 'interior' in results and 'lower' in results and 'upper' in results:
        _print_bounds_sensitivity(results['interior'], results['lower'], results['upper'])

    prob.cleanup()
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Total-derivative and bounds-sensitivity checks for SpaJeti.'
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument('--lower', action='store_true', help='Check at lower DV bounds only')
    grp.add_argument('--upper', action='store_true', help='Check at upper DV bounds only')
    grp.add_argument('--all',   action='store_true',
                     help='Check interior + lower + upper (full bounds sensitivity)')
    grp.add_argument('--profile', action='store_true',
                     help='Time each component compute() to identify coloring bottleneck')
    parser.add_argument('--method', default='cs', choices=['cs', 'fd'],
                        help='Finite-difference method for check_totals (default: cs)')
    parser.add_argument('--n-evals', type=int, default=3,
                        help='Evaluations per component for timing (default: 3)')
    args = parser.parse_args()

    if args.profile:
        try:
            from .run_horizontal_small_uav import build_problem
        except ImportError:
            from run_horizontal_small_uav import build_problem
        _prob, _ = build_problem()
        _prob.run_model()
        profile_component_timing(_prob, n_evals=args.n_evals)
        _prob.cleanup()
    else:
        if args.lower:
            pts = ('lower',)
        elif args.upper:
            pts = ('upper',)
        elif args.all:
            pts = ('interior', 'lower', 'upper')
        else:
            pts = ('interior',)
        main(points=pts, method=args.method)
