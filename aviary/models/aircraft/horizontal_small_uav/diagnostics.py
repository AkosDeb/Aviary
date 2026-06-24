"""Derivative and nonfinite diagnostics for the SpaJeti optimization model."""

from __future__ import annotations

import argparse
import contextlib
import io
import math
from pprint import pformat

import numpy as np

import aviary.api as av

try:
    from . import run_horizontal_small_uav as run_model
except ImportError:
    import run_horizontal_small_uav as run_model


TOTALS_OF = [
    'BEAM_MODAL_3DOF_PK_FLUTTER_SPEED_MARGIN',
    'aeroelasticity:pk_flutter_speed_margin',
    'aeroelasticity:flutter_speed_margin',
    'structural_empty_mass',
    'vtp_structural_mass',
    'mach_crit_margin',
]

TOTALS_WRT = [
    av.Aircraft.Wing.SPAN,
    'wing_section_tc',
    av.Aircraft.VerticalTail.SPAN,
]


def _promoted_outputs(prob):
    abs2prom = prob.model._var_allprocs_abs2prom['output']
    return set(abs2prom.values())


def _promoted_inputs(prob):
    abs2prom = prob.model._var_allprocs_abs2prom['input']
    return set(abs2prom.values())


def _available(names, available_names):
    return [name for name in names if name in available_names]


def _set_design_vars_to_bound(prob, bound):
    dvs = prob.model.get_design_vars()
    for name, meta in dvs.items():
        values = meta.get(bound)
        if values is None:
            continue

        arr = np.asarray(values)
        if arr.size == 0 or np.any(~np.isfinite(arr)):
            continue

        units = meta.get('units')
        prob.set_val(name, arr, units=units)


def _relative_error_summary(totals):
    summary = {}
    for of, wrt_data in totals.items():
        for wrt, data in wrt_data.items():
            rel = np.asarray(data.get('rel error', np.nan), dtype=float)
            finite_rel = rel[np.isfinite(rel)]
            max_rel = float(np.max(finite_rel)) if finite_rel.size else math.nan
            summary[(of, wrt)] = max_rel
    return summary


def check_totals_at_design_point(label, bound=None):
    """Run one model solve and check selected totals at interior/lower/upper DVs."""

    prob, _, _ = run_model.build_problem()
    if bound is not None:
        _set_design_vars_to_bound(prob, bound)

    prob.run_model()

    available_of = _available(TOTALS_OF, _promoted_outputs(prob))
    available_wrt = _available(TOTALS_WRT, _promoted_inputs(prob) | _promoted_outputs(prob))
    skipped_of = sorted(set(TOTALS_OF) - set(available_of))
    skipped_wrt = sorted(set(TOTALS_WRT) - set(available_wrt))

    if not available_of or not available_wrt:
        prob.cleanup()
        raise RuntimeError(
            f'{label}: no requested totals are available. '
            f'skipped_of={skipped_of}, skipped_wrt={skipped_wrt}'
        )

    totals = prob.check_totals(
        of=available_of,
        wrt=available_wrt,
        method='cs',
        compact_print=True,
        out_stream=None,
    )
    summary = _relative_error_summary(totals)
    prob.cleanup()

    return {
        'label': label,
        'of': available_of,
        'wrt': available_wrt,
        'skipped_of': skipped_of,
        'skipped_wrt': skipped_wrt,
        'max_relative_errors': summary,
    }


def run_bounds_sensitivity():
    """Compare baseline, lower-bound, and upper-bound CS total derivative checks."""

    baseline = check_totals_at_design_point('baseline')
    lower = check_totals_at_design_point('lower_bounds', bound='lower')
    upper = check_totals_at_design_point('upper_bounds', bound='upper')
    return [baseline, lower, upper]


def find_nonfinite_outputs(prob):
    """Return promoted output names whose current values contain NaN or Inf."""

    nonfinite = []
    for prom_name in sorted(_promoted_outputs(prob)):
        try:
            value = prob.get_val(prom_name)
        except Exception:
            continue

        arr = np.asarray(value)
        if np.iscomplexobj(arr):
            finite = np.isfinite(arr.real) & np.isfinite(arr.imag)
        else:
            finite = np.isfinite(arr)

        if not np.all(finite):
            nonfinite.append((prom_name, arr.shape, arr))
    return nonfinite


def run_first_iteration_nan_guard(check_partials=True):
    """Run one optimizer iteration, scan outputs, then optionally run check_partials."""

    prob, _, _ = run_model.build_problem()
    optimizer = prob.driver.options['optimizer']
    if optimizer == 'IPOPT':
        prob.driver.opt_settings['max_iter'] = 1
    elif optimizer == 'SLSQP':
        prob.driver.options['maxiter'] = 1

    prob.run_aviary_problem(make_plots=False)
    nonfinite = find_nonfinite_outputs(prob)

    partials = None
    if check_partials:
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            partials = prob.check_partials(compact_print=False)
        partials = {
            'raw': partials,
            'printed': stream.getvalue(),
        }

    prob.cleanup()
    return {
        'optimizer': optimizer,
        'nonfinite_outputs': nonfinite,
        'partials': partials,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'mode',
        choices=['bounds', 'nan-guard'],
        help='Diagnostic to run.',
    )
    parser.add_argument(
        '--skip-partials',
        action='store_true',
        help='For nan-guard, skip the expensive check_partials() call.',
    )
    args = parser.parse_args()

    if args.mode == 'bounds':
        result = run_bounds_sensitivity()
    else:
        result = run_first_iteration_nan_guard(check_partials=not args.skip_partials)

    print(pformat(result, sort_dicts=False))


if __name__ == '__main__':
    main()
