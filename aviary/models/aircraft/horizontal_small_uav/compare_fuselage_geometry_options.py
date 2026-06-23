"""Run old-vs-current fuselage geometry optimization comparisons.

This is intentionally a small executable helper around ``run_horizontal_small_uav``.
It monkeypatches only the fuselage geometry constants before each model build so
the rest of the mission, design variables, constraints, and optimizer are
identical between scenarios.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import time
import warnings
from contextlib import contextmanager
from pathlib import Path

import numpy as np

import aviary.api as av
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetVariables

try:
    from . import horizontal_small_uav_config as cfg
    from . import run_horizontal_small_uav as run
except ImportError:
    import horizontal_small_uav_config as cfg
    import run_horizontal_small_uav as run


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_OUT_DIR = REPO_ROOT / 'outputs' / 'fuselage_geometry_comparison'


SCENARIOS = {
    'old_power_law_fixed_base': {
        'FUSELAGE_NOSE_TYPE': 'power_law',
        'FUSELAGE_NOSE_POWER_EXPONENT': 1.0,
        'FUSELAGE_NOSE_ASPECT_RATIO': 2.0,
        'FUSELAGE_TAIL_LENGTH_FRACTION': 0.35,
        'FUSELAGE_AFT_BASE_MODE': 'fixed',
        'FUSELAGE_BASE_WIDTH_FRACTION': 0.20,
        'FUSELAGE_BASE_HEIGHT_FRACTION': 0.20,
        'description': 'Old conical/power-law nose, 35% aft taper, fixed 20% base.',
    },
    'current_ellipsoid_turbojet_base': {
        'FUSELAGE_NOSE_TYPE': cfg.FUSELAGE_NOSE_TYPE,
        'FUSELAGE_NOSE_POWER_EXPONENT': cfg.FUSELAGE_NOSE_POWER_EXPONENT,
        'FUSELAGE_NOSE_ASPECT_RATIO': cfg.FUSELAGE_NOSE_ASPECT_RATIO,
        'FUSELAGE_TAIL_LENGTH_FRACTION': cfg.FUSELAGE_TAIL_LENGTH_FRACTION,
        'FUSELAGE_AFT_BASE_MODE': cfg.FUSELAGE_AFT_BASE_MODE,
        'FUSELAGE_BASE_WIDTH_FRACTION': cfg.FUSELAGE_BASE_WIDTH_FRACTION,
        'FUSELAGE_BASE_HEIGHT_FRACTION': cfg.FUSELAGE_BASE_HEIGHT_FRACTION,
        'description': 'Current ellipsoid nose, 20% aft taper, turbojet diameter + clearance base.',
    },
}


@contextmanager
def patched_run_constants(overrides):
    keys = [k for k in overrides if k.isupper()]
    old_values = {key: getattr(run, key) for key in keys}
    try:
        for key in keys:
            setattr(run, key, overrides[key])
        yield
    finally:
        for key, value in old_values.items():
            setattr(run, key, value)


def _scalar(prob, name, units=None):
    try:
        value = prob.get_val(name, units=units) if units else prob.get_val(name)
    except Exception:
        return None
    arr = np.asarray(value)
    if arr.size == 0:
        return None
    return float(arr.ravel()[-1])


def _first_scalar(prob, candidates, units=None):
    for name in candidates:
        value = _scalar(prob, name, units)
        if value is not None:
            return value, name
    return None, None


def _collect_results(prob, scenario_name, scenario, elapsed_s):
    engine_mass_name = f'pre_mission.propulsion.{SmallTurbojetVariables.MASS}'
    engine_diameter_name = f'pre_mission.propulsion.{SmallTurbojetVariables.DIAMETER}'

    range_km, range_source = _first_scalar(
        prob,
        [
            av.Mission.RANGE,
            'mission:range',
            'traj.range',
            'traj.dash.states:distance',
            'traj.dash.timeseries.distance',
        ],
        'km',
    )
    cd0_dash, cd0_source = _first_scalar(
        prob,
        [
            'traj.dash.rhs_all.roskam_aero.CD0',
            'traj.phases.dash.rhs_all.roskam_aero.CD0',
            'traj.phases.dash.rhs_all.roskam_aero.RoskamCD0.CD0',
            'traj.phases.dash.rhs_all.CD0',
            'traj.phases.dash.rhs_all.aero.CD0',
            'traj.phases.dash.timeseries.CD0',
            'traj.dash.rhs_all.roskam_aero.RoskamCD0.CD0',
            'traj.dash.rhs_all.CD0',
            'traj.dash.rhs_all.aero.CD0',
            'traj.dash.timeseries.CD0',
            'CD0',
        ],
    )

    result = {
        'scenario': scenario_name,
        'description': scenario['description'],
        'optimizer': run.OPTIMIZER,
        'elapsed_s': elapsed_s,
        'range_km': range_km,
        'range_source': range_source,
        'dash_CD0': cd0_dash,
        'dash_CD0_source': cd0_source,
        'fuselage_structural_mass_kg': _scalar(prob, 'fuselage_structural_mass', 'kg'),
        'fuselage_wetted_area_m2': _scalar(prob, 'fuselage_wetted_area', 'm**2'),
        'fuselage_exposed_wetted_area_m2': _scalar(prob, 'fuselage_exposed_wetted_area', 'm**2'),
        'fuselage_equivalent_diameter_m': _scalar(prob, 'fuselage_equivalent_diameter', 'm'),
        'fuselage_fineness_ratio': _scalar(prob, 'fuselage_fineness_ratio'),
        'fuselage_centroid_x_m_aft': _scalar(prob, 'fuselage_centroid_x', 'm'),
        'fuselage_x_cg_m_forward_frame': _first_scalar(
            prob,
            [
                'fuselage_x_cg',
                'spajeti_mass.fuselage_x_cg',
                'spajeti_mass.fuselage_mass.fuselage_x_cg',
            ],
            'm',
        )[0],
        'aircraft_x_cg_m_forward_frame': _first_scalar(
            prob,
            [
                'aircraft_x_cg',
                'spajeti_mass.aircraft_x_cg',
                'spajeti_mass.cg.aircraft_x_cg',
            ],
            'm',
        )[0],
        'K_wf': _scalar(prob, 'K_wf'),
        'wing_CL_alpha_per_rad': _scalar(prob, 'wing_CL_alpha'),
        'aft_base_diameter_m': _scalar(prob, 'aft_base_diameter', 'm'),
        'fus_base_width_frac': _scalar(prob, 'fus_base_width_frac'),
        'fus_base_height_frac': _scalar(prob, 'fus_base_height_frac'),
        'engine_mass_kg': _scalar(prob, engine_mass_name, 'kg'),
        'engine_diameter_m': _scalar(prob, engine_diameter_name, 'm'),
        'wing_span_m': _scalar(prob, av.Aircraft.Wing.SPAN, 'm'),
        'wing_area_m2': _scalar(prob, av.Aircraft.Wing.AREA, 'm**2'),
        'vtp_span_m': _scalar(prob, av.Aircraft.VerticalTail.SPAN, 'm'),
        'wing_section_tc': _scalar(prob, 'wing_section_tc'),
    }
    return result


def _write_outputs(results, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / 'fuselage_geometry_comparison.json'
    csv_path = out_dir / 'fuselage_geometry_comparison.csv'
    md_path = out_dir / 'fuselage_geometry_comparison.md'

    json_path.write_text(json.dumps(results, indent=2), encoding='utf-8')

    fieldnames = sorted({key for row in results for key in row})
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    lines = [
        '# Fuselage Geometry Optimization Comparison',
        '',
        '| Metric | Old power-law/fixed base | Current ellipsoid/turbojet base |',
        '|---|---:|---:|',
    ]
    old, current = results
    for key in [
        'range_km',
        'dash_CD0',
        'fuselage_structural_mass_kg',
        'fuselage_wetted_area_m2',
        'fuselage_exposed_wetted_area_m2',
        'fuselage_equivalent_diameter_m',
        'fuselage_fineness_ratio',
        'fuselage_centroid_x_m_aft',
        'aircraft_x_cg_m_forward_frame',
        'K_wf',
        'wing_CL_alpha_per_rad',
        'aft_base_diameter_m',
        'fus_base_width_frac',
        'engine_mass_kg',
        'engine_diameter_m',
    ]:
        lines.append(f'| `{key}` | {_fmt(old.get(key))} | {_fmt(current.get(key))} |')

    lines.extend([
        '',
        'Sources:',
        f'- Range old: `{old.get("range_source")}`; current: `{current.get("range_source")}`',
        f'- Dash CD0 old: `{old.get("dash_CD0_source")}`; current: `{current.get("dash_CD0_source")}`',
        '',
    ])
    md_path.write_text('\n'.join(lines), encoding='utf-8')
    return json_path, csv_path, md_path


def _fmt(value):
    if value is None:
        return 'n/a'
    if isinstance(value, str):
        return value
    return f'{value:.6g}'


def run_scenario(name, scenario):
    with patched_run_constants(scenario):
        start = time.perf_counter()
        prob, _ = run.build_problem()
        with warnings.catch_warnings(), np.errstate(invalid='ignore', over='ignore'):
            warnings.simplefilter('ignore', RuntimeWarning)
            prob.run_aviary_problem()
        elapsed = time.perf_counter() - start
        result = _collect_results(prob, name, scenario, elapsed)
        prob.cleanup()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--out-dir',
        type=Path,
        default=DEFAULT_OUT_DIR,
        help='Directory for JSON/CSV/Markdown comparison outputs.',
    )
    args = parser.parse_args()

    os.environ.setdefault('OPENMDAO_USE_MPI', '0')
    results = []
    for name, scenario in SCENARIOS.items():
        print(f'Running {name}: {scenario["description"]}', flush=True)
        results.append(run_scenario(name, scenario))

    json_path, csv_path, md_path = _write_outputs(results, args.out_dir)
    print(f'Wrote {json_path}')
    print(f'Wrote {csv_path}')
    print(f'Wrote {md_path}')


if __name__ == '__main__':
    main()
