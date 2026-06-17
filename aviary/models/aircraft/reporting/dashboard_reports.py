"""Dashboard and HTML report generation helpers for aircraft examples."""

import csv
import math
from pathlib import Path

import numpy as np

import aviary.api as av
from aviary.subsystems.geometry.flops_based.superellipse_fuselage import (
    superellipse_width_height_distribution,
    _superellipse_surface_points,
)
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetVariables
from aviary.utils.print_utils import safe_get

_CONTEXT_READY = False


def configure_dashboard_context(config_module):
    """Load aircraft-specific constants used by the dashboard report writers."""
    global _CONTEXT_READY
    for name in dir(config_module):
        if name.isupper():
            globals()[name] = getattr(config_module, name)
    _CONTEXT_READY = True


def _require_context():
    if not _CONTEXT_READY:
        raise RuntimeError(
            'dashboard_reports.configure_dashboard_context(config_module) must '
            'be called before writing aircraft-specific dashboard reports.'
        )


def premission_propulsion_var(name):
    return f'pre_mission.propulsion.{name}'

def write_payload_range_report(prob):
    _require_context()
    reports_dir = Path(prob.get_reports_dir(force=True))
    csv_path = reports_dir / 'payload_range_data.csv'

    payload_kg = safe_get(prob, av.Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 'kg')
    fuel_kg = safe_get(prob, av.Mission.TOTAL_FUEL, 'kg')
    range_km = safe_get(prob, av.Mission.RANGE, 'km')

    if any(isinstance(value, str) for value in (payload_kg, fuel_kg, range_km)):
        return None

    rows = [
        {
            'Mission Name': 'Zero Range',
            'Payload (kg)': payload_kg,
            'Fuel (kg)': 0.0,
            'Range (km)': 0.0,
        },
        {
            'Mission Name': 'Fallout Mission',
            'Payload (kg)': payload_kg,
            'Fuel (kg)': fuel_kg,
            'Range (km)': range_km,
        },
    ]

    with csv_path.open('w', newline='') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                'Mission Name',
                'Payload (kg)',
                'Fuel (kg)',
                'Range (km)',
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    return csv_path


def write_spajeti_aircraft_3d_report(prob):
    """Write a lightweight SpaJeti-specific 3D geometry report for the dashboard."""
    _require_context()
    reports_dir = Path(prob.get_reports_dir(force=True))
    html_path = reports_dir / 'spajeti_aircraft_3d.html'
    subsystems_dir = reports_dir / 'subsystems'
    subsystem_link_path = subsystems_dir / 'spajeti_3d.md'

    def _num(value, default):
        return default if isinstance(value, str) else float(np.atleast_1d(value)[0])

    fus_len = _num(safe_get(prob, av.Aircraft.Fuselage.LENGTH, 'm'), 2.0)
    fus_w = _num(safe_get(prob, 'fus_max_width', 'm'), FUSELAGE_MAX_WIDTH_M)
    fus_h = _num(safe_get(prob, 'fus_max_height', 'm'), FUSELAGE_MAX_HEIGHT_M)

    wing_span = _num(safe_get(prob, av.Aircraft.Wing.SPAN, 'm'), 1.8)
    wing_area = _num(safe_get(prob, av.Aircraft.Wing.AREA, 'm**2'), 0.45)
    wing_taper = _num(safe_get(prob, av.Aircraft.Wing.TAPER_RATIO), 0.6)
    wing_sweep = math.radians(_num(safe_get(prob, av.Aircraft.Wing.SWEEP, 'deg'), 0.0))
    wing_dihedral = math.radians(_num(safe_get(prob, av.Aircraft.Wing.DIHEDRAL, 'deg'), 0.0))
    wing_cr = 2.0 * wing_area / (wing_span * (1.0 + wing_taper))
    wing_ct = wing_cr * wing_taper
    wing_semispan = 0.5 * wing_span

    vtp_span = _num(safe_get(prob, av.Aircraft.VerticalTail.SPAN, 'm'), VTP_SPAN_INITIAL_M)
    vtp_area = _num(safe_get(prob, av.Aircraft.VerticalTail.AREA, 'm**2'), 0.08)
    vtp_taper = _num(safe_get(prob, av.Aircraft.VerticalTail.TAPER_RATIO), 0.4)
    vtp_sweep = math.radians(_num(safe_get(prob, av.Aircraft.VerticalTail.SWEEP, 'deg'), 20.0))
    vtp_cr = 2.0 * vtp_area / (vtp_span * (1.0 + vtp_taper))
    vtp_ct = vtp_cr * vtp_taper

    engine_diameter = _num(
        safe_get(prob, premission_propulsion_var(SmallTurbojetVariables.DIAMETER), 'm'),
        0.12,
    )
    engine_length = 0.35 * engine_diameter

    def p(x_aft, y_right, z_down):
        # A-Frame: x = aircraft aft/forward, y = up, z = right.
        return f'{x_aft - 0.5 * fus_len:.5f} {-z_down:.5f} {y_right:.5f}'

    def tri(a, b, c, color, opacity='0.82'):
        return (
            f'<a-triangle vertex-a="{a}" vertex-b="{b}" vertex-c="{c}" '
            f'color="{color}" opacity="{opacity}" side="double"></a-triangle>'
        )

    def quad(a, b, c, d, color, opacity='0.82'):
        return tri(a, b, c, color, opacity) + '\n' + tri(a, c, d, color, opacity)

    def fuselage_entities(n_x=18, n_t=24):
        x_norm = np.linspace(0.0, 1.0, n_x)
        widths, heights = superellipse_width_height_distribution(
            x_norm,
            fus_w,
            fus_h,
            FUSELAGE_NOSE_LENGTH_FRACTION,
            FUSELAGE_TAIL_LENGTH_FRACTION,
            FUSELAGE_BASE_WIDTH_FRACTION,
            FUSELAGE_BASE_HEIGHT_FRACTION,
        )
        theta = np.linspace(0.0, 2.0 * np.pi, n_t, endpoint=False)
        rings = []
        for x_frac, width, height in zip(x_norm, widths, heights):
            y_vals, z_vals = _superellipse_surface_points(
                width,
                height,
                FUSELAGE_SUPERELLIPSE_EXPONENT,
                theta,
            )
            rings.append([p(x_frac * fus_len, y, z_down) for y, z_down in zip(y_vals, z_vals)])

        faces = []
        for i in range(n_x - 1):
            for j in range(n_t):
                jp = (j + 1) % n_t
                faces.append(quad(
                    rings[i][j],
                    rings[i + 1][j],
                    rings[i + 1][jp],
                    rings[i][jp],
                    '#c9ced6',
                    '0.92',
                ))
        return '\n'.join(faces)

    def wing_panel(sign):
        y0 = 0.0
        yt = sign * wing_semispan
        x_le_root = WING_X_APEX_M
        z_root = WING_Z_APEX_M
        x_le_tip = x_le_root + 0.25 * wing_cr + abs(yt) * math.tan(wing_sweep) - 0.25 * wing_ct
        z_tip = z_root - abs(yt) * math.tan(wing_dihedral)
        return quad(
            p(x_le_root, y0, z_root),
            p(x_le_root + wing_cr, y0, z_root),
            p(x_le_tip + wing_ct, yt, z_tip),
            p(x_le_tip, yt, z_tip),
            '#3b82f6',
        )

    def vtp_panel(y_sign, z_sign):
        y = y_sign * wing_semispan
        x_le_root = WING_X_APEX_M + 0.25 * wing_cr + wing_semispan * math.tan(wing_sweep) - 0.25 * wing_ct
        z_mid = WING_Z_APEX_M - wing_semispan * math.tan(wing_dihedral)
        z_tip = z_mid + z_sign * 0.5 * vtp_span
        x_le_tip = x_le_root + 0.25 * vtp_cr + 0.5 * vtp_span * math.tan(vtp_sweep) - 0.25 * vtp_ct
        return quad(
            p(x_le_root, y, z_mid),
            p(x_le_root + vtp_cr, y, z_mid),
            p(x_le_tip + vtp_ct, y, z_tip),
            p(x_le_tip, y, z_tip),
            '#0f766e',
            '0.88',
        )

    entities = '\n'.join([
        fuselage_entities(),
        wing_panel(1.0),
        wing_panel(-1.0),
        vtp_panel(1.0, 1.0),
        vtp_panel(1.0, -1.0),
        vtp_panel(-1.0, 1.0),
        vtp_panel(-1.0, -1.0),
        '<a-cylinder radius="{:.5f}" height="{:.5f}" rotation="0 0 90" '
        'position="{}" color="#111827" opacity="0.90" segments-radial="32"></a-cylinder>'.format(
            0.42 * engine_diameter,
            engine_length,
            p(fus_len + 0.5 * engine_length, 0.0, 0.0),
        ),
    ])

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>SpaJeti 3D Geometry</title>
  <script src="https://aframe.io/releases/1.5.0/aframe.min.js"></script>
  <style>
    body {{ margin: 0; background: #eef3f8; font-family: Arial, sans-serif; }}
    .note {{
      position: fixed; left: 16px; top: 12px; z-index: 10;
      background: rgba(255,255,255,0.88); padding: 10px 12px; border-radius: 6px;
      color: #1f2937; font-size: 13px; line-height: 1.35;
    }}
    .fallback {{
      position: fixed; right: 16px; bottom: 16px; z-index: 10;
      width: 300px; max-width: 34vw;
      background: rgba(255,255,255,0.88); padding: 8px; border-radius: 6px;
      color: #1f2937; font-size: 12px;
    }}
    .fallback svg {{ display: block; width: 100%; height: auto; }}
  </style>
</head>
<body>
  <div class="note">
    <b>SpaJeti optimized geometry</b><br>
    b = {wing_span:.3f} m, VTP span = {vtp_span:.3f} m, fuselage = {fus_len:.3f} m<br>
    Fuselage uses the rounded-square superellipse mesh. VTPs mirror up/down from each wingtip.
    Dark aft disk is the small turbojet nozzle.
  </div>
  <div class="fallback">
    <b>Plan view fallback</b>
    <svg viewBox="0 0 300 150" aria-label="SpaJeti H-wing planform fallback">
      <rect x="0" y="0" width="300" height="150" fill="#dbeafe"/>
      <ellipse cx="150" cy="75" rx="58" ry="20" fill="#c9ced6" stroke="#94a3b8"/>
      <polygon points="90,70 210,70 238,62 238,68 210,78 90,78 62,68 62,62" fill="#3b82f6" opacity="0.82"/>
      <polygon points="56,51 68,51 74,75 68,99 56,99 50,75" fill="#0f766e" opacity="0.88"/>
      <polygon points="232,51 244,51 250,75 244,99 232,99 226,75" fill="#0f766e" opacity="0.88"/>
      <circle cx="211" cy="75" r="5" fill="#111827" opacity="0.90"/>
    </svg>
  </div>
  <a-scene background="color: #dbeafe">
    <a-camera position="0 1.8 7.5" rotation="-12 0 0"></a-camera>
    <a-entity light="type: ambient; intensity: 0.62"></a-entity>
    <a-entity light="type: directional; intensity: 0.72" position="-2 4 3"></a-entity>
    <a-entity id="aircraft" rotation="0 -35 0">
{entities}
    </a-entity>
    <a-grid color="#94a3b8" opacity="0.35" position="0 -0.45 0"></a-grid>
  </a-scene>
</body>
</html>
"""
    html_path.write_text(html, encoding='utf-8')
    subsystems_dir.mkdir(parents=True, exist_ok=True)
    subsystem_link_path.write_text(
        '# SpaJeti 3D Geometry\n\n'
        'The SpaJeti H-wing geometry report is also available as a standalone '
        'interactive HTML file:\n\n'
        '[Open SpaJeti 3D Geometry](../spajeti_aircraft_3d.html)\n\n'
        'If the dashboard was launched with an older Aviary entry point and the '
        'Results tab is missing, use this subsystem report link or open '
        '`reports/spajeti_aircraft_3d.html` directly.\n',
        encoding='utf-8',
    )
    return html_path



