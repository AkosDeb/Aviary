import unittest

import numpy as np
import openmdao.api as om

from aviary.models.external_subsystems.aeroelasticity.model.flutter import QuasiSteadyFlutterScreen
from aviary.models.external_subsystems.aeroelasticity.model.vtp_inertia import VTPTipInertia
from aviary.models.external_subsystems.aeroelasticity.variables import Aeroelasticity as AE
from aviary.variable_info.variables import Aircraft

# SpaJeti wing baseline (constant throughout sweep)
_WING_AREA = 0.8    # m²
_WING_SPAN = 2.0    # m
_WING_TAPER = 1.0
_ELASTIC_AXIS = 0.375

# SpaJeti VTP (constant throughout sweep)
_VTP_MASS = 0.15    # kg
_VTP_SPAN = 0.30    # m
_VTP_ROOT_CHORD = 0.20  # m
_VTP_TAPER = 1.0
_VTP_SWEEP_DEG = 0.0

# Sweep range from study definition
_OFFSETS = np.linspace(-0.10, 0.20, 7)  # m


def _make_vtp_flutter_problem():
    prob = om.Problem(name='vtp_x_sweep', reports=False)
    prob.model.add_subsystem('vtp', VTPTipInertia(), promotes=['*'])
    prob.model.add_subsystem('flutter', QuasiSteadyFlutterScreen(), promotes=['*'])
    prob.setup()

    prob.set_val(Aircraft.Wing.AREA, _WING_AREA, units='m**2')
    prob.set_val(Aircraft.Wing.SPAN, _WING_SPAN, units='m')
    prob.set_val(Aircraft.Wing.TAPER_RATIO, _WING_TAPER)
    prob.set_val(AE.ELASTIC_AXIS_FRACTION, _ELASTIC_AXIS)

    prob.set_val(Aircraft.VerticalTail.MASS, _VTP_MASS, units='kg')
    prob.set_val(Aircraft.VerticalTail.SPAN, _VTP_SPAN, units='m')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, _VTP_ROOT_CHORD, units='m')
    prob.set_val(Aircraft.VerticalTail.TAPER_RATIO, _VTP_TAPER)
    prob.set_val(Aircraft.VerticalTail.SWEEP, _VTP_SWEEP_DEG, units='deg')

    # Wing structural inputs — held fixed so only VTP x-location varies
    prob.set_val(AE.MASS_PER_UNIT_SPAN, 1.5, units='kg/m')
    prob.set_val(AE.PITCH_INERTIA_PER_UNIT_SPAN, 0.05, units='kg*m')
    prob.set_val(AE.CONTROL_INERTIA_PER_UNIT_SPAN, 0.005, units='kg*m')
    prob.set_val(AE.BENDING_STIFFNESS_PLUNGE, 2.0e5, units='N/m')
    prob.set_val(AE.TORSIONAL_STIFFNESS, 1.5e4, units='N*m/rad')
    prob.set_val(AE.CONTROL_STIFFNESS, 5.0e3, units='N*m/rad')
    prob.set_val(AE.REQUIRED_SPEED, 1.15 * 550.0 / 3.6, units='m/s')
    prob.set_val(AE.FLUTTER_MAX_SPEED, 250.0, units='m/s')

    return prob


class TestVTPXLocationSweep(unittest.TestCase):
    """Sensitivity study: move VTP root LE x-offset and observe aeroelastic response.

    Only the VTP x-location changes; all wing structural properties are held
    constant so the trends are attributable solely to VTP mass placement.

    Expected physics (simplified flutter model):
    - dx_to_elastic_axis increases linearly with offset.
    - pitch_static_unbalance increases linearly with offset (proportional to dx).
    - pitch_inertia_per_unit_span is quadratic in dx; minimum is interior to the sweep.
    - flutter outputs stay finite and physically meaningful across the full range.
    """

    @classmethod
    def setUpClass(cls):
        prob = _make_vtp_flutter_problem()
        cls._results = []
        for x_offset in _OFFSETS:
            prob.set_val(AE.VTP_ROOT_LE_X_OFFSET, x_offset, units='m')
            prob.run_model()
            cls._results.append({
                'offset': x_offset,
                'dx': float(prob.get_val(AE.VTP_DX_TO_ELASTIC_AXIS, units='m')[0]),
                'inertia': float(prob.get_val(AE.VTP_PITCH_INERTIA_PER_UNIT_SPAN, units='kg*m')[0]),
                'static_unbalance': float(prob.get_val(AE.VTP_PITCH_STATIC_UNBALANCE, units='kg')[0]),
                'flutter_speed': float(prob.get_val(AE.FLUTTER_SPEED, units='m/s')[0]),
                'max_eigenvalue': float(prob.get_val(AE.MAX_REAL_EIGENVALUE_AT_DESIGN, units='1/s')[0]),
            })

    def test_dx_to_ea_tracks_offset_monotonically(self):
        dxs = [r['dx'] for r in self._results]
        for i in range(1, len(dxs)):
            self.assertGreater(
                dxs[i], dxs[i - 1],
                msg=f"dx_to_elastic_axis not monotone at offset {self._results[i]['offset']:.3f} m",
            )

    def test_static_unbalance_tracks_offset_monotonically(self):
        su = [r['static_unbalance'] for r in self._results]
        for i in range(1, len(su)):
            self.assertGreater(
                su[i], su[i - 1],
                msg=f"pitch_static_unbalance not monotone at offset {self._results[i]['offset']:.3f} m",
            )

    def test_pitch_inertia_minimum_is_interior_to_sweep(self):
        # Inertia = m_eq*(dx² + z²) is quadratic in dx; minimum occurs when dx ≈ 0,
        # which falls inside the sweep range [-0.10, 0.20] m for this geometry.
        inertias = [r['inertia'] for r in self._results]
        i_min = inertias.index(min(inertias))
        self.assertGreater(i_min, 0, msg="Inertia minimum at leading edge of sweep range")
        self.assertLess(i_min, len(inertias) - 1, msg="Inertia minimum at trailing edge of sweep range")

    def test_flutter_outputs_are_finite_and_positive_across_sweep(self):
        for r in self._results:
            self.assertGreater(
                r['flutter_speed'], 0.0,
                msg=f"flutter_speed non-positive at offset {r['offset']:.3f} m",
            )
            self.assertTrue(
                np.isfinite(r['max_eigenvalue']),
                msg=f"max_real_eigenvalue_at_design not finite at offset {r['offset']:.3f} m",
            )


if __name__ == '__main__':
    unittest.main()
