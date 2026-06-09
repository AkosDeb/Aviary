"""
Mean Aerodynamic Chord geometry for trapezoidal lifting surfaces.

Coordinate frame (body-fixed, origin at nose tip)
---------------------------------------------------
    x  --  fuselage station, positive AFT from nose          [m]
    y  --  butt line,         positive STARBOARD (right)     [m]
    z  --  waterline,         positive DOWN                  [m]

This is the aircraft-station frame: x=0 at the nose tip, x increases
towards the tail so all component positions are positive.  y-right and
z-down are as requested (FLOPS uses z-up; this frame inverts z).

All apex inputs are the root leading-edge position in this frame.

MAC formulas -- linear taper (Anderson 2nd ed., consistent with user image)
---------------------------------------------------------------------------
Given: root chord c_r, taper ratio lambda = c_t/c_r, full span b.

    c_r    = 2 * S / (b * (1 + lambda))            derived from trapezoidal area

    c_mac  = (2/3) * c_r * (1 + lambda + lambda^2) / (1 + lambda)

    y_mac  = (b / 6) * (1 + 2*lambda) / (1 + lambda)   from BL 0 (centreline)

Leading-edge sweep from quarter-chord sweep (standard trapezoidal identity):
    tan(Lambda_LE) = tan(Lambda_c4) + c_r * (1 - lambda) / b

MAC leading-edge and quarter-chord stations:
    x_mac_le = x_apex + y_mac * tan(Lambda_LE)
    z_mac_le = z_apex - y_mac * tan(dihedral)    [minus: up-dihedral lifts MAC above root]
    x_mac_c4 = x_mac_le + c_mac / 4

SpaJeti example (b=1.8m, S=0.45m^2, lambda=0.6, sweep_c4=0deg, dih=3deg, x_apex=0.80m, z_apex=0.0m):
    c_r      = 0.313 m
    c_mac    = 0.255 m
    y_mac    = 0.413 m
    tan_LE   = 0.0694  =>  Lambda_LE = 3.97 deg  (from taper, even at zero c/4 sweep)
    x_mac_le = 0.829 m   (0.80 + 0.413*0.0694)
    z_mac_le = -0.022 m  (above nose datum: dihedral lifts MAC, z is down so z_mac < z_apex)
    x_mac_c4 = 0.893 m   (0.829 + 0.255/4)
"""

import numpy as np
import openmdao.api as om


class MACGeometryComp(om.ExplicitComponent):
    """Mean Aerodynamic Chord geometry for a trapezoidal lifting surface.

    Derives root chord from area, span and taper so MAC position responds
    correctly when span or area changes as a design variable.

    Inputs
    ------
    surface_area     [m^2]  reference area  S = b * c_r * (1 + lambda) / 2
    surface_span     [m]    full span b
    surface_taper    [-]    taper ratio lambda = c_t / c_r
    surface_sweep_c4 [deg]  quarter-chord sweep Lambda_c4
    dihedral_deg     [deg]  dihedral angle (positive = tip UP = tip has lower z)
    x_apex           [m]    x-station of root LE from nose (positive aft)
    z_apex           [m]    z-station of root LE from nose datum (positive down)

    Outputs
    -------
    root_chord  [m]   c_r = 2*S / (b*(1+lambda))
    c_mac       [m]   mean aerodynamic chord
    y_mac       [m]   spanwise BL of MAC from centreline
    x_mac_le    [m]   x-station of MAC leading edge
    z_mac_le    [m]   z-station of MAC leading edge
    x_mac_c4    [m]   x-station of MAC quarter-chord (aerodynamic centre x)
    """

    def setup(self):
        self.add_input('surface_area',     val=0.45,  units='m**2',
                       desc='Reference area S = b * c_r * (1 + lambda) / 2')
        self.add_input('surface_span',     val=1.8,   units='m',
                       desc='Full span b')
        self.add_input('surface_taper',    val=0.6,   units='unitless',
                       desc='Taper ratio lambda = c_tip / c_root')
        self.add_input('surface_sweep_c4', val=0.0,   units='deg',
                       desc='Quarter-chord sweep angle Lambda_c4')
        self.add_input('dihedral_deg',     val=0.0,   units='deg',
                       desc='Dihedral angle; positive = tip UP (z_tip < z_root since z is down)')
        self.add_input('x_apex',           val=0.80,  units='m',
                       desc='x-station of root LE from nose (positive aft)')
        self.add_input('z_apex',           val=0.0,   units='m',
                       desc='z-station of root LE from nose datum (positive down)')

        self.add_output('root_chord', val=0.3125, units='m',
                        desc='Root chord c_r = 2*S / (b * (1 + lambda))')
        self.add_output('c_mac',    val=0.255,  units='m',
                        desc='Mean aerodynamic chord')
        self.add_output('y_mac',    val=0.413,  units='m',
                        desc='Spanwise BL of MAC from centreline (BL 0)')
        self.add_output('x_mac_le', val=0.829,  units='m',
                        desc='x-station of MAC leading edge from nose (positive aft)')
        self.add_output('z_mac_le', val=-0.022, units='m',
                        desc='z-station of MAC leading edge from nose datum (positive down)')
        self.add_output('x_mac_c4', val=0.893,  units='m',
                        desc='x-station of MAC quarter-chord from nose (aerodynamic centre x)')

    def setup_partials(self):
        self.declare_partials('*', '*', method='cs')

    def compute(self, inputs, outputs):
        S      = inputs['surface_area']
        b      = inputs['surface_span']
        lam    = inputs['surface_taper']
        phi_c4 = inputs['surface_sweep_c4'] * (np.pi / 180.0)
        dih    = inputs['dihedral_deg']     * (np.pi / 180.0)
        x_ap   = inputs['x_apex']
        z_ap   = inputs['z_apex']

        # Root chord from trapezoidal planform area
        c_r = 2.0 * S / (b * (1.0 + lam))

        # MAC chord
        c_mac = (2.0 / 3.0) * c_r * (1.0 + lam + lam ** 2) / (1.0 + lam)

        # Spanwise station of MAC from centreline
        y_mac = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)

        # LE sweep from quarter-chord sweep
        # tan(Lambda_LE) = tan(Lambda_c4) + c_r * (1 - lambda) / b
        tan_le = np.tan(phi_c4) + c_r * (1.0 - lam) / b

        # MAC LE station
        # z: positive dihedral lifts tip above root -> z_tip < z_root (z is down)
        x_mac_le = x_ap + y_mac * tan_le
        z_mac_le = z_ap - y_mac * np.tan(dih)

        outputs['root_chord'] = c_r
        outputs['c_mac']      = c_mac
        outputs['y_mac']      = y_mac
        outputs['x_mac_le']   = x_mac_le
        outputs['z_mac_le']   = z_mac_le
        outputs['x_mac_c4']   = x_mac_le + c_mac / 4.0
