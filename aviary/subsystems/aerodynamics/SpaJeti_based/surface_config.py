"""
SurfaceConfig -- constructor-time configuration for a LiftingSurfaceGroup.

Bundles airfoil data with role flags so that WingSurface, VTPSurface, and
future surface types (HTPSurface, CanardSurface, ...) can all be instantiated
from a single immutable config object.
"""

from dataclasses import dataclass

from aviary.subsystems.aerodynamics.SpaJeti_based.airfoil_data import AirfoilData


@dataclass(frozen=True)
class SurfaceConfig:
    """Immutable constructor-time configuration for a LiftingSurfaceGroup.

    Parameters
    ----------
    name : str
        Human-readable label used in print output and N2 descriptions.
    airfoil : AirfoilData
        Section aerodynamic properties.  ``cl_alpha_per_rad`` feeds
        ``LiftCurveSlopePolhamus`` via the ``AirfoilConstantsComp`` inside the Group.
    endplate_correction : bool
        True  -> add ``ScholzWingletARCorrection`` upstream of Polhamus
                 (wing with VTP endplates).
        False -> AR_eff = geometric AR, k_eff = 1.0 (standard isolated surface).
    k_wl : float
        Winglet effectiveness penalty for the Scholz correction.
        Only used when ``endplate_correction=True``.
        2.45 = experimental average (Dubs/Zimmer); 2.0 = theoretical optimum.
    split_penalty : float
        Symmetric split-winglet penalty.  Only used when ``endplate_correction=True``.
        0.90 = H-tail symmetric VTPs produce ~90 % of a standard winglet's benefit.
    has_control_surface : bool
        True  -> wire ``CyDeltaRudder`` for rudder effectiveness (``CY_delta_r`` output).
        False -> no ``CY_delta_r`` output.
    fuselage_diameter : float
        Equivalent fuselage diameter [m] for the K_wf correction in Polhamus.
        0.0 disables the correction (K_wf = 1.0).  Used by WingSurface only.
    """

    name:                str
    airfoil:             AirfoilData
    endplate_correction: bool  = False
    k_wl:                float = 2.45
    split_penalty:       float = 0.90
    has_control_surface: bool  = False
    fuselage_diameter:   float = 0.0
