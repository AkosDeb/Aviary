import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.model.max_rpm import MaxRPM
from aviary.subsystems.propulsion.small_turbojet.model.max_thrust import MaxThrust
from aviary.subsystems.propulsion.small_turbojet.model.max_weight import MaxWeight
from aviary.subsystems.propulsion.small_turbojet.model.sfc import SFC


class SmallTurbojetPreMission(om.Group):
    """
    Static design regressions for the small turbojet.

    Diameter and length are the design inputs. The pre-mission chain computes
    max RPM, mass, rated sea-level static thrust, and SFC. These values are
    treated as constant during the current simplified mission model.
    """

    def setup(self):
        self.add_subsystem('max_rpm', MaxRPM(), promotes=['*'])
        self.add_subsystem('max_weight', MaxWeight(), promotes=['*'])
        self.add_subsystem('max_thrust', MaxThrust(), promotes=['*'])
        self.add_subsystem('sfc', SFC(), promotes=['*'])
