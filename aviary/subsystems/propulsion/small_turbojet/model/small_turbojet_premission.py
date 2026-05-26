import openmdao.api as om

from aviary.subsystems.propulsion.small_turbojet.model.max_diameter import MaxDiameter
from aviary.subsystems.propulsion.small_turbojet.model.max_rpm import MaxRPM
from aviary.subsystems.propulsion.small_turbojet.model.max_weight import MaxWeight
from aviary.subsystems.propulsion.small_turbojet.model.sfc import SFC
from aviary.variable_info.variables import Aircraft


class SmallTurbojetPreMission(om.Group):
    """
    Static design regressions for the small turbojet.

    Max SLS thrust is the single design input. The pre-mission chain computes:
        thrust  →  diameter  →  max_rpm  →  mass  →  sfc

    All components are promoted with '*' so variables are shared by name.
    """

    def setup(self):
        design_vars = om.IndepVarComp()
        design_vars.add_output(Aircraft.Engine.SCALED_SLS_THRUST, val=394.0, units='N')
        self.add_subsystem('design_vars', design_vars, promotes=['*'])

        self.add_subsystem('max_diameter', MaxDiameter(), promotes=['*'])
        self.add_subsystem('max_rpm',      MaxRPM(),      promotes=['*'])
        self.add_subsystem('max_weight',   MaxWeight(),   promotes=['*'])
        self.add_subsystem('sfc',          SFC(),         promotes=['*'])
