import aviary.api as av
from aviary.models.missions.energy_state_default import phase_info

# Suppress outputs by setting verbosity as zero (quiet mode)
prob = av.AviaryProblem(verbosity=1)

# Load aircraft and options data from provided sources
prob.load_inputs('models/aircraft/test_aircraft/aircraft_for_bench_FwFm.csv', phase_info)

# Sanity check inputs and guess initial conditions for mission phases
prob.check_and_preprocess_inputs()

# Have Aviary build the OpenMDAO model with pre-mission, mission, and post-mission components
prob.build_model()

# Selecting optimizer and iteration limit are optional
prob.add_driver('SLSQP', max_iter=20)

# Add the default design variables needed to size the aircraft
prob.add_design_variables()

# Add wing area and engine scaling as additional design variables
prob.model.add_design_var(av.Aircraft.Engine.SCALE_FACTOR, lower=0.8, upper=1.2, ref=1)
prob.model.add_design_var(av.Aircraft.Wing.AREA, lower=1200, upper=1800, units='ft**2', ref=1400)

# Add the default objective function (minimum fuel burn)
prob.add_objective()

# Add constraints for wing loading and thrust-to-weight ratio
prob.model.add_constraint(av.Aircraft.Design.WING_LOADING, lower=120, units='lbf/ft**2')
prob.model.add_constraint(av.Aircraft.Design.THRUST_TO_WEIGHT_RATIO, lower=0.35)

# Standard OpenMDAO problem setup step
prob.setup()

prob.run_model()
data = prob.check_totals(
    of=[av.Mission.RANGE],
    wrt=[av.Aircraft.Wing.SPAN],
    compact_print=True,
)

# Run the optimization problem
prob.run_aviary_problem()