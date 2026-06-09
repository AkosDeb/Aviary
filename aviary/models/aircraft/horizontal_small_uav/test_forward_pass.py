"""Forward-pass test for the H-tail load factor chain.

No optimizer needed — just checks that the subsystem graph wires up correctly
and prints Ny, Nz, T/W at the baseline geometry.

Run from the repo root:
    python aviary/models/aircraft/horizontal_small_uav/test_forward_pass.py
"""
import os
import shutil
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).parent))   # so bare imports find phase_info

os.environ.setdefault('OPENMDAO_USE_MPI', '0')

import aviary.api as av
from aviary.subsystems.propulsion.small_turbojet import SmallTurbojetModel

from run_horizontal_small_uav import (
    AIRCRAFT_DATA, OUTPUT_ROOT, OUTPUT_DIR, PROBLEM_NAME,
    EMPTY_MASS_KG, FUEL_CAPACITY_KG, MAX_TAKEOFF_MASS_KG,
    CONSTRAINT_MACH, CONSTRAINT_Q_PA, FUSELAGE_EQUIV_DIAMETER_M,
    VTP_SPAN_INITIAL_M, NY_MIN, NZ_MIN, TW_MIN,
    apply_aircraft_mass_and_fuel_limits,
    add_load_factor_subsystems,
    add_fuel_budget_constraint,
    AVAILABLE_FUEL, FUEL_BUDGET_MARGIN,
)
from phase_info import phase_info

OUTPUT_ROOT.mkdir(exist_ok=True)
if OUTPUT_DIR.exists():
    shutil.rmtree(OUTPUT_DIR)

prob = av.AviaryProblem(
    problem_type=av.ProblemType.FALLOUT,
    verbosity=av.Verbosity.BRIEF,
    name=PROBLEM_NAME,
    work_dir=OUTPUT_ROOT,
)
prob.load_inputs(aircraft_data=AIRCRAFT_DATA, phase_info=phase_info)
prob.load_external_subsystems([SmallTurbojetModel()])
prob.check_and_preprocess_inputs()
apply_aircraft_mass_and_fuel_limits(prob)

prob.add_pre_mission_systems()
add_load_factor_subsystems(prob)
prob.add_phases()
prob.add_post_mission_systems()
add_fuel_budget_constraint(prob)
prob.link_phases()

# No driver — just run a single model evaluation
prob.add_design_variables()
prob.add_objective()
prob.setup()
prob.set_initial_guesses()
prob.set_val(av.Aircraft.Design.EMPTY_MASS, EMPTY_MASS_KG, 'kg')
prob.set_val(av.Aircraft.Design.GROSS_MASS, MAX_TAKEOFF_MASS_KG, 'kg')
prob.set_val(av.Aircraft.VerticalTail.SPAN, VTP_SPAN_INITIAL_M, 'm')
prob.set_val('wing_polhamus.mach',              CONSTRAINT_MACH)
prob.set_val('wing_polhamus.section_lift_slope', 2.0 * np.pi)
prob.set_val('wing_polhamus.fuselage_diameter', FUSELAGE_EQUIV_DIAMETER_M)
prob.set_val('vtp_polhamus.mach',               CONSTRAINT_MACH)
prob.set_val('vtp_polhamus.section_lift_slope',  2.0 * np.pi)
prob.set_val('vtp_polhamus.fuselage_diameter',   0.0)
prob.set_val('long_load.alpha_max_deg',          12.0)

prob.run_model()

print('\n' + '=' * 60)
print('FORWARD PASS — baseline geometry')
print('=' * 60)

def p(label, val, unit=''):
    print(f'  {label:<30} = {val:>10.4f}  {unit}')

p('Wing span',          prob.get_val(av.Aircraft.Wing.SPAN,             'm')[0],      'm')
p('Wing AR',            prob.get_val(av.Aircraft.Wing.ASPECT_RATIO)[0])
p('Wing AR_eff',        prob.get_val('AR_eff')[0])
p('Wing k_h',           prob.get_val('k_h')[0])
p('Wing K_wf',          prob.get_val('K_wf')[0])
p('Wing CL_alpha',      prob.get_val('wing_CL_alpha')[0],               '/rad')
p('VTP span',           prob.get_val(av.Aircraft.VerticalTail.SPAN,     'm')[0],      'm')
p('VTP area',           prob.get_val(av.Aircraft.VerticalTail.AREA,     'm**2')[0],   'm²')
p('VTP AR',             prob.get_val(av.Aircraft.VerticalTail.ASPECT_RATIO)[0])
p('VTP CL_alpha_v',     prob.get_val('CL_alpha_v')[0],                  '/rad')
p('CY_beta_vtp',        prob.get_val('CY_beta_vtp')[0],                 '/rad')
p('CY_delta_r',         prob.get_val('CY_delta_r')[0],                  '/rad')
print()
ny  = prob.get_val('Ny')[0]
nz  = prob.get_val('Nz')[0]
sls = prob.get_val(av.Aircraft.Engine.SCALED_SLS_THRUST, 'N')[0]
tw  = sls / (MAX_TAKEOFF_MASS_KG * 9.80665)
p('Ny  (lateral)',  ny,  f'[min {NY_MIN}]  {"OK" if ny  >= NY_MIN  else "VIOLATION"}')
p('Nz  (vertical)', nz,  f'[min {NZ_MIN}]  {"OK" if nz  >= NZ_MIN  else "VIOLATION"}')
p('T/W at SLS',     tw,  f'[min {TW_MIN}]  {"OK" if tw  >= TW_MIN  else "VIOLATION"}')
print('=' * 60)

prob.cleanup()
