"""Diagnostic: find NaN source in the climb ODE."""
import sys, warnings
warnings.filterwarnings('ignore')

import openmdao.api as om
import aviary.core.aviary_problem as ap
import numpy as np

# Patch run_aviary_problem to stop after run_model and report NaN
_orig_rap = ap.AviaryProblem.run_aviary_problem

def diag_rap(self, *args, **kwargs):
    print('=== DIAGNOSTIC run_model ===')
    try:
        self.run_model()
        print('run_model() PASSED — checking for NaN outputs')
    except Exception as e:
        print(f'run_model() FAILED: {type(e).__name__}: {e}')

    # Walk the model and report NaN/Inf outputs
    found = False
    for comp in self.model.system_iter(include_self=False, recurse=True):
        try:
            for name, val in comp._outputs.items():
                arr = np.asarray(val)
                if np.any(np.isnan(arr)) or np.any(np.isinf(arr)):
                    print(f'  NaN/Inf  {comp.pathname}.{name} = {arr.flat[:4]}')
                    found = True
        except Exception:
            pass
    if not found:
        print('  (no NaN/Inf outputs found)')
    sys.exit(0)

ap.AviaryProblem.run_aviary_problem = diag_rap

# Run the actual script
sys.argv = ['run_horizontal_small_uav.py']
import runpy
runpy.run_path(
    'aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py',
    run_name='__main__',
)
