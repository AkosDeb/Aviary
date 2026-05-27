# MDAO with OpenMDAO — Detail Reference

## What is MDAO?

**Multidisciplinary Design, Analysis, and Optimization (MDAO)** is the practice of optimizing a system where multiple disciplines (aerodynamics, structures, propulsion, trajectory) are coupled — meaning the output of one feeds the input of another.

The challenge: if you optimize each discipline separately you miss cross-discipline trade-offs (e.g. a heavier wing that allows a better cruise altitude that saves more fuel than the extra weight costs). MDAO solves all disciplines simultaneously as one coupled system.

OpenMDAO is NASA's open-source MDAO framework that makes this tractable in Python.

---

## 1. Core Concept — Components and Data Flow

Everything in OpenMDAO is a **Component** or a **Group** of components.

```
Group
 ├── Component A  (inputs → computes → outputs)
 ├── Component B  (takes output of A as input)
 └── Component C  (takes outputs of A and B)
```

Each component declares:
- `setup()` — what inputs and outputs it has, with units
- `compute()` — how to compute outputs from inputs
- `compute_partials()` — analytic derivatives of outputs w.r.t. inputs

Example:
```python
class WingMass(om.ExplicitComponent):
    def setup(self):
        self.add_input('wing_area', units='m**2')
        self.add_input('wing_span', units='m')
        self.add_output('wing_mass', units='kg')

    def compute(self, inputs, outputs):
        S = inputs['wing_area']
        b = inputs['wing_span']
        outputs['wing_mass'] = 3.2 * S**0.6 * b**0.4

    def compute_partials(self, inputs, J):
        S = inputs['wing_area']
        b = inputs['wing_span']
        J['wing_mass', 'wing_area'] = 3.2 * 0.6 * S**-0.4 * b**0.4
        J['wing_mass', 'wing_span'] = 3.2 * S**0.6 * 0.4 * b**-0.6
```

---

## 2. Why OpenMDAO is Faster Than Standard Optimization

### The standard approach (finite differences)

To compute the gradient of objective `f` w.r.t. design variable `x`:
```
∂f/∂x ≈ [f(x + δ) − f(x)] / δ
```

For `N` design variables this requires `N` extra model evaluations per iteration.
An aircraft model with 50 design variables = **50 full aircraft evaluations per optimizer step**.

Problems:
- Expensive (scales linearly with number of design variables)
- Inaccurate (step size δ is a compromise between truncation and round-off error)
- Completely breaks at implicit solver boundaries (e.g. iterative Newton loops inside a component)

### The OpenMDAO approach (MAUD — unified analytic derivatives)

OpenMDAO uses the **Modular Analysis and Unified Derivatives (MAUD)** architecture.

Each component provides its own partial derivatives analytically. OpenMDAO assembles them into one global **unified derivative equation**:

```
[dR/dx] · [dx/dp] = −[dR/dp]
```

Where:
- `R` = residuals of the entire coupled system
- `x` = all state/output variables
- `p` = design parameters (what the optimizer controls)

This is solved as a **single linear system** — one linear solve gives you the gradient w.r.t. all design variables at once.

### Forward vs. Adjoint mode

| Mode | Cost | Best when |
|---|---|---|
| **Forward (direct)** | One solve per design variable | Few design vars, many outputs |
| **Adjoint (reverse)** | One solve per objective/constraint | Many design vars, few objectives ← typical aircraft case |

Aircraft optimization usually has 10–100+ design variables but only 1–2 objectives (minimize fuel, maximize range). Adjoint mode costs the same regardless of how many design variables you add.

### Derivative coloring (sparsity)

Most outputs don't depend on most inputs. OpenMDAO uses **graph coloring** to identify the sparse structure of the Jacobian and only computes non-zero entries. In practice this reduces derivative cost by **10–100×** for large models.

### Summary comparison

| | Finite Differences | OpenMDAO (MAUD + adjoint) |
|---|---|---|
| Gradient cost | O(N) model runs | O(1) linear solves |
| Accuracy | Step-size limited | Machine precision |
| Scales with design vars | Poorly | Designed for 100s–1000s |
| Implicit solvers inside model | Breaks | Handled natively |
| Coupled disciplines | Manual iteration | Automatic via Newton solver |

---

## 3. Solvers

Before the optimizer runs, OpenMDAO must solve the coupled system (all disciplines consistent with each other). Two solver types:

### Nonlinear solvers (converge the physics)

| Solver | When to use |
|---|---|
| `NewtonSolver` | Tightly coupled systems, fastest when analytic partials are available |
| `NonlinearBlockGS` | Loosely coupled, simpler to set up, slower convergence |

```python
prob.model.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
prob.model.nonlinear_solver.options['maxiter'] = 30
prob.model.nonlinear_solver.options['atol'] = 1e-8
```

### Linear solvers (solve the derivative equation)

| Solver | When to use |
|---|---|
| `DirectSolver` | Small/medium models — factorizes the full Jacobian |
| `LinearBlockGS` | Large sparse models — iterative, cheaper per iteration |
| `PETScKrylov` | Large parallel models — uses Krylov subspace methods |

```python
prob.model.linear_solver = om.DirectSolver()
```

---

## 4. Optimization Algorithms (Drivers)

OpenMDAO wraps external optimizers via `pyoptsparse`. You set the driver before calling `setup()`.

### IPOPT — Interior Point OPTimizer
```python
prob.driver = om.pyOptSparseDriver()
prob.driver.options['optimizer'] = 'IPOPT'
prob.driver.opt_settings['max_iter'] = 500
prob.driver.opt_settings['tol'] = 1e-6
prob.driver.opt_settings['print_level'] = 5        # 0=silent, 5=detailed
prob.driver.opt_settings['mu_strategy'] = 'adaptive'
prob.driver.opt_settings['bound_push'] = 1e-2      # keeps vars away from bounds early on
```

- Best for: large, constrained, nonlinear problems (typical Aviary run)
- Interior-point method: adds a barrier term to keep solution inside bounds
- Scales well with many variables and constraints
- Requires `pyoptsparse`

### SLSQP — Sequential Least Squares Quadratic Programming
```python
prob.driver = om.ScipyOptimizeDriver()
prob.driver.options['optimizer'] = 'SLSQP'
prob.driver.options['tol'] = 1e-6
prob.driver.options['maxiter'] = 200
```

- Built into `scipy` — no extra install needed
- Good for small/medium problems
- Gradient-based, handles equality and inequality constraints
- Slower than IPOPT for large models

### SNOPT — Sparse Nonlinear OPTimizer
```python
prob.driver.options['optimizer'] = 'SNOPT'
prob.driver.opt_settings['Major iterations limit'] = 200
prob.driver.opt_settings['Major optimality tolerance'] = 1e-6
prob.driver.opt_settings['Major feasibility tolerance'] = 1e-6
```

- Commercial (license required)
- Very robust for ill-conditioned problems
- Uses an active-set SQP method
- Good alternative to IPOPT when IPOPT struggles to converge

### DOE / Design of Experiments (non-gradient)
```python
prob.driver = om.DOEDriver(om.FullFactorialGenerator(levels=3))
```

- Runs the model at a grid of design variable combinations
- No optimization — used for sensitivity studies and surrogate model building
- Useful before setting up the real optimization to understand design space

### Choosing an optimizer

| Situation | Recommendation |
|---|---|
| Standard Aviary run, constrained | IPOPT |
| Quick test, no pyoptsparse | SLSQP |
| Convergence issues with IPOPT | SNOPT |
| Mapping the design space first | DOE |

---

## 5. Design Variables, Constraints, Objectives

```python
# Design variable — what the optimizer can change
prob.model.add_design_var(
    Aircraft.Wing.SPAN,
    lower=1.0, upper=3.0,
    units='m',
    ref=1.8,        # scaling reference — keep near 1.0 for optimizer numerics
)

# Constraint — rule that must be satisfied
prob.model.add_constraint(
    'fuel_budget_margin',
    lower=0.0,
    units='kg',
    ref=5.0,
)

# Objective — what to minimize (use negative for maximization)
prob.model.add_objective(
    Mission.Objectives.FUEL,   # minimize fuel burned
    ref=10.0,
)
```

**Scaling matters.** The optimizer works best when all variables and constraints are order-of-magnitude 1.0. Use `ref` (and `ref0` for lower bound) to scale. Poorly scaled problems cause slow convergence or failure.

---

## 6. Outputs and Reports

After a run Aviary writes an output folder (e.g. `outputs/run_my_uav_out/`). OpenMDAO generates several files inside.

### N2 Diagram
```
n2.html
```
An interactive HTML diagram showing every component, every variable connection, and data flow through the model. Open in a browser.

- Diagonal = components
- Off-diagonal = data connections between components
- Color coding shows solver groupings
- Hover to see variable names and values
- **Use this first when debugging** — missing connections and wrong variable paths are immediately visible

```powershell
openmdao n2 run_my_uav.py   # generate standalone N2 without running
```

### Optimizer Output (`opt_report.html`)
- Iteration history: objective value, constraint violations, step size per iteration
- Shows whether the optimizer converged, stalled, or hit iteration limit
- Lists active constraints at the final solution

### IPOPT/SNOPT log (`IPOPT.out` / `SNOPT_print.out`)
- Line-by-line solver output
- Key columns: `iter`, `objective`, `inf_pr` (primal infeasibility = constraint violation), `inf_du` (dual infeasibility = optimality condition)
- Converged when both `inf_pr` and `inf_du` drop below tolerance

### Aviary Dashboard
```powershell
aviary dashboard outputs/run_my_uav_out
```

Opens a web dashboard with:
- Mission trajectory plots (altitude, Mach, fuel burn vs. time)
- Mass breakdown (pie chart of component weights)
- Aerodynamic polars
- Subsystem reports
- Link to N2

### Timeseries outputs (Dymos)
```python
phase.add_timeseries_output('aero.CL', output_name='CL', units='unitless')
```
Adds any internal variable to the recorded trajectory. Accessible in results:
```python
CL = prob.get_val('traj.cruise.timeseries.CL')
```

### Case recorder (detailed data log)
```python
recorder = om.SqliteRecorder('cases.sql')
prob.add_recorder(recorder)
prob.driver.add_recorder(recorder)
```

Records every variable at every iteration to a SQLite file. Read back with:
```python
cr = om.CaseReader('cases.sql')
cases = cr.list_cases('root')
case = cr.get_case(cases[-1])
val = case['aircraft:wing:mass']
```

### Check partials report
```python
data = prob.check_partials(compact_print=True)
```
Compares your analytic `compute_partials` against finite-difference estimates. Use this during development of custom components to verify derivatives are correct before handing the model to the optimizer.

---

## 7. Debugging Workflow

```
1. Run fails to converge
   → Open IPOPT.out — check inf_pr and inf_du trends
   → If inf_pr stays high: constraints are infeasible, check bounds and initial guesses

2. Weird results / optimizer goes to boundary
   → Open N2 — check all connections are wired correctly
   → Check scaling (ref values)

3. Slow convergence
   → Run check_partials — bad analytic derivatives destroy convergence
   → Check linear solver tolerance (too loose = bad gradients)

4. Component giving NaN
   → Add print statements inside compute()
   → Use prob.check_totals() to isolate which component breaks
```

---

## Sources

- [OpenMDAO Documentation](https://openmdao.org/newdocs/versions/latest/main.html)
- [MAUD Architecture Paper](https://link.springer.com/article/10.1007/s00158-019-02211-z)
- [Dymos: Why Dymos?](https://openmdao.github.io/dymos/#why-dymos)
- [Aviary: Optimization Algorithms](https://openmdao.github.io/Aviary/theory_guide/optimization_algorithms.html)
- [pyoptsparse](https://mdolab-pyoptsparse.readthedocs-hosted.com/en/latest/)
