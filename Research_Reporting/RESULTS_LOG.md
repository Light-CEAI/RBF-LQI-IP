# Results Log

**Status:** Living evidence register. No numerical result is accepted without execution evidence.

## How to use this file

Add a dated entry for each executed test. Preserve failed tests and unexpected results; do not overwrite them. Link the exact code commit and parameter-set version. Attach plots or raw data in a clearly named subfolder when available.

## Test record template

### [TEST-ID] — [Test name]

- **Execution date/time:**
- **Research branch and commit:**
- **Script and version:**
- **MATLAB release/environment:**
- **Parameter-set ID and source:**
- **Purpose / hypothesis:**
- **Test conditions:** initial state, disturbance, uncertainty, actuator limits, simulation horizon, solver and sample time.
- **Expected result / acceptance criterion:**
- **Actual output:** paste unedited command-window output or link raw data.
- **Figures/data files:**
- **Pass/fail/inconclusive:**
- **Observed anomalies:**
- **Interpretation:**
- **Limitations:**
- **Follow-up action:**

---

## Current status

| Test ID | Test | Status | Evidence |
|---|---|---|---|
| V-001 | Nonlinear equations, inertia positive definiteness and upright finite-difference linearisation | **Passed for illustrative parameter set** | `Analyzed_System/verification/verify_nonlinear_model.m`; user-supplied MATLAB output recorded below |
| V-002 | Time-varying attached-body Euler–Lagrange identity, inertia condition and constant-parameter limit | **Passed for illustrative smooth trajectory** | `Analyzed_System/verification/verify_time_varying_lagrange_model.m`; user-supplied MATLAB output recorded below |

## Accepted results

### V-001 — Nonlinear model algebraic and numerical consistency

- **Execution date/time:** Reported by user on 2026-11-04; exact execution time and MATLAB release not supplied.
- **Research branch and commit:** `analyzed-system`; verification script commit `97d195cf3c3d1cbbeff13ee866332a08370409ba`.
- **Script and version:** `Analyzed_System/verification/verify_nonlinear_model.m`.
- **MATLAB release/environment:** MATLAB environment not specified.
- **Parameter-set ID and source:** Illustrative verification set embedded in the script; not a validated hardware parameter set.
- **Purpose / hypothesis:** Check positive definiteness of the cart-pendulum inertia matrix over θ ∈ [-π,π] and verify analytical upright-equilibrium A, B and E Jacobians against finite differences.
- **Test conditions:** Full angular interval [-π,π]; illustrative constant parameters. No hardware, time-varying payload, actuator or closed-loop controller test.
- **Actual output (as reported):**
  - D(0) = 0.009675
  - Minimum det(H), θ ∈ [-π,π] = 0.009675
  - Minimum eig(H), θ ∈ [-π,π] = 0.0148119676429
  - Upright Jacobian error A = 1.114e-11
  - Upright Jacobian error B = 2.220e-16
  - Upright Jacobian error E = 2.220e-16
  - Inertia positive definite = PASS
  - Linearisation finite-diff = PASS
  - MATLAB returned `passedInertia = 1` and `passedLinearisation = 1`.
- **Pass/fail/inconclusive:** **Pass**, restricted to the two implemented checks and the illustrative parameter set.
- **Observed anomalies:** None reported.
- **Interpretation:** The computed inertia matrix remained positive definite over the tested angular interval. The analytical upright-equilibrium Jacobians agreed with finite-difference estimates to the reported numerical precision. This supports algebraic/numerical consistency of the implemented constant-parameter model and its local linearisation.
- **Limitations:** This result does not validate the physical parameter values, the physical system boundary, time-varying mass/geometry equations, motor/actuator dynamics, saturation or slew-rate effects, disturbance modelling, RBF approximation, or any closed-loop stability claim. It is not hardware validation.
- **Follow-up action:** Retain the illustrative-set status; complete parameter provenance and physical interpretation before using a hardware-derived nominal set. Proceed with the separate derivation review for time-varying parameters; do not infer controller stability from V-001.

### V-002 — Time-varying attached-body Lagrangian model verification

- **Execution date/time:** Reported by user on 2026-11-04; exact execution time and MATLAB release not supplied.
- **Research branch and commit:** `analyzed-system`; script commit `203dcd77e2919f467ca8887081d376c7f395720d`.
- **Script and version:** `Analyzed_System/verification/verify_time_varying_lagrange_model.m`.
- **MATLAB release/environment:** MATLAB environment not specified.
- **Parameter-set ID and source:** Illustrative smooth attached-body parameter set embedded in the script; not a hardware-derived or identified set.
- **Purpose / hypothesis:** Independently evaluate the Euler–Lagrange expression along a prescribed smooth trajectory and compare it with the explicit time-varying equations; check the pivot inertia condition and constant-parameter limit.
- **Test conditions:** Prescribed smooth attached-body parameter/trajectory variation. This is not a mass-flow/open-system model and does not include abrupt payload impact.
- **Actual output (as reported by user):**
  - Euler–Lagrange / explicit-equation error = 4.033e-13
  - Force-balance residual at selected input = 2.831e-01
  - Norm of parameter-rate terms = 0.00040875
  - Constant-parameter rate-term norm = 0.000e+00
  - Pivot inertia margin I-m*l^2 = 0.004 kg m^2
  - Equation identity = PASS
  - Physical inertia condition = PASS
  - Constant-parameter limit = PASS
- **Pass/fail/inconclusive:** **Pass for the three implemented identity/condition checks**, restricted to the illustrative smooth trajectory and parameter set.
- **Observed anomalies / separate diagnostic:** The force-balance residual is nonzero (2.831e-01). This is distinct from the Euler–Lagrange / explicit-equation identity error and is not itself reported as a pass. Its physical interpretation and acceptance criterion need separate review.
- **Interpretation:** The equation-identity error is very small, supporting numerical consistency between the independently evaluated Euler–Lagrange expression and the explicit time-varying equations for this trajectory. The positive inertia margin satisfies the implemented physical condition for this parameter set. The parameter-rate terms vanish in the constant-parameter limit, as expected.
- **Limitations:** Does not validate open-system mass flow, abrupt payload events or impact/reset dynamics, hardware parameters, actuator dynamics, or closed-loop stability. It does not establish that the illustrative trajectory/parameter variation represents a physical apparatus.
- **Follow-up action:** Inspect the force-balance residual definition and units before interpreting it; retain the current equation identity result without relabelling the residual as zero. Review the derivation's physical system boundary and time-varying attached-body assumptions before using the equations in controller design.

## Failed / inconclusive tests

_None recorded yet._
