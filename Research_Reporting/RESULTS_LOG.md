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

## Failed / inconclusive tests

_None recorded yet._
