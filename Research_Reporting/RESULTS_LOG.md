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
- **Original output (reported by user from the pre-correction script):**
  - Euler–Lagrange / explicit-equation error = 4.033e-13 (the original unscaled norm mixed N and N*m components; treat as a historical numerical indicator, not a dimensionally homogeneous scalar).
  - Original generalized-force mismatch norm = 2.831e-01 (dimensionally unsuitable as a single scalar because it mixed N and N*m; do not interpret physically).
  - Original parameter-rate term norm = 0.00040875 (also combined N and N*m; do not interpret as a single-unit magnitude).
  - Constant-parameter rate-term norm = 0.000e+00.
  - Pivot inertia margin I-m*l^2 = 0.004 kg m^2.
  - Equation identity = PASS; physical inertia condition = PASS; constant-parameter limit = PASS.
- **Pass/fail/inconclusive:** The three original checks were reported as passed, but V-002's diagnostic presentation is being corrected. Re-execution of the corrected componentwise script is required before closing V-002.
- **Diagnostic correction:** The previous script applied an ordinary infinity norm to generalized-force vectors whose first component is in N and second component is in N*m. The corrected script reports cart-equation and pendulum-equation identity errors separately, with separate unit-labelled tolerances. It also reports the generalized-force mismatch and parameter-rate contributions componentwise. The generalized-force mismatch is diagnostic only: the prescribed trajectory and independently selected generalized input need not satisfy the forced equations of motion.
- **Interpretation pending rerun:** The previous very small identity indicator suggests close numerical agreement, but the corrected componentwise outputs are now the acceptance evidence. The positive pivot-inertia margin and constant-parameter-limit result were reported previously; confirm again with the corrected script.
- **Limitations:** Does not validate open-system mass flow, abrupt payload events or impact/reset dynamics, hardware parameters, actuator dynamics, or closed-loop stability. It does not establish that the illustrative trajectory/parameter variation represents a physical apparatus.
- **Follow-up action:** Run the corrected V-002 script and return the complete output. Do not change equations or tolerances to force a pass.

## Failed / inconclusive tests

_None recorded yet._


### V-002 — Corrected componentwise rerun (accepted)

- **Execution date/time:** Reported by user on 2026-11-04; exact execution time and MATLAB release not supplied.
- **Research branch and commit:** `analyzed-system`; corrected diagnostic script commit `094fbe091156520169b36e83e909447d9c7e416b`.
- **Script and version:** `Analyzed_System/verification/verify_time_varying_lagrange_model.m`; corrected componentwise diagnostics.
- **MATLAB release/environment:** Not supplied.
- **Parameter-set ID and source:** Illustrative smooth attached-body parameter set embedded in the script; not hardware-derived.
- **Purpose / hypothesis:** Verify the Euler–Lagrange identity componentwise, the pivot-inertia condition, and the constant-parameter limit without combining quantities of unlike units.
- **Actual output (user-supplied):**
  - Cart equation identity error: (4.032590\times10^{-13}\) N; tolerance (2.0\times10^{-7}\) N — PASS.
  - Pendulum equation identity error: (4.527628\times10^{-14}\) N·m; tolerance (2.0\times10^{-7}\) N·m — PASS.
  - Generalized-force mismatch: (-2.831167\times10^{-1}\) N and (-6.485389\times10^{-2}\) N·m — diagnostic only, not an identity-test failure.
  - Parameter-rate contribution: (4.087503\times10^{-4}\) N and (6.800509\times10^{-6}\) N·m.
  - Constant-parameter rate terms: 0 N and 0 N·m — PASS.
  - Pivot inertia margin (I-ml^2=0.004\) kg·m² — PASS.
  - MATLAB returned `passedIdentityX=1`, `passedIdentityTheta=1`, `passedEquationIdentity=1`, `passedPhysicalInertia=1`, and `passedConstantParameterLimit=1`.
- **Pass/fail/inconclusive:** **PASS**, for the implemented identities and checks on the illustrative smooth trajectory.
- **Interpretation:** The explicit time-varying equations agree with the independently evaluated Euler–Lagrange expressions to substantially tighter precision than the componentwise tolerances. The physical pivot-inertia margin is positive, and parameter-rate contributions vanish in the constant-parameter limit. The nonzero generalized-force mismatch is expected to be reported separately because the selected trajectory/input is not required to satisfy the forced equations of motion; it does not invalidate the identity check.
- **Limitations:** This is an equation-consistency verification for an illustrative prescribed smooth attached-body trajectory. It does not validate open-system mass flow, abrupt payload events/impact resets, hardware parameters, actuator dynamics, or closed-loop stability.
- **Follow-up action:** V-002 is closed for its stated scope. Continue with parameter provenance and the rigorous actual-versus-nominal residual and matched/unmatched projection derivation.