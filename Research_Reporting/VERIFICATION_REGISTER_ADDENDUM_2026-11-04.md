# Verification Register Addendum — 2026-11-04

This addendum records changes after the original test-register snapshot. Consult it alongside `TEST_REGISTER.md`.

| ID | Updated status | Evidence |
|---|---|---|
| V-001 | **Passed for the illustrative parameter set only**, based on user-supplied MATLAB output. This does not validate hardware parameters or stability. | `Research_Reporting/RESULTS_LOG.md` |
| V-002 | **Corrected diagnostic committed; rerun required before closure.** Earlier user-supplied output reported three checks as passed, but its scalar diagnostics mixed force and torque units. | [Corrected MATLAB script](../Analyzed_System/verification/verify_time_varying_lagrange_model.m); [run instructions](../Analyzed_System/verification/README_TIME_VARYING.md); detailed record in `Research_Reporting/RESULTS_LOG.md` |

## V-002 scope

V-002 independently evaluates the Euler–Lagrange expression for a prescribed smooth attached-body model and compares it with the explicit time-varying equations. It is not an open-system mass-flow model and does not establish stability.

## Diagnostic correction

The previous script used unscaled norms across a generalized-force vector with a translational component in N and a rotational component in N*m. Such a scalar has no single physical unit and must not be interpreted as a force-balance error.

The corrected script:
- compares the Euler–Lagrange and explicit equations componentwise;
- reports cart-equation identity error in N and pendulum-equation identity error in N*m;
- uses separate numerical tolerances for the two identity checks;
- reports generalized-force mismatch and parameter-rate contributions componentwise;
- treats generalized-force mismatch as diagnostic only, not as an identity pass/fail condition.

A prescribed kinematic trajectory and independently selected generalized input need not satisfy the forced equations of motion. Thus a nonzero generalized-force mismatch is generally expected and does not by itself indicate a failed Euler–Lagrange identity.

## Required user action

Download the corrected script from the `analyzed-system` branch and run:

```matlab
results = verify_time_varying_lagrange_model();
```

Return the complete Command Window output. Do not edit the equations or tolerances to force a pass. V-002 is closed only after the corrected componentwise identity checks, pivot-inertia condition and constant-parameter limit are confirmed.

## Limitations

This result does not validate open-system mass flow, abrupt payload events or impact/reset dynamics, hardware parameters, actuator dynamics, or closed-loop stability. The trajectory and parameters are illustrative, not hardware-identified.
