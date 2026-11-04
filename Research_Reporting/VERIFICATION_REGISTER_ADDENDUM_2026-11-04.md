# Verification Register Addendum — 2026-11-04

This addendum records changes after the original test-register snapshot. It is append-only; consult it alongside `TEST_REGISTER.md`.

| ID | Updated status | Evidence |
|---|---|---|
| V-001 | **Passed for the illustrative parameter set only**, based on user-supplied MATLAB output. This does not validate hardware parameters or stability. | `Research_Reporting/RESULTS_LOG.md` |
| V-002 | **Passed for the illustrative smooth attached-body trajectory**, based on user-supplied MATLAB output. | `Analyzed_System/verification/verify_time_varying_lagrange_model.m`; detailed record in `Research_Reporting/RESULTS_LOG.md` |

## V-002 scope

V-002 independently evaluates the Euler–Lagrange expression for a prescribed smooth attached-body model and compares it with the explicit time-varying equations. It is not an open-system mass-flow model and does not establish stability.

### User-executed result

- Euler–Lagrange / explicit-equation error: `4.033e-13`
- Force-balance residual at selected input: `2.831e-01`
- Norm of parameter-rate terms: `0.00040875`
- Constant-parameter rate-term norm: `0.000e+00`
- Pivot inertia margin `I-m*l^2`: `0.004 kg m^2`
- Equation identity: PASS
- Physical inertia condition: PASS
- Constant-parameter limit: PASS

**Interpretation:** The very small equation-identity error supports numerical consistency between the independently evaluated Euler–Lagrange expression and the explicit time-varying equations for this illustrative smooth trajectory. The pivot inertia margin is positive for the tested parameter set, and the parameter-rate contribution vanishes in the constant-parameter limit as expected.

**Important distinction:** The reported force-balance residual is nonzero. It is a separate diagnostic and is not part of the equation-identity pass. It must not be described as a zero-residual force-balance validation. Its physical interpretation and acceptance threshold require separate examination.

**Limitations:** This result does not validate open-system mass flow, abrupt payload events or impact/reset dynamics, hardware parameters, actuator dynamics, or closed-loop stability. The trajectory and parameters are illustrative, not hardware-identified.

Do not change the tolerance or model equations merely to force a pass; investigate any failure and report it.
