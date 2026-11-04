# Verification and Simulation Test Register

Tests are staged. A later-stage controller test must not be treated as authorised or meaningful while its prerequisite derivations remain unresolved.

| ID | Test | Prerequisite | Current status | Evidence/script |
|---|---|---|---|---|
| V-001 | Inertia positive definiteness and upright finite-difference linearisation | Constant-parameter nonlinear equations | Script committed; run pending | `Analyzed_System/verification/verify_nonlinear_model.m` |
| V-002 | Time-varying parameter Euler–Lagrange derivation checks | Define physical meaning of changing masses/geometry and system boundary | Not started | To be created |
| V-003 | Parameter provenance and physical consistency | Exact reference configuration documentation | In progress | Parameter register to be created |
| V-004 | Nominal linear model and controllability/stabilisability | Frozen nominal parameters and verified Jacobians | Blocked by V-003 | To be created |
| V-005 | LQI augmentation and Riccati solution | V-004 plus specified Q/R and output integrator | Not started | To be created |
| V-006 | Matched/unmatched projection identity | One consistent residual and actual-input definition | Not started | To be created |
| V-007 | RBF approximation domain and error evaluation | Defined RBF regressors, scales, centres and data coverage | Not started | To be created |
| V-008 | Adaptive update and projection property | Completed Lyapunov derivation | Not started | To be created |
| V-009 | Actuator/motor and anti-windup model | Verified actuator data and chosen drive model | Not started | To be created |
| V-010 | Complete UUB analysis checks | All boundedness, invariance and actuator assumptions | Not started | To be created |
| S-001 | Nominal LQI nonlinear simulation | V-003 to V-005 | Blocked | To be created |
| S-002 | LQI versus RBF-augmented LQI comparison | V-006 to V-010 and accepted controller derivation | Not authorised | To be created |
| S-003 | Parameter uncertainty sweep | Accepted plant/controller and declared ranges | Not authorised | To be created |
| S-004 | Disturbance rejection tests | Accepted disturbance model and controller | Not authorised | To be created |
| S-005 | Actuator saturation/slew-rate tests | V-009 and accepted anti-windup design | Not authorised | To be created |
| S-006 | Sensitivity and repeatability tests | Stable test harness and fixed scenarios | Not authorised | To be created |

## Test acceptance rules

1. Every test has a stated hypothesis and numerical acceptance criterion before execution.
2. Keep the complete console output, including warnings and errors.
3. Record the exact source commit and parameter-set version.
4. Do not alter the model or test conditions silently to obtain a pass.
5. A failed test is evidence and must remain in the log.
6. Numerical checks support but do not replace analytical proofs.
