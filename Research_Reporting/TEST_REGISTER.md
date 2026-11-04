# Verification and Simulation Test Register

Tests are staged. A later-stage controller test must not be treated as authorised or meaningful while its prerequisite derivations remain unresolved. The authoritative research scope and gate sequence are in [DESIGN_CONTROL_DOCUMENT.md](../Analyzed_System/DESIGN_CONTROL_DOCUMENT.md).

## Verification tests

| ID | Test | Prerequisite | Current status | Evidence/script |
|---|---|---|---|---|
| V-001 | Inertia positive definiteness and upright finite-difference linearisation | Constant-parameter nonlinear equations | **Executed; passed for illustrative parameter set** | [Script](../Analyzed_System/verification/verify_nonlinear_model.m); [instructions](../Analyzed_System/verification/README.md); result in RESULTS_LOG.md |
| V-002 | Time-varying attached-body Euler–Lagrange identity, pivot-inertia condition and constant-parameter limit | Defined attached-body model and system boundary | **Executed; three checks passed for illustrative smooth trajectory. Diagnostic correction outstanding.** | [Script](../Analyzed_System/verification/verify_time_varying_lagrange_model.m); [instructions](../Analyzed_System/verification/README_TIME_VARYING.md); result in RESULTS_LOG.md |
| V-003 | Parameter provenance and physical consistency | Exact reference configuration documentation | In progress; parameter set not frozen | [Parameter register](PARAMETER_REGISTER.md) |
| V-004 | Nominal linear model, stabilisability and integral augmentation conditions | V-003 plus verified nonlinear model | Blocked pending V-003 | To be created |
| V-005 | LQI augmentation and Riccati solution | V-004 plus specified Q/R and integrated output | Not started | To be created |
| V-006 | Actual-versus-nominal residual and matched/unmatched projection identity | Consistent residual, actual-input definition and actuator direction | Not started | To be derived and tested |
| V-007 | RBF regressor, approximation domain and error evaluation | Defined signals, scales, centres and data coverage | Not started | To be specified |
| V-008 | Adaptive update law and projection property | Completed Lyapunov derivation | Not started | To be derived |
| V-009 | Actuator/drive model, force and slew limits, anti-windup | Verified actuator data and selected drive model | Not started | To be specified |
| V-010 | Complete conditional boundedness/UUB analysis | V-006 to V-009 and all boundedness/invariance assumptions | Not started | To be derived and independently reviewed |

## Comparative simulation tests

| ID | Test | Prerequisite | Current status | Evidence |
|---|---|---|---|---|
| S-001 | Nominal nonlinear-plant LQI simulation | V-003 to V-005 | Blocked | To be created |
| S-002 | LQI versus fixed/offline and online-adaptive RBF-augmented LQI | Accepted residual/RBF/actuator design and stability analysis | **Not authorised** | To be created |
| S-003 | Parameter uncertainty sweep | Accepted plant/controller and declared ranges | **Not authorised** | To be created |
| S-004 | Horizontal force and pivot torque disturbance rejection | Accepted disturbance model and controller | **Not authorised** | To be created |
| S-005 | Force saturation and slew-rate tests | V-009 and accepted anti-windup design | **Not authorised** | To be created |
| S-006 | Sensitivity, adverse-case and repeatability tests | Stable test harness and fixed scenarios | **Not authorised** | To be created |

## V-002 diagnostic clarification

The previously reported scalar value formed by taking an ordinary norm of a two-component generalized-force residual is dimensionally unsuitable because its components have different units: newtons and newton-metres. Do not interpret or report that mixed-unit scalar as a force-balance error or as a pass/fail measure.

Revise the diagnostic to report the translational generalized-force residual in N and rotational generalized-force residual in N m separately. The prescribed trajectory and independently selected generalized input need not satisfy the forced equations of motion; a nonzero generalized-force mismatch is therefore not, by itself, a failure of the Euler–Lagrange identity check. The equation-identity, physical inertia and constant-parameter-limit checks remain distinct.

## Test acceptance rules

1. Every test has a stated hypothesis and numerical acceptance criterion before execution.
2. Keep complete console output, including warnings and errors.
3. Record exact source commit, parameter-set ID, MATLAB release and solver settings where available.
4. Do not alter the model or test conditions silently to obtain a pass.
5. Preserve failed, adverse and inconclusive results.
6. A numerical identity/Jacobian check supports only the implemented equation and tested domain; it does not establish physical validity or closed-loop stability.
7. Simulation results do not replace analytical stability proofs.
8. Do not mark a test passed unless its declared acceptance checks have actually been executed and their evidence recorded.


## V-002 closure — 2026-11-04

The corrected componentwise MATLAB script was executed by the user. Both Euler–Lagrange identity checks passed: cart equation error (4.032590\times10^{-13}\) N against (2.0\times10^{-7}\) N tolerance; pendulum equation error (4.527628\times10^{-14}\) N·m against (2.0\times10^{-7}\) N·m tolerance. The pivot inertia margin was (0.004\) kg·m² (positive), and both constant-parameter rate contributions were exactly zero. MATLAB reported all five pass flags as true.

The generalized-force mismatch was (-0.2831167\) N and (-0.06485389\) N·m. These are explicitly diagnostic only and do not enter the identity-test pass/fail decision.

**Status: PASSED and closed for the stated illustrative smooth attached-body model and test domain.** This does not validate open-system mass flow, abrupt payload events, hardware parameters, actuator dynamics, or closed-loop stability. See the detailed user-supplied output in [RESULTS_LOG.md](RESULTS_LOG.md).