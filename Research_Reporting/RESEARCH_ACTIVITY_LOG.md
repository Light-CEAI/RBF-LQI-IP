# Research Activity and Decision Log

Record significant research work chronologically. Keep entries concise but traceable to commits, derivations, tests and decisions.

## Entry template

### YYYY-MM-DD — [Activity title]
- **Objective:**
- **Work completed:**
- **Evidence / files:**
- **Decision:**
- **Open issues:**
- **Next action:**

---

## 2026-11-04 — Establish reporting and verification workflow
- **Objective:** Create living research-reporting records and establish an evidence-led MATLAB test workflow.
- **Work completed:** Reporting templates created for methodology, results and discussion. Initial nonlinear-model verification scripts and instructions are maintained under Analyzed_System/verification/.
- **Evidence:** See repository commits and linked files in the research reporting folder.
- **Decision:** MATLAB scripts should be downloaded from the repository and executed in MATLAB Online or desktop MATLAB. Preserve complete command-window output and generated figures/data.
- **Open issues:** Physical parameter provenance, actuator model, residual projection, RBF adaptation and stability proof remain open.
- **Next action:** Resolve the outstanding V-002 diagnostic interpretation, then complete the parameter set and nominal LQI prerequisites.

## 2026-11-04 — Consolidate research design control and scope
- **Objective:** Establish a durable GitHub reference for the agreed research scope, architecture, assumptions, evidence standards and development sequence.
- **Work completed:** Added Analyzed_System/DESIGN_CONTROL_DOCUMENT.md as the authoritative design-control index. It records the single-pendulum scope, underactuation limitation, matched/unmatched residual philosophy, RBF/LQI architecture, actuator and stability requirements, comparative evaluation plan, evidence discipline and staged development gates. Reconciled the test register with user-reported V-001/V-002 execution evidence and documented the V-002 mixed-unit diagnostic correction required.
- **Evidence:** [Design control document](../Analyzed_System/DESIGN_CONTROL_DOCUMENT.md); [test register](TEST_REGISTER.md); [results log](RESULTS_LOG.md).
- **Decision:** The RBF augmentation is evaluated for practical comparative robustness improvement, not complete cancellation of all residual dynamics. Controller implementation remains gated on accepted derivations and stability analysis.
- **Open issues:** Correct V-002 componentwise diagnostic; freeze traceable parameters; derive and verify the residual/projection; complete RBF adaptation, actuator model and conditional stability analysis.
- **Next action:** Correct V-002 diagnostic, then proceed through the design gates in the design-control document.


## 2026-11-04 — Complete corrected V-002 componentwise verification

- **Objective:** Close the time-varying attached-body Euler–Lagrange identity verification after correcting the dimensionally invalid mixed-unit diagnostics.
- **Work completed:** User executed the corrected `verify_time_varying_lagrange_model.m`. Cart identity error was 4.032590e-13 N and pendulum identity error was 4.527628e-14 N·m, both below their separate 2.0e-07 tolerances. Pivot inertia margin was 0.004 kg·m². Both constant-parameter rate contributions were zero. MATLAB reported all five pass flags true.
- **Evidence / files:** Corrected [verification script](../Analyzed_System/verification/verify_time_varying_lagrange_model.m), [run instructions](../Analyzed_System/verification/README_TIME_VARYING.md), and detailed [results log](RESULTS_LOG.md).
- **Decision:** V-002 is passed and closed for the illustrative smooth attached-body trajectory and the specific identity, inertia and constant-parameter checks. The generalized-force mismatch remains diagnostic only and is not a failure.
- **Open issues:** This result does not validate open-system mass flow, abrupt payload events, hardware parameters, actuator dynamics or stability. Parameter provenance and the actual-versus-nominal residual/projection derivation remain open.
- **Next action:** Proceed to the parameter register and derive the actual-versus-nominal residual and matched/unmatched force projection rigorously before any controller implementation.

## 2026-11-04 — Derive actual-versus-nominal residual and matched/unmatched projection (V-006)

- **Objective:** Establish the exact nonlinear-to-nominal residual identity and isolate the component that can be represented by the single horizontal cart-force input.
- **Work completed:** Added a dedicated derivation defining the actual state input direction, the residual evaluated at the nominal LQI force request, and the exact identity that distinguishes nominal request from applied actuator force. Derived a weighted scalar least-squares projection of the two acceleration residuals onto the actual force direction, the corresponding unmatched residual and its weighted orthogonality. Also proved algebraically that horizontal force disturbance is matched while an independent pivot torque direction is linearly independent of the horizontal-force direction for positive inertia determinant. Aligned the mathematical model's RBF specification with the agreed two-output residual estimator followed by scalar force projection. Added a deterministic MATLAB verification script and run instructions.
- **Evidence / files:** [Residual/projection derivation](../Analyzed_System/ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md); [V-006 MATLAB script](../Analyzed_System/verification/verify_residual_projection.m); [run instructions](../Analyzed_System/verification/README_RESIDUAL_PROJECTION.md); [test register](TEST_REGISTER.md).
- **Decision:** The RBF may estimate both acceleration-residual coordinates, but only the weighted projection is used as a scalar force compensation. The unmatched residual is not independently cancelled. Compensation sign is derived using actual applied force and actuator tracking error. Controller implementation remains blocked pending V-006 execution/review and subsequent RBF, actuator and stability derivations.
- **Open issues:** Justify projection scales and lower-bound conditioning over the final admissible set; establish parameter/input-direction availability or estimate it and bound its error; execute V-006 in MATLAB; derive the adaptive law and complete actuator-inclusive UUB analysis.
- **Next action:** User runs `verify_residual_projection.m` in MATLAB and returns the complete output.

## 2026-11-04 — Execute and close V-006 algebraic residual/projection verification

- **Objective:** Check the documented actual-versus-nominal residual identity and weighted matched/unmatched projection numerically before advancing to RBF adaptation and actuator analysis.
- **Work completed:** Corrected a MATLAB-incompatible direct indexing of a linear-solve expression in `verify_residual_projection.m` (commit `13f63088e08ecc6a4a5ea93fa0036fe00f8a17cd`). User reran the corrected script. All five checks passed over five illustrative deterministic samples. Maximum residual reconstruction error was zero; weighted orthogonality, actual-nominal identity and torque-direction determinant errors were at approximately machine precision. Minimum inertia determinant and projection denominator were positive.
- **Evidence / files:** [V-006 script](../Analyzed_System/verification/verify_residual_projection.m); [residual/projection derivation](../Analyzed_System/ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md); [test register](TEST_REGISTER.md); [results log](RESULTS_LOG.md).
- **Decision:** V-006 is passed and closed for the specific algebraic identities on the illustrative five-sample grid. This is not physical validation, RBF validation, actuator validation or a stability result. Provisional projection scales remain unapproved.
- **Open issues:** Justify metric scales and conditioning over the final operating envelope; establish availability or bounded estimation error for actual input direction and regressors; derive the online RBF update law; model actuator lag, magnitude/rate limits and anti-windup; complete conditional UUB/invariance proof.
- **Next action:** Resolve projection metric and signal-availability assumptions, then derive the RBF adaptive law and its Lyapunov terms. Keep final controller implementation gated.


## 2026-11-04 — Specify projection scaling and online signal availability

- **Objective:** Advance the V-006 follow-on work without inventing metric scales, assuming unavailable plant parameters, or prematurely deriving an adaptive law.
- **Work completed:** Added a companion design specification that checks the dimensions of the weighted projection objective and its denominator; sets out a predeclared envelope-based scale-selection and conditioning procedure; separates the true analysis input direction from the estimated controller direction; and audits the availability of state, nominal request, applied force, plant parameters/rates, disturbances and acceleration targets. Identified two valid but distinct RBF signal routes: parameter-informed approximation with bounded estimates, or state/input-only approximation with a robust bound for hidden parameter variation.
- **Evidence / files:** [Projection metric and online signal availability](../Analyzed_System/PROJECTION_SCALING_AND_SIGNAL_AVAILABILITY.md); [parameter register](PARAMETER_REGISTER.md); [design control document](../Analyzed_System/DESIGN_CONTROL_DOCUMENT.md).
- **Decision:** Do not freeze numerical scales or claim a lower bound from the current five-sample V-006 grid. Existing 0.08 m/s² and 0.10 rad/s² values remain provisional examples. Do not feed true, unmeasured simulation parameters/rates into an online RBF regressor. No adaptation law or controller code is approved yet.
- **Open issues:** Freeze parameter/state/disturbance/force envelopes; specify sensors, state estimation and applied-force reconstruction; decide between parameter-informed and robust state/input-only RBF routes; establish a continuous-domain projection conditioning bound.
- **Next action:** Resolve parameter provenance and admissible operating/sensing assumptions. Then freeze projection scales and regressor inputs, derive the online adaptation law and its Lyapunov contribution, and proceed to actuator-inclusive analysis.


## 2026-11-05 — Prepare MATLAB projection-metric envelope audit (V-011)

- **Objective:** Provide a reproducible MATLAB procedure to evaluate inertia positivity and weighted-projection conditioning once physically justified parameter and operating ranges are entered.
- **Work completed:** Added a parameter-grid audit function, an intentionally unpopulated configuration template, and run/interpretation instructions. The function derives pivot inertia using the parallel-axis theorem, evaluates the inertia determinant and weighted projection denominator over the Cartesian product of supplied ranges, and reports a predeclared sensitivity sweep of the angular-to-cart acceleration scale ratio.
- **Evidence / files:** [Audit function](../Analyzed_System/verification/audit_projection_metric_envelope.m); [configuration template](../Analyzed_System/verification/projection_metric_config_TEMPLATE.m); [README](../Analyzed_System/verification/README_PROJECTION_METRIC_AUDIT.md); [parameter register](PARAMETER_REGISTER.md).
- **Decision:** No physical values or acceleration scales are embedded. The script is a finite-grid diagnostic, not a continuous-domain proof of \(\beta_b\). V-011 is not passed until the user supplies justified ranges, executes the function in MATLAB and returns the complete output.
- **Open issues:** Parameter ranges, admissible angle domain and metric scales remain unapproved. Grid coverage and endpoint inclusion must be reviewed; a formal continuous lower bound still needs an analytic or validated enclosure argument.
- **Next action:** Populate the template only from the approved parameter/operating-envelope register, execute V-011, and review the output before selecting an RBF regressor or deriving its update law.
