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