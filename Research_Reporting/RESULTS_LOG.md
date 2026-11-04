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
| V-001 | Nonlinear equations, inertia positive definiteness and upright finite-difference linearisation | Script committed; execution pending | `Analyzed_System/verification/verify_nonlinear_model.m` |

No MATLAB execution result has yet been received for V-001. Therefore it is **pending**, not passed.

## Accepted results

_None recorded yet._

## Failed / inconclusive tests

_None recorded yet._
