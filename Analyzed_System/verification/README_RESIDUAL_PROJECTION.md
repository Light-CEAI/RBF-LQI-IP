# Actual-versus-Nominal Residual and Projection Verification

**Test ID:** V-006  
**Branch:** `analyzed-system`  
**Status:** Script committed; user MATLAB execution pending.

## Purpose

Check the algebraic identities in [ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md](../ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md) over a deterministic grid of illustrative states and smooth attached-body parameter values.

## Run

1. Download `verify_residual_projection.m` from this branch.
2. Make the downloaded file's folder MATLAB's Current Folder (or add it to the MATLAB path).
3. Run:

```matlab
results = verify_residual_projection();
```

4. Send the complete Command Window output and the returned `results` structure.

Do not modify the script or tolerances to obtain a pass.

## Checks performed

- Positive inertia determinant for every sample.
- Positive weighted projection denominator \(b_q^TSb_q\).
- Exact reconstruction \(r=B_p\delta_m+r_u\).
- Weighted projection orthogonality \(b_q^TSr_{q,u}=0\).
- Actual-versus-nominal dynamic identity with an arbitrary applied force and two disturbances.
- Determinant identity \(\det[b_q\;g_\theta]=1/D\), confirming that the independent pivot-torque acceleration direction is not collinear with the horizontal-force direction when \(D>0\).

The script uses illustrative values and a deterministic grid, not a frozen physical parameter set. Its scales \(a_s=0.08\) m/s² and \(\alpha_s=0.10\) rad/s² are provisional numerical values used only to exercise the projection calculation; they are **not approved design scales**. A later sensitivity and operating-envelope study must justify the metric.

## Limitations

Passing V-006 verifies the implemented algebraic identities on the sampled grid. It does not validate physical parameter provenance, identify unknown parameters, establish an RBF approximation bound, validate actuator dynamics, or prove closed-loop stability. The RBF adaptation law and UUB proof remain blocked.
