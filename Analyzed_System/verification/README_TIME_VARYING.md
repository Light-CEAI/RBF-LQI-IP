# Time-Varying Attached-Body Model Verification

**Test ID:** V-002  
**Branch:** `analyzed-system`  
**Script status:** Corrected componentwise diagnostic committed; rerun required.

## Purpose

Numerically audit the time-varying attached-rigid-body Euler–Lagrange equations (6)–(7) in the mathematical model document. The script independently evaluates

`d/dt(∂L/∂qdot) - ∂L/∂q`

along smooth prescribed coordinate and parameter trajectories, then compares that result with the explicit equations containing the `Mdot`, `mdot`, `ldot`, and `Idot` terms.

## What changed in the diagnostic

The earlier script used an ordinary infinity norm on two generalized-force coordinates. That combined a translational component measured in N with a rotational component measured in N*m, so the resulting scalar had no valid single physical unit.

The corrected script now:
- reports the Euler–Lagrange identity error for the cart equation separately in N;
- reports the identity error for the pendulum equation separately in N*m;
- applies separate, unit-labelled numerical tolerances to those two identity checks;
- reports the generalized-force mismatch against the selected input separately in N and N*m, as a diagnostic only;
- reports the parameter-rate contributions separately in N and N*m;
- never uses the mixed-coordinate generalized-force mismatch as a pass/fail condition.

The prescribed kinematic trajectory and independently selected generalized input are not required to satisfy the forced equations of motion. A nonzero generalized-force mismatch is therefore expected in general and does not mean the Euler–Lagrange identity is wrong.

## Run in MATLAB

1. Download the corrected `verify_time_varying_lagrange_model.m` from this branch.
2. In MATLAB, make the folder containing the downloaded file the Current Folder (or add it to the MATLAB path).
3. Run:

```matlab
results = verify_time_varying_lagrange_model();
```

4. Copy the complete Command Window output and send it back. Also include the values returned in `results` if MATLAB displays them.

Do not edit the model equations or tolerances to force a pass.

## What the script checks

1. Componentwise agreement between the independently evaluated Euler–Lagrange equations and explicit time-varying equations.
2. The illustrative pivot inertia condition `I > m*l^2`.
3. Vanishing of each parameter-rate contribution in the constant-parameter limit.

## Important interpretation and scope

This is a numerical identity check for a prescribed **attached-body** model whose kinetic energy is explicitly time-dependent. It does not derive or validate open-system variable-mass dynamics. In particular, it does not model propellant exhaust momentum flux, material entering or leaving through ports, payload impact impulses, or hybrid reset maps.

The parameter and trajectory values are illustrative and are not hardware specifications or a frozen nominal set. The script does not test a controller or establish stability.

## Acceptance

The script uses a central finite-difference approximation for the time derivative of generalized momentum. Separate componentwise tolerances are numerical checks, not a formal proof. If either identity component fails, preserve the output and investigate finite-difference step-size sensitivity before changing the equations or tolerances.
