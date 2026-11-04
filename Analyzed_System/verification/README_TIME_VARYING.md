# Time-Varying Attached-Body Model Verification

**Test ID:** V-002  
**Branch:** `analyzed-system`  
**Status:** Script committed; MATLAB execution pending.

## Purpose

Numerically audit the time-varying attached-rigid-body Euler–Lagrange equations (6)–(7) in the mathematical model document. The script independently evaluates

`d/dt(∂L/∂qdot) - ∂L/∂q`

along smooth prescribed coordinate and parameter trajectories, then compares that result with the explicit equations containing the `Mdot`, `mdot`, `ldot`, and `Idot` terms.

## Run in MATLAB

Download `verify_time_varying_lagrange_model.m` and run:

```matlab
results = verify_time_varying_lagrange_model();
```

Return the complete Command Window output, including any warnings or errors.

## What the script checks

1. The independently evaluated Euler–Lagrange expression agrees with the explicit time-varying equations to a finite-difference tolerance.
2. The illustrative pivot inertia satisfies `I > m*l^2`.
3. The parameter-rate contribution vanishes in the constant-parameter limit.

## Important interpretation

This is a numerical identity check for a prescribed **attached-body** model whose kinetic energy is explicitly time-dependent. It does not derive or validate open-system variable-mass dynamics. In particular, it does not model propellant exhaust momentum flux, material entering or leaving through ports, payload impact impulses, or hybrid reset maps.

The parameter and trajectory values are illustrative and are not hardware specifications or a frozen nominal set. The script does not test a controller or establish stability.

## Acceptance

The script uses a central finite-difference approximation for the time derivative of generalized momentum. The tolerance is therefore numerical, not a formal proof. If the test fails, preserve the output and investigate step-size sensitivity before changing the equations or tolerance.
