# Nonlinear Model Verification: First Pass

**Branch:** `analyzed-system`  
**Status:** Verification plan and executable MATLAB checks; not a controller implementation.

## Purpose

Independently check the constant-parameter single cart–inverted-pendulum equations and their upright linearisation before controller coding. The companion script is:

- [`verify_nonlinear_model.m`](verify_nonlinear_model.m)

Run in MATLAB from this directory:

```matlab
results = verify_nonlinear_model();
```

## Scope of the first verification pass

The script checks:

1. Physical parameter consistency at the selected numerical test point, including (I\ge ml^2) where (I) is pivot inertia.
2. Positive definiteness of the (2\times2) inertia matrix over a full ([-\pi,\pi]) angle sweep for that fixed parameter set.
3. The analytic upright linearisation matrices (A_0,B_0,E_0) against central finite-difference Jacobians of the nonlinear equations.
4. The nonlinear state derivative at a non-equilibrium sample point, retained in the returned results for inspection.

The numerical values inside the script are **illustrative test values only**. They are not a final project parameter set and must not be described as measured or manufacturer-specified values.

## Independent model identities to check

With (q=[x,\theta]^T), state (X=[x,\dot x,\theta,\dot\theta]^T), and angle measured from upright:

[
H(q)\ddot q+h(q,\dot q)=
\begin{bmatrix}F+d_x\\d_\theta\end{bmatrix},
\quad
H=\begin{bmatrix}M+m&ml\cos\theta\\ml\cos\theta&I\end{bmatrix}.
]

[
D(\theta)=(M+m)I-m^2l^2\cos^2\theta.
]

For (M>0,m>0,I\ge ml^2), the matrix is positive definite for all (	heta), since

[
D(\theta)\ge (M+m)I-m^2l^2
=MI+m(I-ml^2)>0.
]

The upright linearisation uses (D_0=(M+m)I-(ml)^2):

[
B_0=\begin{bmatrix}0&I/D_0&0&-ml/D_0\end{bmatrix}^{T},
]

[
E_0=\begin{bmatrix}
0&0\\ I/D_0&-ml/D_0\\0&0\\-ml/D_0&(M+m)/D_0
\end{bmatrix}.
]

The gravity terms in (A_0) must retain the signs implied by (	heta=0) upright: cart acceleration coefficient (-m^2gl^2/D_0), angular acceleration coefficient (+(M+m)mgl/D_0).

## First-pass algebraic findings

- The constant-parameter Lagrangian equations in the mathematical specification are consistent with the stated upright-angle convention.
- The displayed (A_0,B_0,E_0) expressions are algebraically consistent with inversion of the coupled inertia matrix.
- The horizontal force disturbance enters through the same acceleration direction as the actuator force; the independent pivot torque has a distinct input direction and is generally unmatched.
- The condition (I\ge ml^2) is a physically meaningful rigid-body condition when (I) is measured about the pivot and (l) is the pivot-to-COM distance. It is stronger and clearer than merely checking (I>0).

These are derivation-level checks, not a claim that MATLAB has been executed. The script must be run in MATLAB and its output retained before recording a numerical pass.

## Explicit exclusions

This first script does **not** verify:

- time-varying attached-mass equations (6)–(7);
- open-system mass flow or rocket dynamics;
- abrupt payload-event reset maps;
- friction identification or a hardware parameter set;
- motor, gearbox, amplifier, voltage-to-force, saturation or slew-rate dynamics;
- LQI stabilisability/detectability or Riccati solution;
- RBF projection, adaptation, anti-windup or UUB;
- any controller performance or stability claim.

## Next verification tasks

1. Execute this script in MATLAB and record MATLAB release, output and any warnings.
2. Independently derive the time-varying-parameter Euler–Lagrange equations and verify every \\(\dot M,\dot m,\dot l,\dot I\\) term symbolically.
3. Freeze the physical parameter register with provenance and admissible intervals.
4. Build a separate nominal-LQI verification script only after the nominal parameter set is approved.
5. Derive and verify the matched/unmatched projection from one internally consistent nonlinear residual definition, including the actual input direction and actuator-force convention.

No RBF-LQI controller code is authorised by this first pass.
