# RBF-assisted adaptive LQI for the inverted pendulum

This branch adds a second architecture, distinct from the fixed-LQI-plus-RBF-force-compensation baseline in `feature/simple-lqi-rbf`.

## Architecture

1. Design a nominal LQI controller for the upright linearized cart-pendulum.
2. Train a multi-output Gaussian RBF network offline to estimate nonlinear cart- and pendulum-acceleration residuals relative to the nominal model. Inputs: `z = [theta, theta_dot, x, x_dot, force]`.
3. Evaluate the RBF output and its analytical Jacobian online.
4. Form local matrices `Ahat = A0 + d(rhat)/d(state)` and `Bhat = B0 + d(rhat)/d(force)` by mapping the residual derivatives into the acceleration rows.
5. Recompute the augmented LQI gain using the local matrices, then blend the candidate gain with the previous gain. Accept a candidate only when its local augmented closed-loop poles are in the left half-plane.
6. Apply the retuned LQI feedback law. The RBF estimate is used to adapt the local model and LQI gains; it is not added directly as a force input.

## Run

```matlab
results = run_rbf_adaptive_lqi();
```

Requires MATLAB Control System Toolbox (`lqr`); `ode45` is standard MATLAB.

## Study conditions

- Initial angles: 20, 23, 24, 25, 26, 27, 28, 29, and 30 degrees.
- RBF adaptation period: 50 ms; gain blending factor: 0.25.
- Fixed LQI and RBF-adaptive LQI use identical initial states and nominal plant parameters.
- LQI weights remain fixed; the gain is retuned as the RBF-updated local plant matrices change.

## Scope and validation

This is a research simulation baseline, not hardware-ready. It excludes actuator saturation, anti-windup, friction, observers, online RBF weight learning, and embedded deployment. The local affine residual offset is not explicitly cancelled; the RBF Jacobian is used to update the local plant Jacobian and tune LQI gains.

The script has not yet been executed in MATLAB in this workflow. Verify the RBF fit, accepted gain updates, closed-loop stability, and responses before using results in a report or publication.
