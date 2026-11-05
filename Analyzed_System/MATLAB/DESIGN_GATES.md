# MATLAB scaffold: design gates

This folder is an executable research scaffold, not a completed robust adaptive controller.

## Included
- Nonlinear single cart-pendulum attached-body dynamics.
- Upright fixed-parameter linearisation and disturbance matrix.
- Nominal LQI synthesis (Control System Toolbox required).
- Gaussian RBF basis and two-output prediction.
- Weighted matched/unmatched force projection.
- Static force saturation and force slew-rate command limiter.
- Finite-difference model Jacobian check.

## Not implemented or approved
1. Online RBF weight adaptation law.
2. Robustifying term for unmatched residuals and pivot torque.
3. Motor electrical/mechanical model or validated actuator tracking-error bound.
4. Actuator-inclusive Lyapunov proof.
5. UUB theorem and verified forward-invariant approximation region.
6. Payload jump/reset dynamics.

The function adapt_rbf_weights_NOT_APPROVED intentionally stops execution. This is a research safeguard.

## Demo data
demo_config.m contains illustrative software-test values only. They are not manufacturer data, approved physical ranges, final projection scales, actuator ratings or accepted LQI tuning.

## Run
Set MATLAB current folder to this directory and execute run_project_demo.
Requires Control System Toolbox for lqr and ctrb.

Do not report the demo as evidence of robust stability, RBF learning, disturbance rejection, actuator feasibility or hardware readiness.
