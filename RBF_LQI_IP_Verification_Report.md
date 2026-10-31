# RBF-LQI-IP Verification Report

## Scope

This report reviews the repository implementation for an RBF-augmented LQI controller applied to an inverted pendulum under parametric uncertainty.

## Executive Summary

The implementation is structurally sound and internally consistent. The design follows a standard architecture:

- nominal LQI design based on a linearized inverted-pendulum model,
- state observer for partial measurements,
- RBF neural compensator for model mismatch and parameter drift,
- saturation and anti-windup applied to the combined command, not just the LQI term.

This is the correct control architecture for compensating real-time uncertainty while maintaining stability under saturation.

## Repository Structure Review

### Root files

- startup.m
  - Correctly resolves the project root and adds all subfolders to MATLAB path.
  - Loads the default configuration into the workspace.
  - This is a clean MATLAB bootstrap pattern.

- parameters.m
  - Centralizes all physical and control parameters.
  - Separates Config A (wheel-driven cart with torque input) and Config B (direct-force cart).
  - Includes realistic ranges for mass, friction, and length variation.
  - Provides RBF and embedded-control settings in a single place.

## Model Review

### Models/Models/nonlinear_plant.m

This file implements the nonlinear cart-pendulum dynamics using the standard Lagrangian formulation.

Observations:

- State ordering is consistent with the project: [x; xdot; theta; thetadot].
- The mass matrix and forcing terms are consistent with a pendulum mounted on a cart.
- The actuator input conversion is correctly handled by config:
  - Config A: torque converted to an equivalent force via wheel radius.
  - Config B: direct force input.
- The dynamics are appropriate for simulation and for testing robustness under parameter uncertainty.

### Models/Models/linearized_model.m

The linearization is consistent with the common small-angle inverted-pendulum model around the upright equilibrium.

Observations:

- The system matrix A follows the standard form for a cart-pendulum linearization.
- The B matrix is scaled correctly for Config A, where the actual plant input is motor torque.
- The model is suitable for LQI design and for discrete-time observer design.

### Models/Models/discretize_model.m

The zero-order-hold discretization is implemented correctly using the augmented matrix exponential.

Observations:

- This is a valid discretization strategy for a continuous-time linear model.
- It avoids dependence on the Control System Toolbox c2d call.
- It matches the project’s goal of portable and transparent simulation logic.

## Controller Review

### Controllers/Controllers/design_lqi.m

This file designs an LQI controller by augmenting the plant with an integral state on the regulated cart position.

Observations:

- The augmented system is standard for integral action.
- Weighting choices are reasonable and consistent with a controller that must regulate both cart position and pendulum angle.
- The script includes a stability warning if the closed-loop augmented system is not asymptotically stable.

### Controllers/Controllers/lqi_controller.m

This file returns the raw unsaturated LQI law.

This is the correct design choice for an RBF-augmented controller because it separates the nominal linear feedback from the adaptive nonlinear compensator. The key point is that saturation should happen only after combining the linear and RBF terms.

### Controllers/Controllers/actuator_saturation.m

Saturation is implemented in a clean config-aware way.

Observations:

- Config A uses motor torque limit.
- Config B uses control force limit.
- This is consistent with the actuator model and with the rest of the project.

## Observer Review

### Observers/Observers/observer_design.m

The observer design is standard for a cart-pendulum state estimator with partial measurement.

Observations:

- Measurement vector is consistent with only measuring cart position and pendulum angle.
- Kalman and Luenberger variants are both supported.
- Stability diagnostics are included.

### Observers/Observers/observer_update.m

The observer update is mathematically correct.

State update:

- prediction: xhat_pred = Ad * xhat + Bd * u
- innovation: y - C_meas * xhat_pred
- correction: xhat_next = xhat_pred + L * innovation

This is the standard discrete-time observer structure.

## RBF Network Review

### RBF/RBF/rbf_initialize.m

This file initializes the RBF basis function network and handles host or embedded use cases.

Observations:

- The fallback initialization is sensible when the saved center file is unavailable.
- Sigma values are reasonable for the different state dimensions.
- The design is practical and consistent with a real-time compensation strategy.

### RBF/RBF/rbf_forward.m

The Gaussian RBF forward pass is implemented correctly.

Observations:

- The basis activations are computed as Gaussian functions centered on the hidden nodes.
- The normalized squared distance is computed with the sigma scaling.
- The scalar compensation output is the standard weighted sum of basis activations.

### RBF/RBF/rbf_update.m

The online update rule is a standard gradient descent / delta-rule update with leakage.

Observations:

- The update is consistent with adaptive control theory.
- Leakage prevents unbounded weight drift.
- Weight clipping bounds the network output.

## Core Simulation Review

### Simulations/Simulations/simulation_main.m

This is the most important file in the repository from a correctness standpoint.

The simulation engine does the following correctly:

- builds nominal and perturbed plant models,
- designs controller and observer on the nominal model,
- computes RBF compensation,
- combines the RBF term with the LQI term,
- applies actuator saturation once to the combined command,
- freezes the integral state according to the combined saturation condition.

This architecture is the correct approach for an adaptive compensator added to a baseline LQI controller. In particular, anti-windup must consider the total command signal, not just the LQI signal alone.

This is a key design correction and matches the overall goal of robust performance under parameter uncertainty.

### Simulations/Simulations/test_parametric_drift.m

This simulation is well designed for evaluating adaptive compensation under slowly varying mass.

Observations:

- It compares LQI-only vs LQI+RBF on the same varying mass profile.
- That is the right way to isolate the effect of the RBF component.
- The test is aligned with the project goal of handling real-time parametric drift.

## Overall Verdict

The repository is logically well structured and internally coherent. The combination of:

- linearized model,
- nominal LQI design,
- observer-based state estimation,
- additive RBF compensation,
- combined-signal anti-windup,
- and robustness simulations

is a valid and realistic design for an inverted-pendulum controller under parameter variations.

The strongest correctness point is the implementation of saturation and anti-windup on the total command u_lqi + f_rbf rather than only on the raw LQI term. That is a sound and necessary design choice when an adaptive nonlinearity is layered on top of the nominal controller.

## Recommendation

The project is suitable for MATLAB simulation and research-level evaluation. For physical deployment, the next step would be:

1. validate the linear model against the real pendulum rig,
2. verify sensor noise and quantization assumptions,
3. confirm actuator saturation and rate limits on the actual hardware,
4. run the same robustness tests against measured plant data.

## Conclusion

The repository demonstrates a coherent and technically sound implementation of an RBF-augmented LQI controller for an inverted pendulum under parametric uncertainty.

The code is appropriate for simulation and comparison studies, and the control architecture is consistent with control theory best practices.

---

Prepared for: Light-CEAI / RBF-LQI-IP
Date: 2026-10-31
