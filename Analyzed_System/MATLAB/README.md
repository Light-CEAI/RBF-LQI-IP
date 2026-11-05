# MATLAB scripts — analyzed-system branch

A staged MATLAB scaffold for the single cart-mounted inverted pendulum.

## Quick start
1. Download the MATLAB folder from the analyzed-system branch.
2. Set MATLAB current folder to Analyzed_System/MATLAB.
3. Run run_project_demo.

The demo checks the nonlinear model Jacobian, synthesises a nominal LQI gain, evaluates an RBF basis/prediction, demonstrates weighted projection and exercises a software actuator limiter.

## File guide
- demo_config.m — illustrative software-test configuration.
- cart_pendulum_dynamics.m — nonlinear attached-body plant.
- linearize_cart_pendulum.m — upright linear model.
- design_lqi_controller.m — nominal augmented LQI.
- rbf_basis.m and rbf_predict.m — RBF features and two-output prediction.
- matched_unmatched_projection.m — scalar force projection.
- actuator_limit_step.m — force and slew-rate limiter.
- verify_model_consistency.m — finite-difference Jacobian check.
- adapt_rbf_weights_NOT_APPROVED.m — deliberate implementation gate.
- DESIGN_GATES.md — limitations and outstanding derivations.

This is not the final robust RBF-LQI controller. No adaptive update or UUB/stability guarantee is claimed. The limiter is not an electrical motor model.
