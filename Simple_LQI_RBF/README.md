# Simple LQI + RBF branch

This branch isolates a minimal simulation study of a direct-force cart–inverted-pendulum plant.

## Control law

`u = u_LQI + f_RBF`

- LQI is designed once from the nominal upright linearization.
- The RBF is trained offline to estimate the force-equivalent projection of the nonlinear acceleration residual.
- The RBF correction is added directly to the LQI force command.
- The nonlinear plant is frictionless and accepts a single direct horizontal force.

## Deliberately excluded

No A/B configuration switch, actuator saturation, anti-windup, observer, embedded deployment, Coulomb friction, online RBF adaptation, or actuator model.

## Run

In MATLAB with Control System Toolbox:

```matlab
addpath(genpath('Simple_LQI_RBF'));
results = run_simple_lqi_rbf();
```

The script compares LQI and LQI+RBF at initial pendulum angles of 0, 20, 25 and 30 degrees. It reports force-residual training RMSE and plots angle and force responses.

## Interpretation

This is a controlled baseline for isolating the additive RBF idea. It is not a validated stability result or a hardware-ready controller. Inspect the simulated responses and force demand before making performance claims.
