# RBF-LQI-IP

Augmenting LQI controller for an IP with RBFNN to cater for realtime parametric variations

\# RBF-LQI-IP



\*\*Augmenting an LQI Controller for an Inverted Pendulum with an RBF Neural Network to cater for real-time parametric variations\*\*



\## Project structure



| Folder | Contents |

|---|---|

| `Controllers/` | LQI design, control law, actuator saturation |

| `Models/` | Nonlinear plant, linearization, ZOH discretization |

| `Observers/` | Kalman / Luenberger observer design and update |

| `RBF/` | RBF network: initialization, forward pass, online update, offline training |

| `Deployment/` | Single-precision embedded path, codegen-safe functions, sensor/actuator calibration |

| `Simulations/` | All test scripts: nominal, robustness, stress, variant comparisons |

| `Utilities/` | Performance metrics, plotting, Monte Carlo, stability margins |

| `Visualizations/` | Animation, video generation, summary plots |



\## How to run



1\. Open MATLAB, set current folder to the project root

2\. Run `startup` — adds all subfolders to path and loads default parameters

3\. Run `cd Simulations` then `run\_all\_tests` — runs the full test suite



\## Configuration



All system parameters are in `parameters.m`. Switch between Config A (wheel-driven cart) and Config B (direct force) by changing `DEFAULT\_CONFIG` at the top of that file.



\## Requirements



\- MATLAB R2021a or later

\- Control System Toolbox (for `lqr`, `dlqe`, `place`, `margin`)

\- Optional: Statistics and Machine Learning Toolbox (for `kmeans` in `rbf\_train\_multiscenario.m`, paired t-test in `monte\_carlo\_evaluation.m`)



## Seminar I corrected simulation workflow

Use the `seminar1-corrections` branch for the corrected Seminar I implementation.
The Config A actuator remains ideal: the commanded motor torque is converted
directly to cart force using `F = tau/r`. Motor current, back-EMF, winding
inductance, voltage dynamics, and PWM dynamics are intentionally not included
in this stage.

Before running the controller comparisons, open MATLAB at the repository root,
run `startup`, then execute:

```matlab
params = parameters();
rbf = rbf_train_offline(params, 3000);
```

This trains the RBF target in actuator-command units. The offline angular-
acceleration residual is converted through the local input gain of the nonlinear
plant before it is fitted. The resulting `RBF/RBF/rbf_centers.mat` is loaded
by the host simulation; if it is missing, the host uses zero initial weights.

Then run the required Seminar I checks from `Simulations` (or use the project
test runner):

```matlab
cd Simulations
run_all_tests
```

For embedded-equivalent checks only, regenerate the exported RBF constants
after offline training and regenerate controller constants after the LQI /
observer design has been confirmed:

```matlab
cd ../RBF
rbf_export_for_embedded('rbf_centers.mat', 'rbf_centers_const.m');
cd ../Deployment
% Generate control_consts.m using the current design_lqi and observer gains.
```

The online RBF update uses the cart-position tracking error consistently in
the host and embedded controller paths. It is an empirical adaptation rule;
it is not presented as a formal Lyapunov-based stability law. Treat the
comparison and robustness outputs as results to be re-established by running
the corrected scripts in MATLAB, not as pre-validated performance claims.
