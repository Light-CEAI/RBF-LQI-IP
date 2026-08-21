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

