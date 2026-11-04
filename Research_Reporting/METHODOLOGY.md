# Research Methodology

**Project:** Robust RBF-Augmented LQI Control of a Single Inverted Pendulum under Parametric Uncertainty, External Disturbances and Actuator Constraints  
**Status:** Living document — update as methods are derived, reviewed and verified.  
**Research branch:** `analyzed-system`

## 1. Research design

The study follows a model-based control-design and computational-validation methodology. The physical system is a single rigid pendulum mounted on a horizontally translating cart. The controller under investigation is an LQI baseline augmented by a radial-basis-function (RBF) approximation mechanism. The intended robustness study considers parameter uncertainty, external disturbances and actuator limitations.

The present stage is mathematical modelling and verification. No final RBF-LQI controller implementation or performance claim is made until the relevant derivations have been reviewed and the corresponding tests have been executed.

## 2. Physical system and modelling

### 2.1 Coordinates and state

Use generalised coordinates (q=[x,\theta]^T), where (x) is cart displacement and (	heta=0) denotes the upright pendulum equilibrium. The state is
[
X=[x,\dot{x},\theta,\dot{\theta}]^T.
]

The plant is modelled as one cart and one rigid pendulum link. No second pendulum link is included.

### 2.2 Governing equations

The constant-parameter model is derived from the Euler–Lagrange equations and written as
[
H(q,p)\ddot q+h(q,\dot q,p)=
\begin{bmatrix}F_a+d_x\\d_\theta\end{bmatrix}.
]
All parameter definitions, sign conventions, assumptions and derivation steps are maintained in `Analyzed_System/MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md`.

### 2.3 Parameter provenance

Every parameter must be labelled as one of:
- published: taken from an identifiable technical manual or data sheet;
- derived: calculated from published/measured quantities using a stated law;
- measured: obtained experimentally, with measurement procedure and uncertainty;
- estimated: identified from data using a stated estimation method;
- assumed: selected for analysis, with justification and sensitivity range.

A Quanser laboratory source may be used as a source of realistic parameter values. This does not define the research objective or limit the generality of the controller.

## 3. Nominal model and LQI baseline

After the nonlinear plant and nominal parameters are verified, linearise about the upright equilibrium. Record the Jacobians and compare them with independent numerical finite-difference estimates. Verify stabilisability and the integral-augmentation conditions before solving the LQI problem.

Record the state weighting, integral weighting, input weighting, sampling time, solver settings and resulting closed-loop poles for every run.

## 4. Uncertainty and disturbance design

The study considers:
- variation in cart and pendulum mass properties;
- horizontal external force disturbance;
- pendulum pivot torque disturbance;
- actuator magnitude and slew-rate limits.

Each test case must identify its nominal value, perturbed value, waveform or time profile, application interval and rationale. Do not provide unknown true plant parameters to the controller unless the test explicitly evaluates a measured-parameter controller.

## 5. RBF approximation and adaptation

The selected architecture uses two RBF outputs to estimate cart- and angular-acceleration residuals. Their projection onto the instantaneous actuator input direction separates the matched component from the unmatched residual.

Before coding, document:
- the exact residual definition and the input signals used to evaluate it;
- basis-function centres, widths, scaling and approximation domain;
- ideal approximation and residual-error assumptions;
- projection metric and non-degeneracy condition;
- online weight update law and projection bounds;
- treatment of noisy or estimated signals and time-varying ideal weights.

The controller must use signals that are measurable or practically estimable, not hidden simulation truth.

## 6. Actuator and implementation model

The force-input plant is an intermediate mechanical model. A later implementation-ready model must connect the control command to applied cart force through the selected motor, transmission and amplifier model. Include force/voltage/current limits, slew rate, actuator dynamics and LQI integrator anti-windup. State explicitly which actuator quantities are measured, specified or identified.

## 7. Stability analysis

The target is a conditional uniform ultimate boundedness result. State all assumptions, including bounded approximation error, bounded disturbances, bounded actuator mismatch, parameter variation, projection properties and forward invariance of the operating region. The proof must account for actuator dynamics and any time-varying ideal RBF weights. A negative Lyapunov derivative outside a ball is not, by itself, sufficient to claim a complete UUB result unless the relevant boundedness and invariance conditions are established.

## 8. Computational verification and experiments

Use staged tests:
1. nonlinear model and inertia-matrix verification;
2. upright linearisation verification;
3. nominal LQI verification;
4. matched/unmatched projection verification;
5. RBF approximation verification;
6. adaptive-law numerical verification;
7. actuator and anti-windup verification;
8. integrated closed-loop comparison;
9. uncertainty and disturbance sweeps;
10. sensitivity, repeatability and computational-cost tests.

For each test, retain source-code commit, MATLAB release, execution date, parameter-set version, solver/sample-time settings, raw numerical output and generated figures.

## 9. Performance measures

Select measures before comparing controllers. Candidate measures include:
- settling time and peak pendulum-angle deviation;
- cart-position peak and RMS error;
- steady-state error;
- control-force peak and RMS;
- total control effort;
- saturation duration and slew-rate limiting;
- RBF residual prediction error;
- adaptation-weight boundedness;
- disturbance-rejection recovery time;
- minimum distance to rail limits;
- computational time per sample.

Define each metric mathematically and use identical initial conditions, disturbances and parameter cases for fair comparisons.

## 10. Reproducibility and reporting

Every reported result must be reproducible from a named script and parameter-set version. Separate:
- analytical derivations;
- numerical verification;
- simulation observations;
- hardware measurements;
- interpretation and limitations.

Never report a test as passed until it has been executed and its output reviewed. Do not infer stability from a plot alone. Update the results log and discussion only from actual evidence.
