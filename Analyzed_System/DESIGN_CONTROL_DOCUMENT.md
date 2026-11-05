# Research Design Control Document

**Project title:** Robust RBF-Augmented LQI Control of a Single Inverted Pendulum under Parametric Uncertainty, External Disturbances and Actuator Constraints  
**Repository:** `Light-CEAI/RBF-LQI-IP`  
**Controlled branch:** `analyzed-system`  
**Document status:** Living, authoritative design reference; mathematical design is not yet approved for controller implementation.  
**Last design consolidation:** 2026-11-04

> **Authority and change control.** This document records the currently agreed research scope, design decisions, development sequence and acceptance gates. Detailed equations and evidence remain in the linked companion documents. If a detailed document conflicts with this control document, record the discrepancy and resolve it explicitly; do not silently choose one version. Every material design change must be justified in the activity/decision log and reflected in all affected documents.

## 1. Research purpose and design philosophy

The study investigates whether an RBF augmentation can improve the practical robustness and disturbance-handling performance of a conventional LQI controller for a single cart-mounted inverted pendulum, under declared parameter variation, external disturbances and actuator constraints.

The RBF is an augmentation to a conventional feedback controller, not a replacement for the physical actuator or a means of cancelling arbitrary plant dynamics. The plant is underactuated: it has two generalized coordinates and one horizontal force input. Therefore, arbitrary two-coordinate acceleration residuals cannot in general be independently cancelled. Complete residual cancellation is neither an objective nor an acceptance criterion.

The research must make proportionate claims. Any improvement must be demonstrated against a conventional LQI baseline under matched test conditions. Cases where augmentation gives little benefit or degrades performance must also be reported. Stability claims must follow from a complete proof with explicit hypotheses; simulation alone does not prove stability.

## 2. Fixed research scope

### 2.1 Included

- One cart/base translating horizontally and one rigid inverted-pendulum link.
- Two generalized coordinates, cart position (x) and pendulum angle (	heta), with (	heta=0) at upright.
- Variation in cart/base and pendulum mass properties, within physically admissible bounds.
- Horizontal external force disturbance and independent pendulum-pivot torque disturbance.
- A conventional LQI baseline with fixed nominal feedback gains during the initial study.
- RBF approximation of cart- and angular-acceleration residuals, with online adaptation of RBF weights as the selected adaptive architecture.
- A single actuator force command, subject to magnitude and slew-rate constraints.
- Comparative simulation over a declared operating envelope and a staged mathematical/numerical verification programme.

### 2.2 Excluded from the present plant and claims

- A double inverted pendulum or multi-link model.
- A complete rocket/open-system variable-mass model.
- A complete human biomechanics, exoskeleton, prosthesis or walking-contact model.
- Claims that the controller can independently command both generalized accelerations.
- Claims of exact cancellation of all matched and unmatched uncertainty.
- Global stability, guaranteed superiority, or hardware validation unless separately established.
- Final controller implementation before the mathematical derivation and stability review are accepted.

Other application areas may motivate the general problem, but no application-specific result transfers without a separate plant model and analysis.

### 2.3 Research/hardware boundary

The research is framed as a general single cart-mounted inverted-pendulum control study, not as a controller designed specifically for a named commercial platform. Manufacturer documentation may be used as a source of realistic parameter values. The provisional 1.0 m project rail length is a design assumption, not a manufacturer claim or usable-travel specification. Physical rack length, usable cart travel and safety margin must be distinguished.

## 3. Agreed architecture and design decisions

| Design item | Current decision | Qualification / open condition |
|---|---|---|
| Plant | Single rigid pendulum on cart | Verify physical parameter consistency and system boundary |
| Coordinates | (q=[x,	heta]^T), upright at (	heta=0) | Maintain this sign convention in all derivations and code |
| State | (X=[x,dot{x},	heta,dot{	heta}]^T) | State ordering is controlled |
| Nominal controller | LQI | Confirm stabilisability, integral augmentation and Riccati solution |
| Adaptation | Online RBF weight adaptation; nominal LQI gains remain fixed initially | Adaptation law and projection proof are not yet accepted |
| RBF outputs | Two acceleration residual estimates: cart and pendulum | Regressors must use available/measurable or estimable signals |
| Actuator | One horizontal force input | Include force and slew-rate constraints; actuator/drive dynamics remain to be specified |
| Disturbances | Horizontal force and pivot torque | Distinguish matched and unmatched effects |
| Uncertainty | Cart/base and pendulum mass properties vary | Declare bounded ranges and rates; abrupt changes require separate event treatment |
| Projection | Weighted instantaneous projection onto the actuator input direction | Must be derived with consistent units, conditioning and parameter knowledge |
| Stability target | Conditional UUB/practical boundedness is a candidate objective | Claim only if complete proof, invariance and actuator hypotheses are established |
| Evaluation | Compare conventional LQI and RBF-augmented variants | Same plant, initial conditions, disturbance, actuator limits and solver conditions |

## 4. Physical model and sign conventions

The constant-parameter rigid-body model is

[
H(q,p)ddot q+h(q,dot q,p)=
egin{bmatrix}F_a+d_x\d_\thetaend{bmatrix},
]

with

[
H=
egin{bmatrix}
M+m & mlcos\theta\
mlcos\theta & I
end{bmatrix},qquad
D=(M+m)I-m^2l^2cos^2\theta.
]

Here (I) is pendulum inertia about the pivot, (l) is pivot-to-COM distance, (F_a) is actual applied horizontal force, (d_x) is horizontal disturbance force, and (d_\theta) is pivot disturbance torque. The complete equations, energy derivation, state-space linearisation and time-varying attached-body model are maintained in the mathematical companion document.

Physical admissibility includes positive mass properties and positive-definite inertia matrix over the declared operating region. For a rigid pendulum, use (I=I_c+ml^2) when the centre-of-mass inertia (I_c) is the source quantity. Parameter values must carry provenance labels: published, derived, measured, estimated or assumed. Do not freeze a parameter set until source configuration and uncertainty are documented.

**Variable-parameter caveat:** the existing time-varying Lagrangian formulation is for a prescribed attached-body kinetic-energy model. It is not a general open-system mass-flow/rocket equation. Smooth parameter variation and abrupt payload events are different modelling cases; abrupt events may require momentum-consistent reset maps and separate stability analysis.

## 5. RBF-to-actuator interface: governing principle

The RBF estimates an acceleration-residual vector

[
hat r_q=[hat r_{ddot{x}},hat r_{ddot\theta}]^T.
]

The controller has only one horizontal force input. Therefore the two-output estimate must not be added directly to the scalar LQI force command. A mathematically defined projection onto the instantaneous actuator input direction may produce a scalar equivalent-force estimate in newtons. The projection metric must include physically meaningful acceleration scales so the weighted objective is dimensionally consistent.

The component represented by the actuator direction is the matched component. The orthogonal remainder under the selected metric is unmatched and remains in the closed-loop dynamics. It must be bounded/evaluated in the analysis and tests, not assumed cancelled. An independent pivot torque disturbance is generally not directly matched by the horizontal actuator.

The residual definition must consistently compare actual and nominal dynamics, including the difference between actual and nominal input directions and the distinction between commanded force and applied force. Derive the sign of compensation from that definition; do not assume a sign by intuition. The actual actuator force, not merely the requested command, enters the plant equations.

## 6. Adaptation, actuator and stability requirements

Before accepting the online adaptation law, specify the RBF regressor, basis centres/widths, input scaling, approximation domain, ideal-weight assumptions, approximation-error bound, weight constraint set and projection operator. Address measurement noise, unavailable signals, time-varying ideal weights and parameter-rate effects.

The actuator model must distinguish requested force, limited command and actual applied force. Force saturation, slew-rate limitation, actuator lag, drive limits and LQI integrator anti-windup must be treated consistently. Do not assume actuator tracking error is small when the actuator is saturated.

A candidate Lyapunov analysis must include the LQI-augmented plant state and RBF weight error. Explicitly account for unmatched residuals, approximation error, external disturbances, actuator mismatch and any robust term. Establish boundedness of the adaptive weights, forward invariance of the operating domain, and that the relevant Lyapunov sublevel set remains inside the region where assumptions hold. If ideal approximation weights vary with time, include or bound their derivative term. A negative derivative outside a ball is not, by itself, a complete UUB proof.

## 7. Comparative evaluation plan

The minimum controller set is:
1. Conventional LQI.
2. Fixed/offline RBF-augmented LQI, if retained as a useful intermediate benchmark.
3. Online-adaptive RBF-augmented LQI.

Use identical plant conditions, initial states, disturbances, actuator constraints, simulation horizon and numerical solver settings for each comparison. Predeclare parameter ranges and disturbance profiles. The test matrix should cover:
- initial-angle recovery over the agreed operating range;
- cart and pendulum mass-property variation;
- horizontal force disturbances;
- pivot torque disturbances;
- simultaneous uncertainty and disturbances;
- force saturation and slew-rate limitation;
- sensitivity to RBF centres, widths, learning rate and projection bounds;
- repeatability and cases in which augmentation does not improve results.

Report at least cart-position error/travel, pendulum angle and angular rate, settling/recovery time, peak deviation, control effort, actuator saturation duration, slew-rate activity, adaptation-weight bounds and any constraint violations. Define exact metrics and acceptance thresholds before each test. Report median/worst-case or distributional results across declared sweeps where appropriate, not only a favourable trajectory.

## 8. Evidence and claim discipline

- A derivation is not verified merely because it is written in symbolic form.
- A numerical identity check verifies only the implemented identity and tested domain.
- A finite-difference Jacobian check supports local algebraic consistency, not physical model validity.
- Simulation supports performance claims only for the tested conditions and implementation.
- Parameter provenance is separate from mathematical correctness.
- A literature citation supports a general method/theorem, not automatically the satisfaction of its hypotheses by this system.
- Preserve failed, inconclusive and adverse results.
- Never state that MATLAB was run unless execution evidence was supplied or generated in the relevant environment.

## 9. Development gates and sequence

| Gate | Required work / evidence | Exit condition |
|---|---|---|
| G0 — Design control | This document and linked registers aligned | Scope and decisions recorded; contradictions resolved |
| G1 — Model verification | Constant-parameter model, time-varying model qualification and diagnostic correction | Equations, units, physical constraints and verification interpretation reviewed |
| G2 — Parameter set | Traceable nominal set and uncertainty ranges | Sources/configuration/provenance and derived quantities checked |
| G3 — Nominal LQI | Upright Jacobians, stabilisability, integral augmentation and Riccati design | Nominal baseline analytically and numerically checked |
| G4 — Residual/projection | Actual-vs-nominal residual, input direction, matched/unmatched split | Identity, units, conditioning and compensation sign verified |
| G5 — RBF/adaptation | Regressor, approximation assumptions, online law and projection | Boundedness properties and signal availability reviewed |
| G6 — Actuator | Force/slew/lag model and anti-windup | Requested/applied force relationship and limits verified |
| G7 — Stability | Complete conditional boundedness/UUB analysis | All assumptions, invariance and actuator terms accounted for |
| G8 — Simulation | Controlled comparison and robustness test suite | Reproducible evidence logged; claims restricted to tested envelope |
| G9 — Reporting | Methods, results, discussion and traceability | Report matches repository evidence and limitations |

**Controller implementation is not authorised by this plan until G1–G7 have been reviewed and the mathematical design is explicitly accepted.** Verification scripts for the model may be developed before controller code.

## 10. Current evidence and unresolved work (2026-11-04)

- **V-001:** User-reported MATLAB checks passed for the illustrative constant-parameter set: inertia positive definiteness over the tested angle interval and finite-difference agreement of upright (A,B,E) Jacobians. This does not validate physical parameters, actuator behaviour or closed-loop stability.
- **V-002:** Corrected componentwise MATLAB rerun passed and is closed for the illustrative smooth attached-body trajectory. Cart and pendulum identity errors passed separate unit-labelled tolerances; pivot inertia margin was positive; constant-parameter rate terms were zero. The generalized-force mismatch is retained as a componentwise diagnostic only. See RESULTS_LOG.md.
- **Parameter register:** not frozen; physical values and actuator data require exact source/configuration and uncertainty.
- **V-006 residual/projection:** Exact actual-versus-nominal identity and weighted matched/unmatched projection are documented in ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md; V-006 MATLAB algebraic checks passed on the illustrative five-sample grid (see RESULTS_LOG.md). **RBF adaptation, actuator model, complete stability proof and controller simulations:** open; no final controller implementation is approved.

## 11. Controlled repository references

- [Mathematical model and stability analysis](MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md)
- [Literature and derivation traceability](LITERATURE_AND_DERIVATION_TRACEABILITY.md)
- [Methodology](../Research_Reporting/METHODOLOGY.md)
- [Parameter register](../Research_Reporting/PARAMETER_REGISTER.md)
- [Verification and simulation test register](../Research_Reporting/TEST_REGISTER.md)
- [Results log](../Research_Reporting/RESULTS_LOG.md)
- [Discussion](../Research_Reporting/DISCUSSION.md)
- [Research activity and decision log](../Research_Reporting/RESEARCH_ACTIVITY_LOG.md)
- [Verification scripts and instructions](verification/)

## 12. Required maintenance

At the end of each substantive research session:
1. Update this document if scope, architecture, assumptions, gate status or decisions changed.
2. Update the relevant detailed technical document and literature traceability.
3. Update test register and results log only from actual evidence.
4. Record decisions, rejected alternatives, open issues and next action in the activity log.
5. Ensure links, branch names, equation numbering and test statuses agree.
6. Commit related changes with descriptive messages; record the resulting commit(s) here or in the activity log.

This document is a design-control reference, not a substitute for the full derivations, literature sources, test outputs or stability proof.


### Projection/signal-availability gate update (2026-11-04)

A companion specification, [Projection metric and online signal availability](PROJECTION_SCALING_AND_SIGNAL_AVAILABILITY.md), now formalises the dimensional requirements for the weighted metric, the procedure for selecting its scales from a declared operating envelope, and the distinction between the true analysis input direction and an implementable estimated direction. It also audits the online availability of state, nominal request, applied force, varying parameters/rates, disturbances and acceleration targets.

**Decision:** No numerical projection scales or continuous-domain lower bound are approved because the parameter register and operating envelope remain unfrozen. The earlier 0.08 m/s² and 0.10 rad/s² values remain provisional examples only. The RBF regressor cannot silently use true simulation parameters or rates. Two possible routes are retained for review: parameter-informed approximation using bounded online estimates, or state/input-only approximation with a defensible robust bound for hidden parameter variation. Neither route is selected yet. The controller implementation gate remains closed.


### MATLAB projection-metric audit prepared (V-011)

Added [audit function](verification/audit_projection_metric_envelope.m), [configuration template](verification/projection_metric_config_TEMPLATE.m), and [run instructions](verification/README_PROJECTION_METRIC_AUDIT.md). The tool evaluates \(D(\theta)\) and \(b_q^TSb_q\) over user-supplied parameter/angle grids and reports a predeclared scale-ratio sensitivity sweep. All configuration values are intentionally blank until the parameter and operating-envelope register is justified. V-011 is pending user MATLAB execution; even a pass will be a finite-grid numerical diagnostic, not a certified continuous-domain lower bound or stability result.
