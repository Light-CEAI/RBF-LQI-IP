# Literature and Derivation Traceability Register

**Repository:** `Light-CEAI/RBF-LQI-IP`  
**Branch:** `analyzed-system`  
**Companion document:** [Mathematical System Model and Stability Analysis](MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md)

## 1. Purpose and mandatory evidence standard

This register identifies the literature basis for the mathematical model and the proposed control architecture. It is intended to make the derivation auditable: a reader should be able to trace each important modelling choice, equation family, design method, assumption, and stability claim to a recognised source.

A reference supports the underlying method or theorem; it does **not** automatically prove that the method's hypotheses hold for this particular plant. All system-specific algebra, dimensions, sign conventions, parameter restrictions, and Lyapunov inequalities must still be derived and checked in this project.

### Mandatory rules for future commits

1. Cite sources in the body immediately after the equation, modelling statement, assumption, or method they support, using author–year citations (for example, `[Spong, Hutchinson & Vidyasagar, 2006]`).
2. Give complete bibliographic entries and DOI/publisher links where available in Section 4.
3. Distinguish **standard result**, **system-specific derivation**, and **design assumption**. Do not cite our own derivation as if it were a published result.
4. For a theorem, state all hypotheses used (regularity, boundedness, compact operating set, positive definiteness, stabilizability, actuator feasibility, etc.) and show where each is verified or remains an assumption.
5. Prefer original peer-reviewed papers for specific algorithms/results and established textbooks/monographs for foundational theory. Avoid citing search snippets, lecture slides, blogs, or unreviewed material as the sole authority for a central claim.
6. Cite the exact edition/version consulted. Add page, chapter, theorem, or equation numbers after checking the source itself; do not invent pinpoint references.
7. A simulation result is not a theorem and cannot validate a uniform approximation bound or prove stability.
8. Any new modelling domain (rocket, exoskeleton, prosthesis) requires its own domain-specific model and sources. The cart–pendulum analogy is not a transfer-of-proof.

## 2. Equation- and section-level traceability

The equation numbers below refer to the current companion mathematical document. They are a first-pass source map; in-text citations must also be inserted into that document during the next documentation pass.

| Companion document location | Mathematical content / claim | Primary supporting literature | Project-specific obligation |
|---|---|---|---|
| §§2–3.1, kinematics | Generalized coordinates, rigid-body centre-of-mass kinematics, upright-angle convention | Spong, Hutchinson & Vidyasagar (2006); Murray, Li & Sastry (1994) | Re-derive coordinates from the chosen angle convention; independently verify signs. |
| §3.2, (T,V,\mathcal L), Eqs. (1)–(5) | Euler–Lagrange formulation, coupled inertia matrix, nonlinear equations of motion | Spong et al. (2006); Murray et al. (1994) | Show the two generalized-force equations and matrix inversion; verify symmetry and positive definiteness of (H). |
| §3.2, Eq. (4) | Inertia determinant and admissibility | Spong et al. (2006); Murray et al. (1994) | Verify (D>0) over the complete declared angle/parameter set, not only at the upright point. |
| §3.3, Eqs. (6)–(7) | Time-dependent Lagrangian parameters | Standard analytical mechanics; see Goldstein, Poole & Safko (2002) | Clearly restrict this to the prescribed attached-mass kinetic-energy model. It is not a general open-system variable-mass equation. |
| §3.3, rocket qualification | Open-system momentum balance, exhaust momentum flux and rocket equation | Sutton & Biblarz (2017); Anderson (2017) | Derive mass-flow and thrust terms for any rocket-specific model; do not transplant Eqs. (6)–(7) as a rocket model. |
| §3.3, abrupt payload changes | Hybrid reset and momentum-consistent impact/event treatment | Leine & Nijmeijer (2004) | Specify pre/post-event states, impulses, conservation law, and reset map for each physical event. |
| §4, Eqs. (8)–(10) | Upright linearization, state-space matrices, integral augmentation and LQI/LQR | Franklin, Powell & Emami-Naeini (2019/2021 edition as consulted); Anderson & Moore (2007) | Recompute Jacobians from the nonlinear model; test stabilizability/detectability and the augmented pair before accepting an LQI gain. |
| §5, uncertainty decomposition | Matched/unmatched uncertainty relative to the input distribution | Isidori (1995); Khalil (2002) | Define the actual nonlinear input vector field and show the projection/decomposition algebra. A single input cannot cancel arbitrary two-coordinate acceleration residuals. |
| §6, actuator model | Magnitude/rate saturation and actuator dynamics | Hu & Lin (2001); Galeani, Onori, Teel & Zaccarian (2008) | Treat the actuator state and rate limit in the closed-loop model; do not merely assume the tracking error is small. |
| §6, anti-windup | Integral windup and anti-windup synthesis | Zaccarian & Teel (2011) | Choose a specific anti-windup architecture and include its state in the stability proof. |
| §7, Gaussian RBF | RBF basis functions and approximation on compact sets | Park & Sandberg (1991); Lewis, Jagannathan & Yeşildirek (1998) | Define the compact approximation domain, centres, widths, scaling, and a defensible residual bound. Universal approximation alone gives no numerical uniform error bound for a finite chosen network. |
| §8, RBF compensation | Adaptive neural/RBF control with Lyapunov analysis | Lewis et al. (1998); Polycarpou & Ioannou (1996); Ioannou & Sun (1996) | Derive the scalar matched-channel target and sign from the plant equations. |
| §8, projection update | Parameter projection to keep estimates in a compact admissible set | Ioannou & Sun (1996); Lavretsky & Wise (2013) | State the exact projection convention and prove the required non-positive Lyapunov contribution under that convention. |
| §9, Lyapunov/UUB | Lyapunov stability, ultimate boundedness and robust adaptive control | Khalil (2002); Ioannou & Sun (1996); Polycarpou & Ioannou (1996) | Complete the derivative inequality for the actual augmented plant; establish forward invariance of the approximation/operating set and bound every residual term. |
| §9, time-varying parameters | Slowly varying parameters and time-varying ideal weights | Ioannou & Sun (1996); Khalil (2002) | Include the \\(\dot W^*\\) term if ideal weights vary; alternatively define a fixed approximator over the full joint state/parameter domain. |
| §9, jumps | Hybrid Lyapunov/jump conditions | Leine & Nijmeijer (2004) | Prove or bound the Lyapunov jump and state reset; continuous-time UUB does not cover discontinuous events by itself. |
| §10.1, rocket | Rocket propulsion and variable-mass dynamics | Sutton & Biblarz (2017); Anderson (2017) | Separate thrust, exhaust flux, changing centre of gravity/inertia, aerodynamics, and slosh where applicable. |
| §10.2, human-device system | Human–robot interaction, wearable robotics and gait/contact dynamics | Pons (2008); dollar & Herr (2008) | Build a multi-link/contact model before making claims about exoskeleton or prosthesis performance or safety. |

## 3. Critical mathematical checks attached to the references

These are not optional literature formalities; they are conditions that the project must verify.

### 3.1 Euler–Lagrange model

- The inertia matrix must be symmetric and positive definite over the declared physical set.
- (I) must be defined consistently: pivot inertia (I=I_c+ml^2), not centre-of-mass inertia substituted directly.
- The sign of gravity and the input direction must follow the upright-angle convention.
- Nonlinear acceleration equations must be obtained by inverting the coupled inertia matrix, not by independently approximating the two accelerations.

### 3.2 Time-varying mass and open systems

- A time-dependent Lagrangian for attached bodies is not interchangeable with a control-volume balance for mass entering or leaving a system.
- Rocket propellant depletion requires momentum flux of the exhaust; rotational models may also require angular-momentum flux.
- Payload attachment/removal may involve impact impulses and a reset map. A parameter step with unchanged velocity is only a modelling idealisation if momentum transfer is neglected deliberately and justified.

### 3.3 RBF approximation and adaptation

- The approximation statement is local to a specified compact domain unless a stronger theorem and its conditions are established.
- Finite-network approximation error must be bounded or conservatively estimated over the domain; training RMSE is not a uniform bound.
- The RBF input must contain only signals available to the controller. True but unmeasured simulation parameters cannot be treated as online measurements.
- If (\Theta^*(t)) is time varying, the Lyapunov derivative contains the ideal-weight-rate contribution. A fixed ideal weight is justified only when the approximator is defined over a joint domain that includes the varying parameters and relevant rates.
- The projection operator's exact inequality and dimensions must match the selected matrix-weight convention.

### 3.4 Matched-channel projection

For an acceleration residual (r_q\in\mathbb R^2) and actual scalar-force acceleration direction (b_q\in\mathbb R^2), a weighted least-squares projection is a system-specific construction:
[
\delta_m=\arg\min_{\delta\in\mathbb R}(r_q-b_q\delta)^T S(r_q-b_q\delta)
=\frac{b_q^T S r_q}{b_q^T S b_q},
\]
where (S=S^T>0). The residual (r_{q,u}=r_q-b_q\delta_m) satisfies (b_q^T S r_{q,u}=0).

This formula follows by differentiating the strictly convex scalar quadratic objective; it is not a special adaptive-control theorem. The metric (S), physical units/scales, and the lower bound (b_q^TSb_q\ge\beta_b>0) must be justified. Use a symbol such as (S) for the metric and reserve (Theta) for RBF weights.

Crucially, if (b_q) depends on unknown parameters, the controller cannot use the true (b_q) unless those parameters are measured. Use an estimated input direction and include its projection/model error in the stability bound.

### 3.5 UUB and actuator limits

- A derivative that is negative outside a ball in the plant error does not automatically prove boundedness of the complete adaptive state unless the weight error is also bounded and the operating set is forward invariant.
- A bounded weight estimate from projection does not prove that the plant remains in the RBF approximation domain.
- The actuator tracking error cannot simply be declared small under arbitrary command saturation. Its dynamics, force/rate limits, command derivative and integral anti-windup must be included or bounded through a justified regional argument.
- The robust term must be explicitly designed. If no robust term is selected, set it to zero and prove the corresponding practical/UUB result for the bounded unmatched terms.
- Abrupt parameter jumps require hybrid jump conditions; slow-variation UUB does not automatically extend through jumps.

## 4. Bibliography

The following references are the initial core library. Verify exact edition and pinpoint pages before adding page-level citations in the derivation document.

1. Anderson, J. D. (2017). *Introduction to Flight* (8th ed.). McGraw-Hill Education. Foundational aircraft/flight mechanics text; use alongside a dedicated propulsion source for rocket mass-flow derivations.
2. Anderson, B. D. O., & Moore, J. B. (2007). *Optimal Control: Linear Quadratic Methods*. Dover Publications. LQ optimal control and Riccati theory.
3. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). *Feedback Control of Dynamic Systems* (8th ed.). Pearson. State-space feedback, linear systems and control design. Confirm the edition/year against the copy actually consulted.
4. Galeani, S., Onori, S., Teel, A. R., & Zaccarian, L. (2008). A magnitude and rate saturation model and its use in the solution of a static anti-windup problem. *Systems & Control Letters, 57*(1), 1–9. https://doi.org/10.1016/j.sysconle.2007.06.011
5. Goldstein, H., Poole, C., & Safko, J. (2002). *Classical Mechanics* (3rd ed.). Addison-Wesley. Lagrangian mechanics and time-dependent generalized formulations.
6. Hu, T., & Lin, Z. (2001). *Control Systems with Actuator Saturation: Analysis and Design*. Birkhäuser. https://doi.org/10.1007/978-1-4612-0205-9
7. Ioannou, P. A., & Sun, J. (1996). *Robust Adaptive Control*. Prentice Hall. Adaptive laws, robustness modifications and projection-based parameter constraints.
8. Isidori, A. (1995). *Nonlinear Control Systems* (3rd ed.). Springer. Nonlinear systems, input distributions and geometric control foundations.
9. Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall. Lyapunov stability, invariant sets and ultimate boundedness.
10. Lavretsky, E., & Wise, K. A. (2013). *Robust and Adaptive Control: With Aerospace Applications*. Springer. https://doi.org/10.1007/978-1-4471-4396-3
11. Leine, R. I., & Nijmeijer, H. (2004). *Dynamics and Bifurcations of Non-Smooth Mechanical Systems*. Springer. Hybrid and nonsmooth mechanical dynamics.
12. Lewis, F. L., Jagannathan, S., & Yeşildirek, A. (1998). *Neural Network Control of Robot Manipulators and Non-Linear Systems*. Taylor & Francis/CRC Press. Neural adaptive control and Lyapunov-based designs.
13. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press. Rigid-body mechanics and manipulator dynamics.
14. Park, J., & Sandberg, I. W. (1991). Universal approximation using radial-basis-function networks. *Neural Computation, 3*(2), 246–257. https://doi.org/10.1162/neco.1991.3.2.246
15. Pons, J. L. (Ed.). (2008). *Wearable Robots: Biomechatronic Exoskeletons*. Wiley. Wearable robotics and human-device mechanics.
16. Polycarpou, M. M., & Ioannou, P. A. (1996). A robust adaptive nonlinear control design. *Automatica, 32*(3), 423–427. https://doi.org/10.1016/0005-1098(95)00147-6
17. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. Lagrangian rigid-body dynamics and robot control.
18. Sutton, G. P., & Biblarz, O. (2017). *Rocket Propulsion Elements* (9th ed.). Wiley. Rocket propulsion, thrust and variable-mass propulsion.
19. Zaccarian, L., & Teel, A. R. (2011). *Modern Anti-windup Synthesis: Control Augmentation for Actuator Saturation*. Princeton University Press. https://doi.org/10.23943/princeton/9780691147321.001.0001
20. Dollar, A. M., & Herr, H. (2008). Lower extremity exoskeletons and active orthoses: challenges and state-of-the-art. *IEEE Transactions on Robotics, 24*(1), 144–158. https://doi.org/10.1109/TRO.2008.915453

## 5. Reference verification queue

Before the mathematical specification is frozen:

- [ ] Verify the exact Franklin et al. edition/year and correct the entry to match the edition actually used.
- [ ] Verify all publisher metadata and DOI details against publisher/Crossref records.
- [ ] Add page/chapter/equation pointers for the Lagrangian derivation, RBF approximation theorem, projection property, anti-windup method, and UUB theorem.
- [ ] Add an authoritative source for the specific weighted projection/orthogonality construction if it is presented as a standard method; otherwise label it explicitly as our weighted least-squares derivation.
- [ ] Add an authoritative source for the exact hybrid payload reset model selected for the physical event.
- [ ] Add human–robot interaction/contact references before constructing an exoskeleton-specific model.
- [ ] Insert the corresponding author–year citations directly into the companion mathematical document, not only in this register.

## 6. Scope statement

This register supports a cart–inverted-pendulum benchmark and the mathematical methods proposed for it. It does not certify a rocket controller, exoskeleton controller, or prosthesis controller. Those are distinct physical systems requiring separate governing equations, sensing assumptions, safety constraints, and stability proofs.
