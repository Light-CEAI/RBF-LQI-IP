# Mathematical System Model and Stability Analysis
## Analyzed-system branch — RBF-augmented LQI under parametric variation and disturbances

**Status:** Mathematical design specification only. No controller implementation is included in this document.

## 1. Scope and modelling decisions

This study develops a physically grounded cart–inverted-pendulum model as a tractable benchmark for an underactuated mechanical system subject to changing parameters, external disturbances, and bounded actuation.

The agreed design decisions are:

- Both cart/base and pendulum mass properties may vary.
- External horizontal force and pendulum torque disturbances are included.
- Online adaptation is restricted initially to RBF weights. The nominal LQI gains remain fixed.
- The actuator has both force saturation and a finite force slew-rate limit.
- The controller is to be developed only after the model, uncertainty decomposition, and stability argument are reviewed.

The nine existing initial-angle tests are retained: 20, 23, 24, 25, 26, 27, 28, and 29, 30 degrees, measured from the upright equilibrium.

### 1.1 What this model represents

The cart–pendulum is a benchmark for an underactuated mechanical system with two generalized coordinates and one principal control input. Its changing parameters can represent payload changes, changing attached equipment, or a controlled abstraction of changing mass distribution.

It is **not** a complete rocket model or a complete human–exoskeleton model. Rocket propellant depletion is an open-system momentum problem; human walking involves multi-link biomechanics, contact, impacts, and hybrid gait phases. Those applications motivate the uncertainty and disturbance cases, but require their own plant models before application-specific claims can be made.

## 2. Coordinates, parameters, and sign convention

Let the generalized coordinates be

\[
q = [x,\theta]^T,
\]

where \(x\) is horizontal cart displacement and \(\theta\) is the pendulum angle measured from the vertically upright position. Thus \(\theta=0\) is the desired unstable equilibrium.

The state is

\[
X=[x,\dot{x},\theta,\dot{\theta}]^T.
\]

The physical quantities are:

| Symbol | Definition | SI unit |
|---|---|---|
| \(M(t)\) | Cart/base mass | kg |
| \(m(t)\) | Pendulum rigid-body mass | kg |
| \(l(t)\) | Pivot-to-pendulum-centre-of-mass distance | m |
| \(I(t)\) | Pendulum mass moment of inertia about the pivot | kg m² |
| \(g\) | Gravitational acceleration | m/s² |
| \(F_a\) | Actual actuator force applied to the cart | N |
| \(d_x\) | External horizontal force on the cart/base | N |
| \(d_\theta\) | External torque about the pendulum pivot | N m |
| \(b_x\) | Cart viscous damping coefficient | N s/m |
| \(b_\theta\) | Pivot viscous damping coefficient | N m s/rad |

The inertia \(I\) is about the pivot, not the pendulum centre of mass. If the centre-of-mass inertia is \(I_c\), then \(I=I_c+ml^2\) by the parallel-axis theorem.

Assume initially that the pendulum is a rigid body, its centre of mass lies a distance \(l\) from the pivot, the cart moves on a horizontal track, and the actuator applies a horizontal force. The model includes viscous damping and external disturbances. Coulomb friction, backlash, track end-stops, flexible links, actuator electrical dynamics, and contact impacts are outside this first model.

## 3. Nonlinear equations from first principles

### 3.1 Kinematics

With the angle measured from upright,

\[
x_c=x+l\sin\theta,\qquad y_c=l\cos\theta.
\]

For constant parameters,

\[
\dot{x}_c=\dot{x}+l\dot{\theta}\cos\theta,\qquad
\dot{y}_c=-l\dot{\theta}\sin\theta.
\]

Therefore,

\[
v_c^2=\dot{x}^2+2l\dot{x}\dot{\theta}\cos\theta+l^2\dot{\theta}^2.
\]

### 3.2 Energies for the constant-parameter rigid-body model

The kinetic and potential energies are

\[
T=\frac12(M+m)\dot{x}^2+ml\dot{x}\dot{\theta}\cos\theta
+\frac12 I\dot{\theta}^2,
\]

\[
V=mgl\cos\theta.
\]

The Lagrangian is \(\mathcal{L}=T-V\). Applying Lagrange's equations with generalized forces \(Q_x=F_a+d_x-b_x\dot{x}\) and \(Q_\theta=d_\theta-b_\theta\dot{\theta}\) gives

\[
(M+m)\ddot{x}+ml\cos\theta\,\ddot{\theta}
-ml\sin\theta\,\dot{\theta}^{2}
+b_x\dot{x}=F_a+d_x,
\tag{1}
\]

\[
ml\cos\theta\,\ddot{x}+I\ddot{\theta}
-mgl\sin\theta+b_\theta\dot{\theta}=d_\theta.
\tag{2}
\]

Equivalently,

\[
H(q,p)\ddot q+h(q,\dot q,p)=
\begin{bmatrix}F_a+d_x\\d_\theta\end{bmatrix},
\tag{3}
\]

where

\[
H(q,p)=
\begin{bmatrix}
M+m & ml\cos\theta\\
ml\cos\theta & I
\end{bmatrix},
\]

\[
h(q,\dot q,p)=
\begin{bmatrix}
-ml\sin\theta\,\dot{\theta}^{2}+b_x\dot{x}\\
-mgl\sin\theta+b_\theta\dot{\theta}
\end{bmatrix}.
\]

The determinant of the inertia matrix is

\[
D(\theta,p)=(M+m)I-m^2l^2\cos^2\theta.
\tag{4}
\]

A physically admissible parameter set must satisfy \(M>0\), \(m>0\), \(I>0\), and \(D(\theta,p)>0\) over the operating region. For a physically consistent rigid pendulum, \(I\ge ml^2\); with \(M>0\), this ensures positive definiteness of \(H\).

The explicit acceleration equations are obtained by solving the coupled 2-by-2 system, not by separately approximating each coordinate:

\[
\ddot q=H^{-1}
\left(
\begin{bmatrix}F_a+d_x\\d_\theta\end{bmatrix}
-h(q,\dot q,p)
\right).
\tag{5}
\]

Equation (5) is the governing nonlinear plant for the constant-parameter case and the instantaneous-parameter approximation.

### 3.3 Time-varying mass properties: an essential qualification

If \(M,m,l,I\) vary with time, it is not generally correct to substitute time-varying values into equations (1)–(2) and ignore all derivative and mass-flow terms.

For a prescribed, attached rigid-body model whose kinetic energy is represented by

\[
T(t,q,\dot q)=\frac12(M+m)\dot{x}^2
+ml\dot{x}\dot{\theta}\cos\theta+\frac12I\dot{\theta}^2,
\]

the time-dependent Lagrange equations produce additional terms:

\[
(M+m)\ddot{x}+ml\cos\theta\,\ddot{\theta}
-ml\sin\theta\,\dot{\theta}^{2}
+b_x\dot{x}
+(\dot M+\dot m)\dot{x}
+(\dot m\,l+m\dot l)\dot{\theta}\cos\theta
=F_a+d_x,
\tag{6}
\]

\[
ml\cos\theta\,\ddot{x}+I\ddot{\theta}
-mgl\sin\theta+b_\theta\dot{\theta}
+(\dot m\,l+m\dot l)\dot{x}\cos\theta
+\dot I\dot{\theta}
=d_\theta.
\tag{7}
\]

These equations describe the specified time-varying kinetic-energy model. They do **not** automatically describe an open system that gains or loses mass through ports. In a mass-flow system, momentum and angular-momentum flux terms associated with incoming or expelled material must be added from a control-volume or variable-mass derivation.

For the first controller study, use two clearly separated parameter-change regimes:

1. **Slow parameter drift / attached payload variation:** use equations (6)–(7), provided the physical interpretation is a changing attached mass distribution and the parameter trajectories are prescribed or measured.
2. **Abrupt payload attachment/removal:** treat the event as a hybrid transition. Preserve the appropriate total linear and angular momentum across the event, including any impact impulse, and update the mass properties after the event. Do not model a discontinuous mass jump by differentiating a step function numerically.
   
The initial simulation should implement smooth, bounded parameter trajectories and separately documented step-change events. A later rocket-specific model must include propellant exhaust momentum flux, thrust, centre-of-mass motion, inertia variation, and slosh where relevant.

## 4. State-space form and nominal upright linearization

Let \(X=[x,\dot{x},\theta,\dot{\theta}]^T\), and define \(u=F_a\). For a fixed nominal parameter vector \(p_0=[M_0,m_0,l_0,I_0]^T\), linearize equations (1)–(2) at \(X=0,u=0,d=0\). Since \(\sin\theta\approx\theta\), \(\cos\theta\approx1\), and second-order velocity products vanish, define

\[
D_0=(M_0+m_0)I_0-(m_0l_0)^2.
\]

The linear model is

\[
\dot X=A_0X+B_0u+E_0d,
\tag{8}
\]

where \(d=[d_x,d_\theta]^T\), and

\[
A_0=
\begin{bmatrix}
0&1&0&0\\
0&-\frac{I_0b_x}{D_0}&-\frac{m_0^2g l_0^2}{D_0}&\frac{m_0l_0b_\theta}{D_0}\\
0&0&0&1\\
0&\frac{m_0l_0b_x}{D_0}&\frac{(M_0+m_0)m_0g l_0}{D_0}&-\frac{(M_0+m_0)b_\theta}{D_0}
\end{bmatrix},
\]

\[
B_0=
\begin{bmatrix}
0\\
\frac{I_0}{D_0}\\
0\\
-\frac{m_0l_0}{D_0}
\end{bmatrix},
\qquad
E_0=
\begin{bmatrix}
0&0\\
\frac{I_0}{D_0}&-\frac{m_0l_0}{D_0}\\
0&0\\
-\frac{m_0l_0}{D_0}&\frac{M_0+m_0}{D_0}
\end{bmatrix}.
\tag{9}
\]

The negative sign in the angular acceleration component of \(B_0\) follows from the upright-angle convention. It must be preserved in the implementation and independently checked against the nonlinear model.

For a cart-position integral state \(\xi\), define \(\dot\xi=r-x\). For regulation \(r=0\), the augmented state is \(X_a=[X^T,\xi]^T\), and

\[
A_a=\begin{bmatrix}A_0&0\\-C_x&0\end{bmatrix},
\quad
B_a=\begin{bmatrix}B_0\\0\end{bmatrix},
\quad C_x=[1\;0\;0\;0].
\tag{10}
\]

The nominal LQI law is \(u_c=-KX-K_i\xi\). The weights \(Q\succeq0\) and \(R>0\) must be selected transparently and reported. Before using LQI, verify stabilizability of \((A_0,B_0)\), detectability of the weighted pair, and controllability/stabilizability of the integral augmentation. The selected output integral must not introduce an uncontrollable pole at the origin.

## 5. Parametric uncertainty and disturbance decomposition

Let \(p(t)\) lie in a known compact admissible set

\[
\mathcal P=\{p:\underline p_i\le p_i\le\overline p_i,\;D(\theta,p)\ge D_{\min}>0\}.
\]

The nonlinear plant can be written relative to the nominal model as

\[
\dot X=A_0X+B_0u_c+f_\Delta(X,u_c,p,\dot p)+E_0d+w_a,
\tag{11}
\]

where \(f_\Delta\) is the total model discrepancy induced by nonlinear terms and parameter mismatch, and \(w_a\) represents effects not captured by the nominal matched-input representation, including actuator mismatch where appropriate.

This is a bookkeeping decomposition, not a claim that every component of \(f_\Delta\) is matched to the single cart-force input. The system has two mechanical degrees of freedom and one actuator; it is underactuated. In particular, a general two-component acceleration residual cannot be exactly cancelled by one scalar force.

Separate uncertainty into:

- **Matched component:** uncertainty that enters the state dynamics in the range of \(B_0\), and can in principle be counteracted through the cart force.
- **Unmatched component:** uncertainty or disturbance outside the range of \(B_0\), including general independent pivot torque effects. It cannot be cancelled exactly by a scalar cart force and must be tolerated through closed-loop robustness, bounded-state objectives, or a different actuator architecture.

The disturbance vector includes both horizontal force and pivot torque:

\[
d(t)= [d_x(t),d_\theta(t)]^T.
\]

Test bounded smooth disturbances, finite-duration force/torque pulses, and combinations with parameter drift. Ideal impulses should be analysed through jump conditions, not approximated as arbitrarily high sampled forces without a time-step study.

## 6. Bounded actuator model

The controller computes a requested force \(u_c\), but the plant receives the actual force \(F_a\). Include both magnitude and rate limits:

\[
u_{\mathrm{sat}}(t)=\operatorname{clip}(u_c(t),-F_{\max},F_{\max}),
\]

\[
\dot F_a=
\operatorname{clip}
\left(
\frac{u_{\mathrm{sat}}-F_a}{\tau_a},
-\dot F_{\max},\dot F_{\max}
\right).
\tag{12}
\]

Here \(F_{\max}\) is the force limit, \(\dot F_{\max}\) the slew-rate limit, and \(\tau_a>0\) a nominal actuator response time. A direct discrete rate limiter may instead be used if its sampling-time semantics are defined consistently. The plant equations must use \(F_a\), not the unsaturated request \(u_c\).

Because LQI contains an integrator, saturation can cause integrator windup. The final controller must include a justified anti-windup mechanism or a provably bounded integrator state. The earlier frictionless, unsaturated simulations do not validate this bounded-actuator design.

## 7. RBF approximation objective

The RBF is to estimate only the selected matched uncertainty channel for force compensation in the first implementation. It must not be trained to output arbitrary independent cart and pendulum acceleration corrections and then treated as if both were directly actuated.

Let \(z\) contain measured/available state and parameter information, for example

\[
z=[x,\dot{x},\theta,\dot{\theta},\hat M,\hat m,\hat l,\hat I]^T.
\]

Use Gaussian basis functions

\[
\phi_i(z)=\exp\left[-\frac12(z-c_i)^T\Sigma_i^{-1}(z-c_i)\right],
\quad \Phi(z)=[\phi_1,\ldots,\phi_N]^T.
\]

The matched uncertainty estimate is

\[
\hat\delta_m(z)=\hat W^T\Phi(z).
\tag{13}
\]

The parameter vector in the RBF input must be available to the controller. If masses/inertia are unknown and not measured, they cannot simply be supplied as true simulation values; an estimator or parameter-independent feature design is required. This distinction must be reflected in all tests.

The RBF approximation assumption is local to a compact operating set \(\Omega_z\):

\[
\delta_m(z)=W^{*T}\Phi(z)+\varepsilon(z),
\qquad |\varepsilon(z)|\le\bar\varepsilon.
\tag{14}
\]

The approximation bound must be estimated empirically over held-out samples and the tested operating region. Training RMSE alone does not establish a uniform error bound.

The initial architecture keeps \(K,K_i\) fixed. Only \(\hat W\) adapts online. This isolates the effect of nonlinear matched-disturbance estimation and avoids the unverified Jacobian-based gain retuning used in the earlier experimental branch.

## 8. Candidate robust RBF-augmented LQI structure

The nominal feedback request is

\[
u_{\mathrm{LQI}}=-KX-K_i\xi.
\]

If the matched uncertainty is expressed in equivalent input-force units, a candidate compensation law is

\[
u_c=u_{\mathrm{LQI}}-\hat\delta_m(z)+u_r,
\tag{15}
\]

where \(u_r\) is a bounded robustifying term designed to cover residual matched estimation error. The sign convention in (15) assumes the plant contains \(+B_0\delta_m\); it must be verified from the chosen decomposition before implementation.

A robust term may use a boundary-layer saturation function, but its sliding/error variable and gain must be derived from the closed-loop Lyapunov inequality. It is not acceptable to select a discontinuous sign term or arbitrary gain solely because it improves a plot.

A candidate projected/leaky weight update is

\[
\dot{\hat W}=
\operatorname{Proj}_{\Omega_W}
\left[
\Gamma\Phi(z)s^T-\sigma\Gamma\hat W
\right],
\tag{16}
\]

where \(s\) is the specifically derived error signal, \(\Gamma>0\) is an adaptation-rate matrix, \(\sigma\ge0\) is a leakage coefficient, and \(\Omega_W\) is a compact admissible weight set. Equation (16) is a design template, not yet a fully derived update law: dimensions and the definition of \(s\) depend on the final matched-channel reduction. The exact update law must be derived so that cross terms in \(\dot V\) cancel or are bounded. A generic RBF weight update cannot be asserted stable without that derivation.

## 9. Stability analysis: what can and cannot be claimed

### 9.1 Nominal linear closed-loop stability

For the unsaturated nominal augmented LQI system, solve the continuous-time algebraic Riccati equation and obtain \(K_a\). If the augmented pair is stabilizable and the LQI assumptions hold, the resulting nominal linear closed loop can be made Hurwitz. This establishes local nominal linear stability only.

### 9.2 Nonlinear plant and uncertainty

Let \(e\) denote the selected regulation error and \(\tilde W=W^*-\hat W\). A candidate Lyapunov function has the form

\[
V=e^TPe+\tilde W^T\Gamma^{-1}\tilde W,
\quad P=P^T>0,\;\Gamma>0.
\tag{17}
\]

The derivation must use the actual error dynamics, including unmatched disturbances, approximation residual, actuator tracking error, and saturation. The desired result is an inequality such as

\[
\dot V\le
-\alpha\|e\|^2
+c_\varepsilon\bar\varepsilon^2
+c_d\|d_u\|^2
+c_a|F_a-u_c|^2
+c_p\|\dot p\|^2,
\tag{18}
\]

for positive constants over a specified compact operating set, with any omitted terms justified. Equation (18) is a **target inequality**, not a proven result at this stage. Its constants and validity conditions must be derived from the selected controller and model.

If the residual, unmatched disturbance, parameter rate, and actuator tracking error are bounded, and the negative-definite term dominates outside a calculable neighbourhood, the appropriate conclusion may be uniform ultimate boundedness (UUB), not asymptotic convergence to zero. If a nonzero persistent unmatched torque is allowed, exact convergence to the upright equilibrium cannot generally be guaranteed by a single horizontal actuator.

### 9.3 Actuator saturation and rate limiting

The actual input differs from the requested input. Define the actuator error \(e_a=F_a-u_c\). Its effect enters the state dynamics through \(B_0e_a\), while rate limiting introduces additional actuator state dynamics. The Lyapunov analysis must include \(e_a\) or an actuator storage function and establish conditions under which saturation does not destroy the stability result.

A robust controller cannot guarantee recovery from arbitrary initial conditions or arbitrarily large disturbances with finite actuator force. The theorem must specify a region of attraction or a bounded disturbance/initial-condition set. Anti-windup is mandatory for the LQI integral state under sustained saturation.

### 9.4 Parameter variation

For slow variation, the stability result must include bounds on \(\|\dot p\|\) and the admissible parameter set. For abrupt changes, a continuous-time \(\dot p\) bound is not applicable at the jump. A hybrid stability argument is needed: show that the state remains in the admissible region across the jump and that the Lyapunov function does not increase excessively, or explicitly bound the jump in \(V\). Momentum-consistent reset maps must be used for physical mass attachment/removal events.

### 9.5 Claims deliberately not made

At this stage we do not claim:

- Global asymptotic stability of the nonlinear plant.
- Exact cancellation of unmatched pendulum torque disturbances.
- Stability under arbitrary mass jumps or unbounded disturbances.
- Stability under arbitrary actuator saturation.
- Generalisation to rockets, exoskeletons, or prostheses without application-specific models.
- That an RBF training RMSE proves a uniform approximation bound.
- That a stable frozen-time LQI design guarantees stability for a rapidly time-varying plant.

## 10. Physical scenarios and how they map to the model

### 10.1 Rocket fuel depletion analogy

A rocket loses propellant through an exhaust stream. Its mass and centre of gravity change, and its rotational inertia may change. The translational variable-mass equation includes exhaust momentum flux; in a simple ideal rocket this produces thrust \(T=v_e\dot m_f\). Rotational dynamics may include \(\dot I\,\omega\) and angular-momentum flux, plus slosh and structural modes.

The pendulum benchmark can test slow parameter drift and changes in inertia, but it does not contain rocket thrust-vector dynamics, exhaust flux, aerodynamics, flexible-body modes, or slosh. Those terms must be derived in a rocket model before transferring the controller.

### 10.2 Exoskeleton/prosthesis analogy

Human-device dynamics include changing effective inertia, joint torques, interaction forces, and gait-dependent contact constraints. A push can be represented as a bounded external force/torque pulse for a first disturbance-rejection benchmark. Foot strike and falls are hybrid impact/contact events, not merely smooth disturbances. A realistic exoskeleton model requires multiple links, joint actuation, human interaction, contact constraints, and safety-critical limits.

The cart-pendulum benchmark can therefore assess robustness concepts, not certify human-assistive performance or safety.

## 11. Required validation programme

Use identical plant parameters, initial conditions, disturbance profiles, actuator limits, and numerical tolerances for each controller comparison.

| Test | Variation/disturbance | Main question |
|---|---|---|
| V1 | Nominal fixed parameters, no disturbance | Does the baseline reproduce the nominal response? |
| V2 | Slow increase/decrease in cart/base mass | Does performance remain bounded under base-mass drift? |
| V3 | Slow pendulum mass and inertia variation | Does the estimator handle changing pendulum dynamics? |
| V4 | Smooth centre-of-mass shift | Is the model robust to changing \(l\) and \(I\)? |
| V5 | Finite horizontal push pulse | How quickly does the pendulum recover? |
| V6 | Finite pivot torque pulse | How does the underactuated system respond to unmatched disturbance? |
| V7 | Combined parameter drift and force/torque pulse | Does simultaneous uncertainty remain bounded? |
| V8 | Payload attachment/removal event | Is the hybrid reset and momentum model physically consistent? |
| V9 | Force saturation and slew-rate limiting | Does the controller remain bounded without integrator windup? |
| V10 | Combined worst-case admissible scenario | Does the system stay within the declared operating region? |

Compare at minimum: conventional fixed LQI, fixed LQI with RBF matched-channel compensation, and robust RBF-augmented LQI. The earlier Jacobian-adaptive LQI may be retained as a comparator, but not presumed superior.

Report angle and cart-position peak values, overshoot, settling time, IAE, ISE, ITAE, peak and RMS actual actuator force, time spent at saturation, recovery time after disturbance, RBF held-out estimation error, and any violation of state/parameter/actuator bounds. Report repeated trials or a parameter sweep rather than a single favourable trajectory.

## 12. Decisions required before implementation

The following must be fixed in a parameter/configuration specification before coding:

1. Nominal values and admissible ranges for \(M,m,l,I\), with physically consistent inertia constraints.
2. Whether each changing parameter is measured, estimated, or unknown to the controller.
3. Smooth parameter trajectories and event/reset maps for sudden payload changes.
4. Maximum horizontal disturbance and pivot torque, including pulse duration and slew.
5. Actuator force, slew-rate, and actuator time-constant limits.
6. Cart travel and pendulum-angle operating bounds.
7. Sampling period and numerical integration tolerances.
8. LQI weighting matrices and the exact integrated output.
9. The matched-channel projection used to define the RBF target.
10. A complete Lyapunov derivation for the selected RBF update, robust term, actuator model, and anti-windup mechanism.

## 13. Design conclusion

The physically defensible starting point is the coupled nonlinear model (1)–(3), with time-varying attached-mass terms (6)–(7) used only under their stated assumptions. The plant is underactuated: one cart force cannot independently cancel arbitrary force and torque disturbances. The proposed RBF must therefore estimate a carefully defined matched uncertainty channel, while unmatched disturbances are handled by a robust stability design and bounded-performance objective.

The initial controller design should retain fixed nominal LQI gains and adapt only the RBF weights. Force and slew-rate limits must be represented at the plant input, and integral windup must be addressed. The stability objective is a defensible local/compact-set result—likely uniform ultimate boundedness under bounded approximation error and disturbances—rather than an unsupported claim of global asymptotic stability.

**No MATLAB controller code should be written until Sections 3–9 are reviewed, the uncertainty channel is mathematically defined, and the Lyapunov inequality including actuator and integral-state effects has been completed.**
