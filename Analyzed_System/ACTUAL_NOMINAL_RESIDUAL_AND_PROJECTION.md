# Actual-versus-Nominal Residual and Matched/Unmatched Projection

**Branch:** `analyzed-system`  
**Status:** Derivation for review; no controller implementation authorised.  
**Purpose:** Establish an exact residual identity, define a physically meaningful equivalent-force projection, and expose all terms that remain outside the horizontal actuator channel.

## 1. Scope and conventions

Use the state and upright-angle convention in [MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md](MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md):
\[
X=[x,\dot x,\theta,\dot\theta]^T,\qquad q=[x,\theta]^T,\qquad \theta=0\text{ upright}.
\]
The scalar physical actuator input is the **applied cart force** \(F_a\), in N. The LQI signal \(u_L\) is the nominal requested force before RBF compensation. Do not identify \(F_a\), the limited actuator command, and \(u_L\) as the same signal.

The derivation applies to the attached-body time-varying Lagrangian model in equations (6)–(7) of the companion model. It does not create an open-system mass-flow model. The nonlinear mechanics follow the standard Lagrangian formulation for rigid-body systems [Spong, Hutchinson & Vidyasagar, 2006; Goldstein, Poole & Safko, 2002]. The projection below is a project-specific weighted least-squares construction, not a theorem attributed to those sources.

## 2. Actual nonlinear plant and input direction

Let \(p(t)=(M,m,l,I,b_x,b_\theta)\), and define
\[
H(q,p)=\begin{bmatrix}M+m&ml\cos\theta\\ml\cos\theta&I\end{bmatrix},
\qquad D=(M+m)I-m^2l^2\cos^2\theta.
\]
For the attached-body time-varying model, collect the velocity, gravity, damping and parameter-rate terms into
\[
h_v(X,p,\dot p)=
\begin{bmatrix}
-ml\sin\theta\,\dot\theta^2+b_x\dot x
+(\dot M+\dot m)\dot x+(\dot m\,l+m\dot l)\dot\theta\cos\theta\\
-mgl\sin\theta+b_\theta\dot\theta
+(\dot m\,l+m\dot l)\dot x\cos\theta+\dot I\dot\theta
\end{bmatrix}.
\]
Then
\[
H\ddot q+h_v=
\begin{bmatrix}F_a+d_x\\d_\theta\end{bmatrix}.
\tag{V6.1}
\]
The inverse inertia matrix is
\[
H^{-1}=\frac1D
\begin{bmatrix}I&-ml\cos\theta\\-ml\cos\theta&M+m\end{bmatrix}.
\]
Consequently, the actual state dynamics can be written
\[
\dot X=f_p(X,t)+B_p(X,t)F_a+G_p(X,t)d,
\quad d=[d_x,d_\theta]^T,
\tag{V6.2}
\]
where
\[
f_p=\begin{bmatrix}\dot x\\[1mm] -[H^{-1}h_v]_1\\\dot\theta\\[1mm]-[H^{-1}h_v]_2\end{bmatrix},
\quad
B_p=\begin{bmatrix}0\\I/D\\0\\-ml\cos\theta/D\end{bmatrix},
\tag{V6.3}
\]
and
\[
G_p=\begin{bmatrix}
0&0\\
I/D&-ml\cos\theta/D\\
0&0\\
-ml\cos\theta/D&(M+m)/D
\end{bmatrix}.
\tag{V6.4}
\]
The acceleration-level horizontal-force direction is
\[
b_q=\frac1D\begin{bmatrix}I\\-ml\cos\theta\end{bmatrix},
\tag{V6.5}
\]
which is the acceleration part of \(B_p\). These expressions require \(D>0\). The input direction is state- and parameter-dependent; it is not generally the constant nominal vector \(B_0\).

**Unit check:** the two entries of \(b_q\) have units of acceleration per newton (with radians dimensionless); multiplying by a force gives \([\ddot x,\ddot\theta]^T\). The second disturbance column in \(G_p\) maps N·m to accelerations.

## 3. Define the residual against the nominal LQI model

Let the nominal upright linear model be
\[
\dot X=A_0X+B_0u_L.
\tag{V6.6}
\]
Evaluate the actual zero-disturbance vector field at the same nominal force request \(u_L\). Define
\[
\boxed{r(X,u_L,p,\dot p)=f_p(X,t)+B_p(X,t)u_L-A_0X-B_0u_L.}
\tag{V6.7}
\]
This definition deliberately includes both:
1. nonlinear and parameter/time-variation drift mismatch \(f_p-A_0X\); and
2. input-direction mismatch \((B_p-B_0)u_L\).

It is not valid to omit the second term when actual and nominal input directions differ.

Since both models have kinematic rows \(\dot x\) and \(\dot\theta\), the residual has the form
\[
r=\begin{bmatrix}0&r_x&0&r_\theta\end{bmatrix}^T,
\qquad r_q=\begin{bmatrix}r_x\\r_\theta\end{bmatrix}.
\tag{V6.8}
\]
Here \(r_x\) is an acceleration residual in m/s² and \(r_\theta\) is an angular-acceleration residual in rad/s².

For the actual applied force \(F_a\), add and subtract the nominal-force contribution:
\[
\boxed{\dot X=A_0X+B_0u_L+r+B_p(F_a-u_L)+G_pd.}
\tag{V6.9}
\]
Equation (V6.9) is an exact algebraic identity under definitions (V6.2) and (V6.7), not an approximation. It is the key bookkeeping identity that prevents commanded-force and applied-force terms from being conflated.

## 4. Weighted projection onto the single force channel

The plant has two acceleration coordinates but only one scalar horizontal actuator. Therefore an arbitrary \(r_q\in\mathbb R^2\) cannot generally be cancelled by a scalar force.

Choose positive acceleration scales \(a_s>0\) and \(\alpha_s>0\), and define
\[
S=\operatorname{diag}(a_s^{-2},\alpha_s^{-2})\succ0.
\tag{V6.10}
\]
The scales make the two residual coordinates dimensionless in the weighted objective; they must be declared from the intended operating envelope and must not be selected merely to improve a result.

Define the equivalent matched force as the minimiser
\[
\delta_m=\arg\min_{\delta\in\mathbb R}
(r_q-b_q\delta)^TS(r_q-b_q\delta).
\tag{V6.11}
\]
Differentiating the scalar quadratic gives
\[
-2b_q^TS(r_q-b_q\delta)=0.
\]
Provided
\[
b_q^TSb_q\ge\beta_b>0
\tag{V6.12}
\]
on the declared operating set, the minimiser is unique:
\[
\boxed{\delta_m=\frac{b_q^TSr_q}{b_q^TSb_q}.}
\tag{V6.13}
\]
Its unit is N: numerator and denominator combine the acceleration-per-newton direction and acceleration residual so that the fitted scalar is a force.

Define the unmatched acceleration residual
\[
\boxed{r_{q,u}=r_q-b_q\delta_m.}
\tag{V6.14}
\]
By construction,
\[
b_q^TSr_{q,u}=0.
\tag{V6.15}
\]
This is orthogonality in the chosen weighted metric, not necessarily Euclidean orthogonality. It does not mean the unmatched residual is zero or dynamically harmless.

Lift the unmatched term to state coordinates:
\[
r_u=\begin{bmatrix}0&r_{q,u,1}&0&r_{q,u,2}\end{bmatrix}^T.
\]
Then
\[
\boxed{r=B_p\delta_m+r_u.}
\tag{V6.16}
\]
This is an exact decomposition for the chosen \(S\) and the actual input direction \(B_p\). It does not assert exact cancellation of all model error.

### Conditioning and interpretation

- The denominator \(b_q^TSb_q\) must be bounded away from zero. Since \(D>0\) and \(I>0\), \(b_q\ne0\) for this model, but the quantitative lower bound over the chosen parameter/state set still needs to be calculated.
- A poorly scaled \(S\) can make the fitted force dominated by one coordinate. Report \(a_s,\alpha_s\) and perform sensitivity checks.
- The fitted \(\delta_m\) is the best scalar force only under the selected weighted acceleration-error criterion. It is not necessarily the force minimising a state-space or energy norm.
- If \(p\) is unknown, the controller cannot evaluate the true \(b_q\) or \(\delta_m\) exactly. An estimated direction \(\hat b_q\) creates an additional projection/model error that must be bounded. Do not feed true simulation parameters to the controller unless the sensing/estimation assumption explicitly allows it.

## 5. External disturbances: horizontal force versus pivot torque

From (V6.4), the horizontal-force disturbance column is exactly the same direction as the actuator:
\[
G_{p,x}=B_p.
\tag{V6.17}
\]
Thus \(d_x\) is directly matched to the horizontal actuator channel (though it may not be measured or known for feedforward cancellation).

The pivot-torque disturbance acceleration direction is
\[
g_\theta=\frac1D
\begin{bmatrix}-ml\cos\theta\\M+m\end{bmatrix}.
\tag{V6.18}
\]
It is linearly independent of the horizontal-force direction because
\[
\det\begin{bmatrix}
I/D&-ml\cos\theta/D\\
-ml\cos\theta/D&(M+m)/D
\end{bmatrix}
=\frac{1}{D}\ne0.
\tag{V6.19}
\]
Therefore a nonzero pure pivot torque produces an acceleration vector that cannot be exactly generated by a scalar horizontal force at any admissible point with \(D>0\). It is genuinely unmatched relative to this one-input mechanical actuation.

The complete disturbance term is
\[
G_pd=B_pd_x+\begin{bmatrix}0\\-ml\cos\theta\,d_\theta/D\\0\\(M+m)d_\theta/D\end{bmatrix}.
\tag{V6.20}
\]
The torque term may be projected onto the force direction for analysis, but its unmatched component remains nonzero; doing so does not make the physical torque directly actuated.

## 6. Closed-loop identity and compensation sign

Use the candidate command
\[
u_c=u_L-\hat\delta_m+u_r,
\tag{V6.21}
\]
where \(u_r\) is a future robustifying term (not yet designed). Define the actuator tracking error by
\[
e_a=F_a-u_c.
\tag{V6.22}
\]
Then
\[
F_a-u_L=-\hat\delta_m+u_r+e_a.
\]
Substituting (V6.16) and this identity into (V6.9) gives
\[
\boxed{
\dot X=A_0X+B_0u_L
+B_p(\delta_m-\hat\delta_m+u_r+e_a)
+r_u+G_pd.}
\tag{V6.23}
\]
This establishes the sign: if \(\hat\delta_m=\delta_m\), \(u_r=0\), and \(e_a=0\), the matched residual term is removed from (V6.23), while \(r_u\) and external disturbances remain. This is a conditional algebraic cancellation statement, not a stability claim and not a claim that the RBF can cancel unmatched dynamics.

For cart-position integral augmentation, let
\[
\zeta=\begin{bmatrix}X\\\xi\end{bmatrix},\quad
\dot\xi=-C_xX,\quad
A_c=\begin{bmatrix}A_0+B_0K_L&-B_0K_i\\-C_x&0\end{bmatrix}
\]
when \(u_L=-K_LX-K_i\xi\). Then (V6.23) becomes
\[
\dot\zeta=A_c\zeta+
\bar B_p(\delta_m-\hat\delta_m+u_r+e_a)
+\bar r_u+\bar G_pd,
\tag{V6.24}
\]
where \(\bar B_p=[B_p^T,0]^T\), \(\bar r_u=[r_u^T,0]^T\), and \(\bar G_p=[G_p^T,0]^T\). This augmentation is only valid for the stated integrator definition and fixed nominal LQI gains.

## 7. RBF estimation target and what it does not estimate

If a two-output RBF is retained as an intermediate estimator,
\[
\hat r_q=\hat\Theta^T\Phi(z),\qquad \hat\Theta\in\mathbb R^{N\times2},
\tag{V6.25}
\]
then form its force projection using the same metric and input direction:
\[
\boxed{\hat\delta_m=\frac{b_q^TS\hat r_q}{b_q^TSb_q}.}
\tag{V6.26}
\]
The RBF is not instructed to cancel \(r_{q,u}\). The estimation error in the force channel is
\[
\delta_m-\hat\delta_m
=\frac{b_q^TS(r_q-\hat r_q)}{b_q^TSb_q}.
\tag{V6.27}
\]
If \(r_q=\Theta^{*T}\Phi+\varepsilon\), then
\[
\delta_m-\hat\delta_m
=\frac{b_q^TS(\Theta^{*T}-\hat\Theta^T)\Phi}{b_q^TSb_q}
+\frac{b_q^TS\varepsilon}{b_q^TSb_q}.
\tag{V6.28}
\]
The unmatched part of the RBF residual prediction is not a second control input. It must not be subtracted independently from the two accelerations.

**Signal availability warning:** if \(z\) contains \(p\), \(\dot p\), or \(u_L\), every one of those quantities must be available online through measurement, estimation, or a known command/reference. True plant parameters available only inside a simulation are not admissible controller inputs.

## 8. What is now established, and what remains open

### Established by algebra in this document

1. The actual nonlinear plant has one scalar force input direction \(B_p\).
2. The residual definition (V6.7) includes drift and input-direction mismatch.
3. The exact actual-versus-nominal identity (V6.9) follows.
4. The weighted scalar projection (V6.13) uniquely minimises the declared objective when (V6.12) holds.
5. The unmatched residual is metric-orthogonal to the input direction.
6. The horizontal disturbance is matched; the independent pivot torque direction is not collinear with the actuator direction.
7. The compensation sign in (V6.21) follows from the applied-force identity, including actuator error.

### Not established yet

- A defensible numerical choice of \(a_s,\alpha_s\), and a lower bound \(\beta_b\) over the final admissible parameter/state set.
- Whether actual \(p,\dot p\) or the input direction are measured/estimated; if not, a practical estimated-direction projection and its error bound.
- The RBF regressor, basis centres/widths, domain coverage, ideal-weight assumptions, and approximation-error bound.
- A valid online adaptation law for the projected force estimation error.
- The robust term \(u_r\), if any.
- A closed-loop proof including unmatched residual, torque disturbance, actuator lag/saturation/rate limits, anti-windup, weight projection, and forward invariance.
- Hybrid momentum/reset analysis for abrupt payload events.
- Numerical verification of the residual and projection identities.

## 9. Literature traceability

The Lagrangian plant structure and inertia/input mapping are standard rigid-body mechanics [Spong, Hutchinson & Vidyasagar, 2006; Goldstein, Poole & Safko, 2002]. The distinction between uncertainty in the input distribution and unmatched dynamics is consistent with nonlinear control treatments of input distributions and normal forms [Isidori, 1995; Khalil, 2002]. Adaptive approximation and projection-based parameter constraints require a separate Lyapunov derivation [Lewis, Jagannathan & Ye\u015fildirek, 1998; Ioannou & Sun, 1996; Lavretsky & Wise, 2013]. The weighted projection formula itself is derived directly by minimising (V6.11); it is a project-specific least-squares definition and is not claimed as a published adaptive-control theorem.

Consult the [literature and derivation traceability register](LITERATURE_AND_DERIVATION_TRACEABILITY.md) for full bibliographic details and the outstanding pinpoint-reference verification queue.
