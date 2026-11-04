# Physical and Mathematical Parameter Register

**Status:** Not frozen. This register is the single source of truth for parameter values used in future scripts and reports.

## Required provenance labels

- **Published:** source document, exact platform/configuration, table/page and units recorded.
- **Derived:** equation and all source quantities recorded.
- **Measured:** apparatus, instrument, calibration, procedure, repetitions and uncertainty recorded.
- **Estimated:** identification data, method, fitting interval and confidence/uncertainty recorded.
- **Assumed:** rationale, admissible range and sensitivity plan recorded.

## Register

| Symbol | Meaning | Value | Unit | Provenance | Source / derivation | Uncertainty / range | Status |
|---|---|---:|---|---|---|---|---|
| (M) | Cart/base mass | TBD | kg | TBD | TBD | TBD | Open |
| (m) | Pendulum mass | TBD | kg | TBD | TBD | TBD | Open |
| (l) | Pivot-to-COM distance | TBD | m | TBD | TBD | TBD | Open |
| (I_c) | Pendulum inertia about COM | TBD | kg m² | TBD | TBD | TBD | Open |
| (I) | Pendulum inertia about pivot | TBD | kg m² | Derived | (I=I_c+ml^2) | Propagate source uncertainties | Open |
| (b_x) | Cart viscous damping | TBD | N s/m | TBD | TBD | TBD | Open |
| (b_\theta) | Pivot viscous damping | TBD | N m s/rad | TBD | TBD | TBD | Open |
| (g) | Gravitational acceleration | TBD | m/s² | Assumed/standard | State chosen value and location convention | TBD | Open |
| (L_r) | Project rail length | 1.0 (provisional) | m | Design-assumed | Project design specification, not a manufacturer travel claim | Safe travel interval TBD | Provisional |
| (F_{max}) | Force limit | TBD | N | TBD | Motor/drive model or measured limit | TBD | Open |
| (dot F_{max}) | Force slew-rate limit | TBD | N/s | TBD | Actuator model or measurement | TBD | Open |
| (	au_a) | Actuator time constant | TBD | s | TBD | Data sheet or identification | TBD | Open |
| (R_m) | Motor armature resistance | TBD | ohm | TBD | Exact motor data sheet | TBD | Open |
| (K_t) | Motor torque constant | TBD | N m/A | TBD | Exact motor data sheet | TBD | Open |
| (K_e) | Back-EMF constant | TBD | V s/rad | TBD | Exact motor data sheet | TBD | Open |
| (K_g) | Gear ratio | TBD | dimensionless | TBD | Exact transmission data | TBD | Open |
| (r_p) | Pinion radius | TBD | m | TBD | Exact transmission data | TBD | Open |
| (eta) | Transmission efficiency | TBD | dimensionless | TBD | Manufacturer or identification | TBD | Open |
| (T_s) | Controller sample period | TBD | s | Design choice | Sampling/real-time rationale | TBD | Open |

## Parameter consistency checks

For a rigid pendulum with (l) measured from pivot to centre of mass and (I) about the pivot:
[
I=I_c+ml^2,\qquad I\ge ml^2.
]

For the constant-parameter cart-pendulum inertia matrix:
[
D(\theta)=(M+m)I-m^2l^2\cos^2\theta.
]
Require positive definiteness over the full declared operating domain, not merely at the upright equilibrium.

## Configuration control

Do not mix parameters from different cart bases, pendulum assemblies, motor/gearbox combinations or amplifier configurations without explicitly modelling the differences. Each frozen set receives an ID (e.g. `PSET-001`) and is archived with its source documents and derivation notes.
