# Projection Metric Operating-Grid Audit

**Branch:** `analyzed-system`  
**Purpose:** Provide a MATLAB numerical tool for examining inertia positivity and weighted-projection conditioning over parameter and angle grids supplied by the researcher.

## Files

- `audit_projection_metric_envelope.m` — function that evaluates the Cartesian product of user-supplied parameter and angle vectors.
- `projection_metric_config_TEMPLATE.m` — configuration template with intentionally blank values. Complete it only after the parameter register and operating envelope have been justified.

## Run

1. Download both files into the same MATLAB folder.
2. Open `projection_metric_config_TEMPLATE.m`.
3. Replace each `[]` with a documented vector/range from the approved parameter and operating-envelope register. Do not copy the illustrative V-006 values as if they were verified physical bounds.
4. Enter positive `a_s) and `alpha_s) only after justifying their characteristic acceleration meaning and units.
5. Run:

```matlab
run('projection_metric_config_TEMPLATE.m');
results = audit_projection_metric_envelope(cfg);
```

## What it checks

- (I=I_c+ml^2) using the parallel-axis theorem.
- (D=(M+m)I-m^2l^2\cos^2\theta>0) at every supplied grid point.
- The weighted projection denominator (b_q^TSb_q>0) at every supplied grid point.
- Sensitivity of the denominator to a predeclared sweep of the ratio (alpha_s/a_s), holding (a_s) fixed.

## Interpretation and limitations

A positive minimum over a finite grid is not a certified lower bound over the continuous parameter/state set. Grid density and inclusion of range endpoints matter. A formal (eta_b) requires an analytic inequality, interval method, or another validated enclosure over the full declared set.

The ratio sweep is diagnostic. Do not choose scales by selecting whichever ratio produces the best closed-loop tracking result. The script does not select scales, fit a controller, train an RBF, estimate unknown parameters, or establish UUB/stability.

No nominal parameter values or physical ranges are embedded in the template. This is deliberate because the project parameter register is not yet frozen.
