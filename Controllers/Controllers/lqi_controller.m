function u_lqi_raw = lqi_controller(xhat, xi, r, K, Ki)
%LQI_CONTROLLER Raw (UNSATURATED) LQI control law evaluation.
%
%   u_lqi_raw = lqi_controller(xhat, xi, r, K, Ki)
%
%   Deliberately returns the RAW command, with NO saturation and NO
%   anti-windup decision made here. Both must happen ONCE, on the
%   COMBINED command (u_lqi + f_rbf), at the caller level (see
%   simulation_main.m). Saturating or freezing the integrator inside
%   this function would make that decision blind to what f_rbf adds
%   afterward -- which was the actual root cause of an integrator
%   windup / instability found during +-20% uncertainty + disturbance
%   testing (xi diverged to -200 while saturation showed as 0%, because
%   the old version saturated the LQI term alone before RBF was added).
%
%   This now matches control_step.m's structure, which already computed
%   the raw command this same way.

u_lqi_raw = -K*xhat - Ki*xi;

end