function [u, xhat_next, xi_next, rbf_next, diag] = control_step( ...
    y, u_prev, xhat_prev, xi_prev, rbf, r, c)
%CONTROL_STEP One fixed-rate embedded control cycle — SINGLE PRECISION.
%
%   [u, xhat_next, xi_next, rbf_next, diag] = control_step(y, u_prev, xhat_prev, xi_prev, rbf, r, c)
%
%   c = control_consts() — generated gains/limits, all single.
%   rbf = rbf_initialize(params, 'embedded') — single-precision RBF struct.
%   All other inputs (y, u_prev, xhat_prev, xi_prev, r) MUST be single on
%   entry; cast once at the sensor boundary (sensor_calibration.m), not
%   here, so this function has no implicit double->single conversions
%   for MATLAB Coder to silently insert (or refuse to, in strict typing).

xhat_pred  = c.Ad*xhat_prev + c.Bd*u_prev;
innovation = y - c.C_meas*xhat_pred;
xhat_next  = xhat_pred + c.L*innovation;

err   = r - xhat_next(1);
u_lqi = -c.K*xhat_next - c.Ki*xi_prev;

rbfInput = [xhat_next(3); xhat_next(4); xhat_next(1); xhat_next(2)];
[f_rbf, phi] = rbf_forward(rbf, rbfInput);

u_unsat = u_lqi + f_rbf;
u = max(min(u_unsat, c.actuator_limit), -c.actuator_limit);

saturated_high = u_unsat > u + c.sat_tol;
saturated_low  = u_unsat < u - c.sat_tol;
freeze = (saturated_high && err > 0) || (saturated_low && err < 0);
if freeze
    xi_dot = single(0);
else
    xi_dot = err;
end
xi_next = xi_prev + xi_dot * c.Ts;

rbf_error = r - xhat_next(1);
rbf_next = rbf_update(rbf, phi, rbf_error);

diag.saturated   = saturated_high || saturated_low;
diag.theta       = xhat_next(3);
diag.x           = xhat_next(1);
diag.weight_norm = norm(rbf_next.weights);

end