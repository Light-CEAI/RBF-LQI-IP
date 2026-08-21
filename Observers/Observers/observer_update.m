function [xhat_next, innovation] = observer_update(xhat, u, y, Ad, Bd, L, C_meas, params)
%OBSERVER_UPDATE One discrete predict-correct step of the state observer.
%
%   [xhat_next, innovation] = observer_update(xhat, u, y, Ad, Bd, L, C_meas, params)
%
%   Now returns the innovation vector (y - C_meas*xhat_pred) as a second
%   output. This reflects the instantaneous model-plant mismatch seen by
%   the observer, and is used by run_controller_variant.m as part of the
%   RBF error signal to better target unmodeled dynamics compensation.

xhat_pred  = Ad*xhat + Bd*u;
innovation = y - C_meas*xhat_pred;
xhat_next  = xhat_pred + L*innovation;

end %#ok<INUSD>