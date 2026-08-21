function metrics = performance_metrics(results, params)
%PERFORMANCE_METRICS Compute standard control metrics AND pass/fail safety
%   checks for one simulation_main.m results struct.
%
%   metrics = performance_metrics(results, params)
%
%   This is the single shared helper test_*.m scripts should call instead
%   of each hand-rolling their own pass/fail logic.
%
%   metrics fields:
%     .max_theta_deg, .max_x_m, .max_u, .control_effort (integral u^2 dt)
%     .settling_time_s   (time after which |theta| stays < 1 deg for good,
%                          NaN if it never settles)
%     .steady_state_x_err (|x(end) - r| over the last 1s window)
%     .pass               (true/false overall)
%     .fail_reasons       (cellstr, empty if pass)

t = results.t;
x = results.x_true(1,:);
theta = results.x_true(3,:);
u = results.u;
r = results.scenario.r;
Ts = results.scenario.Ts;

metrics.max_theta_deg = rad2deg(max(abs(theta)));
metrics.max_x_m       = max(abs(x));
metrics.max_u         = max(abs(u));
metrics.control_effort = sum(u.^2) * Ts;

% ---- Settling time: last time |theta| crossed above 1 deg threshold ----
thetaThreshold = deg2rad(1);
aboveThresh = abs(theta) > thetaThreshold;
if any(aboveThresh)
    lastAboveIdx = find(aboveThresh, 1, 'last');
    if lastAboveIdx == numel(t)
        metrics.settling_time_s = NaN; % never settled within Tfinal
    else
        metrics.settling_time_s = t(lastAboveIdx);
    end
else
    metrics.settling_time_s = 0; % started within threshold
end

% ---- Steady-state error: average over the last 1s of the run -----------
nLast = max(1, round(1 / Ts));
metrics.steady_state_x_err = mean(abs(x(end-nLast+1:end) - r));

metrics.iae_theta = trapz(t, abs(theta));
metrics.iae_x     = trapz(t, abs(x - r));
% ---- Pass/fail safety checks (uses params.safety / params.mech) --------
fail_reasons = {};

if metrics.max_theta_deg > params.safety.theta_trip_deg
    fail_reasons{end+1} = sprintf('theta exceeded trip angle (%.2f > %.2f deg)', ...
        metrics.max_theta_deg, params.safety.theta_trip_deg);
end

x_limit = params.mech.track_length/2 - params.safety.x_trip_margin;
if metrics.max_x_m > x_limit
    fail_reasons{end+1} = sprintf('x exceeded safe travel (%.3f > %.3f m)', ...
        metrics.max_x_m, x_limit);
end

if isnan(metrics.settling_time_s)
    fail_reasons{end+1} = 'theta never settled within Tfinal';
end

if any(isnan(x)) || any(isinf(x)) || any(isnan(theta)) || any(isinf(theta))
    fail_reasons{end+1} = 'NaN/Inf encountered in simulated states';
end

metrics.fail_reasons = fail_reasons;
metrics.pass = isempty(fail_reasons);

end