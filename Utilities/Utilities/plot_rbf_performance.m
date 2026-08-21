%% plot_rbf_performance.m
% Generates RBF-specific diagnostic plots:
%   1. RBF weight norm evolution over time (is it learning, plateauing, drifting?)
%   2. RBF compensation contribution f_rbf alongside u_lqi (how much is it doing?)
%   3. RBF-ON vs RBF-OFF comparison on the same disturbance scenario, to
%      show what the RBF compensator is actually buying you over plain LQI.
%
% Note: (3) runs two FRESH simulations (not loaded from Results/), since
% no prior test script saved a useRBF=false case for direct comparison.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');

%% ---- 1 & 2: weight evolution and compensation contribution, nominal run -
load(fullfile(resultsDir, 'results_nominal.mat')); % loads `results`, `metrics`

t = results.t;
tu = t(1:end-1);

figure('Name', 'RBF Weight Evolution');
plot(t, vecnorm(results.rbf_weights, 2, 1), 'r-', 'LineWidth', 1.3);
xlabel('time [s]'); ylabel('||RBF weights||_2'); grid on;
title('RBF Weight Norm Over Time (nominal test)');

figure('Name', 'RBF Compensation Contribution');
plot(tu, results.u_lqi, 'b-', 'LineWidth', 1.1); hold on;
plot(tu, results.f_rbf, 'g-', 'LineWidth', 1.1);
plot(tu, results.u, 'k--', 'LineWidth', 1.0);
legend('u_{lqi}', 'f_{rbf}', 'u (total, saturated)', 'Location', 'best');
xlabel('time [s]'); ylabel('command'); grid on;
title('LQI vs RBF Contribution (nominal test)');

%% ---- 3: RBF-ON vs RBF-OFF, same disturbance scenario --------------------
scenario_base = struct();
scenario_base.Ts             = params.embedded.Ts;
scenario_base.Tfinal         = 8;
scenario_base.initAngleDeg   = 10;
scenario_base.observerMethod = 'kalman';
scenario_base.Fd_func        = @(t) 2.0 * (t >= 2 && t < 2.5); % 2N pulse, mid-range case
scenario_base.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

scenario_on = scenario_base;  scenario_on.useRBF  = true;
scenario_off = scenario_base; scenario_off.useRBF = false;

results_on  = simulation_main(scenario_on);
results_off = simulation_main(scenario_off);

metrics_on  = performance_metrics(results_on, params);
metrics_off = performance_metrics(results_off, params);

fprintf('--- RBF ON vs OFF (2N disturbance @ t=2s) ---\n');
fprintf('RBF ON:  max|theta|=%.2f deg, settling=%.2fs, control_effort=%.4f\n', ...
    metrics_on.max_theta_deg, metrics_on.settling_time_s, metrics_on.control_effort);
fprintf('RBF OFF: max|theta|=%.2f deg, settling=%.2fs, control_effort=%.4f\n', ...
    metrics_off.max_theta_deg, metrics_off.settling_time_s, metrics_off.control_effort);

figure('Name', 'RBF ON vs OFF Comparison');
subplot(2,1,1);
plot(results_on.t, rad2deg(results_on.x_true(3,:)), 'g-', 'LineWidth', 1.3); hold on;
plot(results_off.t, rad2deg(results_off.x_true(3,:)), 'b--', 'LineWidth', 1.3);
legend('RBF ON', 'RBF OFF', 'Location', 'best');
ylabel('\theta [deg]'); grid on;
title('Theta Response: RBF ON vs OFF (2N disturbance pulse)');

subplot(2,1,2);
plot(results_on.t, results_on.x_true(1,:), 'g-', 'LineWidth', 1.3); hold on;
plot(results_off.t, results_off.x_true(1,:), 'b--', 'LineWidth', 1.3);
legend('RBF ON', 'RBF OFF', 'Location', 'best');
xlabel('time [s]'); ylabel('x [m]'); grid on;
title('Position Response: RBF ON vs OFF (2N disturbance pulse)');

save(fullfile(resultsDir, 'results_rbf_comparison.mat'), ...
    'results_on', 'results_off', 'metrics_on', 'metrics_off');

fprintf('Generated 3 RBF performance figures, saved comparison data.\n');