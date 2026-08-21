%% test_combined_worst_case.m
% Combined stress test, comparing LQI-only (baseline) vs LQI+RBF at the
% same worst-case boundary, plus an explicit xi (integrator) check to
% rule out a recurrence of the anti-windup bug before this is reported
% as a genuine actuator-authority finding.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

base = struct('Ts', params.embedded.Ts, 'Tfinal', 10, ...
    'initAngleDeg', 20, 'observerMethod', 'kalman', ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
    'massPct', -0.20, 'frictionPct', -0.20, 'lengthPct', -0.20, ...
    'Fd_func', @(t) 2.5 * (t >= 3 && t < 3.5));

scenario_on = base;  scenario_on.useRBF = true;
scenario_off = base; scenario_off.useRBF = false;

results_on  = simulation_main(scenario_on);
results_off = simulation_main(scenario_off);

metrics_on  = performance_metrics(results_on, params);
metrics_off = performance_metrics(results_off, params);

fprintf('--- Combined Worst-Case Stress Test ---\n');
fprintf('20deg initial + 2.5N pulse @t=3s + -20%% combined uncertainty\n\n');
fprintf('LQI+RBF: max|theta|=%.2f deg, max|x|=%.3f m, PASS=%d, final xi=%.4f\n', ...
    metrics_on.max_theta_deg, metrics_on.max_x_m, metrics_on.pass, results_on.xi(end));
if ~metrics_on.pass, fprintf('  FAIL: %s\n', strjoin(metrics_on.fail_reasons, '; ')); end

fprintf('LQI only: max|theta|=%.2f deg, max|x|=%.3f m, PASS=%d, final xi=%.4f\n', ...
    metrics_off.max_theta_deg, metrics_off.max_x_m, metrics_off.pass, results_off.xi(end));
if ~metrics_off.pass, fprintf('  FAIL: %s\n', strjoin(metrics_off.fail_reasons, '; ')); end

% ---- Diagnostic: is xi bounded (anti-windup working) or runaway? --------
xi_on_bounded = abs(results_on.xi(end)) < 50 && max(abs(results_on.xi)) < 100;
xi_off_bounded = abs(results_off.xi(end)) < 50 && max(abs(results_off.xi)) < 100;
fprintf('\nxi bounded (anti-windup OK)?  RBF: %d   LQI-only: %d\n', xi_on_bounded, xi_off_bounded);
if ~xi_on_bounded || ~xi_off_bounded
    fprintf('WARNING: xi appears unbounded -- this may indicate an anti-windup\n');
    fprintf('gap rather than a genuine actuator-authority limit. Investigate\n');
    fprintf('before reporting this as a clean envelope-boundary finding.\n');
end

figure('Name', 'Combined Worst-Case: LQI-only vs LQI+RBF');
subplot(3,1,1);
plot(results_on.t, rad2deg(results_on.x_true(3,:)), 'g-', 'LineWidth',1.2); hold on;
plot(results_off.t, rad2deg(results_off.x_true(3,:)), 'b--', 'LineWidth',1.2);
legend('LQI+RBF','LQI only','Location','best'); ylabel('theta [deg]'); grid on;
title('Combined Worst-Case Stress Test');

subplot(3,1,2);
plot(results_on.t, results_on.xi, 'g-', 'LineWidth',1.2); hold on;
plot(results_off.t, results_off.xi, 'b--', 'LineWidth',1.2);
ylabel('integral state \xi'); grid on;

subplot(3,1,3);
plot(results_on.t(1:end-1), results_on.u, 'g-', 'LineWidth',1.0); hold on;
plot(results_off.t(1:end-1), results_off.u, 'b--', 'LineWidth',1.0);
ylabel('u'); xlabel('time [s]'); grid on;

save(fullfile(resultsDir,'results_combined_worst_case.mat'), ...
    'results_on', 'results_off', 'metrics_on', 'metrics_off');