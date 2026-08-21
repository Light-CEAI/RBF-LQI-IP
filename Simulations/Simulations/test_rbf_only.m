%% test_rbf_only.m
% Standalone diagnostic of the RBF compensator running WITHOUT LQI
% feedback. This is NOT a deployable configuration -- RBF was designed
% as an additive correction on top of LQI, never as a stabilizer in its
% own right. The purpose here is to characterize HOW it fails (instant
% runaway vs. slow drift vs. brief stability), which is itself an
% informative result for a thesis: it demonstrates that LQI provides
% the actual stabilizing authority, with RBF only ever a refinement.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

scenario = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'x0', [0;0;deg2rad(10);0], 'r', 0, 'Fd_func', @(t) 0, ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std]);

results = run_controller_variant('rbf_only', params, scenario);

if results.diverged
    fprintf('RBF-only: DIVERGED at t=%.3fs (|theta| exceeded 90deg)\n', results.divergence_time_s);
else
    fprintf('RBF-only: did not diverge within Tfinal=%.0fs. max|theta|=%.2fdeg\n', ...
        scenario.Tfinal, rad2deg(max(abs(results.x_true(3,:)))));
end

% Find the valid (pre-divergence/NaN) portion for detailed plotting
validIdx = ~isnan(results.x_true(3,:));
t_valid = results.t(validIdx);
theta_valid = rad2deg(results.x_true(3,validIdx));
x_valid = results.x_true(1,validIdx);

figure('Name','RBF-Only Diagnostic (no LQI feedback)');
subplot(3,1,1);
plot(t_valid, theta_valid, 'r-', 'LineWidth',1.3);
ylabel('theta [deg]'); grid on;
title(sprintf('RBF-only response (%s)', ternary(results.diverged, ...
    sprintf('diverged at t=%.2fs', results.divergence_time_s), 'stable')));

subplot(3,1,2);
plot(t_valid, x_valid, 'r-', 'LineWidth',1.3);
ylabel('x [m]'); grid on;

subplot(3,1,3);
validU = ~isnan(results.u);
plot(results.t(1:end-1).*validU + nan(size(validU)).*(~validU), results.u, 'k-', 'LineWidth',1.0);
hold on;
plot(results.t(validU), results.u(validU), 'k-', 'LineWidth',1.0);
ylabel('u (= f\_rbf only)'); xlabel('time [s]'); grid on;

save(fullfile(resultsDir,'results_rbf_only.mat'), 'results');

function out = ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end