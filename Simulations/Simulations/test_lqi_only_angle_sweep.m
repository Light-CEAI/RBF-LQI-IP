%% test_lqi_only_angle_sweep.m
% Standalone LQI (no RBF) at increasing initial angles, specifically
% beyond the +-20deg range used everywhere else in this project, to find
% where the small-angle-linearized LQI design actually starts to break
% down. This directly establishes the operating boundary at which RBF
% augmentation would be justified, rather than assuming it.
%
% Rail bound is temporarily extended to +-1.0m (from the design value of
% +-0.35m safe travel) SPECIFICALLY for this diagnostic, so that a large
% x excursion is recorded as data rather than masked by an early
% safety-trip classification. This does NOT change params.mech.* for any
% other script -- the extension is local to this file only.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% Local-only override, does not persist or affect other scripts
params_diag = params;
params_diag.mech.track_length = 2.0; % +-1.0m, diagnostic only

angleLevels = [10, 15, 26, 28, 30]; % includes the existing 10deg baseline for reference
colors = lines(numel(angleLevels));

results_sweep = cell(numel(angleLevels),1);

for i = 1:numel(angleLevels)
    scenario = struct('Ts', params_diag.embedded.Ts, 'Tfinal', 8, ...
        'x0', [0;0;deg2rad(angleLevels(i));0], 'r', 0, 'Fd_func', @(t) 0, ...
        'measNoiseStd', [params_diag.sensor.x_quant_std; params_diag.sensor.theta_quant_std]);

    results_sweep{i} = run_controller_variant('lqi_only', params_diag, scenario);

    if results_sweep{i}.diverged
        fprintf('%ddeg: DIVERGED at t=%.3fs\n', angleLevels(i), results_sweep{i}.divergence_time_s);
    else
        m = performance_metrics(struct('t', results_sweep{i}.t, 'x_true', results_sweep{i}.x_true, ...
            'u', results_sweep{i}.u, 'scenario', scenario), params_diag);
        fprintf('%ddeg: max|theta|=%.2fdeg, max|x|=%.3fm, settling=%.2fs\n', ...
            angleLevels(i), m.max_theta_deg, m.max_x_m, m.settling_time_s);
    end
end

%% ---- Single comparison plot, all angles overlaid -------------------------
figure('Name','LQI-Only Angle Sweep (find the breakdown boundary)','Position',[80 80 1000 600]);

subplot(2,1,1);
hold on;
legendEntries = cell(numel(angleLevels),1);
for i = 1:numel(angleLevels)
    validIdx = ~isnan(results_sweep{i}.x_true(3,:));
    plot(results_sweep{i}.t(validIdx), rad2deg(results_sweep{i}.x_true(3,validIdx)), ...
        'Color', colors(i,:), 'LineWidth', 1.3);
    if results_sweep{i}.diverged
        legendEntries{i} = sprintf('%d%s (diverged @%.2fs)', angleLevels(i), char(176), results_sweep{i}.divergence_time_s);
    else
        legendEntries{i} = sprintf('%d%s', angleLevels(i), char(176));
    end
end
legend(legendEntries, 'Location', 'best');
ylabel('theta [deg]'); grid on;
title('LQI-only, increasing initial angle (no RBF)');

subplot(2,1,2);
hold on;
for i = 1:numel(angleLevels)
    validIdx = ~isnan(results_sweep{i}.x_true(1,:));
    plot(results_sweep{i}.t(validIdx), results_sweep{i}.x_true(1,validIdx), ...
        'Color', colors(i,:), 'LineWidth', 1.3);
end
yline(1.0, 'k--', 'extended rail limit (+-1.0m)');
ylabel('x [m]'); xlabel('time [s]'); grid on;

%% ---- Summary: where exactly does it break? --------------------------------
fprintf('\n=== LQI-Only Boundary Summary ===\n');
firstDivergeIdx = find(cellfun(@(r) r.diverged, results_sweep), 1);
if isempty(firstDivergeIdx)
    fprintf('LQI alone remained stable across all tested angles (%s).\n', mat2str(angleLevels));
    fprintf('No divergence boundary found within this range -- RBF augmentation is\n');
    fprintf('not justified by large-angle stabilization within %d-%ddeg.\n', min(angleLevels), max(angleLevels));
else
    fprintf('First divergence at %ddeg initial angle (t=%.3fs).\n', ...
        angleLevels(firstDivergeIdx), results_sweep{firstDivergeIdx}.divergence_time_s);
    if firstDivergeIdx > 1
        fprintf('Last stable angle tested: %ddeg.\n', angleLevels(firstDivergeIdx-1));
        fprintf('This brackets the LQI-only stability boundary between %ddeg and %ddeg --\n', ...
            angleLevels(firstDivergeIdx-1), angleLevels(firstDivergeIdx));
        fprintf('this is the range where RBF augmentation would plausibly be justified.\n');
    end
end

save(fullfile(resultsDir,'results_lqi_only_angle_sweep.mat'), 'results_sweep', 'angleLevels');