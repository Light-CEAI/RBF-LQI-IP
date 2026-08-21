%% test_three_way_nominal.m
% Side-by-side: LQI-only, RBF-only, LQI+RBF, at 10deg nominal initial
% condition. RBF-only panel auto-scales independently and reports its
% own divergence time, rather than sharing an axis scale that would
% make the other two panels unreadable.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

scenario = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'x0', [0;0;deg2rad(10);0], 'r', 0, 'Fd_func', @(t) 0, ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std]);

modes = {'lqi_only','rbf_only','lqi_rbf'};
titles = {'LQI only', 'RBF only', 'LQI + RBF'};
results_nom = cell(1,3);

for i = 1:3
    results_nom{i} = run_controller_variant(modes{i}, params, scenario);
    if results_nom{i}.diverged
        fprintf('%s: DIVERGED at t=%.3fs\n', titles{i}, results_nom{i}.divergence_time_s);
    else
        fprintf('%s: max|theta|=%.2fdeg, max|x|=%.3fm\n', titles{i}, ...
            rad2deg(max(abs(results_nom{i}.x_true(3,:)))), max(abs(results_nom{i}.x_true(1,:))));
    end
end

figure('Name','Three-Way Comparison: Nominal 10deg','Position',[80 80 1200 500]);
for i = 1:3
    subplot(2,3,i);
    plot(results_nom{i}.t, rad2deg(results_nom{i}.x_true(3,:)), 'b-','LineWidth',1.2);
    if results_nom{i}.diverged
        title(sprintf('%s (diverged @%.2fs)', titles{i}, results_nom{i}.divergence_time_s));
    else
        title(titles{i});
    end
    ylabel('theta [deg]'); grid on; % auto y-axis per panel, no shared ylim

    subplot(2,3,i+3);
    plot(results_nom{i}.t(1:end-1), results_nom{i}.u, 'k-','LineWidth',1.0);
    ylabel('u'); xlabel('time [s]'); grid on;
end
sgtitle('Nominal (10deg initial angle, no disturbance)');

save(fullfile(resultsDir,'results_three_way_nominal.mat'), 'results_nom', 'modes', 'titles');