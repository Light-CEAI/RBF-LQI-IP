%% test_three_way_drift.m
% Side-by-side: LQI-only, RBF-only, LQI+RBF, under sinusoidal mass drift.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

bias = 0.10; amplitude = 0.20; t_start = 5; omega = 2*pi*0.1;
M_nom = params.cart.M;
M_of_t = @(tt) M_nom * (1 + (tt>=t_start).*(bias + amplitude*sin(omega*(tt-t_start))));

scenario = struct('Ts', params.embedded.Ts, 'Tfinal', 15, ...
    'x0', zeros(4,1), 'r', 0, 'Fd_func', @(t) 0, 'M_of_t', M_of_t, ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std]);

modes = {'lqi_only','rbf_only','lqi_rbf'};
titles = {'LQI only', 'RBF only', 'LQI + RBF'};
results_drift = cell(1,3);

for i = 1:3
    results_drift{i} = run_controller_variant(modes{i}, params, scenario);
    if results_drift{i}.diverged
        fprintf('%s: DIVERGED at t=%.3fs\n', titles{i}, results_drift{i}.divergence_time_s);
    else
        fprintf('%s: max|theta|=%.2fdeg, max|x|=%.3fm\n', titles{i}, ...
            rad2deg(max(abs(results_drift{i}.x_true(3,:)))), max(abs(results_drift{i}.x_true(1,:))));
    end
end

figure('Name','Three-Way Comparison: Sinusoidal Mass Drift','Position',[80 80 1200 500]);
for i = 1:3
    subplot(2,3,i);
    plot(results_drift{i}.t, rad2deg(results_drift{i}.x_true(3,:)), 'b-','LineWidth',1.2);
    if results_drift{i}.diverged
        title(sprintf('%s (diverged @%.2fs)', titles{i}, results_drift{i}.divergence_time_s));
    else
        title(titles{i});
    end
    ylabel('theta [deg]'); grid on;

    subplot(2,3,i+3);
    plot(results_drift{i}.t(1:end-1), results_drift{i}.u, 'k-','LineWidth',1.0);
    ylabel('u'); xlabel('time [s]'); grid on;
end
sgtitle('Sinusoidal mass drift (+-30%% boundary, onset at t=5s)');

save(fullfile(resultsDir,'results_three_way_drift.mat'), 'results_drift', 'modes', 'titles');