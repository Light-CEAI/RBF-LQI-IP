%% test_length_variation.m
% Robustness to pendulum length mismatch, per datasheet Section 5: +-10-15%.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

lengthPctLevels = [-0.15, -0.10, 0, 0.10, 0.15];

length_results = cell(numel(lengthPctLevels), 1);
length_metrics = cell(numel(lengthPctLevels), 1);
for i = 1:numel(lengthPctLevels)
    scenario = struct();
    scenario.Ts             = params.embedded.Ts;
    scenario.Tfinal         = 8;
    scenario.initAngleDeg   = 10;
    scenario.useRBF         = true;
    scenario.observerMethod = 'kalman';
    scenario.lengthPct      = lengthPctLevels(i);
    scenario.Fd_func        = @(t) 0;
    scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

    length_results{i} = simulation_main(scenario);
    length_metrics{i} = performance_metrics(length_results{i}, params);

    fprintf('Length variation %+.0f%%: max|theta|=%.2f deg, max|u|=%.3f, PASS=%d\n', ...
        lengthPctLevels(i)*100, length_metrics{i}.max_theta_deg, length_metrics{i}.max_u, length_metrics{i}.pass);
    if ~length_metrics{i}.pass
        fprintf('  FAIL: %s\n', strjoin(length_metrics{i}.fail_reasons, '; '));
    end
end
save(fullfile(resultsDir, 'results_length_variation.mat'), ...
    'length_results', 'lengthPctLevels', 'length_metrics');