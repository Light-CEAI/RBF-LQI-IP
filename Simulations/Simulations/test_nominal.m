%% test_nominal.m
% Baseline test: no parameter perturbation, no disturbance force, small
% initial angle. Establishes the reference behavior everything else is
% compared against.

params = parameters();

scenario = struct();
scenario.Ts             = params.embedded.Ts;
scenario.Tfinal         = 8;
scenario.initAngleDeg   = 5;       % small, well within linear region
scenario.r              = 0;
scenario.useRBF         = true;
scenario.observerMethod = 'kalman';
scenario.massPct        = 0;
scenario.frictionPct    = 0;
scenario.lengthPct      = 0;
scenario.Fd_func        = @(t) 0;
scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

results = simulation_main(scenario);
metrics = performance_metrics(results, params);

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
save(fullfile(resultsDir, 'results_nominal.mat'), 'results', 'metrics');

fprintf('test_nominal: max|theta|=%.2f deg, settling=%.2fs, PASS=%d\n', ...
    metrics.max_theta_deg, metrics.settling_time_s, metrics.pass);
if ~metrics.pass
    fprintf('  FAIL reasons: %s\n', strjoin(metrics.fail_reasons, '; '));
end

% plot_states(results);          % uncomment once Utilities/ exists
% plot_control(results);         % uncomment once Utilities/ exists