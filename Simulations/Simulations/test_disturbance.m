%% test_disturbance.m
% Robustness to (a) external force disturbances and (b) larger initial
% angle offsets, per datasheet Section 5:
%   Force disturbance: 0.5 to 3 N
%   Initial angle disturbance: +-5 to +-20 deg

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% ---- Part A: force disturbance sweep, applied as a step at t=2s -------
forceLevels = linspace(params.test.force_dist_range_N(1), ...
    params.test.force_dist_range_N(2), 4);

force_results = cell(numel(forceLevels), 1);
force_metrics = cell(numel(forceLevels), 1);
for i = 1:numel(forceLevels)
    Fmag = forceLevels(i);

    scenario = struct();
    scenario.Ts             = params.embedded.Ts;
    scenario.Tfinal         = 8;
    scenario.initAngleDeg   = 0;
    scenario.useRBF         = true;
    scenario.observerMethod = 'kalman';
    scenario.Fd_func        = @(t) Fmag * (t >= 2 && t < 2.5); % 0.5s pulse
    scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

    force_results{i} = simulation_main(scenario);
    force_metrics{i} = performance_metrics(force_results{i}, params);

    fprintf('Force disturbance %.2f N: max|theta|=%.2f deg, PASS=%d\n', ...
        Fmag, force_metrics{i}.max_theta_deg, force_metrics{i}.pass);
    if ~force_metrics{i}.pass
        fprintf('  FAIL: %s\n', strjoin(force_metrics{i}.fail_reasons, '; '));
    end
end
save(fullfile(resultsDir, 'results_disturbance_force.mat'), ...
    'force_results', 'forceLevels', 'force_metrics');

%% ---- Part B: initial angle sweep ---------------------------------------
angleLevels = [5, 10, 15, 20]; % deg, matches datasheet range

angle_results = cell(numel(angleLevels), 1);
angle_metrics = cell(numel(angleLevels), 1);
for i = 1:numel(angleLevels)
    scenario = struct();
    scenario.Ts             = params.embedded.Ts;
    scenario.Tfinal         = 8;
    scenario.initAngleDeg   = angleLevels(i);
    scenario.useRBF         = true;
    scenario.observerMethod = 'kalman';
    scenario.Fd_func        = @(t) 0;
    scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

    angle_results{i} = simulation_main(scenario);
    angle_metrics{i} = performance_metrics(angle_results{i}, params);

    fprintf('Initial angle %.0f deg: max|theta|=%.2f deg, PASS=%d\n', ...
        angleLevels(i), angle_metrics{i}.max_theta_deg, angle_metrics{i}.pass);
    if ~angle_metrics{i}.pass
        fprintf('  FAIL: %s\n', strjoin(angle_metrics{i}.fail_reasons, '; '));
    end
end
save(fullfile(resultsDir, 'results_disturbance_angle.mat'), ...
    'angle_results', 'angleLevels', 'angle_metrics');