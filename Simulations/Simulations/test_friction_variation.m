%% test_friction_variation.m
% Robustness to friction mismatch, per datasheet Section 5: up to +300%.
% NOTE: "up to 300%" is interpreted as the TRUE friction being up to 3x
% the nominal design value; datasheet does not specify a lower bound, so
% -100% (frictionless) is included as the other extreme since that's
% also physically meaningful (e.g. lubricated rail) and stresses the
% controller differently (less natural damping).

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

frictionPctLevels = [-1.0, 0, 1.0, 2.0, 3.0]; % -100% (frictionless) .. +300%

friction_results = cell(numel(frictionPctLevels), 1);
friction_metrics = cell(numel(frictionPctLevels), 1);
for i = 1:numel(frictionPctLevels)
    scenario = struct();
    scenario.Ts             = params.embedded.Ts;
    scenario.Tfinal         = 8;
    scenario.initAngleDeg   = 10;
    scenario.useRBF         = true;
    scenario.observerMethod = 'kalman';
    scenario.frictionPct    = frictionPctLevels(i);
    scenario.Fd_func        = @(t) 0;
    scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

    friction_results{i} = simulation_main(scenario);
    friction_metrics{i} = performance_metrics(friction_results{i}, params);

    fprintf('Friction variation %+.0f%%: max|theta|=%.2f deg, max|u|=%.3f, PASS=%d\n', ...
        frictionPctLevels(i)*100, friction_metrics{i}.max_theta_deg, friction_metrics{i}.max_u, friction_metrics{i}.pass);
    if ~friction_metrics{i}.pass
        fprintf('  FAIL: %s\n', strjoin(friction_metrics{i}.fail_reasons, '; '));
    end
end
save(fullfile(resultsDir, 'results_friction_variation.mat'), ...
    'friction_results', 'frictionPctLevels', 'friction_metrics');