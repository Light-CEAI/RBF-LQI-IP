%% test_mass_variation.m
% Robustness to mass mismatch between design-time (nominal) and
% true plant mass, per datasheet Section 5: +-30%.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

massPctLevels = [-0.30, -0.15, 0, 0.15, 0.30];

mass_results = cell(numel(massPctLevels), 1);
mass_metrics = cell(numel(massPctLevels), 1);
for i = 1:numel(massPctLevels)
    scenario = struct();
    scenario.Ts             = params.embedded.Ts;
    scenario.Tfinal         = 8;
    scenario.initAngleDeg   = 10;
    scenario.useRBF         = true;
    scenario.observerMethod = 'kalman';
    scenario.massPct        = massPctLevels(i);
    scenario.Fd_func        = @(t) 0;
    scenario.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

    mass_results{i} = simulation_main(scenario);
    mass_metrics{i} = performance_metrics(mass_results{i}, params);

    fprintf('Mass variation %+.0f%%: max|theta|=%.2f deg, max|u|=%.3f, PASS=%d\n', ...
        massPctLevels(i)*100, mass_metrics{i}.max_theta_deg, mass_metrics{i}.max_u, mass_metrics{i}.pass);
    if ~mass_metrics{i}.pass
        fprintf('  FAIL: %s\n', strjoin(mass_metrics{i}.fail_reasons, '; '));
    end
end
save(fullfile(resultsDir, 'results_mass_variation.mat'), ...
    'mass_results', 'massPctLevels', 'mass_metrics');