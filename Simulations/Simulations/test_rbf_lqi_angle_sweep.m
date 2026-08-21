%% test_rbf_lqi_angle_sweep.m
% Diagnostic script to evaluate RBF-Augmented LQI at large initial angles
% Features: Dynamic wide-span grid generation, boosted learning updates,
% and active warning suppression for clean console telemetry output.

% 1. Load Base Configurations
params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% 2. Clean Console Clutter (Suppress the expected missing mat-file warning)
warning('off', 'rbf_initialize:noFile');

% 3. Set Up Diagnostic Overrides for Wide-Envelope Recovery
params_diag = params;
params_diag.mech.track_length = 2.0;       % Allow +-1.0m travel for recovery recording

% --- Architectural Grid Expansion ---
params_diag.rbf.x_range = [-1.0, 1.0];     % Wide-span tracking box boundaries
params_diag.rbf.n_hidden = 64;             % Node count for the wider tracking envelope

% --- High-Velocity Update Tuning Overrides ---
params_diag.rbf.learning_rate = params.rbf.learning_rate * 2.5;  % Boost adaptation speed
params_diag.rbf.sigma_x = 0.65;                                  % Widen Gaussian position kernel

% 4. Initialize Simulation Settings
angleLevels = [20, 25, 30]; 
colors = [0 0.5 0; 0.75 0.5 0; 0 0 0.8];   % Green, Orange, Blue

results_sweep_rbf = cell(numel(angleLevels),1);

fprintf('=== Running Fully Repaired RBF-LQI Angle Sweep ===\n');
for i = 1:numel(angleLevels)
    scenario = struct('Ts', params_diag.embedded.Ts, 'Tfinal', 8, ...
        'x0', [0; 0; deg2rad(angleLevels(i)); 0], 'r', 0, 'Fd_func', @(t) 0, ...
        'measNoiseStd', [params_diag.sensor.x_quant_std; params_diag.sensor.theta_quant_std]);

    % Execute the simulation loop via the updated execution core wrapper
    results_sweep_rbf{i} = run_controller_variant('lqi_rbf', params_diag, scenario);

    % Diagnostic Telemetry Prints
    w_init = results_sweep_rbf{i}.rbf_weights(:, 1);
    w_final = results_sweep_rbf{i}.rbf_weights(:, end);

    fprintf('\n---> %ddeg Telemetry:\n', angleLevels(i));
    if results_sweep_rbf{i}.diverged
        fprintf('  Status: DIVERGED at t=%.3fs\n', results_sweep_rbf{i}.divergence_time_s);
    else
        fprintf('  Status: STABLE Recovery achieved!\n');
    end
    fprintf('  Initial Weights Magnitude Sum: %f\n', sum(abs(w_init), 'omitnan'));
    fprintf('  Final Weights Magnitude Sum:   %f\n', sum(abs(w_final), 'omitnan'));
end

% Restore warnings back to default behavior after the sweep runs
warning('on', 'rbf_initialize:noFile');

%% 5. Visualization Layout -------------------------------------------------
figure('Name','RBF-LQI Angle Sweep Evaluation','Position',[100 100 1000 600]);

subplot(2,1,1);
hold on;
legendEntries = cell(numel(angleLevels),1);
for i = 1:numel(angleLevels)
    validIdx = ~isnan(results_sweep_rbf{i}.x_true(3,:));
    plot(results_sweep_rbf{i}.t(validIdx), rad2deg(results_sweep_rbf{i}.x_true(3,validIdx)), ...
        'Color', colors(i,:), 'LineWidth', 1.5);

    if results_sweep_rbf{i}.diverged
        legendEntries{i} = sprintf('RBF-LQI %d%s (Diverged)', angleLevels(i), char(176));
    else
        legendEntries{i} = sprintf('RBF-LQI %d%s (Stable)', angleLevels(i), char(176));
    end
end
legend(legendEntries, 'Location', 'best');
ylabel('theta [deg]'); grid on;
title('RBF-Augmented LQI, Large Initial Angle Dynamic Response');

subplot(2,1,2);
hold on;
for i = 1:numel(angleLevels)
    validIdx = ~isnan(results_sweep_rbf{i}.x_true(1,:));
    plot(results_sweep_rbf{i}.t(validIdx), results_sweep_rbf{i}.x_true(1,validIdx), ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
yline(1.0, 'r--', 'Extended Rail Limit (+-1.0m)', 'LineWidth', 1.2);
yline(0.35, 'k:', 'Design Safe Envelope (+-0.35m)', 'LineWidth', 1.2);
ylabel('x [m]'); xlabel('time [s]'); grid on;

% Save data workspace structures
save(fullfile(resultsDir,'results_rbf_lqi_angle_sweep.mat'), 'results_sweep_rbf', 'angleLevels');