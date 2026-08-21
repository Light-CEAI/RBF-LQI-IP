%% run_visualizations.m
% Generates the visualization deliverables using the CURRENT, post-fix
% control architecture (saturation/anti-windup bug fixed, leakage
% restored, Q retuned). Run from the Visualizations folder.

%% run_visualizations.m
params = parameters();

% UPDATED: Points exactly to your deep subfolder structure
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results', 'videos'); 

%% ---- 1. Nominal case: clean baseline animation + video -----------------
% ... (rest of code stays exactly the same) ...
%% ---- 1. Nominal case: clean baseline animation + video -----------------
scenario_nom = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'initAngleDeg', 10, 'observerMethod', 'kalman', 'Fd_func', @(t) 0, ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
    'useRBF', true);
results_nom = simulation_main(scenario_nom);
metrics_nom = performance_metrics(results_nom, params);

% Create distinct Window for Nominal Plot
figure(1); 
plot_results(results_nom, metrics_nom, 'Nominal (10deg)');

% Create distinct Window for Nominal Animation
figAnim1 = figure(2); 
animate_pendulum(results_nom, params, 2); 
create_video(results_nom, params, fullfile(resultsDir, 'video_nominal.avi'));

%% ---- 2. The previously-catastrophic case, now fixed --------------------
scenario_fixed = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'initAngleDeg', 0, 'observerMethod', 'kalman', ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
    'massPct', -0.20, 'frictionPct', -0.20, 'lengthPct', -0.20, ...
    'Fd_func', @(t) 2.0 * (t >= 2 && t < 2.5), 'useRBF', true);
results_fixed = simulation_main(scenario_fixed);
metrics_fixed = performance_metrics(results_fixed, params);

% FORCE MATLAB Online to open brand new windows for the second run
figure(3); 
plot_results(results_fixed, metrics_fixed, '2N disturbance, -20% uncertainty (post-fix)');

figure(4); 
animate_pendulum(results_fixed, params, 2); 

create_video(results_fixed, params, fullfile(resultsDir, 'video_2N_minus20pct_fixed.avi'));
fprintf('Visualizations complete. Videos saved to %s\n', resultsDir);