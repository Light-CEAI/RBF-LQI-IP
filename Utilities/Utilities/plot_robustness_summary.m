%% plot_robustness_summary.m
% Generates summary plots across all robustness/disturbance test sweeps:
% max|theta|, max|u|, and settling time vs. the varied parameter, for
% each of: force disturbance, initial angle, mass variation, friction
% variation, length variation.
%
% Run AFTER run_all_tests.m (or the individual test_*.m scripts), since
% it reads from the saved Results/*.mat files rather than re-simulating.

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');

%% ---- Force disturbance --------------------------------------------------
s = load(fullfile(resultsDir, 'results_disturbance_force.mat'));
maxTheta = cellfun(@(m) m.max_theta_deg, s.force_metrics);
maxX     = cellfun(@(m) m.max_x_m, s.force_metrics);
passFlag = cellfun(@(m) m.pass, s.force_metrics);

figure('Name', 'Force Disturbance Robustness');
subplot(2,1,1);
plot(s.forceLevels, maxTheta, 'b-o', 'LineWidth', 1.3); hold on;
plot(s.forceLevels(~passFlag), maxTheta(~passFlag), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('Disturbance force [N]'); ylabel('Max |\theta| [deg]'); grid on;
title('Force Disturbance: angle response');
legend('all cases', 'FAIL cases', 'Location', 'best');

subplot(2,1,2);
plot(s.forceLevels, maxX, 'b-o', 'LineWidth', 1.3); hold on;
yline(0.35, 'r--', 'safe travel limit');
plot(s.forceLevels(~passFlag), maxX(~passFlag), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('Disturbance force [N]'); ylabel('Max |x| [m]'); grid on;
title('Force Disturbance: position response');

%% ---- Initial angle disturbance -----------------------------------------
s = load(fullfile(resultsDir, 'results_disturbance_angle.mat'));
settling = cellfun(@(m) m.settling_time_s, s.angle_metrics);

figure('Name', 'Initial Angle Disturbance');
plot(s.angleLevels, settling, 'b-o', 'LineWidth', 1.3);
xlabel('Initial angle [deg]'); ylabel('Settling time [s]'); grid on;
title('Settling Time vs Initial Angle Disturbance');

%% ---- Mass variation ------------------------------------------------------
s = load(fullfile(resultsDir, 'results_mass_variation.mat'));
maxTheta = cellfun(@(m) m.max_theta_deg, s.mass_metrics);
maxU     = cellfun(@(m) m.max_u, s.mass_metrics);

figure('Name', 'Mass Variation Robustness');
subplot(2,1,1);
plot(s.massPctLevels*100, maxTheta, 'b-o', 'LineWidth', 1.3);
xlabel('Mass variation [%]'); ylabel('Max |\theta| [deg]'); grid on;
title('Mass Variation: angle response');

subplot(2,1,2);
plot(s.massPctLevels*100, maxU, 'b-o', 'LineWidth', 1.3);
xlabel('Mass variation [%]'); ylabel('Max |u|'); grid on;
title('Mass Variation: control effort');

%% ---- Friction variation --------------------------------------------------
s = load(fullfile(resultsDir, 'results_friction_variation.mat'));
maxTheta = cellfun(@(m) m.max_theta_deg, s.friction_metrics);
maxU     = cellfun(@(m) m.max_u, s.friction_metrics);

figure('Name', 'Friction Variation Robustness');
subplot(2,1,1);
plot(s.frictionPctLevels*100, maxTheta, 'b-o', 'LineWidth', 1.3);
xlabel('Friction variation [%]'); ylabel('Max |\theta| [deg]'); grid on;
title('Friction Variation: angle response');

subplot(2,1,2);
plot(s.frictionPctLevels*100, maxU, 'b-o', 'LineWidth', 1.3);
xlabel('Friction variation [%]'); ylabel('Max |u|'); grid on;
title('Friction Variation: control effort');

%% ---- Length variation ------------------------------------------------------
s = load(fullfile(resultsDir, 'results_length_variation.mat'));
maxTheta = cellfun(@(m) m.max_theta_deg, s.length_metrics);
maxU     = cellfun(@(m) m.max_u, s.length_metrics);

figure('Name', 'Length Variation Robustness');
subplot(2,1,1);
plot(s.lengthPctLevels*100, maxTheta, 'b-o', 'LineWidth', 1.3);
xlabel('Length variation [%]'); ylabel('Max |\theta| [deg]'); grid on;
title('Length Variation: angle response');

subplot(2,1,2);
plot(s.lengthPctLevels*100, maxU, 'b-o', 'LineWidth', 1.3);
xlabel('Length variation [%]'); ylabel('Max |u|'); grid on;
title('Length Variation: control effort');

fprintf('Generated 5 robustness summary figures.\n');