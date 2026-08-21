%% test_config_AB_comparison.m
% Direct comparison of Config A (wheels/torque) vs Config B (sliding
% cart/direct force) on the identical scenario. Neither config has been
% explicitly exercised by any prior test script -- this generates the
% first real A-vs-B dataset.
%
% NOTE: the pendulum animation/draw_frame.m will look IDENTICAL between
% configs regardless -- it draws generic [x,theta] geometry, not the
% force-application mechanism. The meaningful comparison is in u
% (interpreted differently: N*m torque for A, N force for B) and the
% resulting state trajectories, not the visual.

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

scenario_base = struct('Tfinal', 8, 'initAngleDeg', 10, 'observerMethod', 'kalman', ...
    'Fd_func', @(t) 1.5*(t>=2 && t<2.5), 'useRBF', true);

paramsA = parameters('A');
scenarioA = scenario_base; scenarioA.config = 'A'; scenarioA.Ts = paramsA.embedded.Ts;
scenarioA.measNoiseStd = [paramsA.sensor.x_quant_std; paramsA.sensor.theta_quant_std];
resultsA = simulation_main(scenarioA);
metricsA = performance_metrics(resultsA, paramsA);

paramsB = parameters('B');
scenarioB = scenario_base; scenarioB.config = 'B'; scenarioB.Ts = paramsB.embedded.Ts;
scenarioB.measNoiseStd = [paramsB.sensor.x_quant_std; paramsB.sensor.theta_quant_std];
resultsB = simulation_main(scenarioB);
metricsB = performance_metrics(resultsB, paramsB);

fprintf('Config A: max|theta|=%.2fdeg, max|u|=%.4f N*m (torque), actuator limit=%.2f N*m\n', ...
    metricsA.max_theta_deg, metricsA.max_u, paramsA.cart.tau_max);
fprintf('Config B: max|theta|=%.2fdeg, max|u|=%.4f N (force), actuator limit=%.2f N\n', ...
    metricsB.max_theta_deg, metricsB.max_u, paramsB.cart.u_max);

figure('Name','Config A vs B Comparison');
subplot(2,1,1);
plot(resultsA.t, rad2deg(resultsA.x_true(3,:)), 'b-', 'LineWidth',1.2); hold on;
plot(resultsB.t, rad2deg(resultsB.x_true(3,:)), 'r--', 'LineWidth',1.2);
legend('Config A (wheels)', 'Config B (direct force)', 'Location','best');
ylabel('theta [deg]'); grid on; title('State Response: Config A vs B (geometry-identical, dynamics differ)');

subplot(2,1,2);
plot(resultsA.t(1:end-1), resultsA.u, 'b-', 'LineWidth',1.0); hold on;
plot(resultsB.t(1:end-1), resultsB.u, 'r--', 'LineWidth',1.0);
legend('Config A: u [N*m]', 'Config B: u [N]', 'Location','best');
ylabel('actuator command (different units!)'); xlabel('time [s]'); grid on;

save(fullfile(resultsDir,'results_config_AB_comparison.mat'), 'resultsA','resultsB','metricsA','metricsB');