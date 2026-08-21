%% test_rbf_vs_lqi_comparison.m
% Direct test of Objectives 2 & 3: does LQI+RBF outperform standalone LQI
% by the claimed margins (>=15% settling time reduction, >=20% IAE
% reduction under disturbance), and does it remain stable under +-20%
% combined parametric uncertainty?
%
% Re-run AFTER the simulation_main.m / lqi_controller.m architecture fix
% (saturation + anti-windup now applied ONCE to the combined u_lqi+f_rbf
% command, not twice) -- all prior runs of this script predate that fix
% and are invalid.
%
% Saves results_rbf_comparison.mat (single nominal-vs-RBF case, for the
% LQI-vs-RBF response plots) and results_rbf_objective_check.mat
% (caseA/caseB/caseC sweep structs, for the objective_validation_plots.py
% script) -- field names match what those downstream plot scripts expect.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% ---- Single comparison case: for LQI-vs-RBF response plots -------------
scenario_base = struct();
scenario_base.Ts             = params.embedded.Ts;
scenario_base.Tfinal         = 8;
scenario_base.initAngleDeg   = 10;
scenario_base.observerMethod = 'kalman';
scenario_base.Fd_func        = @(t) 2.0 * (t >= 2 && t < 2.5); % 2N pulse, mid-range case
scenario_base.measNoiseStd   = [params.sensor.x_quant_std; params.sensor.theta_quant_std];

scenario_on = scenario_base;  scenario_on.useRBF  = true;
scenario_off = scenario_base; scenario_off.useRBF = false;

results_on  = simulation_main(scenario_on);
results_off = simulation_main(scenario_off);

metrics_on  = performance_metrics(results_on, params);
metrics_off = performance_metrics(results_off, params);

fprintf('--- RBF ON vs OFF (2N disturbance @ t=2s, 10deg) ---\n');
fprintf('RBF ON:  max|theta|=%.2f deg, settling=%.2fs, IAE=%.4f\n', ...
    metrics_on.max_theta_deg, metrics_on.settling_time_s, metrics_on.iae_theta);
fprintf('RBF OFF: max|theta|=%.2f deg, settling=%.2fs, IAE=%.4f\n', ...
    metrics_off.max_theta_deg, metrics_off.settling_time_s, metrics_off.iae_theta);

save(fullfile(resultsDir, 'results_rbf_comparison.mat'), ...
    'results_on', 'results_off', 'metrics_on', 'metrics_off');

%% ---- Case A: Large-angle stabilization (no disturbance) ----------------
angleLevels = [15, 18, 20];
caseA = struct('angle', {}, 'settling_on', {}, 'settling_off', {}, ...
    'iae_on', {}, 'iae_off', {}, 'settling_pct_improve', {}, 'iae_pct_improve', {});

for i = 1:numel(angleLevels)
    base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
        'initAngleDeg', angleLevels(i), 'observerMethod', 'kalman', ...
        'Fd_func', @(t) 0, 'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std]);

    sc_on = base; sc_on.useRBF = true;
    sc_off = base; sc_off.useRBF = false;

    r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
    r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

    if isnan(m_off.settling_time_s) || m_off.settling_time_s == 0
        settling_pct = NaN;
    else
        settling_pct = 100 * (m_off.settling_time_s - m_on.settling_time_s) / m_off.settling_time_s;
    end
    if m_off.iae_theta == 0
        iae_pct = NaN;
    else
        iae_pct = 100 * (m_off.iae_theta - m_on.iae_theta) / m_off.iae_theta;
    end

    caseA(i).angle = angleLevels(i);
    caseA(i).settling_on = m_on.settling_time_s; caseA(i).settling_off = m_off.settling_time_s;
    caseA(i).iae_on = m_on.iae_theta; caseA(i).iae_off = m_off.iae_theta;
    caseA(i).settling_pct_improve = settling_pct;
    caseA(i).iae_pct_improve = iae_pct;

    fprintf('Angle %ddeg: settling ON=%.2fs OFF=%.2fs (%.1f%% improve); IAE ON=%.3f OFF=%.3f (%.1f%% improve)\n', ...
        angleLevels(i), m_on.settling_time_s, m_off.settling_time_s, settling_pct, ...
        m_on.iae_theta, m_off.iae_theta, iae_pct);
end

%% ---- Case B: Disturbance rejection (IAE under a force pulse) -----------
forceLevels = [1.0, 2.0, 2.5];
caseB = struct('force', {}, 'iae_on', {}, 'iae_off', {}, 'iae_pct_improve', {});

for i = 1:numel(forceLevels)
    base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, 'initAngleDeg', 0, ...
        'observerMethod', 'kalman', 'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
        'Fd_func', @(t) forceLevels(i) * (t >= 2 && t < 2.5));

    sc_on = base; sc_on.useRBF = true;
    sc_off = base; sc_off.useRBF = false;

    r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
    r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

    if m_off.iae_theta == 0
        iae_pct = NaN;
    else
        iae_pct = 100 * (m_off.iae_theta - m_on.iae_theta) / m_off.iae_theta;
    end

    caseB(i).force = forceLevels(i);
    caseB(i).iae_on = m_on.iae_theta; caseB(i).iae_off = m_off.iae_theta;
    caseB(i).iae_pct_improve = iae_pct;

    fprintf('Force %.1fN: IAE ON=%.3f OFF=%.3f (%.1f%% improve)\n', ...
        forceLevels(i), m_on.iae_theta, m_off.iae_theta, iae_pct);
end

%% ---- Case C: +-20% combined parametric uncertainty, stability check ----
uncertaintyLevels = [-0.20, 0, 0.20];
caseC = struct('pct', {}, 'stable_on', {}, 'stable_off', {});

for i = 1:numel(uncertaintyLevels)
    pct = uncertaintyLevels(i);
    base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, 'initAngleDeg', 10, ...
        'observerMethod', 'kalman', 'Fd_func', @(t) 0, ...
        'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
        'massPct', pct, 'frictionPct', pct, 'lengthPct', pct);

    sc_on = base; sc_on.useRBF = true;
    sc_off = base; sc_off.useRBF = false;

    r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
    r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

    caseC(i).pct = pct;
    caseC(i).stable_on  = m_on.pass  && ~isnan(m_on.settling_time_s);
    caseC(i).stable_off = m_off.pass && ~isnan(m_off.settling_time_s);

    fprintf('Combined uncertainty %+.0f%%: stable ON=%d, stable OFF=%d\n', ...
        pct*100, caseC(i).stable_on, caseC(i).stable_off);
end

%% ---- Objective pass/fail summary ----------------------------------------
fprintf('\n=== OBJECTIVE CHECK ===\n');
avgSettlingImprove = mean([caseA.settling_pct_improve], 'omitnan');
avgIAEImprove_disturbance = mean([caseB.iae_pct_improve], 'omitnan');
allStable = all([caseC.stable_on]);

fprintf('Avg settling time improvement (large-angle cases): %.1f%% (target >=15%%)\n', avgSettlingImprove);
fprintf('Avg IAE improvement (disturbance cases): %.1f%% (target >=20%%)\n', avgIAEImprove_disturbance);
fprintf('Stable under +-20%% combined uncertainty (RBF ON): %d\n', allStable);

save(fullfile(resultsDir, 'results_rbf_objective_check.mat'), 'caseA', 'caseB', 'caseC', ...
    'avgSettlingImprove', 'avgIAEImprove_disturbance', 'allStable');