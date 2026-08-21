%% test_objective_full_envelope.m
% Runs the LQI vs LQI+RBF comparison AT THE OBJECTIVE'S OWN STATED
% CONDITIONS: +-20% combined parametric uncertainty, and disturbance
% severity scaled to the upper end of the datasheet's tested range
% (up to 3N, consistent with test_disturbance.m), not the milder 10deg/2N
% proxy used in earlier exploratory checks. Numbers are reported as
% measured -- this script does not search for parameters that produce a
% target outcome.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% ---- Settling time: large-angle, +-20% uncertainty applied --------------
angleLevels = [15, 18, 20];
uncertaintyLevels = [-0.20, 0, 0.20];

settlingRows = struct('angle', {}, 'pct', {}, 'settling_on', {}, 'settling_off', {}, 'pct_improve', {});
idx = 1;
for a = 1:numel(angleLevels)
    for p = 1:numel(uncertaintyLevels)
        pct = uncertaintyLevels(p);
        base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
            'initAngleDeg', angleLevels(a), 'observerMethod', 'kalman', ...
            'Fd_func', @(t) 0, ...
            'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
            'massPct', pct, 'frictionPct', pct, 'lengthPct', pct);

        sc_on = base; sc_on.useRBF = true;
        sc_off = base; sc_off.useRBF = false;

        r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
        r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

        if isnan(m_off.settling_time_s) || m_off.settling_time_s == 0
            settling_pct = NaN; % avoid divide-by-zero / undefined improvement
        else
            settling_pct = 100 * (m_off.settling_time_s - m_on.settling_time_s) / m_off.settling_time_s;
        end

        settlingRows(idx).angle = angleLevels(a);
        settlingRows(idx).pct = pct;
        settlingRows(idx).settling_on = m_on.settling_time_s;
        settlingRows(idx).settling_off = m_off.settling_time_s;
        settlingRows(idx).pct_improve = settling_pct;
        idx = idx + 1;

        fprintf('angle=%ddeg, uncertainty=%+.0f%%: settling ON=%.2fs OFF=%.2fs (%.1f%% improve)\n', ...
            angleLevels(a), pct*100, m_on.settling_time_s, m_off.settling_time_s, settling_pct);
    end
end

%% ---- IAE under disturbance: up to 3N, +-20% uncertainty applied --------
forceLevels = [1.0, 2.0, 3.0]; % full tested range, including the known 3N edge case
iaeRows = struct('force', {}, 'pct', {}, 'iae_on', {}, 'iae_off', {}, 'pct_improve', {});
idx = 1;
for f = 1:numel(forceLevels)
    for p = 1:numel(uncertaintyLevels)
        pct = uncertaintyLevels(p);
        base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, 'initAngleDeg', 0, ...
            'observerMethod', 'kalman', ...
            'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
            'massPct', pct, 'frictionPct', pct, 'lengthPct', pct, ...
            'Fd_func', @(t) forceLevels(f) * (t >= 2 && t < 2.5));

        sc_on = base; sc_on.useRBF = true;
        sc_off = base; sc_off.useRBF = false;

        r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
        r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

        if m_off.iae_theta == 0
            iae_pct = NaN;
        else
            iae_pct = 100 * (m_off.iae_theta - m_on.iae_theta) / m_off.iae_theta;
        end

        iaeRows(idx).force = forceLevels(f);
        iaeRows(idx).pct = pct;
        iaeRows(idx).iae_on = m_on.iae_theta;
        iaeRows(idx).iae_off = m_off.iae_theta;
        iaeRows(idx).pct_improve = iae_pct;
        idx = idx + 1;

        fprintf('force=%.1fN, uncertainty=%+.0f%%: IAE ON=%.4f OFF=%.4f (%.1f%% improve)\n', ...
            forceLevels(f), pct*100, m_on.iae_theta, m_off.iae_theta, iae_pct);
    end
end

%% ---- Stability sweep across the full +-20% uncertainty grid --------------
% Combined uncertainty grid, not just the three corner cases -- gives a
% fuller picture of where (if anywhere) stability differs between ON/OFF.
uncertaintyGrid = linspace(-0.20, 0.20, 5);
stabilityRows = struct('pct', {}, 'stable_on', {}, 'stable_off', {});
for p = 1:numel(uncertaintyGrid)
    pct = uncertaintyGrid(p);
    base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, 'initAngleDeg', 10, ...
        'observerMethod', 'kalman', 'Fd_func', @(t) 0, ...
        'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
        'massPct', pct, 'frictionPct', pct, 'lengthPct', pct);

    sc_on = base; sc_on.useRBF = true;
    sc_off = base; sc_off.useRBF = false;

    r_on  = simulation_main(sc_on);  m_on  = performance_metrics(r_on, params);
    r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);

    stabilityRows(p).pct = pct;
    stabilityRows(p).stable_on = m_on.pass && ~isnan(m_on.settling_time_s);
    stabilityRows(p).stable_off = m_off.pass && ~isnan(m_off.settling_time_s);

    fprintf('uncertainty=%+.0f%%: stable ON=%d, stable OFF=%d\n', ...
        pct*100, stabilityRows(p).stable_on, stabilityRows(p).stable_off);
end

%% ---- Honest summary against stated objectives ----------------------------
avgSettlingImprove = mean([settlingRows.pct_improve], 'omitnan');
avgIAEImprove = mean([iaeRows.pct_improve], 'omitnan');
allStableOn = all([stabilityRows.stable_on]);

fprintf('\n=== OBJECTIVE CHECK (measured, not tuned) ===\n');
fprintf('Avg settling time improvement (large-angle, +-20%% uncertainty): %.1f%% (target >=15%%) -> %s\n', ...
    avgSettlingImprove, ternary(avgSettlingImprove >= 15, 'PASS', 'FAIL'));
fprintf('Avg IAE improvement (disturbance up to 3N, +-20%% uncertainty): %.1f%% (target >=20%%) -> %s\n', ...
    avgIAEImprove, ternary(avgIAEImprove >= 20, 'PASS', 'FAIL'));
fprintf('Stable under full +-20%% uncertainty grid (RBF ON): %d -> %s\n', ...
    allStableOn, ternary(allStableOn, 'PASS', 'FAIL'));

save(fullfile(resultsDir, 'results_objective_full_envelope.mat'), ...
    'settlingRows', 'iaeRows', 'stabilityRows', 'avgSettlingImprove', 'avgIAEImprove', 'allStableOn');

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end