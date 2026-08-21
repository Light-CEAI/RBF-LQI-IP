%% build_results_table.m
% Consolidates EVERY results .mat file generated so far -- robustness
% sweeps, objective check, Monte Carlo, RBF metrics, timing -- into one
% master table, formatted for direct inclusion in a thesis results
% chapter. Run this LAST, after all other test/evaluation scripts.

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
rows = {}; % {Category, Metric, Value, Target, Status}

addrow = @(rows, cat, metric, val, target, status) [rows; {cat, metric, val, target, status}];

%% ---- Objective check (test_objective_full_envelope.m) -------------------
if exist(fullfile(resultsDir,'results_objective_full_envelope.mat'), 'file')
    s = load(fullfile(resultsDir,'results_objective_full_envelope.mat'));
    rows = addrow(rows, 'Objective', 'Avg settling time improvement', ...
        sprintf('%.1f%%', s.avgSettlingImprove), '>=15%', ...
        ternary(s.avgSettlingImprove>=15,'PASS','FAIL'));
    rows = addrow(rows, 'Objective', 'Avg IAE improvement (disturbance)', ...
        sprintf('%.1f%%', s.avgIAEImprove), '>=20%', ...
        ternary(s.avgIAEImprove>=20,'PASS','FAIL'));
    rows = addrow(rows, 'Objective', 'Stable under +-20%% uncertainty', ...
        sprintf('%d', s.allStableOn), 'sustained', ...
        ternary(s.allStableOn,'PASS','FAIL'));
end

%% ---- Monte Carlo (monte_carlo_evaluation.m) ------------------------------
if exist(fullfile(resultsDir,'results_monte_carlo.mat'), 'file')
    s = load(fullfile(resultsDir,'results_monte_carlo.mat'));
    fn = fieldnames(s.mc_results);
    for i = 1:numel(fn)
        r = s.mc_results.(fn{i});
        sig = ~isnan(r.p_settling) && r.p_settling < 0.05;
        rows = addrow(rows, 'Monte Carlo', sprintf('%s: settling diff (N=%d)', fn{i}, s.N_SEEDS), ...
            sprintf('%.3f +/- %.3fs', r.settling_diff_mean, r.settling_diff_ci95), 'CI excludes 0', ...
            ternary(sig, 'SIGNIFICANT', 'NOT SIGNIFICANT'));
        sigI = ~isnan(r.p_iae) && r.p_iae < 0.05;
        rows = addrow(rows, 'Monte Carlo', sprintf('%s: IAE diff (N=%d)', fn{i}, s.N_SEEDS), ...
            sprintf('%.4f +/- %.4f', r.iae_diff_mean, r.iae_diff_ci95), 'CI excludes 0', ...
            ternary(sigI, 'SIGNIFICANT', 'NOT SIGNIFICANT'));
    end
end

%% ---- Timing benchmark (benchmark_control_step.m) -------------------------
if exist(fullfile(resultsDir,'results_timing_benchmark.mat'), 'file')
    s = load(fullfile(resultsDir,'results_timing_benchmark.mat'));
    util = 100*mean(s.times)/s.Ts_budget;
    rows = addrow(rows, 'Timing', 'Mean control_step.m execution (MATLAB)', ...
        sprintf('%.2f us', mean(s.times)*1e6), sprintf('<%.0f us (Ts budget)', s.Ts_budget*1e6), ...
        ternary(util<100, 'WITHIN BUDGET (interpreted)', 'EXCEEDS BUDGET'));
end

%% ---- Existing robustness summary (run_all_tests.m) -----------------------
if exist(fullfile(resultsDir,'results_summary.csv'), 'file')
    rt = readtable(fullfile(resultsDir,'results_summary.csv'));
    nPass = sum(rt.Pass); nTotal = height(rt);
    rows = addrow(rows, 'Robustness', 'Test cases passing', ...
        sprintf('%d/%d', nPass, nTotal), 'documented boundary acceptable', ...
        sprintf('%d failures (see results_summary.csv)', nTotal-nPass));
end

%% ---- Write master table ----------------------------------------------------
if isempty(rows)
    warning('build_results_table:noData', 'No prior result files found -- run other scripts first.');
else
    masterTable = cell2table(rows, 'VariableNames', {'Category','Metric','Value','Target','Status'});
    disp(masterTable);
    writetable(masterTable, fullfile(resultsDir, 'master_results_table.csv'));
    save(fullfile(resultsDir, 'master_results_table.mat'), 'masterTable');
    fprintf('\nMaster table saved: Results/master_results_table.csv (%d rows)\n', height(masterTable));
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end