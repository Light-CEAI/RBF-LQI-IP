%% monte_carlo_evaluation.m
% Addresses the single-largest defensibility gap: every prior IAE/
% settling-time number was from ONE noisy run per condition. This script
% repeats key conditions (nominal, large-angle, disturbance) across N
% independent measurement-noise seeds, reporting mean +/- std and a
% paired comparison (RBF ON vs OFF) rather than a single sample.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

N_SEEDS = 30;
conditions = struct( ...
    'name', {'nominal_10deg', 'large_angle_18deg', 'disturbance_2N'}, ...
    'initAngleDeg', {10, 18, 0}, ...
    'Fd_func', {@(t) 0, @(t) 0, @(t) 2.0*(t>=2 && t<2.5)} ...
);

mc_results = struct();

for c = 1:numel(conditions)
    cond = conditions(c);
    settling_on  = nan(N_SEEDS,1); settling_off = nan(N_SEEDS,1);
    iae_on       = nan(N_SEEDS,1); iae_off      = nan(N_SEEDS,1);

    for s = 1:N_SEEDS
        rng(s); % reproducible seed per repetition

        base = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
            'initAngleDeg', cond.initAngleDeg, 'observerMethod', 'kalman', ...
            'Fd_func', cond.Fd_func, ...
            'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std]);

        sc_on = base; sc_on.useRBF = true;
        r_on = simulation_main(sc_on); m_on = performance_metrics(r_on, params);
        settling_on(s) = m_on.settling_time_s; iae_on(s) = m_on.iae_theta;

        rng(s); % SAME seed for OFF run -> identical noise realization, paired comparison
        sc_off = base; sc_off.useRBF = false;
        r_off = simulation_main(sc_off); m_off = performance_metrics(r_off, params);
        settling_off(s) = m_off.settling_time_s; iae_off(s) = m_off.iae_theta;
    end

    settling_diff = settling_off - settling_on; % positive = RBF faster
    iae_diff = iae_off - iae_on;                % positive = RBF lower IAE

    res.settling_on_mean = mean(settling_on, 'omitnan'); res.settling_on_std = std(settling_on, 'omitnan');
    res.settling_off_mean = mean(settling_off, 'omitnan'); res.settling_off_std = std(settling_off, 'omitnan');
    res.iae_on_mean = mean(iae_on); res.iae_on_std = std(iae_on);
    res.iae_off_mean = mean(iae_off); res.iae_off_std = std(iae_off);

    % Paired comparison: mean diff, 95% CI (normal approx, N=30 is adequate)
    res.settling_diff_mean = mean(settling_diff, 'omitnan');
    res.settling_diff_ci95 = 1.96 * std(settling_diff, 'omitnan') / sqrt(sum(~isnan(settling_diff)));
    res.iae_diff_mean = mean(iae_diff);
    res.iae_diff_ci95 = 1.96 * std(iae_diff) / sqrt(N_SEEDS);

    % Optional paired t-test if Statistics Toolbox available
    try
        [~, p_settling] = ttest(settling_diff);
        [~, p_iae] = ttest(iae_diff);
        res.p_settling = p_settling; res.p_iae = p_iae;
    catch
        res.p_settling = NaN; res.p_iae = NaN;
        fprintf('(Statistics Toolbox ttest unavailable -- reporting CI only, no p-value)\n');
    end

    mc_results.(cond.name) = res;

    fprintf('--- %s (N=%d seeds) ---\n', cond.name, N_SEEDS);
    fprintf('Settling: ON=%.2f+/-%.2fs OFF=%.2f+/-%.2fs | diff=%.3f+/-%.3fs (95%% CI)', ...
        res.settling_on_mean, res.settling_on_std, res.settling_off_mean, res.settling_off_std, ...
        res.settling_diff_mean, res.settling_diff_ci95);
    if ~isnan(res.p_settling), fprintf('  p=%.4f', res.p_settling); end
    fprintf('\n');
    fprintf('IAE:      ON=%.4f+/-%.4f OFF=%.4f+/-%.4f | diff=%.4f+/-%.4f (95%% CI)', ...
        res.iae_on_mean, res.iae_on_std, res.iae_off_mean, res.iae_off_std, ...
        res.iae_diff_mean, res.iae_diff_ci95);
    if ~isnan(res.p_iae), fprintf('  p=%.4f', res.p_iae); end
    fprintf('\n\n');
end

save(fullfile(resultsDir, 'results_monte_carlo.mat'), 'mc_results', 'N_SEEDS');
fprintf('Saved results_monte_carlo.mat (N=%d seeds per condition)\n', N_SEEDS);