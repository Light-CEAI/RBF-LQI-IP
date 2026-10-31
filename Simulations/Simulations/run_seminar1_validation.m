%% run_seminar1_validation.m
% Focused Seminar I validation for the direct additive RBF-LQI controller.
% Run from the repository root after startup.m.
%
% Tests:
%   1. Nominal: 5 deg initial angle, LQI vs RBF-LQI
%   2. Disturbance: 1 N force pulse from 2.0 to 2.5 s, 5 deg initial angle
%   3. Mass uncertainty: -20% and +20%, 5 deg initial angle
%
% Both cart mass and pendulum mass are varied together because
% simulation_main.m applies scenario.massPct to both parameters.
% The nominal LQI and RBF-LQI runs use matched random measurement noise.
%
% Scope: ideal actuator; no motor electrical or PWM dynamics.

clearvars -except params
clc

%% 1. Check paths and train the offline RBF
params = parameters();

simFile = which('simulation_main');
trainFile = which('rbf_train_offline');
if isempty(simFile) || isempty(trainFile)
    error(['Project functions are not on the MATLAB path. ', ...
           'Open MATLAB at the repository root and run startup.m.']);
end

rbfFolder = fileparts(trainFile);
centersFile = fullfile(rbfFolder, 'rbf_centers.mat');

fprintf('\n=== Seminar I validation ===\n');
fprintf('Simulation function: %s\n', simFile);
fprintf('Training function:   %s\n', trainFile);

% Retrain from the current branch code, then verify the saved file.
rbf = rbf_train_offline(params, 3000);
if exist(centersFile, 'file') ~= 2
    error('Trained RBF file was not created: %s', centersFile);
end
rbfCheck = rbf_initialize(params, 'host', centersFile);
if norm(rbfCheck.weights) <= eps
    error('Loaded RBF weights are zero. Stop before running comparisons.');
end
fprintf('Trained RBF file: %s\n', centersFile);
fprintf('Centres: %d | Weight norm: %.8f\n', ...
    size(rbfCheck.centers,1), norm(rbfCheck.weights));

%% 2. Define the limited test matrix
Ts = params.embedded.Ts;
Tfinal = 8;
measurementNoise = [params.sensor.x_quant_std; ...
                    params.sensor.theta_quant_std];

scenarios = struct([]);

scenarios(1).name = 'Nominal';
scenarios(1).massPct = 0;
scenarios(1).Fd_func = @(t) 0;

scenarios(2).name = 'Disturbance_1N';
scenarios(2).massPct = 0;
scenarios(2).Fd_func = @(t) 1.0 * (t >= 2.0 && t < 2.5);

scenarios(3).name = 'Mass_minus20pct';
scenarios(3).massPct = -0.20;
scenarios(3).Fd_func = @(t) 0;

scenarios(4).name = 'Mass_plus20pct';
scenarios(4).massPct = 0.20;
scenarios(4).Fd_func = @(t) 0;

%% 3. Run matched LQI and RBF-LQI simulations
allResults = struct();
rows = cell(0, 1);

for k = 1:numel(scenarios)
    fprintf('\n--- %s ---\n', scenarios(k).name);

    common = struct();
    common.Ts = Ts;
    common.Tfinal = Tfinal;
    common.initAngleDeg = 5;
    common.r = 0;
    common.observerMethod = 'kalman';
    common.massPct = scenarios(k).massPct;
    common.frictionPct = 0;
    common.lengthPct = 0;
    common.Fd_func = scenarios(k).Fd_func;
    common.measNoiseStd = measurementNoise;

    % Reset the random stream before each controller so both see the
    % same measurement-noise realization for this scenario.
    rng(100 + k, 'twister');
    sLQI = common;
    sLQI.useRBF = false;
    resLQI = simulation_main(sLQI);

    rng(100 + k, 'twister');
    sRBF = common;
    sRBF.useRBF = true;
    resRBF = simulation_main(sRBF);

    metLQI = local_seminar_metrics(resLQI);
    metRBF = local_seminar_metrics(resRBF);

    allResults.(scenarios(k).name).LQI = resLQI;
    allResults.(scenarios(k).name).RBF_LQI = resRBF;
    allResults.(scenarios(k).name).metrics_LQI = metLQI;
    allResults.(scenarios(k).name).metrics_RBF_LQI = metRBF;

    rows{end+1,1} = local_row(scenarios(k).name, 'LQI', metLQI); %#ok<SAGROW>
    rows{end+1,1} = local_row(scenarios(k).name, 'RBF-LQI', metRBF); %#ok<SAGROW>

    fprintf('LQI:     peak |theta| %.4f deg | settling %.3f s | IAE %.6g | ISE %.6g\n', ...
        metLQI.peak_theta_deg, metLQI.settling_time_s, metLQI.IAE_theta, metLQI.ISE_theta);
    fprintf('RBF-LQI: peak |theta| %.4f deg | settling %.3f s | IAE %.6g | ISE %.6g\n', ...
        metRBF.peak_theta_deg, metRBF.settling_time_s, metRBF.IAE_theta, metRBF.ISE_theta);
    fprintf('RBF-LQI: max |u_RBF| %.6g | max |u_total| %.6g | saturation %.2f%%\n', ...
        metRBF.max_abs_rbf, metRBF.max_abs_u, metRBF.saturation_pct);

    % Save one angle and one input comparison figure per scenario.
    local_plot_pair(resLQI, resRBF, scenarios(k).name);
end

%% 4. Assemble and save report-ready tables and raw simulation data
T = vertcat(rows{:});
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'seminar1_validation_results.mat'), ...
    'allResults', 'T', 'scenarios', 'params', 'centersFile');
writetable(T, fullfile(resultsDir, 'seminar1_validation_metrics.csv'));

fprintf('\n=== Summary table ===\n');
disp(T);
fprintf('Saved MAT results and CSV metrics in:\n%s\n', resultsDir);
fprintf(['\nInterpretation note: settling time is the first time after which ', ...
    '|theta| remains within 1 deg for the rest of the 8 s run. ', ...
    'Peak angle is reported directly; peak excess is measured relative ', ...
    'to the 5 deg initial displacement.\n']);

%% Local functions
function m = local_seminar_metrics(r)
    t = r.t(:);
    theta = r.x_true(3,:).';
    x = r.x_true(1,:).';
    u = r.u(:);
    f = r.f_rbf(:);
    Ts = r.scenario.Ts;
    ref = r.scenario.r;

    m.peak_theta_deg = rad2deg(max(abs(theta)));
    initialAbs = abs(theta(1));
    m.peak_excess_deg = max(0, m.peak_theta_deg - rad2deg(initialAbs));

    within = abs(theta) <= deg2rad(1);
    settleIdx = [];
    for q = 1:numel(t)
        if all(within(q:end))
            settleIdx = q;
            break
        end
    end
    if isempty(settleIdx)
        m.settling_time_s = NaN;
    else
        m.settling_time_s = t(settleIdx);
    end

    eTheta = theta;
    eX = x - ref;
    m.IAE_theta = trapz(t, abs(eTheta));
    m.ISE_theta = trapz(t, eTheta.^2);
    m.IAE_x = trapz(t, abs(eX));
    m.ISE_x = trapz(t, eX.^2);

    nLast = min(numel(t), max(1, round(1/Ts) + 1));
    m.steady_state_theta_deg = rad2deg(mean(abs(theta(end-nLast+1:end))));
    m.steady_state_x_error_m = mean(abs(eX(end-nLast+1:end)));

    m.max_abs_u = max(abs(u));
    m.control_effort = trapz(t(1:end-1), u.^2);
    limit = r.paramsNominal.cart.tau_max;
    m.saturation_pct = 100 * mean(abs(u) >= limit - 1e-8);
    m.max_abs_rbf = max(abs(f));
    m.rms_rbf = sqrt(mean(f.^2));
    m.max_abs_x = max(abs(x));
end

function row = local_row(testName, controller, m)
    row = table(string(testName), string(controller), ...
        m.peak_theta_deg, m.peak_excess_deg, m.settling_time_s, ...
        m.IAE_theta, m.ISE_theta, m.IAE_x, m.ISE_x, ...
        m.steady_state_theta_deg, m.steady_state_x_error_m, ...
        m.max_abs_u, m.control_effort, m.saturation_pct, ...
        m.max_abs_rbf, m.rms_rbf, m.max_abs_x, ...
        'VariableNames', {'Test','Controller','PeakTheta_deg', ...
        'PeakExcess_deg','SettlingTime_s','IAE_theta','ISE_theta', ...
        'IAE_x','ISE_x','SteadyTheta_deg','SteadyXError_m', ...
        'MaxAbsInput','ControlEffort','Saturation_pct', ...
        'MaxAbsRBF','RMS_RBF','MaxAbsCartPosition_m'});
end

function local_plot_pair(a, b, testName)
    safeName = regexprep(testName, '[^a-zA-Z0-9_]', '_');

    figure('Name', [testName ' - Pendulum angle']);
    plot(a.t, rad2deg(a.x_true(3,:)), 'LineWidth', 1.3);
    hold on
    plot(b.t, rad2deg(b.x_true(3,:)), '--', 'LineWidth', 1.3);
    yline(1, ':'); yline(-1, ':');
    grid on
    xlabel('Time (s)'); ylabel('Pendulum angle (deg)');
    title([testName ' - Pendulum angle']);
    legend('LQI','RBF-LQI','+/-1 deg band','Location','best');
    saveas(gcf, ['seminar1_' safeName '_angle.png']);

    figure('Name', [testName ' - Control input']);
    stairs(a.t(1:end-1), a.u, 'LineWidth', 1.2);
    hold on
    stairs(b.t(1:end-1), b.u, '--', 'LineWidth', 1.2);
    grid on
    xlabel('Time (s)'); ylabel('Actuator command (N m)');
    title([testName ' - Applied control input']);
    legend('LQI','RBF-LQI','Location','best');
    saveas(gcf, ['seminar1_' safeName '_input.png']);
end
