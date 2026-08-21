%% generate_before_after_data.m
% Generates .mat files for the Python visualization script:
%   1. A clean nominal run (current, fixed controller)
%   2. The 2N/-20% case using the CURRENT (fixed) controller -- settles normally
%   3. The SAME 2N/-20% case reproducing the OLD, BUGGY saturation logic
%      (saturate u_lqi alone before adding f_rbf, blind anti-windup) --
%      for side-by-side comparison ONLY. This bug is NOT present in the
%      live lqi_controller.m / simulation_main.m -- it is reconstructed
%      here, isolated, purely to regenerate the historical failure mode
%      for a before/after demonstration.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% ---- Dataset 1: clean nominal run (current, fixed controller) ----------
scenario_nom = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'initAngleDeg', 10, 'observerMethod', 'kalman', 'Fd_func', @(t) 0, ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
    'useRBF', true);
results_nom = simulation_main(scenario_nom);
metrics_nom = performance_metrics(results_nom, params);

%% ---- Dataset 2: 2N/-20% case, CURRENT fixed controller (settles) --------
scenario_fixed = struct('Ts', params.embedded.Ts, 'Tfinal', 8, ...
    'initAngleDeg', 0, 'observerMethod', 'kalman', ...
    'measNoiseStd', [params.sensor.x_quant_std; params.sensor.theta_quant_std], ...
    'massPct', -0.20, 'frictionPct', -0.20, 'lengthPct', -0.20, ...
    'Fd_func', @(t) 2.0 * (t >= 2 && t < 2.5), 'useRBF', true);
results_after = simulation_main(scenario_fixed);
metrics_after = performance_metrics(results_after, params);

%% ---- Dataset 3: SAME case, reproducing the OLD BUGGY logic --------------
results_before = local_run_buggy_version(scenario_fixed, params);
metrics_before = performance_metrics(results_before, params);

fprintf('Nominal:        max|theta|=%.2f deg, PASS=%d\n', metrics_nom.max_theta_deg, metrics_nom.pass);
fprintf('After (fixed):  max|theta|=%.2f deg, PASS=%d\n', metrics_after.max_theta_deg, metrics_after.pass);
fprintf('Before (buggy): max|theta|=%.2f deg, PASS=%d\n', metrics_before.max_theta_deg, metrics_before.pass);

save(fullfile(resultsDir, 'viz_nominal.mat'), 'results_nom', 'metrics_nom');
save(fullfile(resultsDir, 'viz_before_after.mat'), ...
    'results_before', 'metrics_before', 'results_after', 'metrics_after');

fprintf('Saved viz_nominal.mat and viz_before_after.mat to %s\n', resultsDir);

% ==========================================================================
function results = local_run_buggy_version(scenario, paramsNominal)
%LOCAL_RUN_BUGGY_VERSION Reproduces the OLD saturate-before-RBF bug,
%   isolated to this script only, for before/after comparison purposes.
%   Saturates u_lqi alone, then adds f_rbf and saturates again -- the
%   exact double-saturation/blind-anti-windup defect found and fixed in
%   the live simulation_main.m / lqi_controller.m.

scenario = local_fill_defaults(scenario, paramsNominal);

paramsTrue = paramsNominal;
paramsTrue.cart.M = paramsNominal.cart.M * (1 + scenario.massPct);
paramsTrue.pend.m = paramsNominal.pend.m * (1 + scenario.massPct);
paramsTrue.cart.b = paramsNominal.cart.b * (1 + scenario.frictionPct);
paramsTrue.pend.l = paramsNominal.pend.l * (1 + scenario.lengthPct);
paramsTrue.pend.L = paramsNominal.pend.L * (1 + scenario.lengthPct);

[A, B, C, D] = linearized_model(paramsNominal);
[Ad, Bd, ~, ~] = discretize_model(A, B, C, D, scenario.Ts);
[K, Ki] = design_lqi(A, B, paramsNominal);
[L, C_meas] = observer_design(Ad, paramsNominal, scenario.observerMethod);
rbf = rbf_initialize(paramsNominal);

N = round(scenario.Tfinal / scenario.Ts);
t = (0:N) * scenario.Ts;
x_true = zeros(4, N+1); xhat = zeros(4, N+1);
u_log = zeros(1, N); xi_log = zeros(1, N+1);
x_true(:,1) = scenario.x0; xhat(:,1) = scenario.x0;

for kk = 1:N
    tk = t(kk);
    y = C_meas * x_true(:,kk) + scenario.measNoiseStd .* randn(2,1);

    % --- OLD BUGGY LOGIC: saturate u_lqi ALONE first ---
    err = scenario.r - xhat(1,kk);
    u_lqi_unsat = -K*xhat(:,kk) - Ki*xi_log(kk);
    actuator_limit = paramsNominal.cart.tau_max;
    u_lqi_sat = max(min(u_lqi_unsat, actuator_limit), -actuator_limit);
    sat_high = u_lqi_unsat > u_lqi_sat + 1e-9;
    sat_low  = u_lqi_unsat < u_lqi_sat - 1e-9;
    freeze = (sat_high && err > 0) || (sat_low && err < 0); % blind to f_rbf
    if freeze, xi_dot = 0; else, xi_dot = err; end

    rbfInput = [xhat(3,kk); xhat(4,kk); xhat(1,kk); xhat(2,kk)];
    [f_rbf, phi] = rbf_forward(rbf, rbfInput);

    u_unsat2 = u_lqi_sat + f_rbf;  % adding RBF to ALREADY-saturated LQI term
    u = max(min(u_unsat2, actuator_limit), -actuator_limit);

    Fd = scenario.Fd_func(tk);
    [~, xx] = ode45(@(tt,xx) nonlinear_plant(tt, xx, u, paramsTrue, Fd), ...
                     [tk, tk+scenario.Ts], x_true(:,kk));
    x_true(:,kk+1) = xx(end,:)';
    xhat(:,kk+1) = observer_update(xhat(:,kk), u, y, Ad, Bd, L, C_meas, paramsNominal);
    xi_log(kk+1) = xi_log(kk) + xi_dot * scenario.Ts;
    rbf = rbf_update(rbf, phi, scenario.r - xhat(1,kk+1));
    u_log(kk) = u;
end

results.t = t; results.x_true = x_true; results.xhat = xhat; results.u = u_log;
results.xi = xi_log; results.scenario = scenario;
end

function scenario = local_fill_defaults(scenario, paramsNominal)
defaults.Ts=0.01; defaults.Tfinal=8; defaults.x0=zeros(4,1); defaults.r=0;
defaults.massPct=0; defaults.frictionPct=0; defaults.lengthPct=0;
defaults.Fd_func=@(t) 0; defaults.measNoiseStd=[0;0];
flds = fieldnames(defaults);
for i = 1:numel(flds)
    f = flds{i};
    if ~isfield(scenario, f) || isempty(scenario.(f)), scenario.(f) = defaults.(f); end
end
if isfield(scenario,'initAngleDeg') && ~isempty(scenario.initAngleDeg)
    scenario.x0(3) = deg2rad(scenario.initAngleDeg);
end
if ~isfield(scenario,'config') || isempty(scenario.config)
    scenario.config = paramsNominal.config;
end
end