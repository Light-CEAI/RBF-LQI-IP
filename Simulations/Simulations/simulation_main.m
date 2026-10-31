function results = simulation_main(scenario)
%SIMULATION_MAIN Core simulation engine: nonlinear plant + observer + LQI + RBF.
%
%   results = simulation_main(scenario)
%
%   HOST-SIDE, DOUBLE PRECISION ONLY. Saturation and anti-windup are
%   applied ONCE, to the COMBINED command (u_lqi + f_rbf), matching
%   control_step.m's structure. This fixes a prior bug where
%   lqi_controller.m saturated the LQI term internally before f_rbf was
%   added, making the anti-windup freeze decision blind to RBF's
%   contribution -- which caused an undetected integrator windup (xi
%   diverging unboundedly) under combined disturbance + parametric
%   uncertainty testing.
%
%   scenario is a struct; every field is optional and defaults are filled
%   in below.
%
%   Recognized scenario fields (all optional):
%     .config              'A' or 'B'                  default: parameters() default
%     .Ts                  sample time [s]              default: 0.01
%     .Tfinal               sim duration [s]              default: 8
%     .x0                  initial TRUE state (4x1)      default: zeros(4,1)
%     .initAngleDeg         convenience: sets x0(3)       default: [] (unused if x0 given)
%     .r                   position reference            default: 0
%     .useRBF              logical                       default: true
%     .observerMethod      'kalman' | 'luenberger'        default: 'kalman'
%     .massPct             fractional mass perturbation   default: 0
%     .frictionPct         fractional friction perturbation default: 0
%     .lengthPct           fractional length perturbation default: 0
%     .Fd_func             @(t) -> disturbance force [N]  default: @(t) 0
%     .measNoiseStd        (2x1) std dev of [x_meas; theta_meas] noise default: [0;0]
%
%   results fields: t, x_true, xhat, u, u_lqi, f_rbf, xi, rbf_weights,
%                   paramsNominal, paramsTrue, scenario (echoed back)

%% ---- 1. Fill defaults --------------------------------------------------
paramsNominal = parameters();
if isfield(scenario,'config') && ~isempty(scenario.config)
    paramsNominal = parameters(scenario.config);
end

scenario = local_fill_defaults(scenario, paramsNominal);

%% ---- 2. Build TRUE (perturbed) plant parameters ------------------------
paramsTrue = paramsNominal;
paramsTrue.cart.M  = paramsNominal.cart.M  * (1 + scenario.massPct);
paramsTrue.pend.m  = paramsNominal.pend.m  * (1 + scenario.massPct);
paramsTrue.cart.b  = paramsNominal.cart.b  * (1 + scenario.frictionPct);
paramsTrue.pend.l  = paramsNominal.pend.l  * (1 + scenario.lengthPct);
paramsTrue.pend.L  = paramsNominal.pend.L  * (1 + scenario.lengthPct);

%% ---- 3. Design controller / observer / RBF on the NOMINAL model --------
[A, B, C, D]      = linearized_model(paramsNominal);
[Ad, Bd, Cd, Dd]  = discretize_model(A, B, C, D, scenario.Ts); %#ok<ASGLU>
[K, Ki]           = design_lqi(A, B, paramsNominal);
[L, C_meas]       = observer_design(Ad, paramsNominal, scenario.observerMethod);

if scenario.useRBF
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    rbfMatFile = fullfile(projectRoot, 'RBF', 'RBF', 'rbf_centers.mat');
    rbf = rbf_initialize(paramsNominal, 'host', rbfMatFile);
end

actuator_limit = local_actuator_limit(paramsNominal);
sat_tol = 1e-9; % tight tolerance for the double-precision host path

%% ---- 4. Initialize states ----------------------------------------------
N = round(scenario.Tfinal / scenario.Ts);
t = (0:N) * scenario.Ts;

x_true = zeros(4, N+1);
xhat   = zeros(4, N+1);
u_log  = zeros(1, N);
u_lqi_log = zeros(1, N);
f_rbf_log = zeros(1, N);
xi_log = zeros(1, N+1);

x_true(:,1) = scenario.x0;
xhat(:,1)   = scenario.x0;
xi_log(1)   = 0;
u_prev = 0; % observer prediction uses the input applied over the previous interval

if scenario.useRBF
    rbf_weights = zeros(rbf.n_hidden, N+1);
    rbf_weights(:,1) = rbf.weights;
end

%% ---- 5. Main loop: discrete control/observer, continuous plant --------
for kk = 1:N
    tk = t(kk);

    % --- Measurement from the TRUE plant ---
    y = C_meas * x_true(:,kk) + scenario.measNoiseStd .* randn(2,1);

    % --- Observer correction: current measurement + previous applied input ---
    [xhat(:,kk), innovation] = observer_update( ...
        xhat(:,kk), u_prev, y, Ad, Bd, L, C_meas, paramsNominal);

    % --- Raw (unsaturated) LQI command ---
    err = scenario.r - xhat(1,kk);
    u_lqi = lqi_controller(xhat(:,kk), xi_log(kk), scenario.r, K, Ki);

    % --- RBF compensation (additive, on top of LQI) ---
    if scenario.useRBF
        rbfInput = [xhat(3,kk); xhat(4,kk); xhat(1,kk); xhat(2,kk)]; % theta,thetadot,x,xdot
        [f_rbf, phi] = rbf_forward(rbf, rbfInput);
    else
        f_rbf = 0;
    end

    % --- Saturate ONCE, on the COMBINED command ---
    u_unsat = u_lqi + f_rbf;
    u = max(min(u_unsat, actuator_limit), -actuator_limit);

    % --- Anti-windup based on the COMBINED saturation, not LQI alone ----
    saturated_high = u_unsat > u + sat_tol;
    saturated_low  = u_unsat < u - sat_tol;
    freeze = (saturated_high && err > 0) || (saturated_low && err < 0);
    if freeze
        xi_dot = 0;
    else
        xi_dot = err;
    end

    % --- Integrate TRUE nonlinear plant over [tk, tk+Ts] with ZOH input ---
    Fd = scenario.Fd_func(tk);
    [~, xx] = ode45(@(tt,xx) nonlinear_plant(tt, xx, u, paramsTrue, Fd), ...
                     [tk, tk+scenario.Ts], x_true(:,kk));
    x_true(:,kk+1) = xx(end,:)';

    % --- Integral state update (simple forward Euler) ---
    xi_log(kk+1) = xi_log(kk) + xi_dot * scenario.Ts;

    % --- RBF online update: same cart-position error in all code paths ---
    if scenario.useRBF
        rbf_error = scenario.r - xhat(1,kk);
        rbf = rbf_update(rbf, phi, rbf_error);
        rbf_weights(:,kk+1) = rbf.weights;
    end

    u_prev = u;
    u_log(kk)     = u;
    u_lqi_log(kk) = u_lqi;
    f_rbf_log(kk) = f_rbf;
end

%% ---- 6. Package results -------------------------------------------------
results.t = t;
results.x_true = x_true;
results.xhat = xhat;
results.u = u_log;
results.u_lqi = u_lqi_log;
results.f_rbf = f_rbf_log;
results.xi = xi_log;
if scenario.useRBF
    results.rbf_weights = rbf_weights;
end
results.K = K;
results.Ki = Ki;
results.L = L;
results.paramsNominal = paramsNominal;
results.paramsTrue = paramsTrue;
results.scenario = scenario;

end

% ========================================================================
function limit = local_actuator_limit(params)
switch params.config
    case 'A'
        limit = params.cart.tau_max;
    case 'B'
        limit = params.cart.u_max;
end
end

% ========================================================================
function scenario = local_fill_defaults(scenario, paramsNominal)
defaults.Ts             = 0.01;
defaults.Tfinal         = 8;
defaults.x0             = zeros(4,1);
defaults.r              = 0;
defaults.useRBF         = true;
defaults.observerMethod = 'kalman';
defaults.massPct        = 0;
defaults.frictionPct    = 0;
defaults.lengthPct      = 0;
defaults.Fd_func        = @(t) 0;
defaults.measNoiseStd   = [0;0];

flds = fieldnames(defaults);
for i = 1:numel(flds)
    f = flds{i};
    if ~isfield(scenario, f) || isempty(scenario.(f))
        scenario.(f) = defaults.(f);
    end
end

if isfield(scenario,'initAngleDeg') && ~isempty(scenario.initAngleDeg) ...
        && isequal(scenario.x0, defaults.x0)
    scenario.x0(3) = deg2rad(scenario.initAngleDeg);
end

if ~isfield(scenario,'config') || isempty(scenario.config)
    scenario.config = paramsNominal.config;
end
end