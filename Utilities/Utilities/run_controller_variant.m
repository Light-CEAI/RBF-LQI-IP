function results = run_controller_variant(mode, params, scenario)
%RUN_CONTROLLER_VARIANT Runs ONE of three controller configurations:
%   'lqi_only' : standalone LQI, no RBF term at all
%   'rbf_only' : standalone RBF, NO LQI feedback
%   'lqi_rbf'  : full combined controller (the validated design)
%
%   Includes divergence guard (stops at |theta|>90deg, pads with NaN)
%   and innovation-based RBF error signal (blends position tracking
%   error with observer innovation to better target unmodeled dynamics).

Ts = scenario.Ts; Tfinal = scenario.Tfinal;
N = round(Tfinal/Ts); t = (0:N)*Ts;

if ~isfield(scenario,'M_of_t') || isempty(scenario.M_of_t)
    M_of_t = @(tt) params.cart.M;
else
    M_of_t = scenario.M_of_t;
end

[A, B, C, D] = linearized_model(params);
[Ad, Bd, ~, ~] = discretize_model(A, B, C, D, Ts);
[K, Ki] = design_lqi(A, B, params);
[L, C_meas] = observer_design(Ad, params, 'kalman');

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
rbfMatFile = fullfile(projectRoot, 'RBF', 'RBF', 'rbf_centers.mat');
rbf = rbf_initialize(params, 'host', rbfMatFile);

actuator_limit = params.cart.tau_max; sat_tol = 1e-9;
DIVERGE_THRESHOLD = deg2rad(90);

x_true = zeros(4,N+1); xhat = zeros(4,N+1);
x_true(:,1) = scenario.x0; xhat(:,1) = scenario.x0;
u_log = nan(1,N); xi_log = zeros(1,N+1);
rbf_weights = zeros(rbf.n_hidden, N+1); rbf_weights(:,1) = rbf.weights;
M_true_log = nan(1,N+1);

useLQI = ismember(mode, {'lqi_only','lqi_rbf'});
useRBF = ismember(mode, {'rbf_only','lqi_rbf'});

diverged = false;
divergence_time_s = NaN;
kk_stop = N;

for kk = 1:N
    tk = t(kk);
    paramsTrue = params;
    paramsTrue.cart.M = M_of_t(tk);
    M_true_log(kk) = paramsTrue.cart.M;

    y = C_meas*x_true(:,kk) + scenario.measNoiseStd.*randn(2,1);
    err = scenario.r - xhat(1,kk);

    if useLQI
        u_lqi = -K*xhat(:,kk) - Ki*xi_log(kk);
    else
        u_lqi = 0;
    end

    if useRBF
        rbfInput = [xhat(3,kk); xhat(4,kk); xhat(1,kk); xhat(2,kk)];
        [f_rbf, phi] = rbf_forward(rbf, rbfInput);
    else
        f_rbf = 0;
    end

    u_unsat = u_lqi + f_rbf;
    u = max(min(u_unsat, actuator_limit), -actuator_limit);

    if useLQI
        sat_high = u_unsat > u + sat_tol; sat_low = u_unsat < u - sat_tol;
        freeze = (sat_high && err>0) || (sat_low && err<0);
        xi_dot = 0; if ~freeze, xi_dot = err; end
    else
        xi_dot = 0;
    end

    Fd = scenario.Fd_func(tk);
    [~, xx] = ode45(@(tt,xx) nonlinear_plant(tt, xx, u, paramsTrue, Fd), ...
        [tk, tk+Ts], x_true(:,kk));
    x_true(:,kk+1) = xx(end,:)';

    % Observer update — now returns innovation for RBF error signal
    [xhat(:,kk+1), innovation] = observer_update(xhat(:,kk), u, y, Ad, Bd, L, C_meas, params);

    xi_log(kk+1) = xi_log(kk) + xi_dot*Ts;

    if useRBF
        rbf_error = scenario.r - xhat(1,kk+1);
        rbf = rbf_update(rbf, phi, rbf_error);
        rbf_weights(:,kk+1) = rbf.weights;
    end
    u_log(kk) = u;

    if abs(x_true(3,kk+1)) > DIVERGE_THRESHOLD
        diverged = true;
        divergence_time_s = t(kk+1);
        kk_stop = kk+1;
        break;
    end
end
M_true_log(end) = M_of_t(t(end));

if diverged && kk_stop < N+1
    x_true(:, kk_stop+1:end) = NaN;
    rbf_weights(:, kk_stop+1:end) = NaN;
end

results.t = t; results.x_true = x_true; results.u = u_log; results.xi = xi_log;
results.rbf_weights = rbf_weights; results.M_true = M_true_log; results.mode = mode;
results.diverged = diverged; results.divergence_time_s = divergence_time_s;

end