%% test_parametric_drift.m
% Tests adaptation lag under continuously time-varying mass, comparing
% LQI-only (baseline) against LQI+RBF, so any difference in tracking
% behavior can be attributed to RBF specifically rather than assumed.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

Ts = params.embedded.Ts;
Tfinal = 15;
N = round(Tfinal/Ts);
t = (0:N)*Ts;

bias = 0.10; amplitude = 0.20; t_start = 5; omega = 2*pi*0.1;
M_nom = params.cart.M;
M_of_t = @(tt) M_nom * (1 + (tt>=t_start).*(bias + amplitude*sin(omega*(tt-t_start))));

[A, B, C, D] = linearized_model(params);
[Ad, Bd, ~, ~] = discretize_model(A, B, C, D, Ts);
[K, Ki] = design_lqi(A, B, params);
[L, C_meas] = observer_design(Ad, params, 'kalman');

actuator_limit = params.cart.tau_max; sat_tol = 1e-9;

%% ---- Run both configurations through the same drift profile -------------
useRBF_list = [true, false];
runs = struct();

for r = 1:2
    useRBF = useRBF_list(r);
    label = ternary(useRBF, 'on', 'off');

    rbf = rbf_initialize(params);
    x_true = zeros(4, N+1); xhat = zeros(4, N+1);
    u_log = zeros(1,N); xi_log = zeros(1,N+1);
    M_true_log = zeros(1,N+1);
    rbf_weights = zeros(rbf.n_hidden, N+1); rbf_weights(:,1) = rbf.weights;

    for kk = 1:N
        tk = t(kk);
        paramsTrue = params;
        paramsTrue.cart.M = M_of_t(tk);
        M_true_log(kk) = paramsTrue.cart.M;

        y = C_meas*x_true(:,kk) + [params.sensor.x_quant_std; params.sensor.theta_quant_std].*randn(2,1);

        err = -xhat(1,kk);
        u_lqi = -K*xhat(:,kk) - Ki*xi_log(kk);

        if useRBF
            rbfInput = [xhat(3,kk); xhat(4,kk); xhat(1,kk); xhat(2,kk)];
            [f_rbf, phi] = rbf_forward(rbf, rbfInput);
        else
            f_rbf = 0;
        end

        u_unsat = u_lqi + f_rbf;
        u = max(min(u_unsat, actuator_limit), -actuator_limit);
        sat_high = u_unsat > u + sat_tol; sat_low = u_unsat < u - sat_tol;
        freeze = (sat_high && err>0) || (sat_low && err<0);
        xi_dot = 0; if ~freeze, xi_dot = err; end

        [~, xx] = ode45(@(tt,xx) nonlinear_plant(tt, xx, u, paramsTrue, 0), [tk, tk+Ts], x_true(:,kk));
        x_true(:,kk+1) = xx(end,:)';
        xhat(:,kk+1) = observer_update(xhat(:,kk), u, y, Ad, Bd, L, C_meas, params);
        xi_log(kk+1) = xi_log(kk) + xi_dot*Ts;

        if useRBF
            rbf = rbf_update(rbf, phi, -xhat(1,kk+1));
            rbf_weights(:,kk+1) = rbf.weights;
        end
        u_log(kk) = u;
    end
    M_true_log(end) = M_of_t(t(end));

    runs.(['r' label]).x_true = x_true;
    runs.(['r' label]).u = u_log;
    runs.(['r' label]).rbf_weights = rbf_weights;

    fprintf('Drift test, RBF %s: max|theta|=%.4f deg, max|x|=%.4f m\n', ...
        upper(label), rad2deg(max(abs(x_true(3,:)))), max(abs(x_true(1,:))));
end

%% ---- Comparison plot -----------------------------------------------------
figure('Name','Parametric Drift: LQI-only vs LQI+RBF');
subplot(3,1,1);
plot(t, M_true_log, 'r-', 'LineWidth',1.3); yline(M_nom,'k--');
ylabel('True cart mass M(t) [kg]'); grid on; title('Sinusoidal Mass Drift (within +-30%% design boundary)');

subplot(3,1,2);
plot(t, rad2deg(runs.ron.x_true(3,:)), 'g-', 'LineWidth',1.2); hold on;
plot(t, rad2deg(runs.roff.x_true(3,:)), 'b--', 'LineWidth',1.2);
legend('LQI+RBF', 'LQI only', 'Location','best');
ylabel('theta [deg]'); grid on;

subplot(3,1,3);
plot(t, vecnorm(runs.ron.rbf_weights,2,1), 'g-');
ylabel('||RBF weights||_2 (RBF run only)'); xlabel('time [s]'); grid on;

results_drift.t = t; results_drift.M_true = M_true_log; results_drift.runs = runs;
save(fullfile(resultsDir,'results_parametric_drift.mat'), 'results_drift');

function out = ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end