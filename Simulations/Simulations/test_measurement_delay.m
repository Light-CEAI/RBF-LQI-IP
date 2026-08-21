%% test_measurement_delay.m
% Tests robustness to sensor delay/dropout, comparing LQI-only vs
% LQI+RBF at each delay level, to see whether RBF helps, hurts, or is
% irrelevant to the delay margin itself.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

delayLevels_steps = [0, 1, 3, 5];
dropoutProb = 0.05;

[A, B, C, D] = linearized_model(params);
[Ad, Bd, ~, ~] = discretize_model(A, B, C, D, params.embedded.Ts);
[K, Ki] = design_lqi(A, B, params);
[L, C_meas] = observer_design(Ad, params, 'kalman');

results_delay = cell(numel(delayLevels_steps),2); % {delayLevel, useRBF}
useRBF_list = [true, false];

for d = 1:numel(delayLevels_steps)
    nd = delayLevels_steps(d);
    for r = 1:2
        useRBF = useRBF_list(r);
        rbf = rbf_initialize(params);
        Ts = params.embedded.Ts; Tfinal = 8; N = round(Tfinal/Ts); t=(0:N)*Ts;
        x_true = zeros(4,N+1); xhat = zeros(4,N+1); xi_log = zeros(1,N+1);
        x_true(3,1) = deg2rad(10); xhat(3,1) = deg2rad(10);
        y_buffer = repmat(C_meas*x_true(:,1), 1, nd+1);
        actuator_limit = params.cart.tau_max; sat_tol=1e-9;

        for kk = 1:N
            tk = t(kk);
            y_fresh = C_meas*x_true(:,kk) + [params.sensor.x_quant_std; params.sensor.theta_quant_std].*randn(2,1);
            y_buffer = [y_buffer(:,2:end), y_fresh];
            y_used = y_buffer(:,1);

            if nd == max(delayLevels_steps) && rand() < dropoutProb
                y_used = y_buffer(:,2);
            end

            err = -xhat(1,kk);
            u_lqi = -K*xhat(:,kk) - Ki*xi_log(kk);

            if useRBF
                rbfInput = [xhat(3,kk); xhat(4,kk); xhat(1,kk); xhat(2,kk)];
                [f_rbf, phi] = rbf_forward(rbf, rbfInput);
            else
                f_rbf = 0;
            end

            u_unsat = u_lqi+f_rbf;
            u = max(min(u_unsat, actuator_limit), -actuator_limit);
            sat_high = u_unsat>u+sat_tol; sat_low = u_unsat<u-sat_tol;
            freeze = (sat_high && err>0)||(sat_low && err<0);
            xi_dot=0; if ~freeze, xi_dot=err; end

            [~,xx] = ode45(@(tt,xx) nonlinear_plant(tt,xx,u,params,0), [tk,tk+Ts], x_true(:,kk));
            x_true(:,kk+1) = xx(end,:)';
            xhat(:,kk+1) = observer_update(xhat(:,kk), u, y_used, Ad, Bd, L, C_meas, params);
            xi_log(kk+1) = xi_log(kk)+xi_dot*Ts;

            if useRBF
                rbf = rbf_update(rbf, phi, -xhat(1,kk+1));
            end
        end

        results_delay{d,r}.t=t; results_delay{d,r}.x_true=x_true;
        results_delay{d,r}.delay_steps=nd; results_delay{d,r}.useRBF=useRBF;
        fprintf('Delay=%dsteps(%.0fms), RBF=%d: max|theta|=%.2fdeg, max|x|=%.3fm\n', ...
            nd, nd*Ts*1000, useRBF, rad2deg(max(abs(x_true(3,:)))), max(abs(x_true(1,:))));
    end
end

%% ---- Comparison plots: one figure per delay level ------------------------
for d = 1:numel(delayLevels_steps)
    figure('Name', sprintf('Delay=%dms: LQI-only vs LQI+RBF', delayLevels_steps(d)*params.embedded.Ts*1000));
    plot(results_delay{d,1}.t, rad2deg(results_delay{d,1}.x_true(3,:)), 'g-', 'LineWidth',1.3); hold on;
    plot(results_delay{d,2}.t, rad2deg(results_delay{d,2}.x_true(3,:)), 'b--', 'LineWidth',1.3);
    legend('LQI+RBF', 'LQI only', 'Location','best');
    xlabel('time [s]'); ylabel('theta [deg]'); grid on;
    title(sprintf('%dms delay', delayLevels_steps(d)*params.embedded.Ts*1000));
end

save(fullfile(resultsDir,'results_measurement_delay.mat'), 'results_delay', 'delayLevels_steps', 'dropoutProb');