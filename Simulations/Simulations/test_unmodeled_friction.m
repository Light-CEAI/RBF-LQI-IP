%% test_unmodeled_friction.m
% Demonstrates the specific scenario where RBF augmentation IS
% justified: when the TRUE plant has Coulomb friction that the LQI
% design model does not account for. This is the most realistic
% simulation of model-plant mismatch for a rail-based cart system.
%
% Coulomb friction model:
%   F_coulomb = mu_c * (M+m) * g * sign(xdot),  if |xdot| > v_stiction
%             = mu_s * (M+m) * g * sign(tau),    if |xdot| <= v_stiction
%
% This force is added to the TRUE plant's dynamics but is UNKNOWN
% to the LQI controller -- it only knows about the nominal viscous b.
% The RBF's job is to approximate and cancel this unmodeled term online.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% ---- Coulomb friction parameters (lab-feasible range for a rail cart) --
mu_kinetic  = 0.08;  % kinetic friction coefficient
mu_static   = 0.12;  % static friction coefficient
v_stiction  = 0.02;  % velocity below which static friction applies [m/s]

frictionLevels = [0, 0.05, 0.08, 0.12]; % 0=pure viscous baseline, 0.12=significant

modes = {'lqi_only', 'lqi_rbf'};
modeLabels = {'LQI only', 'LQI+RBF'};
colors_lqi = [0.2 0.4 0.8; 0.4 0.6 1.0; 0.6 0.8 1.0; 0.8 0.9 1.0];
colors_rbf = [0.1 0.7 0.3; 0.3 0.8 0.4; 0.5 0.9 0.5; 0.7 1.0 0.7];

settling_lqi = nan(numel(frictionLevels),1);
settling_rbf = nan(numel(frictionLevels),1);
iae_lqi = nan(numel(frictionLevels),1);
iae_rbf = nan(numel(frictionLevels),1);

[A, B, C, D] = linearized_model(params);
[Ad, Bd, ~, ~] = discretize_model(A, B, C, D, params.embedded.Ts);
[K, Ki] = design_lqi(A, B, params);
[L, C_meas] = observer_design(Ad, params, 'kalman');
actuator_limit = params.cart.tau_max; sat_tol = 1e-9;
Ts = params.embedded.Ts; Tfinal = 10; N = round(Tfinal/Ts); t=(0:N)*Ts;

results_friction = struct();

for fi = 1:numel(frictionLevels)
    mu_k = frictionLevels(fi);
    mu_s = mu_k * 1.5; % static always slightly higher than kinetic

    % Coulomb friction as an additional Fd term applied to TRUE plant
    totalMass = (params.cart.M + params.pend.m) * 9.81;
    coulomb_Fd = @(xdot, u_applied) ...
        ternary(abs(xdot) > v_stiction, ...
            -mu_k * totalMass * sign(xdot), ...
            -min(abs(mu_s * totalMass), abs(u_applied)) * sign(u_applied));

    for mi = 1:2
        useRBF = (mi == 2);
        rbfMatFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'RBF', 'rbf_centers.mat');
        rbf = rbf_initialize(params, 'host', rbfMatFile);

        x_true = zeros(4,N+1); xhat = zeros(4,N+1);
        x_true(3,1) = deg2rad(10); xhat(3,1) = deg2rad(10);
        xi_log = zeros(1,N+1); u_log = zeros(1,N);

        for kk = 1:N
            tk = t(kk);
            y = C_meas*x_true(:,kk) + [params.sensor.x_quant_std;params.sensor.theta_quant_std].*randn(2,1);
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
            sat_high = u_unsat>u+sat_tol; sat_low = u_unsat<u-sat_tol;
            freeze = (sat_high&&err>0)||(sat_low&&err<0);
            xi_dot=0; if ~freeze, xi_dot=err; end

            % Coulomb friction added to TRUE plant only
            Fd_coulomb = coulomb_Fd(x_true(2,kk), u);
            [~,xx] = ode45(@(tt,xx) nonlinear_plant(tt,xx,u,params,Fd_coulomb), ...
                [tk,tk+Ts], x_true(:,kk));
            x_true(:,kk+1) = xx(end,:)';
            xhat(:,kk+1) = observer_update(xhat(:,kk), u, y, Ad, Bd, L, C_meas, params);
            xi_log(kk+1) = xi_log(kk)+xi_dot*Ts;

            if useRBF
                rbf = rbf_update(rbf, phi, -xhat(1,kk+1));
            end
            u_log(kk) = u;
        end

        % Metrics
        theta = x_true(3,:);
        thresh = deg2rad(1);
        above = abs(theta) > thresh;
        if any(above)
            last_above = find(above,1,'last');
            sett = ternary(last_above==N+1, NaN, t(last_above));
        else
            sett = 0;
        end
        iae = trapz(t, abs(theta));

        if mi==1
            settling_lqi(fi) = sett; iae_lqi(fi) = iae;
            results_friction.(['f' num2str(fi) '_lqi']).x_true = x_true;
            results_friction.(['f' num2str(fi) '_lqi']).u = u_log;
        else
            settling_rbf(fi) = sett; iae_rbf(fi) = iae;
            results_friction.(['f' num2str(fi) '_rbf']).x_true = x_true;
            results_friction.(['f' num2str(fi) '_rbf']).u = u_log;
        end

        fprintf('mu_k=%.2f, %s: settling=%.2fs, IAE=%.4f\n', mu_k, modeLabels{mi}, sett, iae);
    end
end

%% ---- Comparison plots ---------------------------------------------------
figure('Name','Unmodeled Coulomb Friction: LQI vs LQI+RBF','Position',[80 80 1100 700]);

% Row 1: theta trajectories, LQI only
subplot(3,2,1); hold on;
for fi = 1:numel(frictionLevels)
    r = results_friction.(['f' num2str(fi) '_lqi']);
    plot(t, rad2deg(r.x_true(3,:)), 'Color', colors_lqi(fi,:), 'LineWidth',1.2);
end
legend(arrayfun(@(mu) sprintf('mu_k=%.2f', mu), frictionLevels,'UniformOutput',false),'Location','best','FontSize',8);
ylabel('theta [deg]'); grid on; title('LQI only: varying Coulomb friction');

% Row 1: theta trajectories, LQI+RBF
subplot(3,2,2); hold on;
for fi = 1:numel(frictionLevels)
    r = results_friction.(['f' num2str(fi) '_rbf']);
    plot(t, rad2deg(r.x_true(3,:)), 'Color', colors_rbf(fi,:), 'LineWidth',1.2);
end
legend(arrayfun(@(mu) sprintf('mu_k=%.2f', mu), frictionLevels,'UniformOutput',false),'Location','best','FontSize',8);
ylabel('theta [deg]'); grid on; title('LQI+RBF: varying Coulomb friction');

% Row 2: settling time comparison
subplot(3,2,3);
bar(categorical(arrayfun(@(mu) sprintf('%.2f',mu), frictionLevels,'UniformOutput',false)), ...
    [settling_lqi, settling_rbf]);
legend({'LQI only','LQI+RBF'},'Location','best');
ylabel('Settling time [s]'); grid on;
title('Settling time vs Coulomb friction level');

% Row 2: IAE comparison
subplot(3,2,4);
bar(categorical(arrayfun(@(mu) sprintf('%.2f',mu), frictionLevels,'UniformOutput',false)), ...
    [iae_lqi, iae_rbf]);
legend({'LQI only','LQI+RBF'},'Location','best');
ylabel('IAE [rad\cdots]'); grid on;
title('IAE vs Coulomb friction level');

% Row 3: improvement percentages
subplot(3,2,5);
settling_improve = 100*(settling_lqi - settling_rbf) ./ settling_lqi;
bar(categorical(arrayfun(@(mu) sprintf('%.2f',mu), frictionLevels,'UniformOutput',false)), settling_improve);
yline(15,'r--','15% target'); ylabel('Improvement [%]'); grid on;
title('Settling time improvement (RBF over LQI)');

subplot(3,2,6);
iae_improve = 100*(iae_lqi - iae_rbf) ./ iae_lqi;
bar(categorical(arrayfun(@(mu) sprintf('%.2f',mu), frictionLevels,'UniformOutput',false)), iae_improve);
yline(20,'r--','20% target'); ylabel('Improvement [%]'); grid on;
title('IAE improvement (RBF over LQI)');

sgtitle('Unmodeled Coulomb Friction: LQI vs LQI+RBF (friction unknown to controller)');

save(fullfile(resultsDir,'results_unmodeled_friction.mat'),'results_friction','frictionLevels','settling_lqi','settling_rbf','iae_lqi','iae_rbf');

function out = ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end