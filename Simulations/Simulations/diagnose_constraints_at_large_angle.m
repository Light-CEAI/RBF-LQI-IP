%% diagnose_constraints_at_large_angle.m
% Identifies WHICH physical constraints are active at 25-30deg by
% plotting: actuator saturation duration, available authority margin,
% cart travel, and RBF weight evolution -- so the constraint picture
% is evidenced, not inferred.

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

params_diag = params;
params_diag.mech.track_length = 2.0;

testAngles = [20, 25, 30];
colors = [0 0.6 0.3; 0.9 0.5 0; 0.8 0.1 0.1];

figure('Name','Constraint Analysis at Large Angles','Position',[80 80 1100 750]);

for ai = 1:numel(testAngles)
    ang = testAngles(ai);

    scenario = struct('Ts', params_diag.embedded.Ts, 'Tfinal', 8, ...
        'x0', [0;0;deg2rad(ang);0], 'r', 0, 'Fd_func', @(t) 0, ...
        'measNoiseStd', [params_diag.sensor.x_quant_std; params_diag.sensor.theta_quant_std]);

    r = run_controller_variant('lqi_rbf', params_diag, scenario);
    validIdx = ~isnan(r.x_true(3,:));
    t_v = r.t(validIdx);
    theta_v = rad2deg(r.x_true(3,validIdx));
    x_v = r.x_true(1,validIdx);

    N_valid = sum(validIdx)-1;
    u_v = r.u(1:min(N_valid,numel(r.u)));
    t_u = r.t(1:numel(u_v));

    % Saturation flag
    limit = params_diag.cart.tau_max;
    is_sat = abs(u_v) >= limit - 1e-6;

    % Remaining authority margin (how close to limit)
    authority_remaining = limit - abs(u_v);

    % Gravity torque that must be overcome (requires full state)
    theta_for_grav = r.x_true(3, 1:numel(u_v));
    tau_gravity = params_diag.pend.m * 9.81 * params_diag.pend.l * abs(sin(theta_for_grav));

    % RBF weight norm
    w_norm = vecnorm(r.rbf_weights(:,1:numel(u_v)+1), 2, 1);

    % ---- Panel 1: theta
    subplot(5, 3, ai);
    plot(t_v, theta_v, 'Color', colors(ai,:), 'LineWidth',1.3);
    ylabel('theta [deg]'); grid on;
    if r.diverged
        title(sprintf('%d%s - DIVERGED @%.2fs', ang, char(176), r.divergence_time_s));
    else
        title(sprintf('%d%s - max|theta|=%.1f%s', ang, char(176), max(abs(theta_v)), char(176)));
    end

    % ---- Panel 2: actuator command + saturation
    subplot(5, 3, ai+3);
    plot(t_u, u_v, 'Color', colors(ai,:), 'LineWidth',1.0); hold on;
    yline(limit, 'k--', 'LineWidth',0.8);
    yline(-limit, 'k--', 'LineWidth',0.8);
    area(t_u, is_sat * limit, 'FaceColor', [1 0.7 0.7], 'EdgeColor','none', 'FaceAlpha', 0.4);
    area(t_u, -is_sat * limit, 'FaceColor', [1 0.7 0.7], 'EdgeColor','none', 'FaceAlpha', 0.4);
    ylabel('u [N\cdotm]'); grid on;
    pct_sat = 100*mean(is_sat);
    title(sprintf('Actuator (%.1f%% saturated)', pct_sat));

    % ---- Panel 3: authority margin vs gravity load
    subplot(5, 3, ai+6);
    plot(t_u, authority_remaining, 'Color', colors(ai,:), 'LineWidth',1.0); hold on;
    plot(t_u, tau_gravity, 'k--', 'LineWidth',0.8);
    legend({'authority remaining','gravity torque'}, 'Location','best','FontSize',8);
    ylabel('[N\cdotm]'); grid on;
    title('Authority margin vs gravity load');

    % ---- Panel 4: cart position vs safe travel
    subplot(5, 3, ai+9);
    plot(t_v, x_v, 'Color', colors(ai,:), 'LineWidth',1.0); hold on;
    yline(0.35, 'r--', 'LineWidth',0.8);
    yline(-0.35, 'r--', 'LineWidth',0.8);
    ylabel('x [m]'); grid on;
    title('Cart travel (-- = original safe limit)');

    % ---- Panel 5: RBF adaptation speed
    subplot(5, 3, ai+12);
    plot(r.t(1:numel(w_norm)), w_norm, 'Color', colors(ai,:), 'LineWidth',1.0);
    ylabel('||w||_2'); xlabel('time [s]'); grid on;
    title('RBF weight adaptation');

    fprintf('%ddeg: saturated=%.1f%%, max_grav_tau=%.3fNm, tau_max=%.3fNm, margin=%.3fNm\n', ...
        ang, pct_sat, max(tau_gravity), limit, limit-max(tau_gravity));
end

sgtitle('Constraint Analysis: LQI+RBF at 20/25/30deg');
save(fullfile(resultsDir,'results_constraint_analysis.mat'), 'testAngles');