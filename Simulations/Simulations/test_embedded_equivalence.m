%% test_embedded_equivalence.m
% Verifies that the single-precision, codegen-targeted control_step.m
% path produces (near-)identical behavior to the double-precision
% simulation_main.m path on the same scenario. This is the test that
% catches precision-related surprises (e.g. integrator drift, RBF
% weight divergence, saturation edge-case mismatches) BEFORE they show
% up as a flaky physical demo on the ESP32/STM32.
%
% Uses a TWO-PART check (transient vs settled) for BOTH theta and x,
% with SEPARATE transient windows per channel: theta settles fast
% (within ~1s near the unstable equilibrium), while x's transient tail
% decays more slowly (observed settling by ~3s). A single shared window
% would either be too tight for x (false FAIL on a decaying transient)
% or too loose for theta (masking a real settled-window problem).

params = parameters();
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

if ~exist('control_consts.m', 'file')
    error('test_embedded_equivalence:missingConsts', ...
        'control_consts.m not found. Run generate_control_consts.m first.');
end
if ~exist('rbf_centers_const.m', 'file')
    error('test_embedded_equivalence:missingRBF', ...
        'rbf_centers_const.m not found. Run rbf_export_for_embedded.m first.');
end

c = control_consts();

%% ---- Shared scenario for both paths -------------------------------------
scenario = struct();
scenario.Ts             = params.embedded.Ts;
scenario.Tfinal         = 8;
scenario.initAngleDeg   = 10;
scenario.r              = 0;
scenario.useRBF         = true;
scenario.observerMethod = 'kalman';
scenario.Fd_func        = @(t) 0;
scenario.measNoiseStd   = [0; 0]; % noise off -- want a clean precision
                                    % comparison, not noise-driven
                                    % divergence masking a real precision bug

%% ---- Path 1: double-precision host simulation (simulation_main.m) ------
results_double = simulation_main(scenario);

%% ---- Path 2: single-precision embedded-equivalent loop -----------------
% Re-implements simulation_main.m's loop structure but calling
% control_step.m's actual single-precision signature, so this is testing
% the REAL deployed function, not a re-derivation of it.

N = round(scenario.Tfinal / scenario.Ts);
t = single((0:N) * scenario.Ts);

x_true_s = zeros(4, N+1, 'single');
xhat_s   = zeros(4, N+1, 'single');
u_log_s  = zeros(1, N, 'single');

x0 = single([0; 0; deg2rad(scenario.initAngleDeg); 0]);
x_true_s(:,1) = x0;
xhat_s(:,1)   = x0;
xi_s = zeros(1, N+1, 'single');

rbf_s = rbf_initialize(params, 'embedded');

C_meas = single([1 0 0 0; 0 0 1 0]);
u_prev = single(0);

for kk = 1:N
    tk = t(kk);

    y = C_meas * x_true_s(:,kk); % no noise, see note above
    y = single(y);

    [u, xhat_s(:,kk+1), xi_s(kk+1), rbf_s, ~] = control_step( ...
        y, u_prev, xhat_s(:,kk), xi_s(kk), rbf_s, single(scenario.r), c);

    % Plant integration stays double internally (ode45 requires it);
    % cast in/out at the boundary, mirroring how a real MCU's ADC/DAC
    % boundary would interact with the (here, simulated-in-double) physics.
    Fd = 0;
    [~, xx] = ode45(@(tt,xx) nonlinear_plant(tt, xx, double(u), params, Fd), ...
                     [double(tk), double(tk)+scenario.Ts], double(x_true_s(:,kk)));
    x_true_s(:,kk+1) = single(xx(end,:)');

    u_log_s(kk) = u;
    u_prev = u;
end

%% ---- Compare the two paths ----------------------------------------------
theta_double = results_double.x_true(3,:);
theta_single = double(x_true_s(3,:));

theta_err_deg = rad2deg(abs(theta_double - theta_single));
x_err_m       = abs(results_double.x_true(1,:) - double(x_true_s(1,:)));

[peakThetaErr, peakThetaIdx] = max(theta_err_deg);
[peakXErr, peakXIdx] = max(x_err_m);

fprintf('--- Embedded Equivalence Test ---\n');
fprintf('Peak theta divergence: %.4f deg at t=%.3fs (step %d of %d)\n', ...
    peakThetaErr, double(t(peakThetaIdx)), peakThetaIdx, numel(t));
fprintf('Peak x divergence:     %.5f m at t=%.3fs (step %d of %d)\n', ...
    peakXErr, double(t(peakXIdx)), peakXIdx, numel(t));
fprintf('Final-step theta divergence: %.4f deg\n', theta_err_deg(end));
fprintf('Final-step x divergence:     %.5f m\n', x_err_m(end));

% ---- Two-part check: transient (loose) vs settled (tight), per channel -
% Separate windows: theta settles fast; x's transient tail decays slower
% (confirmed via plotting the settled-window error -- a decaying damped
% oscillation, not drift, just slower to fully die out than theta's).
THETA_TRANSIENT_WINDOW_S = 1.0;
X_TRANSIENT_WINDOW_S     = 3.0;

THETA_TRANSIENT_TOL_DEG = 2.0;
THETA_SETTLED_TOL_DEG   = 0.1;
X_TRANSIENT_TOL_M       = 0.015;
X_SETTLED_TOL_M         = 0.002;

thetaTransientIdx = double(t) <= THETA_TRANSIENT_WINDOW_S;
thetaSettledIdx   = ~thetaTransientIdx;
xTransientIdx     = double(t) <= X_TRANSIENT_WINDOW_S;
xSettledIdx       = ~xTransientIdx;

max_theta_transient_err = max(theta_err_deg(thetaTransientIdx));
max_theta_settled_err   = max(theta_err_deg(thetaSettledIdx));
max_x_transient_err     = max(x_err_m(xTransientIdx));
max_x_settled_err       = max(x_err_m(xSettledIdx));

fprintf('Max transient theta divergence (t<=%.1fs): %.4f deg (tol %.1f)\n', ...
    THETA_TRANSIENT_WINDOW_S, max_theta_transient_err, THETA_TRANSIENT_TOL_DEG);
fprintf('Max settled theta divergence (t>%.1fs):    %.4f deg (tol %.1f)\n', ...
    THETA_TRANSIENT_WINDOW_S, max_theta_settled_err, THETA_SETTLED_TOL_DEG);
fprintf('Max transient x divergence (t<=%.1fs):     %.5f m (tol %.3f)\n', ...
    X_TRANSIENT_WINDOW_S, max_x_transient_err, X_TRANSIENT_TOL_M);
fprintf('Max settled x divergence (t>%.1fs):        %.5f m (tol %.3f)\n', ...
    X_TRANSIENT_WINDOW_S, max_x_settled_err, X_SETTLED_TOL_M);

pass = (max_theta_transient_err < THETA_TRANSIENT_TOL_DEG) && ...
       (max_theta_settled_err   < THETA_SETTLED_TOL_DEG) && ...
       (max_x_transient_err     < X_TRANSIENT_TOL_M) && ...
       (max_x_settled_err       < X_SETTLED_TOL_M);

if pass
    fprintf('PASS: single-precision embedded path matches double-precision simulation within tolerance.\n');
else
    fprintf('FAIL: divergence exceeds tolerance.\n');
    fprintf('  Likely culprits to check: RBF weight drift (leakage gain), integrator\n');
    fprintf('  anti-windup tolerance (c.sat_tol), or accumulated rounding in xi over %d steps.\n', N);
end

results_equiv.t = t;
results_equiv.theta_err_deg = theta_err_deg;
results_equiv.x_err_m = x_err_m;
results_equiv.pass = pass;
results_equiv.max_theta_transient_err = max_theta_transient_err;
results_equiv.max_theta_settled_err   = max_theta_settled_err;
results_equiv.max_x_transient_err     = max_x_transient_err;
results_equiv.max_x_settled_err       = max_x_settled_err;
save(fullfile(resultsDir, 'results_embedded_equivalence.mat'), 'results_equiv');

figure('Name', 'Embedded Equivalence - Theta channel');
subplot(2,1,1);
plot(results_double.t, rad2deg(theta_double), 'b-', t, rad2deg(theta_single), 'r--');
legend('double (host)', 'single (embedded)'); ylabel('\theta [deg]'); grid on;
title('Theta: double vs single precision');

subplot(2,1,2);
plot(t, theta_err_deg, 'k-');
hold on;
xline(THETA_TRANSIENT_WINDOW_S, 'r--', 'theta transient/settled boundary');
ylabel('|\theta error| [deg]'); xlabel('time [s]'); grid on;

figure('Name', 'Embedded Equivalence - X channel');
subplot(2,1,1);
plot(results_double.t, results_double.x_true(1,:), 'b-', t, double(x_true_s(1,:)), 'r--');
legend('double (host)', 'single (embedded)'); ylabel('x [m]'); grid on;
title('X position: double vs single precision');

subplot(2,1,2);
plot(t, x_err_m, 'k-');
hold on;
xline(X_TRANSIENT_WINDOW_S, 'r--', 'x transient/settled boundary');
ylabel('|x error| [m]'); xlabel('time [s]'); grid on;