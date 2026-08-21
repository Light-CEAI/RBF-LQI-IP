function plot_stability_margins(params)
%PLOT_STABILITY_MARGINS Closed-loop pole locations + classical LQR
%   frequency-domain margins.
%
%   plot_stability_margins(params)
%
%   Two distinct analyses, intentionally separated:
%
%   (A) Closed-loop pole plots for the ACTUAL augmented LQI system
%       (A_aug - B_aug*K_aug) and observer error dynamics (Ad - L*C_meas)
%       -- confirms stability BY DESIGN, independent of any simulation run.
%
%   (B) Gain/phase margin of the BASE full-state-feedback LQR loop
%       (K only, no integral state, no observer) -- this is where the
%       textbook LQR guarantee (>=60deg phase margin, infinite gain
%       margin upward) actually applies [Anderson & Moore, Optimal
%       Control: Linear Quadratic Methods]. That guarantee does NOT
%       carry over once an observer (LQG) or integral augmentation is
%       in the loop -- margins for the full system as actually deployed
%       are NOT guaranteed and are not computed here; (A) is the correct
%       check for the augmented/observed system.

if nargin < 1, params = parameters(); end

[A, B, C, D] = linearized_model(params);
[K, Ki, A_aug, B_aug] = design_lqi(A, B, params);
[Ad, ~, ~, ~] = discretize_model(A, B, C, D, params.embedded.Ts);
[L, C_meas] = observer_design(Ad, params, 'kalman');

%% ---- (A) Closed-loop pole plots -----------------------------------------
K_aug = [K, Ki];
eig_ctrl = eig(A_aug - B_aug*K_aug);
eig_obs  = eig(Ad - L*C_meas);

figure('Name', 'Closed-Loop Pole Locations');
subplot(1,2,1);
plot(real(eig_ctrl), imag(eig_ctrl), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
xline(0, 'k--'); yline(0, 'k--'); grid on; axis equal;
xlabel('Real'); ylabel('Imaginary');
title('Augmented LQI Closed-Loop Poles (continuous)');
maxRe = max(real(eig_ctrl));
text(0.02, 0.95, sprintf('max(Re) = %.4f  (%s)', maxRe, ...
    ternary(maxRe < 0, 'STABLE', 'UNSTABLE')), 'Units', 'normalized', ...
    'BackgroundColor', ternary(maxRe<0,[0.85 1 0.85],[1 0.85 0.85]));

subplot(1,2,2);
theta_unit = linspace(0, 2*pi, 200);
plot(cos(theta_unit), sin(theta_unit), 'k--'); hold on;
plot(real(eig_obs), imag(eig_obs), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
grid on; axis equal;
xlabel('Real'); ylabel('Imaginary');
title('Observer Error Dynamics Poles (discrete, unit circle)');
maxAbs = max(abs(eig_obs));
text(0.02, 0.95, sprintf('max(|\\lambda|) = %.4f  (%s)', maxAbs, ...
    ternary(maxAbs < 1, 'STABLE', 'UNSTABLE')), 'Units', 'normalized', ...
    'BackgroundColor', ternary(maxAbs<1,[0.85 1 0.85],[1 0.85 0.85]));

fprintf('--- Stability by Design ---\n');
fprintf('Augmented LQI: max(Re(eig)) = %.4f -> %s\n', maxRe, ternary(maxRe<0,'STABLE','UNSTABLE'));
fprintf('Observer:      max(|eig|)   = %.4f -> %s\n', maxAbs, ternary(maxAbs<1,'STABLE','UNSTABLE'));

%% ---- (B) Classical LQR margins (K only, no Ki, no observer) ------------
sys_OL = ss(A, B, K, 0);  % loop gain L(s) = K*(sI-A)^-1*B, broken at plant input
[Gm, Pm, Wcg, Wcp] = margin(sys_OL);

figure('Name', 'LQR Loop Gain Margins (base K, idealized full-state-feedback)');
margin(sys_OL);
grid on;

GmdB = 20*log10(Gm);
fprintf('\n--- Classical LQR Margins (base K, full-state-feedback assumption) ---\n');
fprintf('Gain margin:  %.2f dB at %.3f rad/s\n', GmdB, Wcg);
fprintf('Phase margin: %.2f deg at %.3f rad/s\n', Pm, Wcp);
fprintf('NOTE: this margin applies ONLY to the idealized full-state-feedback\n');
fprintf('loop (K, no integral term, no observer). It does NOT characterize\n');
fprintf('the margins of the deployed system, which includes Ki and the\n');
fprintf('observer in the loop -- those margins are not guaranteed by LQR\n');
fprintf('theory and are not computed here. Use panel (A) above for the\n');
fprintf('actual deployed system''s stability verification.\n');
if Pm >= 60 || isinf(Pm)
    fprintf('Phase margin >= 60deg: consistent with the textbook LQR guarantee.\n');
end

end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end