function [K, Ki, A_aug, B_aug, Q_aug] = design_lqi(A, B, params, Q_aug, R)
%DESIGN_LQI Compute LQI (LQR + integral action) gains for the linearized plant.
%
%   [K, Ki, A_aug, B_aug, Q_aug] = design_lqi(A, B, params)
%   [K, Ki, A_aug, B_aug, Q_aug] = design_lqi(A, B, params, Q_aug, R)
%
%   Tracked output: cart position x (state 1). An integral state xi is
%   appended so the augmented plant rejects constant disturbances /
%   reference offsets in x with zero steady-state error.
%
%   Q weighting [x, xdot, theta, thetadot, integral-x-error]:
%   Increased x-weight from 10 -> 30 (relative to theta's 100) after the
%   3N force-disturbance test showed the cart exceeding the safe rail
%   travel (0.455m > 0.35m limit) while theta itself stayed controlled --
%   i.e. the old weighting let position drift too far in favor of angle
%   recovery under a strong enough push. Re-run test_disturbance.m after
%   this change to confirm the 3N case now passes.
%
%   NOTE: this is the SINGLE SOURCE for these gains -- every test_*.m
%   script gets this automatically via simulation_main.m -> design_lqi.m,
%   none of them override Q_aug. If you later regenerate control_consts.m
%   for embedded deployment, re-run that generation step too, since it
%   bakes in whatever gains design_lqi.m produces at generation time.

n = size(A,1);
C_track = [1, 0, 0, 0];   % track/regulate cart position x

A_aug = [A,        zeros(n,1);
    -C_track,  0];
B_aug = [B; 0];

if nargin < 4 || isempty(Q_aug)
    Q_aug = diag([30, 1, 100, 1, 50]);   % x weight: 10 -> 30
end
if nargin < 5 || isempty(R)
    R = 1;
end

K_aug = lqr(A_aug, B_aug, Q_aug, R);

K  = K_aug(1:n);
Ki = K_aug(n+1);

Acl = A_aug - B_aug*K_aug;
if any(real(eig(Acl)) >= 0)
    warning('design_lqi:unstable', ...
        'Closed-loop augmented system is not strictly stable with these weights.');
end

end