function results = verify_nonlinear_model()
%VERIFY_NONLINEAR_MODEL Independent checks of the cart-pendulum plant.
%
% This is a MODEL VERIFICATION utility, not a controller implementation.
% The parameter values below are illustrative numerical test values only.
% They are NOT a frozen hardware/reference parameter set.
%
% State: X = [x; xdot; theta; thetadot], theta = 0 at upright.
% Input: F is horizontal cart force; d = [dx; dtheta].
%
% Run from MATLAB:
%   results = verify_nonlinear_model();

clc;

% Illustrative, physically consistent test point. Replace only after the
% project parameter register has been verified and frozen.
p.M = 0.50;          % kg
p.m = 0.15;          % kg
p.l = 0.30;          % m, pivot to pendulum COM
p.I = 0.0180;        % kg m^2, ABOUT THE PIVOT (must be >= m*l^2)
p.g = 9.81;          % m/s^2
p.bx = 0.05;         % N s/m; illustrative only
p.btheta = 0.002;    % N m s/rad; illustrative only

assert(p.M > 0 && p.m > 0 && p.l > 0 && p.I > 0, ...
    'Masses, COM distance, and pivot inertia must be positive.');
assert(p.I >= p.m*p.l^2, ...
    'Physical consistency requires pivot inertia I >= m*l^2.');
assert(p.bx >= 0 && p.btheta >= 0 && p.g > 0, ...
    'Damping must be nonnegative and gravity positive.');

% Non-equilibrium point avoids checking only the trivial origin.
X0 = [0.07; -0.13; 0.11; 0.19];
F0 = 0.42;
d0 = [0.17; -0.035];

[A, B, E, D0] = analytic_linear_model(p);
f0 = plant_rhs(X0, F0, d0, p);

% Central finite-difference Jacobians of the nonlinear state equation.
nx = numel(X0);
Afd = zeros(4,nx);
for k = 1:nx
    h = 1e-6 * max(1,abs(X0(k)));
    xp = X0; xm = X0;
    xp(k) = xp(k) + h;
    xm(k) = xm(k) - h;
    Afd(:,k) = (plant_rhs(xp,F0,d0,p) - ...
                plant_rhs(xm,F0,d0,p))/(2*h);
end

hu = 1e-6 * max(1,abs(F0));
Bfd = (plant_rhs(X0,F0+hu,d0,p) - ...
       plant_rhs(X0,F0-hu,d0,p))/(2*hu);

Efd = zeros(4,2);
for k = 1:2
    hd = 1e-6;
    dp = d0; dm = d0;
    dp(k) = dp(k) + hd;
    dm(k) = dm(k) - hd;
    Efd(:,k) = (plant_rhs(X0,F0,dp,p) - ...
                plant_rhs(X0,F0,dm,p))/(2*hd);
end

% The analytic matrices are Jacobians at the upright equilibrium.
% Therefore compare finite differences at X=0, F=0, d=0.
Xeq = zeros(4,1); deq = zeros(2,1);
Afd_eq = zeros(4,4);
for k = 1:4
    h = 1e-6;
    xp = Xeq; xm = Xeq;
    xp(k) = xp(k) + h;
    xm(k) = xm(k) - h;
    Afd_eq(:,k) = (plant_rhs(xp,0,deq,p) - ...
                   plant_rhs(xm,0,deq,p))/(2*h);
end
Bfd_eq = (plant_rhs(Xeq,1e-6,deq,p) - ...
          plant_rhs(Xeq,-1e-6,deq,p))/(2e-6);
Efd_eq = zeros(4,2);
for k = 1:2
    dp = deq; dm = deq;
    dp(k) = 1e-6; dm(k) = -1e-6;
    Efd_eq(:,k) = (plant_rhs(Xeq,0,dp,p) - ...
                   plant_rhs(Xeq,0,dm,p))/(2e-6);
end

% Check inertia positive definiteness over a broad angle sweep.
thetaGrid = linspace(-pi,pi,2001);
minDet = inf;
minEig = inf;
for theta = thetaGrid
    H = [p.M+p.m, p.m*p.l*cos(theta); ...
         p.m*p.l*cos(theta), p.I];
    minDet = min(minDet,det(H));
    minEig = min(minEig,min(eig(H)));
end

tol = 5e-6;
errA = norm(A-Afd_eq,inf);
errB = norm(B-Bfd_eq,inf);
errE = norm(E-Efd_eq,inf);

results = struct();
results.D0 = D0;
results.minimumInertiaDeterminant = minDet;
results.minimumInertiaEigenvalue = minEig;
results.linearisationErrorA = errA;
results.linearisationErrorB = errB;
results.linearisationErrorE = errE;
results.passedInertia = minDet > 0 && minEig > 0;
results.passedLinearisation = max([errA,errB,errE]) < tol;
results.sampleNonlinearDerivative = f0;

fprintf('\nCART-PENDULUM MODEL VERIFICATION\n');
fprintf('D(0)                         : %.12g\n',D0);
fprintf('Minimum det(H), theta [-pi,pi]: %.12g\n',minDet);
fprintf('Minimum eig(H), theta [-pi,pi]: %.12g\n',minEig);
fprintf('Upright Jacobian error A     : %.3e\n',errA);
fprintf('Upright Jacobian error B     : %.3e\n',errB);
fprintf('Upright Jacobian error E     : %.3e\n',errE);
fprintf('Inertia positive definite    : %s\n',passfail(results.passedInertia));
fprintf('Linearisation finite-diff    : %s\n',passfail(results.passedLinearisation));

if ~results.passedInertia || ~results.passedLinearisation
    error('One or more nonlinear-model verification checks failed.');
end
fprintf(['\nPASS: algebraic/numerical consistency checks completed for the ' ...
         'illustrative parameter set. This does not validate hardware ' ...
         'parameters, time-varying mass dynamics, actuator dynamics, or stability.\n']);
end

function dX = plant_rhs(X,F,d,p)
theta = X(3);
xdot = X(2);
thetadot = X(4);

H = [p.M+p.m, p.m*p.l*cos(theta); ...
     p.m*p.l*cos(theta), p.I];
h = [-p.m*p.l*sin(theta)*thetadot^2 + p.bx*xdot; ...
     -p.m*p.g*p.l*sin(theta) + p.btheta*thetadot];

qdd = H \ ([F+d(1); d(2)] - h);
dX = [xdot; qdd(1); thetadot; qdd(2)];
end

function [A,B,E,D] = analytic_linear_model(p)
D = (p.M+p.m)*p.I - (p.m*p.l)^2;
A = [0, 1, 0, 0; ...
     0, -p.I*p.bx/D, -p.m^2*p.g*p.l^2/D, p.m*p.l*p.btheta/D; ...
     0, 0, 0, 1; ...
     0, p.m*p.l*p.bx/D, (p.M+p.m)*p.m*p.g*p.l/D, ...
        -(p.M+p.m)*p.btheta/D];
B = [0; p.I/D; 0; -p.m*p.l/D];
E = [0, 0; ...
     p.I/D, -p.m*p.l/D; ...
     0, 0; ...
     -p.m*p.l/D, (p.M+p.m)/D];
end

function s = passfail(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end
