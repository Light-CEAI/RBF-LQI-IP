%% USER-COMPLETED CONFIGURATION FOR PROJECTION METRIC GRID AUDIT
% Fill only with ranges justified in Research_Reporting/PARAMETER_REGISTER.md
% and the declared admissible operating envelope. SI units throughout.
%
% This template deliberately contains no assumed physical parameter values.
% Do not use simulation-only true parameters as controller measurements.

cfg = struct();

cfg.M = [];       % Cart/base mass grid, kg
cfg.m = [];       % Pendulum mass grid, kg
cfg.l = [];       % Pivot-to-COM distance grid, m
cfg.Ic = [];      % Pendulum COM inertia grid, kg*m^2
cfg.theta = [];   % Admissible angle grid, rad (theta=0 upright)

cfg.a_s = [];       % Characteristic cart-acceleration scale, m/s^2
cfg.alpha_s = [];   % Characteristic angular-acceleration scale, rad/s^2

% Predeclared sensitivity multipliers for alpha_s/a_s.
cfg.scaleRatios = [0.5 0.75 1 1.5 2];

% Enforce nonnegative COM inertia.
cfg.requireIphysical = true;

% After completing the fields, execute:
% results = audit_projection_metric_envelope(cfg);
