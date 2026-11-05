function results = audit_projection_metric_envelope(cfg)
%AUDIT_PROJECTION_METRIC_ENVELOPE
% Grid-based numerical audit of inertia positivity and weighted projection
% conditioning over USER-SUPPLIED parameter and angle ranges.
%
% This is a numerical grid audit, NOT a proof of a continuous-domain bound.
% No physical parameter ranges or projection scales are silently assumed.
%
% Required cfg fields (all vectors, SI units):
%   cfg.M, cfg.m, cfg.l, cfg.Ic, cfg.theta
%   cfg.a_s, cfg.alpha_s
%
% Optional:
%   cfg.scaleRatios = vector of positive multipliers applied to alpha_s/a_s
%                    while holding a_s fixed (default [0.5 0.75 1 1.5 2]).
%   cfg.requireIphysical = true/false (default true; checks Ic >= 0).
%
% Example after completing the parameter register:
%   cfg.M = [...];       % kg, declared range/grid
%   cfg.m = [...];       % kg
%   cfg.l = [...];       % m
%   cfg.Ic = [...];      % kg*m^2, about COM
%   cfg.theta = [...];   % rad, admissible angle grid
%   cfg.a_s = ...;       % m/s^2, justified characteristic scale
%   cfg.alpha_s = ...;   % rad/s^2, justified characteristic scale
%   results = audit_projection_metric_envelope(cfg);
%
% Interpretation:
%   - minDetH > 0: sampled inertia matrices were positive definite.
%   - minDenom > 0: sampled projection denominators were positive.
%   - A finite grid minimum is not beta_b for the full continuous set.
%   - Scale-ratio sensitivity changes the chosen matched-force projection;
%     it is a design sensitivity, not an optimisation objective.
%
% This function does not validate parameter provenance, sensors, parameter
% estimation, RBF approximation, actuator behaviour, or closed-loop stability.

required = {'M','m','l','Ic','theta','a_s','alpha_s'};
if nargin ~= 1 || ~isstruct(cfg)
    error('Supply one configuration struct. See the function header.');
end
for k = 1:numel(required)
    name = required{k};
    if ~isfield(cfg,name) || isempty(cfg.(name))
        error('Missing cfg.%s. Do not replace an unknown range with a guess.',name);
    end
    v = cfg.(name);
    if ~isnumeric(v) || ~isvector(v) || any(~isfinite(v(:)))
        error('cfg.%s must be a finite numeric vector.',name);
    end
end
if any(cfg.M(:)<=0) || any(cfg.m(:)<=0) || any(cfg.l(:)<=0) || ...
        any(cfg.Ic(:)<0) || any(cfg.theta(:)<-pi) || any(cfg.theta(:)>pi)
    error('Require M,m,l > 0, Ic >= 0 and theta in [-pi,pi].');
end
if ~isscalar(cfg.a_s) || ~isscalar(cfg.alpha_s) || ...
        ~isfinite(cfg.a_s) || ~isfinite(cfg.alpha_s) || ...
        cfg.a_s<=0 || cfg.alpha_s<=0
    error('a_s and alpha_s must be positive finite scalar acceleration scales.');
end
if ~isfield(cfg,'scaleRatios') || isempty(cfg.scaleRatios)
    cfg.scaleRatios = [0.5 0.75 1 1.5 2];
end
if any(~isfinite(cfg.scaleRatios(:))) || any(cfg.scaleRatios(:)<=0)
    error('scaleRatios must contain positive finite values.');
end
if ~isfield(cfg,'requireIphysical')
    cfg.requireIphysical = true;
end

% Cartesian product of user-supplied grid values.
[MM,mm,ll,IIc,TT] = ndgrid(cfg.M(:),cfg.m(:),cfg.l(:), ...
                           cfg.Ic(:),cfg.theta(:));
M = MM(:); m = mm(:); l = ll(:); Ic = IIc(:); theta = TT(:);
I = Ic + m.*l.^2; % parallel-axis theorem: pivot inertia
D = (M+m).*I - (m.*l.*cos(theta)).^2;

if cfg.requireIphysical && any(Ic < 0)
    error('Nonphysical COM inertia encountered.');
end
if any(D <= 0)
    bad = find(D<=0,1,'first');
    error(['Inertia matrix is not positive definite at a supplied grid point. ' ...
        'First failing index: %d. Recheck ranges/model.'],bad);
end

% b_q = [I; -m*l*cos(theta)]/D, with units acceleration/N.
bxF = I./D;
bthetaF = -(m.*l.*cos(theta))./D;

nRatio = numel(cfg.scaleRatios);
minDenom = zeros(nRatio,1);
maxDenom = zeros(nRatio,1);
conditionRatio = zeros(nRatio,1);
for j = 1:nRatio
    alpha = cfg.alpha_s * cfg.scaleRatios(j);
    denom = (bxF./cfg.a_s).^2 + (bthetaF./alpha).^2;
    minDenom(j) = min(denom);
    maxDenom(j) = max(denom);
    conditionRatio(j) = maxDenom(j)/minDenom(j);
end

results = struct();
results.sampleCount = numel(D);
results.minDetH = min(D);
results.maxDetH = max(D);
results.minPivotInertia = min(I);
results.minProjectionDenominator = minDenom(cfg.scaleRatios==1);
if isempty(results.minProjectionDenominator)
    results.minProjectionDenominator = NaN;
end
results.scaleRatios = cfg.scaleRatios(:);
results.alphaScales = cfg.alpha_s*cfg.scaleRatios(:);
results.minDenominatorByScaleRatio = minDenom;
results.maxDenominatorByScaleRatio = maxDenom;
results.denominatorGridSpread = conditionRatio;
results.gridOnlyWarning = ['Grid minima are diagnostics only; they are not ' ...
    'certified continuous-domain lower bounds.'];

fprintf('\nPROJECTION METRIC OPERATING-GRID AUDIT\n');
fprintf('Grid combinations                         : %d\n',results.sampleCount);
fprintf('Minimum det(H) [kg^2 m^2]                 : %.12e\n',results.minDetH);
fprintf('Maximum det(H) [kg^2 m^2]                 : %.12e\n',results.maxDetH);
fprintf('Minimum pivot inertia [kg m^2]            : %.12e\n',results.minPivotInertia);
fprintf('\nBase scales: a_s = %.6g m/s^2, alpha_s = %.6g rad/s^2\n', ...
    cfg.a_s,cfg.alpha_s);
fprintf('alpha_s multiplier | alpha_s | min(bq''Sbq) | max/min\n');
for j = 1:nRatio
    fprintf('%18.6g | %8.4g | %12.5e | %12.5e\n', ...
        cfg.scaleRatios(j),results.alphaScales(j), ...
        minDenom(j),conditionRatio(j));
end
fprintf(['\nNOTE: this is a finite-grid numerical audit only. It does not ' ...
    'establish a continuous-domain beta_b or prove stability.\n']);
end
