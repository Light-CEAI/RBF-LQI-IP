function results = verify_residual_projection()
%VERIFY_RESIDUAL_PROJECTION
% Numerical audit of the actual-versus-nominal residual and weighted
% matched/unmatched force projection defined in
% ACTUAL_NOMINAL_RESIDUAL_AND_PROJECTION.md.
%
% This script checks algebraic identities over a deterministic grid of
% illustrative states and admissible attached-body parameters. It does NOT
% validate hardware parameters, RBF approximation, an adaptive law,
% actuator dynamics, or closed-loop stability.
%
% Run:
%   results = verify_residual_projection();

clc;

% Illustrative nominal parameters. These are not a frozen physical set.
p0.M = 0.50; p0.m = 0.15; p0.l = 0.30;
p0.Ic = 0.0040; p0.I = p0.Ic + p0.m*p0.l^2;
p0.bx = 0.045; p0.btheta = 0.0018; p0.g = 9.81;

% Actual parameter/state samples. Rates represent smooth attached-body
% variation; they are prescribed test data, not estimated controller inputs.
scales = [0.08, 0.10]; % [a_s (m/s^2), alpha_s (rad/s^2)]
S = diag(1./(scales.^2));

stateGrid = [ ...
    0.00,  0.00,  0.00,  0.00; ...
    0.04,  0.12,  0.08, -0.15; ...
   -0.06, -0.18, -0.12,  0.22; ...
    0.10,  0.25,  0.20, -0.30; ...
   -0.08,  0.15, -0.25,  0.35]';
parameterGrid = [ ...
    0.50, 0.15, 0.30, 0.0040,  0.00,  0.0000,  0.000,  0.000; ...
    0.54, 0.16, 0.31, 0.0042,  0.01, -0.0020,  0.003, -0.0002; ...
    0.47, 0.14, 0.29, 0.0038, -0.01,  0.0015, -0.002,  0.0002; ...
    0.52, 0.155,0.32, 0.0041,  0.02, -0.0010,  0.001, -0.0001]';
% Rows: M,m,l,Ic,Mdot,mdot,ldot,Idot. I is recomputed from Ic + m*l^2;
% supplied Idot is replaced by the derivative of that pivot-inertia rule.

nState = size(stateGrid,2);
nParam = size(parameterGrid,2);
maxResidualReconstruction = 0;
maxProjectionOrthogonality = 0;
maxActualNominalIdentity = 0;
maxTorqueDeterminantError = 0;
minD = inf;
minProjectionDenominator = inf;

for k = 1:max(nState,nParam)
    X = stateGrid(:,mod(k-1,nState)+1);
    pv = parameterGrid(:,mod(k-1,nParam)+1);

    p.M = pv(1); p.m = pv(2); p.l = pv(3); p.Ic = pv(4);
    p.Mdot = pv(5); p.mdot = pv(6); p.ldot = pv(7);
    p.I = p.Ic + p.m*p.l^2;
    p.Idot = p.mdot*p.l^2 + 2*p.m*p.l*p.ldot;
    p.bx = p0.bx; p.btheta = p0.btheta; p.g = p0.g;

    xdot = X(2); theta = X(3); thetadot = X(4);
    D = (p.M+p.m)*p.I - (p.m*p.l*cos(theta))^2;
    minD = min(minD,D);
    assert(D > 0,'Illustrative parameter grid violates D > 0.');

    H = [p.M+p.m, p.m*p.l*cos(theta); ...
         p.m*p.l*cos(theta), p.I];
    hv = [ ...
       -p.m*p.l*sin(theta)*thetadot^2 + p.bx*xdot ...
       +(p.Mdot+p.mdot)*xdot ...
       +(p.mdot*p.l+p.m*p.ldot)*thetadot*cos(theta); ...
       -p.m*p.g*p.l*sin(theta) + p.btheta*thetadot ...
       +(p.mdot*p.l+p.m*p.ldot)*xdot*cos(theta) ...
       +p.Idot*thetadot];

    Bp = [0; p.I/D; 0; -p.m*p.l*cos(theta)/D];
    Gp = [0,0; p.I/D,-p.m*p.l*cos(theta)/D; ...
          0,0; -p.m*p.l*cos(theta)/D,(p.M+p.m)/D];
    % Use an explicit solve for compatibility with MATLAB releases.
    accDrift = -(H\hv);
    fp = [xdot;accDrift(1);thetadot;accDrift(2)];

    % Nominal upright linearisation at p0.
    D0 = (p0.M+p0.m)*p0.I-(p0.m*p0.l)^2;
    A0 = [0,1,0,0; ...
          0,-p0.I*p0.bx/D0,-p0.m^2*p0.g*p0.l^2/D0,p0.m*p0.l*p0.btheta/D0; ...
          0,0,0,1; ...
          0,p0.m*p0.l*p0.bx/D0,(p0.M+p0.m)*p0.m*p0.g*p0.l/D0,...
          -(p0.M+p0.m)*p0.btheta/D0];
    B0 = [0;p0.I/D0;0;-p0.m*p0.l/D0];

    uL = 0.7*cos(0.4*k); % illustrative nominal LQI force request
    r = fp + Bp*uL - A0*X - B0*uL;
    rq = [r(2);r(4)];
    bq = [p.I/D;-p.m*p.l*cos(theta)/D];

    denom = bq'*S*bq;
    minProjectionDenominator = min(minProjectionDenominator,denom);
    assert(denom > 0,'Projection denominator must be positive.');
    delta = (bq'*S*rq)/denom;
    rqu = rq-bq*delta;
    ru = [0;rqu(1);0;rqu(2)];

    recErr = norm(r-(Bp*delta+ru),inf);
    orthErr = abs(bq'*S*rqu);

    % Check the exact actual-vs-nominal identity for an arbitrary applied
    % force and two disturbances.
    Fa = -0.35 + 0.1*k;
    d = [0.12*sin(k);0.015*cos(k)];
    actual = fp + Bp*Fa + Gp*d;
    decomposed = A0*X+B0*uL+r+Bp*(Fa-uL)+Gp*d;
    dynErr = norm(actual-decomposed,inf);

    % The determinant of [bq, torque direction] must be 1/D.
    gTheta = [-p.m*p.l*cos(theta)/D;(p.M+p.m)/D];
    detErr = abs(det([bq,gTheta])-1/D);

    maxResidualReconstruction = max(maxResidualReconstruction,recErr);
    maxProjectionOrthogonality = max(maxProjectionOrthogonality,orthErr);
    maxActualNominalIdentity = max(maxActualNominalIdentity,dynErr);
    maxTorqueDeterminantError = max(maxTorqueDeterminantError,detErr);
end

tol = 1e-10;
passedResidualReconstruction = maxResidualReconstruction < tol;
passedProjectionOrthogonality = maxProjectionOrthogonality < tol;
passedActualNominalIdentity = maxActualNominalIdentity < tol;
passedTorqueDirectionIndependence = maxTorqueDeterminantError < tol;
passedPhysicalInertia = minD > 0;

results = struct( ...
    'sampleCount',max(nState,nParam), ...
    'minInertiaDeterminant',minD, ...
    'minProjectionDenominator',minProjectionDenominator, ...
    'maxResidualReconstructionError',maxResidualReconstruction, ...
    'maxProjectionOrthogonalityError',maxProjectionOrthogonality, ...
    'maxActualNominalIdentityError',maxActualNominalIdentity, ...
    'maxTorqueDeterminantIdentityError',maxTorqueDeterminantError, ...
    'passedPhysicalInertia',passedPhysicalInertia, ...
    'passedResidualReconstruction',passedResidualReconstruction, ...
    'passedProjectionOrthogonality',passedProjectionOrthogonality, ...
    'passedActualNominalIdentity',passedActualNominalIdentity, ...
    'passedTorqueDirectionIndependence',passedTorqueDirectionIndependence);

fprintf('\nACTUAL-NOMINAL RESIDUAL AND PROJECTION VERIFICATION\n');
fprintf('Samples checked                            : %d\n',results.sampleCount);
fprintf('Minimum det(H)                             : %.12e\n',minD);
fprintf('Minimum bq''*S*bq                           : %.12e\n',minProjectionDenominator);
fprintf('Maximum residual reconstruction error      : %.12e\n',maxResidualReconstruction);
fprintf('Maximum weighted orthogonality error        : %.12e\n',maxProjectionOrthogonality);
fprintf('Maximum actual-nominal identity error       : %.12e\n',maxActualNominalIdentity);
fprintf('Maximum torque-direction determinant error  : %.12e\n',maxTorqueDeterminantError);
fprintf('Physical inertia determinant                : %s\n',passfail(passedPhysicalInertia));
fprintf('Residual reconstruction                     : %s\n',passfail(passedResidualReconstruction));
fprintf('Projection orthogonality                    : %s\n',passfail(passedProjectionOrthogonality));
fprintf('Actual-nominal identity                     : %s\n',passfail(passedActualNominalIdentity));
fprintf('Torque direction independence               : %s\n',passfail(passedTorqueDirectionIndependence));

if ~(passedPhysicalInertia && passedResidualReconstruction && ...
     passedProjectionOrthogonality && passedActualNominalIdentity && ...
     passedTorqueDirectionIndependence)
    error('One or more residual/projection verification checks failed.');
end
fprintf(['\nPASS: algebraic residual and projection identities passed on the ' ...
    'illustrative deterministic grid. This does not validate parameter provenance, ' ...
    'RBF approximation, actuator behaviour, or closed-loop stability.\n']);
end

function s = passfail(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end
