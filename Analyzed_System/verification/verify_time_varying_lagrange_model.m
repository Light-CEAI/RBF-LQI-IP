function results = verify_time_varying_lagrange_model()
%VERIFY_TIME_VARYING_LAGRANGE_MODEL
% Numerical audit of the time-varying attached-rigid-body equations.
%
% This checks equations (6)-(7) in
% MATHEMATICAL_MODEL_AND_STABILITY_ANALYSIS.md by independently evaluating
% d/dt(dL/dqdot) - dL/dq along smooth prescribed q(t), p(t) trajectories.
%
% IMPORTANT SCOPE:
% - This is an attached-body, time-varying kinetic-energy model.
% - It is NOT an open-system variable-mass / rocket equation.
% - No controller, actuator, payload-impact reset, or stability is tested.
% - Illustrative parameters are used only for numerical verification.
%
% Run:
%   results = verify_time_varying_lagrange_model();

clc;

t0 = 0.83;
h = 2e-5;
p = parameter_trajectory(t0);
q = coordinate_trajectory(t0);
u.Fa = 0.35;
u.dx = -0.08;
u.dtheta = 0.012;
bx = 0.045;
btheta = 0.0018;

% Independent Euler-Lagrange evaluation: numerical time derivative of
% generalized momentum, minus the analytical partial derivative dL/dq.
momPlus = generalized_momentum(t0+h);
momMinus = generalized_momentum(t0-h);
momDotFD = (momPlus-momMinus)/(2*h);
dLdq = lagrangian_partial_q(t0);
EL = momDotFD-dLdq;

% Right-hand side of Lagrange equations including damping and disturbances.
Q = [u.Fa+u.dx-bx*q.xdot; u.dtheta-btheta*q.thetadot];

% Explicit left-hand sides of the proposed time-varying equations (6)-(7).
Dterms = [(p.Mdot+p.mdot)*q.xdot + ...
          (p.mdot*p.l+p.m*p.ldot)*q.thetadot*cos(q.theta); ...
          (p.mdot*p.l+p.m*p.ldot)*q.xdot*cos(q.theta) + ...
          p.Idot*q.thetadot];

H = [p.M+p.m, p.m*p.l*cos(q.theta); ...
     p.m*p.l*cos(q.theta), p.I];
hvec = [-p.m*p.l*sin(q.theta)*q.thetadot^2; ...
        -p.m*p.g*p.l*sin(q.theta)];
explicitLHS = H*[q.xddot;q.thetaddot] + hvec + Dterms;

% Compare independent Euler-Lagrange evaluation against explicit equations.
elError = norm(EL-explicitLHS,inf);
forceBalanceError = norm(EL-Q,inf);

% Constant-parameter limit: all parameter-rate terms must vanish.
pc = p;
pc.Mdot = 0; pc.mdot = 0; pc.ldot = 0; pc.Idot = 0;
DtermsConstant = [(pc.Mdot+pc.mdot)*q.xdot + ...
    (pc.mdot*pc.l+pc.m*pc.ldot)*q.thetadot*cos(q.theta); ...
    (pc.mdot*pc.l+pc.m*pc.ldot)*q.xdot*cos(q.theta) + ...
    pc.Idot*q.thetadot];

% Verify pivot inertia is physically consistent at the test instant.
physicalMargin = p.I-p.m*p.l^2;

tol = 2e-7;
results = struct();
results.eulerLagrangeEquationError = elError;
results.forceBalanceResidual = forceBalanceError;
results.parameterRateTermNorm = norm(Dterms,inf);
results.constantParameterRateTermNorm = norm(DtermsConstant,inf);
results.pivotInertiaMargin = physicalMargin;
results.passedEquationIdentity = elError < tol;
results.passedPhysicalInertia = physicalMargin > 0;
results.passedConstantParameterLimit = norm(DtermsConstant,inf) == 0;

fprintf('\nTIME-VARYING ATTACHED-BODY LAGRANGE VERIFICATION\n');
fprintf('Euler-Lagrange / explicit-equation error : %.3e\n',elError);
fprintf('Force-balance residual at selected input : %.3e\n',forceBalanceError);
fprintf('Norm of parameter-rate terms             : %.6g\n',norm(Dterms,inf));
fprintf('Constant-parameter rate-term norm        : %.3e\n',norm(DtermsConstant,inf));
fprintf('Pivot inertia margin I-m*l^2             : %.6g kg m^2\n',physicalMargin);
fprintf('Equation identity                        : %s\n',passfail(results.passedEquationIdentity));
fprintf('Physical inertia condition               : %s\n',passfail(results.passedPhysicalInertia));
fprintf('Constant-parameter limit                  : %s\n',passfail(results.passedConstantParameterLimit));

if ~results.passedEquationIdentity || ~results.passedPhysicalInertia || ...
        ~results.passedConstantParameterLimit
    error('One or more time-varying Lagrange verification checks failed.');
end
fprintf(['\nPASS: numerical identity checks completed for the illustrative ' ...
    'smooth attached-body trajectory. This is not validation of open-system ' ...
    'mass flow, abrupt payload events, hardware parameters, or stability.\n']);

    function mom = generalized_momentum(t)
        pp = parameter_trajectory(t);
        qq = coordinate_trajectory(t);
        Ht = [pp.M+pp.m, pp.m*pp.l*cos(qq.theta); ...
              pp.m*pp.l*cos(qq.theta), pp.I];
        mom = Ht*[qq.xdot;qq.thetadot];
    end

    function partial = lagrangian_partial_q(t)
        pp = parameter_trajectory(t);
        qq = coordinate_trajectory(t);
        % dL/dx = 0; dL/dtheta from T - V.
        partial = [0; ...
            -pp.m*pp.l*qq.xdot*qq.thetadot*sin(qq.theta) + ...
             pp.m*pp.g*pp.l*sin(qq.theta)];
    end
end

function p = parameter_trajectory(t)
% Smooth prescribed attached-body mass-property variation.
p.M = 0.50 + 0.025*sin(0.4*t);
p.Mdot = 0.010*cos(0.4*t);
p.m = 0.15 + 0.008*cos(0.3*t);
p.mdot = -0.0024*sin(0.3*t);
p.l = 0.30 + 0.006*sin(0.5*t);
p.ldot = 0.003*cos(0.5*t);
Ic = 0.0040;                 % illustrative COM inertia, kg m^2
p.I = Ic + p.m*p.l^2;        % pivot inertia by parallel-axis theorem
p.Idot = p.mdot*p.l^2 + 2*p.m*p.l*p.ldot;
p.g = 9.81;
end

function q = coordinate_trajectory(t)
q.x = 0.08*sin(0.7*t);
q.xdot = 0.056*cos(0.7*t);
q.xddot = -0.0392*sin(0.7*t);
q.theta = 0.12*cos(0.5*t);
q.thetadot = -0.06*sin(0.5*t);
q.thetaddot = -0.03*cos(0.5*t);
end

function s = passfail(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end
