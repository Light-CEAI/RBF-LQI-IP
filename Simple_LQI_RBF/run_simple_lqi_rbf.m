function results = run_simple_lqi_rbf()
%RUN_SIMPLE_LQI_RBF Minimal force-input LQI + RBF inverted-pendulum study.
% Run from this folder or add it to the MATLAB path:
%   results = run_simple_lqi_rbf();
%
% State: [x; xdot; theta; thetadot], theta=0 is upright.
% Input: one direct horizontal force u [N].
% Controller: u = u_LQI + f_RBF.
% No configuration switch, actuator saturation, anti-windup, observer,
% embedded-code path, Coulomb friction, or online adaptation is included.
%
% The RBF is trained offline on the force-equivalent projection of the
% nonlinear acceleration residual relative to the nominal linear model.
% This is a clean simulation baseline, not a hardware-ready controller.

clc;
close all;

%% Nominal physical parameters (single direct-force plant)
p.M = 0.50;       % cart mass [kg]
p.m = 0.15;       % pendulum mass [kg]
p.l = 0.30;       % pivot-to-centre-of-mass distance [m]
p.L = 0.60;       % uniform-rod total length [m]
p.g = 9.81;       % gravity [m/s^2]
p.I = p.m*p.L^2/3;

[A,B] = simple_linear_model(p);

%% LQI design: regulate cart position and upright pendulum angle
Cint = [1 0 0 0];
Aaug = [A zeros(4,1); -Cint 0];
Baug = [B; 0];
Qaug = diag([20 2 250 8 30]);
R = 0.8;
Kaug = lqr(Aaug,Baug,Qaug,R);
K = Kaug(1:4);
Ki = Kaug(5);

%% Offline RBF training on the nominal nonlinear residual
rng(7);
nTrain = 5000;
nBasis = 60;
trainRange = [deg2rad(-35), deg2rad(35); -4, 4; -0.35, 0.35; -2, 2];
Xtrain = trainRange(:,1)' + rand(nTrain,4).*(trainRange(:,2)-trainRange(:,1))';
centres = Xtrain(randperm(nTrain,nBasis),:);
sigma = [0.30 2.0 0.25 1.0];
Phi = zeros(nTrain,nBasis);
targetForce = zeros(nTrain,1);
bacc = [p.I/((p.M+p.m)*p.I-(p.m*p.l)^2); ...
       -p.m*p.l/((p.M+p.m)*p.I-(p.m*p.l)^2)];
for k = 1:nTrain
    z = Xtrain(k,:);
    state = [z(3); z(4); z(1); z(2)];
    dxNL = simple_ip_plant(0,state,0,p);
    dxLIN = A*state;
    residualAcc = [dxNL(2)-dxLIN(2); dxNL(4)-dxLIN(4)];
    targetForce(k) = -(bacc'*residualAcc)/(bacc'*bacc);
    Phi(k,:) = rbf_basis(z,centres,sigma)';
end
lambda = 1e-5;
w = (Phi'*Phi + lambda*eye(nBasis))\(Phi'*targetForce);
fprintf('RBF training force-residual RMSE: %.5f N\n', ...
    sqrt(mean((Phi*w-targetForce).^2)));

%% Compare LQI-only and LQI+RBF for nominal and large initial angles
angles = [20 23 24 25 26 27 28 29 30];
Ts = 0.01;
Tend = 8;
results = struct([]);
for a = 1:numel(angles)
    x0 = [0; 0; deg2rad(angles(a)); 0];
    results(a).angle_deg = angles(a);
    results(a).LQI = simulate_case(x0,false,A,B,K,Ki,p,centres,sigma,w,Ts,Tend);
    results(a).LQI_RBF = simulate_case(x0,true,A,B,K,Ki,p,centres,sigma,w,Ts,Tend);
end

%% Plots: angle response and applied force
for a = 1:numel(angles)
    figure('Name',sprintf('Initial angle %g deg',angles(a)));
    subplot(2,1,1);
    plot(results(a).LQI.t,rad2deg(results(a).LQI.x(3,:)),'--', ...
         results(a).LQI_RBF.t,rad2deg(results(a).LQI_RBF.x(3,:)),'LineWidth',1.2);
    grid on; ylabel('Pendulum angle [deg]');
    legend('LQI','LQI + RBF','Location','best');
    title(sprintf('Initial angle = %g deg',angles(a)));
    subplot(2,1,2);
    plot(results(a).LQI.t(1:end-1),results(a).LQI.u,'--', ...
         results(a).LQI_RBF.t(1:end-1),results(a).LQI_RBF.u,'LineWidth',1.2);
    grid on; ylabel('Force [N]'); xlabel('Time [s]');
    legend('LQI','LQI + RBF','Location','best');
end

fprintf('\nInitial-angle summary (simulation only)\n');
fprintf('Angle | LQI peak | LQI+RBF peak | LQI final | LQI+RBF final (deg)\n');
for a = 1:numel(angles)
    e1 = rad2deg(results(a).LQI.x(3,:));
    e2 = rad2deg(results(a).LQI_RBF.x(3,:));
    fprintf('%5.1f | %8.3f | %12.3f | %9.3f | %15.3f\n', ...
        angles(a),max(abs(e1)),max(abs(e2)),e1(end),e2(end));
end
end

function out = simulate_case(x0,useRBF,A,B,K,Ki,p,centres,sigma,w,Ts,Tend)
N = round(Tend/Ts);
out.t = (0:N)*Ts;
out.x = zeros(4,N+1);
out.u = zeros(1,N);
out.fRBF = zeros(1,N);
out.x(:,1) = x0;
xi = 0;
for k = 1:N
    x = out.x(:,k);
    uLQI = -K*x - Ki*xi;
    f = 0;
    if useRBF
        z = [x(3) x(4) x(1) x(2)];
        f = rbf_basis(z,centres,sigma)'*w;
    end
    u = uLQI + f;
    [~,xx] = ode45(@(t,s) simple_ip_plant(t,s,u,p), ...
                   [out.t(k) out.t(k+1)],x);
    out.x(:,k+1) = xx(end,:)';
    xi = xi - x(1)*Ts;
    out.u(k) = u;
    out.fRBF(k) = f;
end
end

function [A,B] = simple_linear_model(p)
Den = (p.M+p.m)*p.I-(p.m*p.l)^2;
A = zeros(4);
A(1,2)=1; A(2,3)=-p.m^2*p.g*p.l^2/Den;
A(3,4)=1; A(4,3)=(p.M+p.m)*p.m*p.g*p.l/Den;
B = [0; p.I/Den; 0; -p.m*p.l/Den];
end

function dx = simple_ip_plant(~,s,u,p)
% Frictionless, unsaturated nonlinear cart-pendulum with direct force input.
xdot=s(2); theta=s(3); thetadot=s(4);
MM=[p.M+p.m, p.m*p.l*cos(theta); ...
    p.m*p.l*cos(theta), p.I];
rhs=[u+p.m*p.l*sin(theta)*thetadot^2; ...
     p.m*p.g*p.l*sin(theta)];
acc=MM\rhs;
dx=[xdot;acc(1);thetadot;acc(2)];
end

function phi = rbf_basis(z,centres,sigma)
d=(centres-z)./sigma;
phi=exp(-0.5*sum(d.^2,2));
end
