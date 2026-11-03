function results = run_rbf_adaptive_lqi()
%RUN_RBF_ADAPTIVE_LQI RBF-estimated nonlinearities with adaptive LQI tuning.
% Run: results = run_rbf_adaptive_lqi();
% State [x; xdot; theta; thetadot], theta=0 upright; input is cart force [N].
% The offline RBF estimates nonlinear acceleration residuals. Its analytical
% Jacobian updates local A,B matrices; LQI is recomputed and smoothly blended.
% The RBF output is NOT added as a force compensation term.
% Research simulation baseline only: no saturation, anti-windup, friction,
% observer, online RBF weight learning, or embedded deployment.

clc; close all;
p.M=0.50; p.m=0.15; p.l=0.30; p.L=0.60; p.g=9.81; p.I=p.m*p.L^2/3;
[A0,B0]=nominal_ip_model(p);
Cint=[1 0 0 0]; Qaug=diag([20 2 250 8 30]); R=0.8;
[K0,Ki0]=design_lqi(A0,B0,Cint,Qaug,R);

%% Train multi-output RBF on nonlinear acceleration residuals
rng(11); nTrain=7000; nBasis=80;
% RBF input z=[theta, theta_dot, x, x_dot, force].
zRange=[-deg2rad(35) deg2rad(35); -4 4; -0.35 0.35; -2 2; -18 18];
Ztrain=zRange(:,1)'+rand(nTrain,5).*(zRange(:,2)-zRange(:,1))';
centres=Ztrain(randperm(nTrain,nBasis),:);
sigma=[0.24 1.5 0.24 1.5 7.0];
Phi=zeros(nTrain,nBasis); Ytrain=zeros(nTrain,2);
for k=1:nTrain
    z=Ztrain(k,:); x=[z(3);z(4);z(1);z(2)]; u=z(5);
    dxNL=nonlinear_ip_plant(0,x,u,p); dxNom=A0*x+B0*u;
    Ytrain(k,:)=[dxNL(2)-dxNom(2),dxNL(4)-dxNom(4)];
    Phi(k,:)=rbf_basis(z,centres,sigma)';
end
ridge=1e-5; W=(Phi'*Phi+ridge*eye(nBasis))\(Phi'*Ytrain);
Yfit=Phi*W; trainRMSE=sqrt(mean((Yfit-Ytrain).^2,1));
fprintf('RBF training residual RMSE: cart %.5f, angular %.5f\\n',trainRMSE(1),trainRMSE(2));

%% Compare fixed and adaptive LQI over the specified initial angles
angles=[20 23 24 25 26 27 28 29 30];
Ts=0.01; Tend=8; adaptEvery=5; gainBlend=0.25;
results=struct([]);
for ia=1:numel(angles)
    x0=[0;0;deg2rad(angles(ia));0];
    results(ia).angle_deg=angles(ia);
    results(ia).LQI=simulate_controller(x0,false,A0,B0,K0,Ki0,p, ...
        centres,sigma,W,Cint,Qaug,R,Ts,Tend,adaptEvery,gainBlend);
    results(ia).RBF_Adaptive_LQI=simulate_controller(x0,true,A0,B0,K0,Ki0,p, ...
        centres,sigma,W,Cint,Qaug,R,Ts,Tend,adaptEvery,gainBlend);
end

%% Plots
for ia=1:numel(angles)
    figure('Name',sprintf('RBF adaptive LQI: initial angle %g deg',angles(ia)));
    subplot(3,1,1);
    plot(results(ia).LQI.t,rad2deg(results(ia).LQI.x(3,:)),'--', ...
        results(ia).RBF_Adaptive_LQI.t,rad2deg(results(ia).RBF_Adaptive_LQI.x(3,:)), ...
        'LineWidth',1.2);
    grid on; ylabel('Angle [deg]'); legend('Fixed LQI','RBF-adaptive LQI','Location','best');
    title(sprintf('Initial angle = %g deg',angles(ia)));
    subplot(3,1,2);
    plot(results(ia).LQI.t(1:end-1),results(ia).LQI.u,'--', ...
        results(ia).RBF_Adaptive_LQI.t(1:end-1),results(ia).RBF_Adaptive_LQI.u,'LineWidth',1.2);
    grid on; ylabel('Force [N]'); legend('Fixed LQI','RBF-adaptive LQI','Location','best');
    subplot(3,1,3);
    plot(results(ia).RBF_Adaptive_LQI.t(1:end-1), ...
        results(ia).RBF_Adaptive_LQI.gainChange,'LineWidth',1.1);
    grid on; ylabel('||K-K_0||'); xlabel('Time [s]');
end

fprintf('\\nInitial angle | fixed LQI peak | adaptive LQI peak | fixed final | adaptive final (deg)\\n');
for ia=1:numel(angles)
    ef=rad2deg(results(ia).LQI.x(3,:));
    ea=rad2deg(results(ia).RBF_Adaptive_LQI.x(3,:));
    fprintf('%13.1f | %15.3f | %17.3f | %11.3f | %14.3f\\n', ...
        angles(ia),max(abs(ef)),max(abs(ea)),ef(end),ea(end));
end
end

function out=simulate_controller(x0,adaptive,A0,B0,K0,Ki0,p,centres,sigma,W, ...
    Cint,Qaug,R,Ts,Tend,adaptEvery,gainBlend)
N=round(Tend/Ts); out.t=(0:N)*Ts;
out.x=zeros(4,N+1); out.u=zeros(1,N);
out.residualNorm=zeros(1,N); out.gainChange=zeros(1,N); out.Knorm=zeros(1,N);
out.x(:,1)=x0; K=K0; Ki=Ki0; xi=0;
for k=1:N
    x=out.x(:,k); u=-K*x-Ki*xi;
    if adaptive && mod(k-1,adaptEvery)==0
        z=[x(3) x(4) x(1) x(2) u];
        [rhat,J]=rbf_residual_and_jacobian(z,centres,sigma,W);
        out.residualNorm(k)=norm(rhat);
        % J columns: [theta, theta_dot, x, x_dot, force].
        Ahat=A0; Bhat=B0;
        Ahat(2,:)=Ahat(2,:)+[J(1,3),J(1,4),J(1,1),J(1,2)];
        Ahat(4,:)=Ahat(4,:)+[J(2,3),J(2,4),J(2,1),J(2,2)];
        Bhat(2)=Bhat(2)+J(1,5); Bhat(4)=Bhat(4)+J(2,5);
        try
            [Kc,Kic]=design_lqi(Ahat,Bhat,Cint,Qaug,R);
            AaugHat=[Ahat zeros(4,1);-Cint 0]; BaugHat=[Bhat;0];
            Kblend=(1-gainBlend)*K+gainBlend*Kc;
            Kiblend=(1-gainBlend)*Ki+gainBlend*Kic;
            if all(isfinite(Kc)) && isfinite(Kic) && ...
                    all(real(eig(AaugHat-BaugHat*[Kc Kic]))<0) && ...
                    all(real(eig(AaugHat-BaugHat*[Kblend Kiblend]))<0)
                K=Kblend;
                Ki=Kiblend;
            end
        catch
            % Retain the last accepted gain if local LQI design fails.
        end
    end
    u=-K*x-Ki*xi;
    [~,xx]=ode45(@(t,s)nonlinear_ip_plant(t,s,u,p),[out.t(k) out.t(k+1)],x);
    out.x(:,k+1)=xx(end,:)'; xi=xi-x(1)*Ts;
    out.u(k)=u; out.gainChange(k)=norm(K-K0); out.Knorm(k)=norm(K);
end
end

function [K,Ki]=design_lqi(A,B,Cint,Qaug,R)
Aaug=[A zeros(4,1);-Cint 0]; Baug=[B;0];
Kaug=lqr(Aaug,Baug,Qaug,R); K=Kaug(1:4); Ki=Kaug(5);
end

function [A,B]=nominal_ip_model(p)
Den=(p.M+p.m)*p.I-(p.m*p.l)^2;
A=zeros(4); A(1,2)=1; A(2,3)=-p.m^2*p.g*p.l^2/Den;
A(3,4)=1; A(4,3)=(p.M+p.m)*p.m*p.g*p.l/Den;
B=[0;p.I/Den;0;-p.m*p.l/Den];
end

function dx=nonlinear_ip_plant(~,s,u,p)
xdot=s(2); theta=s(3); thetadot=s(4);
MM=[p.M+p.m,p.m*p.l*cos(theta);p.m*p.l*cos(theta),p.I];
rhs=[u+p.m*p.l*sin(theta)*thetadot^2;p.m*p.g*p.l*sin(theta)];
acc=MM\rhs; dx=[xdot;acc(1);thetadot;acc(2)];
end

function phi=rbf_basis(z,centres,sigma)
d=(centres-z)./sigma; phi=exp(-0.5*sum(d.^2,2));
end

function [r,J]=rbf_residual_and_jacobian(z,centres,sigma,W)
d=(centres-z)./sigma; phi=exp(-0.5*sum(d.^2,2));
r=(phi'*W)';
% d(phi_i)/d(z_j)=-phi_i*(z_j-centre_ij)/sigma_j^2.
dPhi=-phi.*((z-centres)./(sigma.^2));
J=W'*dPhi;
end
