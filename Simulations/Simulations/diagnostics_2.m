params = parameters();
[A,B,C,D] = linearized_model(params);
[Ad,Bd,~,~] = discretize_model(A,B,C,D,params.embedded.Ts);
[K,Ki] = design_lqi(A,B,params);
[L,C_meas] = observer_design(Ad,params,'kalman');

mu_k = 0.12;
mu_s = mu_k*1.5;
v_stiction = 0.02;
totalMass = (params.cart.M+params.pend.m)*9.81;
coulomb_Fd = @(xdot,u) ternary(abs(xdot)>v_stiction, ...
    -mu_k*totalMass*sign(xdot), ...
    -min(abs(mu_s*totalMass),abs(u))*sign(u));

Ts=params.embedded.Ts; N=round(8/Ts); t=(0:N)*Ts;
x_true=zeros(4,N+1); xhat=zeros(4,N+1);
x_true(3,1)=deg2rad(10); xhat(3,1)=deg2rad(10);
xi_log=zeros(1,N+1);
actuator_limit=params.cart.tau_max; sat_tol=1e-9;

innov_log = zeros(2,N);
rbf_err_log = zeros(1,N);
tracking_err_log = zeros(1,N);
alpha = params.rbf.alpha_innovation;

rbfMatFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'RBF', 'rbf_centers.mat');
rbf = rbf_initialize(params,'host',rbfMatFile);

for kk=1:N
    tk=t(kk);
    y=C_meas*x_true(:,kk)+[params.sensor.x_quant_std;params.sensor.theta_quant_std].*randn(2,1);
    err=-xhat(1,kk);
    u_lqi=-K*xhat(:,kk)-Ki*xi_log(kk);
    rbfInput=[xhat(3,kk);xhat(4,kk);xhat(1,kk);xhat(2,kk)];
    [f_rbf,phi]=rbf_forward(rbf,rbfInput);
    u_unsat=u_lqi+f_rbf;
    u=max(min(u_unsat,actuator_limit),-actuator_limit);
    sat_high=u_unsat>u+sat_tol; sat_low=u_unsat<u-sat_tol;
    freeze=(sat_high&&err>0)||(sat_low&&err<0);
    xi_dot=0; if ~freeze, xi_dot=err; end
    Fd=coulomb_Fd(x_true(2,kk),u);
    [~,xx]=ode45(@(tt,xx) nonlinear_plant(tt,xx,u,params,Fd),[tk,tk+Ts],x_true(:,kk));
    x_true(:,kk+1)=xx(end,:)';
    [xhat(:,kk+1),innovation]=observer_update(xhat(:,kk),u,y,Ad,Bd,L,C_meas,params);
    xi_log(kk+1)=xi_log(kk)+xi_dot*Ts;

    tracking_err=0-xhat(1,kk+1);
    innov_log(:,kk)=innovation;
    tracking_err_log(kk)=tracking_err;
    rbf_err_log(kk)=(1-alpha)*tracking_err+alpha*innovation(2);
    rbf=rbf_update(rbf,phi,rbf_err_log(kk));
end

fprintf('=== Innovation Diagnostic (mu_k=0.12, alpha=%.1f) ===\n', alpha);
fprintf('Innovation(1) [x]:     mean=%.6f, std=%.6f, max=%.6f\n', ...
    mean(innov_log(1,:)), std(innov_log(1,:)), max(abs(innov_log(1,:))));
fprintf('Innovation(2) [theta]: mean=%.6f, std=%.6f, max=%.6f\n', ...
    mean(innov_log(2,:)), std(innov_log(2,:)), max(abs(innov_log(2,:))));
fprintf('Tracking err:          mean=%.6f, std=%.6f\n', ...
    mean(tracking_err_log), std(tracking_err_log));
fprintf('RBF error signal:      mean=%.6f, std=%.6f, max=%.6f\n', ...
    mean(rbf_err_log), std(rbf_err_log), max(abs(rbf_err_log)));
fprintf('Final ||weights||_2 = %.6f\n', norm(rbf.weights));
fprintf('Max |f_rbf| this run: not captured here -- check plot below\n');

figure('Name','Innovation Diagnostic');
subplot(3,1,1);
plot(t(1:N),innov_log(1,:),'b-',t(1:N),innov_log(2,:),'r-');
legend('innovation(1) x','innovation(2) theta'); ylabel('[units]'); grid on;
title('Observer innovation over time (mu_k=0.12)');
subplot(3,1,2);
plot(t(1:N),tracking_err_log,'b-',t(1:N),rbf_err_log,'r-');
legend('tracking error','combined RBF error'); grid on; ylabel('error signal');
subplot(3,1,3);
plot(t(1:N),vecnorm(rbf.weights*ones(1,N),2,1),'g-'); % weight norm (constant after loop)
ylabel('||weights||_2'); xlabel('time [s]'); grid on;

function out=ternary(cond,a,b), if cond, out=a; else, out=b; end, end