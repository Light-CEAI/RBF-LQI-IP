function run_project_demo()
% Executable smoke-test of established components; not final adaptive control.
clc; close all; cfg=demo_config();
fprintf('DEMO ONLY: parameters/scales/limits/Q/R are illustrative.\n');
[A,B,E]=linearize_cart_pendulum(cfg); disp(A); disp(B); disp(E);
report=verify_model_consistency(cfg); disp(report);
Q=diag([10 1 100 1 5]); R=1;
[K,info]=design_lqi_controller(A,B,Q,R);
disp('Illustrative LQI gain:'); disp(K); disp(info.closedLoopPoles);
z=[0;0;deg2rad(5);0;0]; phi=rbf_basis(z,cfg.rbf);
rhat=rbf_predict(phi,cfg.rbf.demoWeights); disp('Demo RBF output:'); disp(rhat);
X=[0;0;deg2rad(5);0]; [~,~,~,bp]=cart_pendulum_dynamics(X,0,cfg);
bq=[bp(2);bp(4)]; S=diag([1/cfg.projection.a_s^2,1/cfg.projection.alpha_s^2]);
[delta,ru,pinfo]=matched_unmatched_projection([0.05;0.02],bq,S);
fprintf('Demo matched force %.6g N; unmatched norm %.6g\n',delta,norm(ru));
disp(pinfo);
aout=actuator_limit_step(0,0.8,cfg.actuator,cfg.sim.Ts);
fprintf('Demo limited force %.6g N\n',aout.force);
fprintf('No adaptive update, robustifying term or UUB claim is included.\n');
end
