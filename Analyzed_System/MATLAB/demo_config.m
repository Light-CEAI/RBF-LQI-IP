function cfg = demo_config()
% Illustrative software smoke-test values only; not physical design data.
cfg.M=0.50; cfg.m=0.15; cfg.l=0.30; cfg.Ic=0.0045;
cfg.I=cfg.Ic+cfg.m*cfg.l^2; cfg.g=9.81;
cfg.bx=0; cfg.btheta=0;
cfg.projection.a_s=0.08; cfg.projection.alpha_s=0.10;
cfg.actuator.forceMax=10; cfg.actuator.forceRateMax=100;
cfg.sim.Ts=0.01;
cfg.rbf.centres=zeros(3,5); cfg.rbf.width=1;
cfg.rbf.demoWeights=zeros(3,2);
end
