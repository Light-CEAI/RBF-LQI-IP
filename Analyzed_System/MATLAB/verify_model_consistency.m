function report = verify_model_consistency(cfg)
% Finite-difference check of upright A and B against nonlinear dynamics.
[A,B,E]=linearize_cart_pendulum(cfg); X=zeros(4,1); h=1e-6;
An=zeros(4); Bn=zeros(4,1);
for k=1:4
 dx=zeros(4,1); dx(k)=h;
 fp=cart_pendulum_dynamics(X+dx,0,cfg);
 fm=cart_pendulum_dynamics(X-dx,0,cfg);
 An(:,k)=(fp-fm)/(2*h);
end
fp=cart_pendulum_dynamics(X,h,cfg); fm=cart_pendulum_dynamics(X,-h,cfg);
Bn=(fp-fm)/(2*h);
report.AError=norm(A-An,'fro'); report.BError=norm(B-Bn);
report.maxError=max(report.AError,report.BError); report.pass=report.maxError<1e-5;
report.E=E;
fprintf('Jacobian errors: A %.3e, B %.3e; pass=%d\n',report.AError,report.BError,report.pass);
end
