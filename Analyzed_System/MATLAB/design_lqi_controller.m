function [K,info] = design_lqi_controller(A,B,Q,R)
% Continuous nominal LQI. Requires Control System Toolbox.
n=size(A,1); Cx=[1 zeros(1,n-1)];
Aa=[A zeros(n,1);-Cx 0]; Ba=[B;0];
K=lqr(Aa,Ba,Q,R);
info.Aa=Aa; info.Ba=Ba; info.closedLoopPoles=eig(Aa-Ba*K);
info.controllabilityRank=rank(ctrb(Aa,Ba));
info.note='Nominal synthesis only; weights and parameters require justification.';
end
