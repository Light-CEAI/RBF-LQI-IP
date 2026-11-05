function [xdot,H,D,bp] = cart_pendulum_dynamics(X,Fa,cfg,dist,pdot)
%CART_PENDULUM_DYNAMICS Attached-body cart-pendulum dynamics.
% X=[x;xdot;theta;thetadot], Fa [N], dist=[dx;dtheta].
if nargin<4 || isempty(dist), dist=[0;0]; end
if nargin<5 || isempty(pdot), pdot=struct(); end
M=cfg.M; m=cfg.m; l=cfg.l; I=cfg.I;
Mdot=getv(pdot,'Mdot'); mdot=getv(pdot,'mdot');
ldot=getv(pdot,'ldot'); Idot=getv(pdot,'Idot');
vx=X(2); th=X(3); w=X(4); c=cos(th); s=sin(th);
H=[M+m,m*l*c;m*l*c,I]; D=det(H);
if D<=0, error('Inertia matrix is not positive definite.'); end
hv=[-m*l*s*w^2+cfg.bx*vx+(Mdot+mdot)*vx+(mdot*l+m*ldot)*w*c;
    -m*cfg.g*l*s+cfg.btheta*w+(mdot*l+m*ldot)*vx*c+Idot*w];
qdd=H\([Fa+dist(1);dist(2)]-hv);
xdot=[vx;qdd(1);w;qdd(2)];
bp=[0;I/D;0;-m*l*c/D];
end
function v=getv(s,n)
v=0; if isfield(s,n), v=s.(n); end
end
