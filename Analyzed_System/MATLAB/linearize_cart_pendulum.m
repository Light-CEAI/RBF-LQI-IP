function [A,B,E] = linearize_cart_pendulum(cfg)
% Upright fixed-parameter linearisation; theta=0 is upright.
M=cfg.M; m=cfg.m; l=cfg.l; I=cfg.I; D=(M+m)*I-(m*l)^2;
if D<=0, error('Nominal inertia determinant must be positive.'); end
A=zeros(4); A(1,2)=1; A(3,4)=1;
A(2,2)=-I*cfg.bx/D; A(2,3)=-m^2*cfg.g*l^2/D;
A(2,4)=m*l*cfg.btheta/D; A(4,2)=m*l*cfg.bx/D;
A(4,3)=(M+m)*m*cfg.g*l/D; A(4,4)=-(M+m)*cfg.btheta/D;
B=[0;I/D;0;-m*l/D];
E=[0 0;I/D -m*l/D;0 0;-m*l/D (M+m)/D];
end
