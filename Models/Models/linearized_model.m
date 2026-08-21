function [A, B, C, D] = linearized_model(params)
%LINEARIZED_MODEL Linearize the cart-pendulum about the upright equilibrium.
%
%   [A, B, C, D] = linearized_model(params)
%
%   Equilibrium: theta = 0, thetadot = 0, xdot = 0 (x free).
%   State:  X = [x; xdot; theta; thetadot]
%   Input:  u = actuator command (torque for Config A, force for Config B)
%   Output: C = eye(4) (full state), D = 0
%
%   This is the continuous-time small-angle linearization of the same
%   Lagrangian model used in nonlinear_plant.m.

m = params.pend.m;
l = params.pend.l;
L = params.pend.L;
g = params.pend.g;
I_p = (1/3) * m * L^2;

M = params.cart.M;
b = params.cart.b;

Den = (M + m)*I_p - (m*l)^2;   % determinant of the linearized mass matrix

A = zeros(4,4);
A(1,2) = 1;
A(2,2) = -I_p*b / Den;
A(2,3) =  (m^2 * g * l^2) / Den;
A(3,4) = 1;
A(4,2) =  (m*l*b) / Den;
A(4,3) =  ((M+m)*m*g*l) / Den;

B = zeros(4,1);
B(2) =  I_p / Den;
B(4) = -m*l / Den;

% B above is with respect to force F. Config A's actual input is torque
% (F = tau/r), so scale by 1/r via the chain rule.
if strcmp(params.config, 'A')
    B = B / params.cart.r;
end

C = eye(4);
D = zeros(4,1);
end