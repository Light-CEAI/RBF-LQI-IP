function [Ad, Bd, Cd, Dd] = discretize_model(A, B, C, D, Ts)
%DISCRETIZE_MODEL Zero-order-hold discretization of a continuous SS model.
%
%   [Ad, Bd, Cd, Dd] = discretize_model(A, B, C, D, Ts)
%
%   Implemented via the augmented matrix exponential method, so it has no
%   dependency on the Control System Toolbox's c2d:
%
%       [Ad Bd]   = expm( [A*Ts  B*Ts] )
%       [0   I ]          [0     0   ]
%
%   Ts = sample period [s]

n = size(A,1);
p = size(B,2);

Maug = [A*Ts, B*Ts; zeros(p, n+p)];
Mexp = expm(Maug);

Ad = Mexp(1:n, 1:n);
Bd = Mexp(1:n, n+1:end);
Cd = C;
Dd = D;
end