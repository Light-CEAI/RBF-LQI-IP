function dxdt = nonlinear_plant(t, state, u_in, params, Fd)
%NONLINEAR_PLANT Full nonlinear equations of motion for the cart-pendulum.
%
%   dxdt = nonlinear_plant(t, state, u_in, params, Fd)
%
%   state = [x; xdot; theta; thetadot]   (theta = 0 is upright, unstable)
%   u_in  = actuator command:
%               Config A -> motor torque tau [N*m]
%               Config B -> input force u    [N]
%   Fd    = optional external disturbance force [N], default 0
%           (used by Simulations/test_disturbance.m)
%
%   Pendulum modeled as a uniform rod pivoted on the cart:
%       I_p = (1/3)*m*L^2   (moment of inertia about the pivot)
%   which is consistent with the datasheet's l = L/2 (COM at rod midpoint).
%
%   Derived via Lagrangian mechanics. Returns dxdt for use with ode45 or
%   a custom integrator.

if nargin < 5 || isempty(Fd)
    Fd = 0;
end

x       = state(1); %#ok<NASGU> (unused in dynamics, kept for clarity)
xdot    = state(2);
theta   = state(3);
thetadot = state(4);

m = params.pend.m;
l = params.pend.l;
L = params.pend.L;
g = params.pend.g;
I_p = (1/3) * m * L^2;     % = m*l^2 + I_cm for a uniform rod, l = L/2

M = params.cart.M;
b = params.cart.b;         % cart-side friction/damping (config A: b_w, config B: b)

% ---- Convert actuator command to a usable force ----------------------
switch params.config
    case 'A'
        F = u_in / params.cart.r;   % torque -> equivalent linear force
    case 'B'
        F = u_in;                   % already a force
end

F_total = F + Fd;

% ---- Mass matrix and RHS (from Euler-Lagrange) ------------------------
Mmat = [ (M + m),          m*l*cos(theta);
    m*l*cos(theta),  I_p             ];

rhs  = [ F_total - b*xdot + m*l*sin(theta)*thetadot^2;
    m*g*l*sin(theta) ];

acc = Mmat \ rhs;   % acc = [xddot; thetaddot]

dxdt = [ xdot;
    acc(1);
    thetadot;
    acc(2) ];
end