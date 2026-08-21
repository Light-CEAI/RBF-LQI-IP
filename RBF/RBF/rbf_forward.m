function [f_rbf, phi] = rbf_forward(rbf, inputVec)
%RBF_FORWARD Forward pass: compute the RBF network's compensation output.
%
%   [f_rbf, phi] = rbf_forward(rbf, inputVec)
%
%   rbf      : struct from rbf_initialize.m (or rbf_update.m)
%   inputVec : (4x1) or (1x4) = [theta; theta_dot; x; x_dot]
%
%   f_rbf : scalar compensation term, added to the LQI command:
%               u = u_lqi + f_rbf
%           (wire this in at the Simulations level, not inside
%           lqi_controller.m, to keep the linear controller and the
%           nonlinear compensator cleanly separable/testable)
%   phi   : (n_hidden x 1) basis activations, returned so rbf_update.m
%           can reuse them without recomputing (cheaper online update)

inputVec = inputVec(:)';   % row vector, 1 x n_inputs

n_hidden = rbf.n_hidden;
phi = zeros(n_hidden, 1);

for k = 1:n_hidden
    diff = inputVec - rbf.centers(k,:);          % 1 x n_inputs
    d2   = sum((diff ./ rbf.sigma).^2);          % normalized squared distance
    phi(k) = exp(-0.5 * d2);
end

f_rbf = rbf.weights' * phi;

end