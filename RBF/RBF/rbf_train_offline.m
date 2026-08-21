function rbf = rbf_train_offline(params, nSamples, mu_k_train)
%RBF_TRAIN_OFFLINE Pre-train RBF centers/weights against the combined
%   nonlinear plant residual, now including a Coulomb friction term so
%   the offline-trained weights have a meaningful starting point for the
%   online friction-compensation task.
%
%   mu_k_train: Coulomb friction coefficient to include in training target.
%               Default 0.08 (mid-range of your tested values).
%               Set to 0 to reproduce original pure-nonlinearity training.
%
%   Run from the RBF folder:
%       cd RBF
%       params = parameters();
%       rbf = rbf_train_offline(params);

if nargin < 2 || isempty(nSamples),   nSamples   = 3000; end
if nargin < 3 || isempty(mu_k_train), mu_k_train = 0.08; end

[A, B] = linearized_model(params);
rbf = rbf_initialize(params);
rbf.weights = zeros(rbf.n_hidden, 1);

ranges = [params.rbf.theta_range;
    params.rbf.thetadot_range;
    params.rbf.x_range;
    params.rbf.xdot_range];

rng(1);
X = zeros(nSamples, 4);
for d = 1:4
    X(:,d) = ranges(d,1) + (ranges(d,2)-ranges(d,1))*rand(nSamples,1);
end

totalMass = (params.cart.M + params.pend.m) * 9.81;
v_stiction = 0.02;
mu_s_train = mu_k_train * 1.5;

target = zeros(nSamples, 1);
u0 = 0;
for i = 1:nSamples
    % State order in X: [theta, thetadot, x, xdot]
    state = [X(i,3); X(i,4); X(i,1); X(i,2)]; % [x;xdot;theta;thetadot]

    % Coulomb friction force at this state
    xdot_i = X(i,4);
    if abs(xdot_i) > v_stiction
        Fd_coulomb = -mu_k_train * totalMass * sign(xdot_i);
    else
        Fd_coulomb = -min(abs(mu_s_train*totalMass), abs(u0)) * sign(u0);
    end

    dxdt_true   = nonlinear_plant(0, state, u0, params, Fd_coulomb);
    dxdt_linear = A*state + B*u0;
    target(i)   = dxdt_true(4) - dxdt_linear(4); % thetaddot residual
end

Phi = zeros(nSamples, rbf.n_hidden);
for i = 1:nSamples
    [~, phi] = rbf_forward(rbf, X(i,:));
    Phi(i,:) = phi';
end

lambda = 1e-3;
w = (Phi'*Phi + lambda*eye(rbf.n_hidden)) \ (Phi'*target);
rbf.weights = max(min(w, rbf.wbounds(2)), rbf.wbounds(1));

centers = rbf.centers;
weights = rbf.weights;
save('rbf_centers.mat', 'centers', 'weights');

fit_err_std = std(Phi*rbf.weights - target);
target_std  = std(target);
fprintf('Offline RBF training complete (mu_k_train=%.2f).\n', mu_k_train);
fprintf('Fit residual std: %.4f (target std: %.4f, ratio: %.3f)\n', ...
    fit_err_std, target_std, fit_err_std/target_std);
fprintf('If ratio > 0.8, the basis functions are not covering the target well.\n');
fprintf('Consider increasing n_hidden or adjusting sigma values.\n');

end