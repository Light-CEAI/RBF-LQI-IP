function rbf = rbf_train_offline(params, nSamples)
%RBF_TRAIN_OFFLINE Train the RBF to output an equivalent actuator command.
%   The target starts as the nonlinear angular-acceleration residual and
%   is divided by the exact local input gain d(theta_ddot)/d(u). Thus the
%   network output has the same units as the plant input: N*m for Config A
%   (motor torque) or N for Config B (direct force).
%
%   Centres retain the 3 x 2 x 5 x 2 (60-unit) grid, with velocity
%   locations moved closer to the normal operating region.
%   The training target uses the same viscous-friction model as the nominal
%   plant. No extra Coulomb friction is injected into this Seminar I model.
%
%   Run from the RBF folder:
%       cd RBF
%       params = parameters();
%       rbf = rbf_train_offline(params);

if nargin < 2 || isempty(nSamples),   nSamples   = 3000; end

[A, B] = linearized_model(params);
rbf = rbf_initialize(params);
rbf.weights = zeros(rbf.n_hidden, 1);

% Build a balanced 60-centre grid with better coverage near zero velocity.
% Keep the original 3 x 2 x 5 x 2 architecture and the existing widths.
theta_centres = linspace(params.rbf.theta_range(1), ...
    params.rbf.theta_range(2), 3);
thetadot_centres = [-1, 1];       % [rad/s], closer to normal operation
x_centres = linspace(-0.2, 0.2, 5); % [m], local cart operating region
xdot_centres = [-0.5, 0.5];       % [m/s], closer to normal operation

[TH, THD, XX, XD] = ndgrid(theta_centres, thetadot_centres, ...
    x_centres, xdot_centres);
rbf.centers = [TH(:), THD(:), XX(:), XD(:)];
rbf.n_hidden = size(rbf.centers, 1);
if rbf.n_hidden ~= params.rbf.n_hidden
    error('rbf_train_offline:centerCount', ...
        'Generated %d centres, but params.rbf.n_hidden is %d.', ...
        rbf.n_hidden, params.rbf.n_hidden);
end

% Train over the same local operating envelope covered by the centre grid.
ranges = [min(rbf.centers, [], 1)', max(rbf.centers, [], 1)'];

rng(1);
X = zeros(nSamples, 4);
for d = 1:4
    X(:,d) = ranges(d,1) + (ranges(d,2)-ranges(d,1))*rand(nSamples,1);
end

target = zeros(nSamples, 1);
u0 = 0;
for i = 1:nSamples
    % State order in X: [theta, thetadot, x, xdot]
    state = [X(i,3); X(i,4); X(i,1); X(i,2)]; % [x;xdot;theta;thetadot]

    dxdt_true   = nonlinear_plant(0, state, u0, params, 0);
    dxdt_linear = A*state + B*u0;
    accel_residual = dxdt_true(4) - dxdt_linear(4); % [rad/s^2]

    % Exact local angular-acceleration gain for the actuator command.
    theta = state(3);
    m = params.pend.m; l = params.pend.l; L = params.pend.L;
    Ip = (1/3) * m * L^2;
    detM = (params.cart.M + m)*Ip - (m*l*cos(theta))^2;
    if strcmp(params.config, 'A')
        input_gain = -(m*l*cos(theta)) / (params.cart.r * detM); % [rad/s^2]/[N*m]
    else
        input_gain = -(m*l*cos(theta)) / detM;                  % [rad/s^2]/[N]
    end
    if abs(input_gain) < 1e-8
        error('rbf_train_offline:singularInputGain', ...
            'Input gain is too small at training sample %d; cannot form an equivalent command.', i);
    end
    target(i) = accel_residual / input_gain;
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
outFile = fullfile(fileparts(mfilename('fullpath')), 'rbf_centers.mat');
save(outFile, 'centers', 'weights');

fit_err_std = std(Phi*rbf.weights - target);
target_std  = std(target);
fprintf('Offline RBF training complete. Output units: actuator command.\n');
fprintf('Fit residual std: %.4f (target std: %.4f, ratio: %.3f)\n', ...
    fit_err_std, target_std, fit_err_std/target_std);
fprintf('If ratio > 0.8, the basis functions are not covering the target well.\n');
fprintf('Consider increasing n_hidden or adjusting sigma values.\n');

end