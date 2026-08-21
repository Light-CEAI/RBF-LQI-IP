function [L, C_meas, info] = observer_design(Ad, params, method, varargin)
%OBSERVER_DESIGN Design a discrete-time state observer (Kalman or Luenberger).
%
%   [L, C_meas, info] = observer_design(Ad, params, method, ...)
%
%   Ad      : discrete-time A matrix (from discretize_model.m)
%   params  : struct from parameters.m
%   method  : 'kalman'     -> steady-state discrete Kalman gain (dlqe)
%             'luenberger' -> pole-placement observer (place)
%
%   Sensor model (assumed realistic instrumentation):
%       Measured directly : x (cart position, encoder), theta (angle,
%                            potentiometer/IMU)
%       Estimated         : xdot, thetadot
%   i.e.  C_meas = [1 0 0 0; 0 0 1 0]
%   Edit C_meas below if your rig also measures velocities directly.
%
%   --- method = 'kalman' ---
%   [L, C_meas, info] = observer_design(Ad, params, 'kalman', Qn, Rn)
%       Qn : process noise covariance (4x4), default 1e-4*I
%       Rn : measurement noise covariance (2x2), default 1e-3*I
%       Requires Control System Toolbox (dlqe).
%
%   --- method = 'luenberger' ---
%   [L, C_meas, info] = observer_design(Ad, params, 'luenberger', poles)
%       poles : 4 desired closed-loop observer poles (inside unit circle,
%               default = [0.55 0.60 0.65 0.70])
%       Requires Control System Toolbox (place).
%
%   info : struct with diagnostic fields (eigenvalues of (Ad - L*C_meas), etc.)

n = size(Ad,1);
C_meas = [1 0 0 0;
    0 0 1 0];

switch lower(method)
    case 'kalman'
        if numel(varargin) >= 1 && ~isempty(varargin{1})
            Qn = varargin{1};
        else
            Qn = eye(n) * 1e-4;
        end
        if numel(varargin) >= 2 && ~isempty(varargin{2})
            Rn = varargin{2};
        else
            Rn = eye(size(C_meas,1)) * 1e-3;
        end

        % dlqe(A, G, C, Qn, Rn) with G = I treats Qn as state-noise cov directly.
        [L, ~, ~] = dlqe(Ad, eye(n), C_meas, Qn, Rn);

    case 'luenberger'
        if numel(varargin) >= 1 && ~isempty(varargin{1})
            poles = varargin{1};
        else
            poles = [0.55, 0.60, 0.65, 0.70];   % stable, reasonably fast
        end
        if numel(poles) ~= n
            error('observer_design:poleCount', ...
                'Expected %d desired poles, got %d.', n, numel(poles));
        end
        L = place(Ad', C_meas', poles)';

    otherwise
        error('observer_design:badMethod', ...
            'method must be ''kalman'' or ''luenberger'', got ''%s''.', method);
end

% ---- Diagnostics --------------------------------------------------------
obsEig = eig(Ad - L*C_meas);
info.method = lower(method);
info.eig_observer = obsEig;
info.stable = all(abs(obsEig) < 1);

if ~info.stable
    warning('observer_design:unstable', ...
        'Observer error dynamics are not stable (|eig| >= 1). Reconsider Qn/Rn or pole placement.');
end

end