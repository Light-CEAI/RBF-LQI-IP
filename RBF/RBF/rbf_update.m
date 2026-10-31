function rbf = rbf_update(rbf, phi, error_signal)
%RBF_UPDATE Online gradient update of the RBF output weights, with leakage.
%
%   rbf = rbf_update(rbf, phi, error_signal)
%
%   Update rule (delta rule + leakage / e-modification, Narendra &
%   Annaswamy, 1987 [IEEE TAC, AC-32(2), 134-145]):
%       w_k <- (1 - lr*leak)*w_k + lr*error_signal*phi_k
%   followed by hard clipping to rbf.wbounds (+-10).
%
%   In this project the online error signal is cart-position tracking
%   error [m]. The learning rate therefore acts as a heuristic scaling
%   from position error to actuator-command weight change. This is an
%   empirical adaptation rule, not a derived Lyapunov update; no formal
%   closed-loop stability guarantee is claimed for the adaptive law.

rbf.weights = (1 - rbf.lr*rbf.leak) * rbf.weights + rbf.lr * error_signal * phi;
rbf.weights = max(min(rbf.weights, rbf.wbounds(2)), rbf.wbounds(1));

end