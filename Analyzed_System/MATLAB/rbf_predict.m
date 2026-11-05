function rhat = rbf_predict(phi,Theta)
% Two-output acceleration-residual estimate [m/s^2; rad/s^2].
if size(Theta,1)~=numel(phi) || size(Theta,2)~=2
 error('Theta must be N-by-2 and match the basis count.');
end
rhat=Theta'*phi(:);
end
