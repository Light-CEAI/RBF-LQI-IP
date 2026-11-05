function ThetaDot = adapt_rbf_weights_NOT_APPROVED(varargin)
% Deliberate guard: no adaptation law has passed the Lyapunov design gate.
error(['RBF adaptation is not implemented. First approve the regressor, ' ...
 'measurable error, projection operator, ideal-weight assumptions and ' ...
 'Lyapunov cancellation terms. Do not substitute a generic gradient law.']);
ThetaDot=[]; %#ok<UNRCH>
end
