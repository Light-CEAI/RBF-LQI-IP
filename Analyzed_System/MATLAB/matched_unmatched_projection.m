function [delta,ru,info] = matched_unmatched_projection(rq,bq,S)
% Weighted least-squares projection onto the scalar cart-force direction.
rq=rq(:); bq=bq(:);
if numel(rq)~=2 || numel(bq)~=2 || ~isequal(size(S),[2 2])
 error('rq and bq must have length 2; S must be 2-by-2.');
end
if min(eig((S+S')/2))<=0, error('S must be positive definite.'); end
den=bq'*S*bq; if den<=eps, error('Projection denominator is not positive.'); end
delta=(bq'*S*rq)/den; ru=rq-bq*delta;
info.denominator=den; info.weightedOrthogonality=bq'*S*ru;
info.reconstructionError=norm(rq-bq*delta-ru);
end
