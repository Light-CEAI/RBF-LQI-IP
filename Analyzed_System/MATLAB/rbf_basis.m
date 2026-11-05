function phi = rbf_basis(z,rbf)
% Gaussian RBF basis; centres are N-by-nInputs.
if size(rbf.centres,2)~=numel(z), error('RBF input dimension mismatch.'); end
if rbf.width<=0, error('RBF width must be positive.'); end
d=rbf.centres-z(:)';
phi=exp(-sum(d.^2,2)/(2*rbf.width^2));
end
