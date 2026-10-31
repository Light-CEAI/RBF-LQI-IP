function rbf = rbf_initialize(params, mode, centersMatFile)
%RBF_INITIALIZE Build (or load) the RBF network used to augment the LQI law.
%
%   rbf = rbf_initialize(params, 'host')                       (default)
%   rbf = rbf_initialize(params, 'embedded')   <- uses rbf_centers_const.m,
%                                                  NO file I/O, codegen-safe

if nargin < 2 || isempty(mode), mode = 'host'; end

switch mode
    case 'embedded'
        [centers, weights] = rbf_centers_const();  %#ok — generated, no load()
        centers = double(centers);
        weights = double(weights);

    case 'host'
        if nargin < 3 || isempty(centersMatFile)
            centersMatFile = fullfile(fileparts(mfilename('fullpath')), 'rbf_centers.mat');
        end
        if exist(centersMatFile, 'file')
            loaded = load(centersMatFile);
            centers = loaded.centers;
            weights = loaded.weights;
        else
            warning('rbf_initialize:noFile', ...
                '%s not found. Using untrained grid centers.', centersMatFile);
            ranges = [params.rbf.theta_range; params.rbf.thetadot_range; ...
                params.rbf.x_range;     params.rbf.xdot_range];
            centers = local_grid_centers(ranges, params.rbf.n_hidden);
            weights = zeros(params.rbf.n_hidden, 1);
        end

    otherwise
        error('rbf_initialize:badMode', 'mode must be ''host'' or ''embedded''.');
end

rbf.centers  = centers;
rbf.sigma    = [params.rbf.sigma_theta, params.rbf.sigma_thetadot, 0.15, 0.50];
rbf.weights  = weights;
rbf.n_hidden = params.rbf.n_hidden;
rbf.n_inputs = params.rbf.n_inputs;
rbf.lr       = params.rbf.learning_rate;
rbf.wbounds  = params.rbf.weight_bounds;
rbf.leak     = params.rbf.leakage;

if strcmp(mode, 'embedded')
    rbf.sigma   = single(rbf.sigma);
    rbf.lr      = single(rbf.lr);
    rbf.wbounds = single(rbf.wbounds);
    rbf.leak    = single(rbf.leak);
end

end

% ------------------------------------------------------------------------
function centers = local_grid_centers(ranges, n_hidden)
%LOCAL_GRID_CENTERS Lay out n_hidden centers via stratified sampling
%   across the 4D operating box. Only used as a fallback if the
%   centersMatFile is missing entirely.

n_inputs = size(ranges,1);
rng(0); % reproducible default layout for the untrained fallback

centers = zeros(n_hidden, n_inputs);
for d = 1:n_inputs
    edges = linspace(ranges(d,1), ranges(d,2), n_hidden+1);
    mids  = (edges(1:end-1) + edges(2:end)) / 2;
    centers(:,d) = mids(randperm(n_hidden));
end
end