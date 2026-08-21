%% benchmark_control_step.m
% Benchmarks control_step.m execution time in MATLAB, as a feasibility
% check against the 100Hz (10ms) real-time budget. NOTE: this times the
% MATLAB-interpreted function, NOT the generated C running on actual
% hardware -- it is a sanity check, not a substitute for on-target
% profiling once codegen + flashing is done.

params = parameters();

if ~exist('control_consts.m', 'file')
    error('benchmark_control_step:missingConsts', 'Run generate_control_consts.m first.');
end
if ~exist('rbf_centers_const.m', 'file')
    error('benchmark_control_step:missingRBF', 'Run rbf_export_for_embedded.m first.');
end

c = control_consts();
rbf = rbf_initialize(params, 'embedded');

y = single([0.01; 0.05]);
u_prev = single(0);
xhat_prev = single([0.01; 0; 0.05; 0]);
xi_prev = single(0);
r = single(0);

N_CALLS = 5000;
times = zeros(N_CALLS, 1);

% Warm up (first call includes JIT compilation overhead, exclude from stats)
[~, ~, ~, ~, ~] = control_step(y, u_prev, xhat_prev, xi_prev, rbf, r, c);

for i = 1:N_CALLS
    tic;
    [u, xhat_prev, xi_prev, rbf, ~] = control_step(y, u_prev, xhat_prev, xi_prev, rbf, r, c);
    times(i) = toc;
    u_prev = u;
end

Ts_budget = params.embedded.Ts;
fprintf('--- control_step.m Timing (MATLAB-interpreted, NOT on-target) ---\n');
fprintf('Mean:   %.2f us\n', mean(times)*1e6);
fprintf('Median: %.2f us\n', median(times)*1e6);
fprintf('Max:    %.2f us\n', max(times)*1e6);
fprintf('99th pct: %.2f us\n', prctile(times,99)*1e6);
fprintf('Budget (Ts=%.0fms): %.0f us\n', Ts_budget*1000, Ts_budget*1e6);
fprintf('Mean utilization of budget: %.4f%%\n', 100*mean(times)/Ts_budget);
fprintf('\nNOTE: MATLAB interpreted execution is many times slower than\n');
fprintf('compiled C on a microcontroller -- this number being well under\n');
fprintf('budget is a necessary but NOT sufficient check. Once codegen is\n');
fprintf('run and flashed, re-profile execution time ON THE ACTUAL TARGET\n');
fprintf('(e.g. toggling a GPIO pin around the control_step() call and\n');
fprintf('measuring with an oscilloscope, or using the target''s own cycle\n');
fprintf('counter) before trusting real-time feasibility.\n');

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
save(fullfile(resultsDir, 'results_timing_benchmark.mat'), 'times', 'Ts_budget');