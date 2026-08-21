function plot_control(results, titleStr)
%PLOT_CONTROL Plot actuator command, its LQI/RBF decomposition, and the
%   integral (LQI) state.
%
%   plot_control(results)
%   plot_control(results, titleStr)

if nargin < 2, titleStr = ''; end

t = results.t;
u = results.u;
tu = t(1:end-1); % u, u_lqi, f_rbf are logged per-step (length N), not N+1

figure('Name', ['Control: ' titleStr]);

subplot(3,1,1);
plot(tu, results.u_lqi, 'b-', 'LineWidth', 1.0); hold on;
plot(tu, results.f_rbf, 'g-', 'LineWidth', 1.0);
plot(tu, u, 'k-', 'LineWidth', 1.4);
legend('u_{lqi}', 'f_{rbf}', 'u (saturated, applied)', 'Location', 'best');
ylabel('command');
if ~isempty(titleStr), title(titleStr); end
grid on;

subplot(3,1,2);
plot(t, results.xi, 'm-');
ylabel('integral state \xi');
grid on;

if isfield(results, 'rbf_weights')
    subplot(3,1,3);
    plot(t, vecnorm(results.rbf_weights, 2, 1), 'r-');
    ylabel('||RBF weights||_2');
    xlabel('time [s]');
    grid on;
else
    subplot(3,1,3);
    xlabel('time [s]');
end

end