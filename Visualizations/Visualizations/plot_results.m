function plot_results(results, metrics, titleStr)
%PLOT_RESULTS Single summary figure: theta, x, control signal, pass/fail.
%
%   plot_results(results, metrics)
%   plot_results(results, metrics, titleStr)

if nargin < 3, titleStr = ''; end

t = results.t;
theta_deg = rad2deg(results.x_true(3,:));
x = results.x_true(1,:);
tu = t(1:end-1);

fig = figure('Name', ['Summary: ' titleStr], 'Position', [100 100 900 600]);

subplot(3,1,1);
plot(t, theta_deg, 'b-', 'LineWidth', 1.3);
ylabel('\theta [deg]'); grid on;
if ~isempty(titleStr), title(titleStr); end

subplot(3,1,2);
plot(t, x, 'b-', 'LineWidth', 1.3);
ylabel('x [m]'); grid on;

subplot(3,1,3);
plot(tu, results.u, 'k-', 'LineWidth', 1.1);
ylabel('u'); xlabel('time [s]'); grid on;

if metrics.pass
    % replaced \u2014 with char(8212) which is the decimal code for an em-dash
    bannerStr = sprintf('PASS %s max|\\theta|=%.1f deg, settling=%.2fs', ...
        char(8212), metrics.max_theta_deg, metrics.settling_time_s);
    bannerColor = [0.85 1 0.85];
else
    bannerStr = sprintf('FAIL \u2014 %s', strjoin(metrics.fail_reasons, '; '));
    bannerColor = [1 0.85 0.85];
end
annotation(fig, 'textbox', [0.15, 0.95, 0.7, 0.04], 'String', bannerStr, ...
    'HorizontalAlignment', 'center', 'BackgroundColor', bannerColor, ...
    'EdgeColor', 'k', 'FitBoxToText', 'off');

end