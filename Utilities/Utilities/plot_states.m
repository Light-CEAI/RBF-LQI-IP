function plot_states(results, titleStr)
%PLOT_STATES Plot true state, estimated state, and estimation error.
%
%   plot_states(results)
%   plot_states(results, titleStr)

if nargin < 2, titleStr = ''; end

t = results.t;
x_true = results.x_true;
xhat   = results.xhat;

labels = {'x [m]', 'xdot [m/s]', 'theta [deg]', 'thetadot [deg/s]'};
scale  = [1, 1, rad2deg(1), rad2deg(1)]; % convert angle rows to degrees for readability

figure('Name', ['States: ' titleStr]);
for i = 1:4
    subplot(4,1,i);
    plot(t, x_true(i,:)*scale(i), 'b-', 'LineWidth', 1.2); hold on;
    plot(t, xhat(i,:)*scale(i), 'r--', 'LineWidth', 1.0);
    ylabel(labels{i});
    if i == 1
        legend('true', 'estimate', 'Location', 'best');
        if ~isempty(titleStr), title(titleStr); end
    end
    if i == 4, xlabel('time [s]'); end
    grid on;
end

% ---- Estimation error subplot would be a 5th panel; kept separate to
% avoid cluttering the main 4-state view:
figure('Name', ['Estimation error: ' titleStr]);
err = x_true - xhat;
for i = 1:4
    subplot(4,1,i);
    plot(t, err(i,:)*scale(i), 'k-');
    ylabel(['err ' labels{i}]);
    if i == 4, xlabel('time [s]'); end
    grid on;
end

end