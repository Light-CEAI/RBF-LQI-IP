function animate_pendulum(results, params, playbackSpeed)
%ANIMATE_PENDULUM Play back a simulation_main.m results struct live.
if nargin < 3 || isempty(playbackSpeed), playbackSpeed = 1; end

fig = figure('Name', 'Pendulum Animation');
ax = axes(fig);

t = results.t;
x_true = results.x_true;
Ts = results.scenario.Ts;
frameSkip = max(1, round(0.02 / Ts)); % aim for ~50 fps of simulated time

for kk = 1:frameSkip:numel(t)
    % CRITICAL FIX: Check if the user closed the figure window
    if ~isgraphics(ax)
        fprintf('Animation window closed by user.\n');
        break; 
    end

    draw_frame(ax, x_true(:,kk), params, t(kk));
    drawnow;
    pause(Ts * frameSkip / playbackSpeed);
end
end