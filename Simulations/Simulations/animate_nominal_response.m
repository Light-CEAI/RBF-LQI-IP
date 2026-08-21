%% animate_nominal_response.m
% Renders a dynamic 2D visual animation of the cart-pendulum system
% using the saved true trajectory states from your nominal test workspace.

clear; clc;

%% 1. Load Simulation Data
resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
nominalPath = fullfile(resultsDir, 'results_nominal.mat');

if ~exist(nominalPath, 'file')
    error('Could not locate nominal results data at: %s\nRun your main simulation first.', nominalPath);
end

load(nominalPath); % Loads 'results' and 'metrics' structure matrices

t = results.t;
x_states = results.x_true; % Matrix dimensions: [4 x N]

cart_x     = x_states(1, :);   % Cart position (m)
pend_theta = x_states(3, :);   % Pendulum angle (rad) - 0 is vertical upright

% Physical dimensions for rendering scale
L_rod = 0.5; % Visual length of the pendulum rod (m)

%% 2. Configure Graphics Canvas Geometry
fig = figure('Name', 'Inverted Pendulum Dynamic Balancing Loop', ...
    'Position', [200, 200, 850, 450], 'Color', [1 1 1]);

% Determine track visualization limits dynamically based on cart travel
min_x = min(cart_x) - 0.5;
max_x = max(cart_x) + 0.5;

ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
set(ax, 'XLim', [min_x, max_x], 'YLim', [-0.2, L_rod + 0.3], 'DataAspectRatio', [1 1 1]);
xlabel(ax, 'Track Position [m]', 'FontWeight', 'bold');
ylabel(ax, 'Height [m]', 'FontWeight', 'bold');
title(ax, 'Closed-Loop Mechanical Balance Profile (Nominal Evaluation)', 'FontSize', 12);

% Draw Ground Surface Track Reference Line
line([min_x, max_x], [0, 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'Parent', ax);

% Initialize Graphic Handles with Dummy Data for Performance Buffering
hCart = plot(ax, 0, 0, 'rs', 'MarkerSize', 18, 'MarkerFaceColor', [0.85 0.15 0.15], 'LineWidth', 1.5);
hRod  = plot(ax, [0, 0], [0, L_rod], 'Color', [0.07 0.20 0.40], 'LineWidth', 3.5);
hTip  = plot(ax, 0, L_rod, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', [0 0 0]);

% Setup On-Screen Real-Time Data Overlay Text
hText = text(min_x + 0.05, L_rod + 0.2, '', 'FontName', 'monospace', 'FontSize', 10, 'Parent', ax);

%% 3. Dynamic Playback Integration Loop
% Downsample slightly if time steps are tight, ensuring a smooth 30-50 FPS look
sampleStride = max(1, round(length(t) / 300)); 

% Measure simulation step execution delays to sync visualization speed
dt_sim = t(2) - t(1);
playback_delay = dt_sim * sampleStride;

for k = 1:sampleStride:length(t)
    % Bail out safely if the user closes the window mid-simulation
    if ~ishandle(fig)
        break;
    end

    % Capture instantaneous state parameters
    curr_x  = cart_x(k);
    curr_th = pend_theta(k);

    % Kinematics Mapping: Compute spatial coordinate updates for the rod tip
    % theta = 0 points straight up; clockwise deflection is positive
    bob_x = curr_x + L_rod * sin(curr_th);
    bob_y = L_rod * cos(curr_th);

    % Update graphic handles directly (avoids standard 'plot' redraw overhead)
    set(hCart, 'XData', curr_x, 'YData', 0);
    set(hRod,  'XData', [curr_x, bob_x], 'YData', [0, bob_y]);
    set(hTip,  'XData', bob_x, 'YData', bob_y);

    % Update data string overlay
    set(hText, 'String', sprintf('Time:  %5.2f s\nAngle: %5.1f deg\nPos:   %5.2f m', ...
        t(k), rad2deg(curr_th), curr_x));

    % Flush graphics queue and match physical time pacing
    drawnow;
    pause(playback_delay);
end

disp('Animation execution completed successfully.');