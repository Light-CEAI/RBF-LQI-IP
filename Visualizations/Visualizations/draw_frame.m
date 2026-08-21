function draw_frame(ax, state, params, t)
%DRAW_FRAME Draw one frame of the cart-pendulum at a given state.
%
%   draw_frame(ax, state, params, t)
%   state : [x; xdot; theta; thetadot] (theta=0 is upright)

x     = state(1);
theta = state(3);

L = params.pend.L;
trackHalf = params.mech.track_length / 2;

cla(ax); hold(ax, 'on'); axis(ax, 'equal');

plot(ax, [-trackHalf, trackHalf], [0, 0], 'k-', 'LineWidth', 1.5);
plot(ax, [-trackHalf, -trackHalf], [-0.03, 0.03], 'r-', 'LineWidth', 2);
plot(ax, [ trackHalf,  trackHalf], [-0.03, 0.03], 'r-', 'LineWidth', 2);

cartW = 0.12; cartH = 0.06;
rectangle(ax, 'Position', [x-cartW/2, 0, cartW, cartH], ...
    'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'k', 'LineWidth', 1.2);

pivotX = x; pivotY = cartH;
tipX = pivotX + L*sin(theta);
tipY = pivotY + L*cos(theta);

plot(ax, [pivotX, tipX], [pivotY, tipY], 'r-', 'LineWidth', 3);
plot(ax, tipX, tipY, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
plot(ax, pivotX, pivotY, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

axLim = trackHalf + L + 0.1;
xlim(ax, [-axLim, axLim]);
ylim(ax, [-0.1, L + cartH + 0.1]);
grid(ax, 'on');
title(ax, sprintf('t = %.2f s   \\theta = %.1f deg   x = %.3f m', t, rad2deg(theta), x));
xlabel(ax, 'x [m]'); ylabel(ax, 'height [m]');

end