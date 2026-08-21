function create_video(results, params, outFile, fps)
%CREATE_VIDEO Render a simulation_main.m results struct to an .avi/.mp4 file.
if nargin < 4 || isempty(fps), fps = 30; end

t = results.t;
x_true = results.x_true;
Ts = results.scenario.Ts;

% CRITICAL FIX 1: Ensure the destination directory actually exists
fileDir = fileparts(outFile);
if ~isempty(fileDir) && ~exist(fileDir, 'dir')
    mkdir(fileDir);
end

% Initialize the video writer
vw = VideoWriter(outFile, 'Motion JPEG AVI');
vw.FrameRate = fps;

% Try opening the file explicitly
try
    open(vw);
catch ME
    error('Failed to open VideoWriter file at "%s". Reason: %s', outFile, ME.message);
end

% CRITICAL FIX 2: Double check that it actually opened successfully
if ~isOpen(vw)
    error('VideoWriter failed to open. Check your write permissions or file path: %s', outFile);
end

fig = figure('Name', 'Rendering video...', 'Visible', 'off');
ax = axes(fig);
frameStep = max(1, round(1/(fps*Ts)));

for kk = 1:frameStep:numel(t)
    % Extra safety check inside the loop
    if ~isOpen(vw)
        break;
    end

    draw_frame(ax, x_true(:,kk), params, t(kk));
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);
close(fig);
fprintf('Video saved: %s (%d frames at %d fps)\n', outFile, numel(1:frameStep:numel(t)), fps);
end