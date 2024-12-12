clc;
clear;
close all;

% Load paths
filename = 'Animation_NoLookahead/adaptive_1.txt';
[x_values, y_values] = extractPath(filename);
for i = 1:length(x_values)
    z_values{i} = zeros(size(x_values{i})); % Add a Z-dimension with constant 0
end
[divergence_index] = findDivergencePoint(x_values, y_values);
q_init = [x_values{1}(1), y_values{1}(1),z_values{1}(1)];
q_goal = [x_values{end}(end), y_values{end}(end),z_values{end}(end)];
% Set up video writer
video_filename = '3DPathAnimation1.avi';
v = VideoWriter(video_filename, 'Motion JPEG AVI');
v.FrameRate = 60; % Set frame rate
open(v);
% Initialize figure
figure(1);
hold on;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Set fixed axis limits to avoid resizing
xlim([-5, 35]);
ylim([-5, 35]);
zlim([0, 10]);

% Set 3D camera view with a tilted perspective
camera_position = [0, 0, 50]; % Camera position (X, Y, Z)
camera_target = [15, 15, 10];    % Camera target (X, Y, Z)
campos(camera_position);         % Set camera position
camtarget(camera_target);        % Set camera target
camup([0, 0, 1]);                % Ensure the Z-axis is up
camva(45);                       % Set camera view angle (field of view)

% Plot obstacles and animate the trajectory
obstacles = plotObstacles3D('Animation_NoLookahead/obstacles_1.txt',x_values,y_values);
sensing_radius = 5; % Define sensing radius
start_index = 1;
marker_size = 100;
% Plot the start and end goals
scatter(q_init(1), q_init(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'HandleVisibility', 'off');
scatter(q_goal(1), q_goal(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'HandleVisibility', 'off');

% Loop through paths and propagate the agent
for i = 1:length(x_values)
    propagateAgent3D(x_values{i}, y_values{i}, z_values{i}, start_index, divergence_index(i), obstacles, sensing_radius,v);
    start_index = divergence_index(i);
    for j =1:30
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
end
% Capture the final frame
frame = getframe(gcf);
writeVideo(v, frame);
% Close the video writer
close(v);
disp(['Video saved as ', video_filename]);