clc;
clear;
close all;

% Load paths
filename = 'Data/adaptive_small_2.txt';
[x_values, y_values] = extractPath(filename);
for i = 1:length(x_values)
    z_values{i} = zeros(size(x_values{i})); % Add a Z-dimension with constant 0
end
[divergence_index] = findDivergencePoint(x_values, y_values);

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
obstacles = plotObstacles3D('Data/obstacles_small_2.txt');
sensing_radius = 5; % Define sensing radius
start_index = 1;

% Loop through paths and propagate the agent
for i = 1:length(x_values)
    propagateAgent3D(x_values{i}, y_values{i}, z_values{i}, start_index, divergence_index(i), obstacles, sensing_radius);
    start_index = divergence_index(i);
end
