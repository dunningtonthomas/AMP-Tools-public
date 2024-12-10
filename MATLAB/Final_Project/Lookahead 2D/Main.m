clc;
clear;
close all;
% Load Paths
filename = 'Data/PathPropagationRRT/adaptive_2.txt';
[x_values,y_values] = extractPath(filename);

% Load Planning Points
planning_points = load('Data/PathPropagationRRT/adaptive_flag_2.txt');
planning_points(end+1,:) = [x_values{end}(end),y_values{end}(end)];

% Find planning and divergence index
planning_index = findPlanningPoint(planning_points, x_values, y_values);
[divergence_index] = findDivergencePoint(x_values,y_values);
q_init = [x_values{1}(1),y_values{1}(1)];
q_goal = [x_values{end}(end),y_values{end}(end)];
sensing_radius = 5; % Define sensing radius for the agent

% Animate Trajectory
start_index = 1; 
figure(1);
hold on;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
marker_size = 100;
% Add labels and title for clarity
xlabel('X');
ylabel('Y');
axis equal; % Fix aspect ratio
axis tight;
axis manual; % Prevent automatic adjustment of axis limits
xlim([-5 35]);
ylim([-5 35]);
title(['Path ', num2str(1)]);
pause(1)
% Plot obstacles and get their data
obstacles = plotObstacles('Data/PathPropagationRRT/obstacles_2.txt');
% Loop to plot and update the paths
for i = 1:length(x_values)
    % Plot the current path with a specific tag
    if i==1
        % Plot the start and end goals
        scatter(q_init(1), q_init(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
        scatter(q_goal(1), q_goal(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    end
    % Propagate the agent along the current path
    %propagateAgent(x_values{i}, y_values{i}, start_index, planning_index, divergence_index, obstacles, sensing_radius,i);
    propagateAgentWithInterpolation(x_values{i}, y_values{i}, start_index, planning_index, divergence_index, obstacles, sensing_radius, i)
    % Update the start index for the next segment
    start_index = divergence_index(i);
    
    % Ensure updates are drawn
    drawnow;
end