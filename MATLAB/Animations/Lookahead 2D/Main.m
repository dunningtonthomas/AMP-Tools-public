clc;
clear;
close all;

% Load Paths
filename = 'Data/Animation_Lookahead/adaptive_3.txt';
[x_values, y_values] = extractPath(filename);

% Load Planning Points
planning_points = load('Data/Animation_Lookahead/adaptive_flag_3.txt');
planning_points(end+1, :) = [x_values{end}(end), y_values{end}(end)];

% Find planning and divergence index
planning_index = findPlanningPoint(planning_points, x_values, y_values);
[divergence_index] = findDivergencePoint(x_values, y_values);
q_init = [x_values{1}(1), y_values{1}(1)];
q_goal = [x_values{end}(end), y_values{end}(end)];
sensing_radius = 5; % Define sensing radius for the agent

% Adjust Obstacle to reflect the tolerances
obstacles = adjustObstacles('Data/Animation_Lookahead/obstacles_3.txt',x_values,y_values);
% Set up video writer
video_filename = 'DynamicPathAnimation3.avi';
v = VideoWriter(video_filename, 'Motion JPEG AVI');
v.FrameRate = 60; % Set frame rate
open(v);

% Initialize Figure
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
xlim([-1 31]);
ylim([-1 31]);
title('Initial Path','FontSize',20);
pause(1);

% Plot obstacles and get their data
%obstacles = plotObstacles('Data/PathPropagationRRT/obstacles_2.txt');

% Loop to plot and update the paths
start_index = 1;
for i = 1:length(x_values)
    % Plot the current path with a specific tag
    if i == 1
        % Plot the start and end goals
        scatter(q_init(1), q_init(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'HandleVisibility', 'off');
        scatter(q_goal(1), q_goal(2), marker_size, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'HandleVisibility', 'off');
    end
    
    % Propagate the agent along the current path
    delete(findobj('Tag', 'state'));
    propagateAgentWithInterpolation(x_values{i}, y_values{i}, start_index, planning_index, divergence_index, obstacles, sensing_radius, i,v);
    
    % Update the start index for the next segment
    start_index = divergence_index(i);

    % Capture the frame and write it to the video
    frame = getframe(gcf); % Capture current figure
    writeVideo(v, frame); % Write frame to video
end

% Update title to indicate final path
title('Final Path','FontSize',20);

% Capture the final frame
frame = getframe(gcf);
writeVideo(v, frame);

% Close the video writer
close(v);

disp(['Video saved as ', video_filename]);
