clc;
clear;
close all;

% Load Paths
filename = 'Data/PathRecalc2_3/adaptive_8.txt';
[x, y] = extractPath(filename);

% Load Planning Points
%planning_points = load('Data/PathPropagationRRT/adaptive_flag_3.txt');
planning_points = load('Data/PathRecalc2_3/adaptive_flag_8.txt');
planning_points(end+1, :) = [x{end}(end), y{end}(end)];

% Load Obstacles
obstacles = load('Data/PathRecalc2_3/obstacles_8.txt');


% Load Intermediate Goal Index
intGoal_index = load('Data/PathRecalc2_3/adaptive_goalIndex_8.txt');
intGoal_index = intGoal_index+1;
intGoal_index = [0;intGoal_index];
intGoal_index(end+1) = length(x{end});
% Find planning and divergence index
planning_index = findPlanningPoint(planning_points, x, y);
[divergence_index] = findDivergencePoint(x, y);
q_init = [x{1}(1), y{1}(1)];
q_goal = [x{end}(end), y{end}(end)];
sensing_radius = 100; % Define sensing radius for the agent

% Set Plot Parameters
font_size = 14;
marker_size = 100;
agent_marker_size = 60;
agent_color = [9, 219, 216] / 255;
goal_color = [24, 143, 17] / 255;
initial_path_color = 'b';
replanned_path_color = [204, 85, 0] / 255;
intersecting_obstacle_color = 'r';
non_intersecting_obstacle_color = 'black';

% Loop Through Each Path
for i = 1:length(x)+1
    figure(i);
    hold on;
    %set(gcf, 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
    xlabel('X', 'FontSize', font_size, 'FontWeight', 'bold');
    ylabel('Y', 'FontSize', font_size, 'FontWeight', 'bold');
    axis equal;
    grid on;
    xlim([-1 31]);
    ylim([-1 31]);
    title(['Path Planning with Obstacles (Step ', num2str(i), ')'], 'FontSize', font_size, 'FontWeight', 'bold');

    % Plot Obstacles
    % obstacles = plotObstacles('Data/PathPropagationRRT/obstacles_3.txt');
    if i ~= length(x)+1
        k = i;
    else
        k = i-1;
    end
        for j = 1:size(obstacles, 1)
            obstacle_center = obstacles(j, 1:2);
            obstacle_radius = obstacles(j, 3);
            theta = linspace(0, 2*pi, 100);
            if checkPathIntersection(x{k}, y{k}, obstacle_center, obstacle_radius, sensing_radius)
                x_circle = obstacle_center(1) + obstacle_radius * cos(theta);
                y_circle = obstacle_center(2) + obstacle_radius * sin(theta);
                % Highlight intersecting obstacles
                fill(x_circle, y_circle, intersecting_obstacle_color,'LineWidth',3 ,'EdgeColor', 'none', 'FaceAlpha', 1, 'HandleVisibility', 'off', 'Tag', ['obstacle_', num2str(i)]);
            else
                x_circle = obstacle_center(1) + (obstacle_radius-.1) * cos(theta);
                y_circle = obstacle_center(2) + (obstacle_radius-.1) * sin(theta);
                % Non-intersecting obstacles
                fill(x_circle, y_circle, non_intersecting_obstacle_color,'LineWidth',3 ,'EdgeColor', 'none', 'FaceAlpha', 1, 'HandleVisibility', 'off', 'Tag', ['obstacle_', num2str(i)]);
            end
        end

    % Plot Start and Goal Points
    scatter(q_init(1), q_init(2), marker_size, 'filled', ...
            'MarkerFaceColor', initial_path_color, ...
            'MarkerEdgeColor', 'k', 'DisplayName', 'Start', 'HandleVisibility', 'off');
    text(q_init(1), q_init(2), 'Start', ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
         'FontSize', font_size, 'FontWeight', 'bold', 'Color', initial_path_color);

    scatter(q_goal(1), q_goal(2), marker_size, 'filled', ...
            'MarkerFaceColor', goal_color, ...
            'MarkerEdgeColor', 'k', 'DisplayName', 'Goal', 'HandleVisibility', 'off');
    text(q_goal(1), q_goal(2), 'Goal', ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ...
         'FontSize', font_size, 'FontWeight', 'bold', 'Color', goal_color);

    % Plot Agent and Paths
    if i == 1
        % Plot agent at initial location
        scatter(q_init(1), q_init(2), agent_marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent', 'HandleVisibility', 'off');
        text(q_init(1), q_init(2), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);
        line_style = ':';
        % Plot Initial Path
        plot(x{1}, y{1}, 'LineWidth', 2, 'Color', initial_path_color,'LineStyle', line_style, 'DisplayName', 'Initial Projected Path');
    elseif i <= length(x)
        % Plot agent location
        scatter(x{i}(planning_index(i-1)), y{i}(planning_index(i-1)), agent_marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent', 'HandleVisibility', 'off');
        text(x{i}(planning_index(i-1)), y{i}(planning_index(i-1)), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);

        % Plot Path Traveled
        line_style = '-';
        plot(x{i}(1:planning_index(i-1)), y{i}(1:planning_index(i-1)), 'LineWidth', 2, 'Color', initial_path_color,'LineStyle', line_style, 'DisplayName', 'Path Traveled');
        % Plot New Path
        new_index = find(x{i-1}(intGoal_index(i)) ==x{i});
        plot(x{i}(planning_index(i-1):new_index), y{i}(planning_index(i-1):new_index), 'LineWidth', 2, ...
             'Color', replanned_path_color, ...
             'DisplayName', ['Replanned Projected Path ', num2str(i)]);
        % Plot Current Projected 
        plot(x{i-1}(intGoal_index(i):end), y{i-1}(intGoal_index(i):end), 'LineWidth', 2,'LineStyle',':',...
        'Color', 'b','DisplayName', 'Initial Projected Path');
    else
        % Plot Agent at the Goal with a Label
        scatter(q_goal(1), q_goal(2), agent_marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent', 'HandleVisibility', 'off');
        text(q_goal(1), q_goal(2), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);
        % Final Path
        plot(x{end}, y{end}, 'LineWidth', 2 ,'LineStyle','-',...
             'Color', 'b', ...
             'DisplayName', 'Final Path');
    end
    % Add Legend
    legend('Location', 'northwest','FontSize',14);
end
