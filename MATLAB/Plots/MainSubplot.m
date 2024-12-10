clc;
clear;
close all;

% Load Paths
filename = 'Data/PathPropagationRRT/adaptive_3.txt';
[x, y] = extractPath(filename);

% Load Planning Points
planning_points = load('Data/PathPropagationRRT/adaptive_flag_3.txt');
planning_points(end+1, :) = [x{end}(end), y{end}(end)];

% Find planning and divergence index
planning_index = findPlanningPoint(planning_points, x, y);
[divergence_index] = findDivergencePoint(x, y);
q_init = [x{1}(1), y{1}(1)];
q_goal = [x{end}(end), y{end}(end)];
sensing_radius = 100; % Define sensing radius for the agent

% Set Plot Parameters
font_size = 14;
marker_size = 100;
agent_color = [9, 219, 216] / 255;
goal_color = [24, 143, 17] / 255;
initial_path_color = 'b';
replanned_path_color = [0.8, 0, 0.8];
intersecting_obstacle_color = 'r';
non_intersecting_obstacle_color = 'black';

% Loop Through Each Path
for i = 1:length(x)
    figure(i);
    hold on;
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
    xlabel('X', 'FontSize', font_size, 'FontWeight', 'bold');
    ylabel('Y', 'FontSize', font_size, 'FontWeight', 'bold');
    axis equal;
    grid on;
    xlim([-5 35]);
    ylim([-5 35]);
    title(['Path Planning with Obstacles (Step ', num2str(i), ')'], 'FontSize', font_size, 'FontWeight', 'bold');

    % Plot Obstacles
    obstacles = plotObstacles('Data/PathPropagationRRT/obstacles_3.txt');
    for j = 1:size(obstacles, 1)
        obstacle_center = obstacles(j, 1:2);
        obstacle_radius = obstacles(j, 3);
        theta = linspace(0, 2*pi, 100);
        x_circle = obstacle_center(1) + obstacle_radius * cos(theta);
        y_circle = obstacle_center(2) + obstacle_radius * sin(theta);

        if checkPathIntersection(x{i}, y{i}, obstacle_center, obstacle_radius, sensing_radius)
            % Highlight intersecting obstacles
            fill(x_circle, y_circle, intersecting_obstacle_color, ...
                 'EdgeColor', intersecting_obstacle_color, ...
                 'LineWidth', 2.0, 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        else
            % Non-intersecting obstacles
            fill(x_circle, y_circle, non_intersecting_obstacle_color, ...
                 'EdgeColor', 'none', ...
                 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        end
    end

    % Plot Start and Goal Points
    scatter(q_init(1), q_init(2), marker_size, 'filled', ...
            'MarkerFaceColor', initial_path_color, ...
            'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
    text(q_init(1), q_init(2), 'Start', ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
         'FontSize', font_size, 'FontWeight', 'bold', 'Color', initial_path_color);

    scatter(q_goal(1), q_goal(2), marker_size, 'filled', ...
            'MarkerFaceColor', goal_color, ...
            'MarkerEdgeColor', 'k', 'DisplayName', 'Goal');
    text(q_goal(1), q_goal(2), 'Goal', ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
         'FontSize', font_size, 'FontWeight', 'bold', 'Color', goal_color);

    % Plot Agent and Paths
    if i == 1
        scatter(q_init(1), q_init(2), marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent');
        text(q_init(1), q_init(2), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);
        line_style = '-';
    elseif i < length(x)
        scatter(x{i}(planning_index(i)), y{i}(planning_index(i)), marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent');
        text(x{i}(planning_index(i)), y{i}(planning_index(i)), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);
        plot(x{i}, y{i}, 'LineWidth', 2, ...
             'Color', replanned_path_color, ...
             'DisplayName', ['Replanned Path ', num2str(i)]);
        line_style = '--';
    else
        scatter(q_goal(1), q_goal(2), marker_size, 'filled', ...
                'MarkerFaceColor', agent_color, ...
                'MarkerEdgeColor', 'k', 'DisplayName', 'Agent');
        text(q_goal(1), q_goal(2), 'Agent', ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
             'FontSize', font_size, 'FontWeight', 'bold', 'Color', agent_color);
        plot(x{i}, y{i}, 'LineWidth', 2, ...
             'Color', replanned_path_color, ...
             'DisplayName', 'Final Path');
        line_style = '--';
    end

    % Plot Initial Path
    plot(x{1}, y{1}, 'LineWidth', 2, ...
         'Color', initial_path_color, ...
         'LineStyle', line_style, 'DisplayName', 'Initial Path');

    % Add Legend
    legend('Location', 'northeastoutside');
end
