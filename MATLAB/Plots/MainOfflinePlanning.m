clc;
clear;
close all;
% Load Paths
filename = 'Data/PathRecalc2_3/adaptive_8.txt';
[x, y] = extractPath(filename);
q_init = [x{1}(1), y{1}(1)];
q_goal = [x{end}(end), y{end}(end)];
% Load Obstacles
obstacles = load('Data/PathRecalc2_3/obstacles_8.txt');


% Set Plot Parameters
font_size = 14;
marker_size = 100;
agent_marker_size = 60;
agent_color = [9, 219, 216] / 255;
goal_color = [24, 143, 17] / 255;
initial_path_color = 'b';
replanned_path_color = [204, 85, 0] / 255;
non_intersecting_obstacle_color = 'black';
sensing_radius = 100;

figure(1);
hold on;
%set(gcf, 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
xlabel('X', 'FontSize', font_size, 'FontWeight', 'bold');
ylabel('Y', 'FontSize', font_size, 'FontWeight', 'bold');
axis equal;
grid on;
xlim([-1 31]);
ylim([-1 31]);
title('Offline RRT Planning', 'FontSize', font_size, 'FontWeight', 'bold');

% Plot Obstacles
for j = 1:size(obstacles, 1)
    obstacle_center = obstacles(j, 1:2);
    obstacle_radius = obstacles(j, 3);
    theta = linspace(0, 2*pi, 100);
    if checkPathIntersection(x{end}, y{end}, obstacle_center, obstacle_radius, sensing_radius)
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

% Plot Agent and Path
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
    % Add Legend
    legend('Location', 'northwest','FontSize',14);

