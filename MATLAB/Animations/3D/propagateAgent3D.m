function propagateAgent3D(x, y, z, start_index, divergence_index, obstacles, sensing_radius,video_writer)
figure(1);
hold on;

% Cleanup workspace
existing_path = findobj('Tag', 'projected_path');
delete(existing_path);
delete(findobj('Tag', 'sensor'));
delete(findobj('Tag', 'state'));
% Initialize obstacle handles
obstacle_handles = gobjects(size(obstacles, 1), 1);
for j = 1:size(obstacles, 1)
    % Color obstacles red initially
    obstacle_handles(j) = findobj('Tag', ['obstacle_', num2str(j)]);
end

% Initialize the agent as a scatter plot
marker_size = 50;
state = scatter3(x(start_index), y(start_index), z(start_index), marker_size, ...
                 'filled', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'black', 'Tag', 'state');

% Initialize path plots
path_plot = plot3(x(1:start_index), y(1:start_index), z(1:start_index), 'LineWidth', 2, 'Color', 'b', 'Tag', 'current_path');
projected_plot = plot3(x(start_index:end), y(start_index:end), z(start_index:end), 'LineWidth', 2, 'Color', 'b', 'LineStyle', '--', 'Tag', 'projected_path');

% Initialize sensor radius
sensor_plot = plot3(nan, nan, nan,'k--', 'Tag', 'sensor', 'LineWidth', 2, ...
        'DisplayName', 'Sensor Radius', 'HandleVisibility', 'on');
theta = linspace(0, 2*pi, 100);
% Iterate through the path
for i = start_index:divergence_index
    % Update agent's position and path
    set(state, 'XData', x(i), 'YData', y(i), 'ZData', z(i));
    set(sensor_plot, 'XData', x(i) + sensing_radius * cos(theta),'YData',y(i) + sensing_radius * sin(theta),'ZData', zeros(length(theta),1));
    set(path_plot, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z(1:i));
    set(projected_plot, 'XData', x(i:end), 'YData', y(i:end), 'ZData', z(i:end));

    for j = 1:size(obstacles, 1)
        obstacle_center = obstacles(j, 1:2);
        obstacle_radius = obstacles(j, 3);
        distance_to_agent = norm([x(i), y(i)] - obstacle_center);

        % Determine obstacle color
        if distance_to_agent - obstacle_radius <= sensing_radius
            if checkPathIntersection(x(i:end), y(i:end), obstacle_center, obstacle_radius+0.05, sensing_radius)
                color = 'r'; % Red
            else
                color = 'g'; % Green
            end
        else
            color = 'black'; % Default (not sensed)
        end

        % Update obstacle color
        set(obstacle_handles(j), 'FaceColor', color);
    end
    frame = getframe(gcf);
    writeVideo(video_writer, frame);
    % Pause for animation
    % pause(0.2);
end
end
