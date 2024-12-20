function propagateAgent(x, y, start_index, divergence_index, obstacles, sensing_radius)
    figure(1);
    hold on;

    % Pre-calculate circle geometry for sensor and obstacles
    theta = linspace(0, 2*pi, 100);

    % Pre-allocate obstacle handles for efficiency
    obstacle_handles = gobjects(size(obstacles, 1), 1);
    for j = 1:size(obstacles, 1)
        % Initialize all obstacles as red
        obstacle_center = obstacles(j, 1:2);
        obstacle_radius = obstacles(j, 3);
        x_circle = obstacle_center(1) + obstacle_radius * cos(theta);
        y_circle = obstacle_center(2) + obstacle_radius * sin(theta);
        obstacle_handles(j) = fill(x_circle, y_circle, 'r', ...
                                   'EdgeColor', 'none', 'FaceAlpha', 1.0, ...
                                   'Tag', ['obstacle_', num2str(j)]);
    end

    % Cleanup workspace
    existing_sensor = findobj('Tag', 'sensor');
    delete(existing_sensor);
    existing_path = findobj('Tag', 'projected_path');
    delete(existing_path);

    % Initialize agent's state as a scatter plot
    marker_size = 50;
    state = scatter(x(start_index), y(start_index), marker_size, 'filled', 'Tag', 'state', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'black');

    % Initialize path plot
    path = plot(x(1:start_index), y(1:start_index), 'LineWidth', 2, 'Color', 'b', 'Tag', 'current_path');
    projected_path = plot(x(start_index:end), y(start_index:end), 'LineWidth', 2, 'Color', 'b', 'LineStyle', '--', 'Tag', 'projected_path');
    % Initialize the sensor plot
    sensor_plot = plot(nan, nan, 'k--', 'Tag', 'sensor', 'LineWidth', 2); % Dashed circle

    % Iterate through the path up to the divergence point
    for i = start_index:divergence_index
        % Update the sensor circle
        sensor_x = x(i) + sensing_radius * cos(theta);
        sensor_y = y(i) + sensing_radius * sin(theta);

        % Update the agent's position and path
        set(state, 'XData', x(i), 'YData', y(i));
        set(sensor_plot, 'XData', sensor_x, 'YData', sensor_y);
        set(path, 'XData', x(1:i), 'YData', y(1:i));
        set(projected_path, 'XData', x(i:end), 'YData', y(i:end));

        % Update obstacle colors based on proximity
        for j = 1:size(obstacles, 1)
            obstacle_center = obstacles(j, 1:2);
            obstacle_radius = obstacles(j, 3);
            distance_to_agent = norm([x(i), y(i)] - obstacle_center);

            % Determine obstacle color
            if distance_to_agent - obstacle_radius <= sensing_radius
                if i == divergence_index && divergence_index ~= length(x)
                    if(checkPathIntersection(x(divergence_index:end), y(divergence_index:end), obstacle_center, obstacle_radius,sensing_radius))
                        color = 'r'; %red
                    else
                        color = 'g';
                    end
                else
                    color = 'g'; % Green
                end
            else
                color = 'black'; % yellow
            end

            % Update obstacle color
            set(obstacle_handles(j), 'FaceColor', color);
        end

        % Pause for animation
        pause(0.2);
    end
end
