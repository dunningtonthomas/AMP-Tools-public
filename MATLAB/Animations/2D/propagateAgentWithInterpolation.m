function propagateAgentWithInterpolation(x, y, start_index, divergence_index, obstacles, sensing_radius, plan_num,video_writer)
    figure(1);
    hold on;
    title('Waypoint Tracking')
    % Declare persistent variables to store obstacle handles
    persistent obstacle_handles;
    % Initialize obstacle handles if not already initialized
    theta = linspace(0, 2*pi, 100);
    if isempty(obstacle_handles) || any(~isgraphics(obstacle_handles))
        obstacle_handles = gobjects(size(obstacles, 1), 1);
        for j = 1:size(obstacles, 1)
            % Initialize all obstacles as black
            obstacle_center = obstacles(j, 1:2);
            obstacle_radius = obstacles(j, 3);
            x_circle = obstacle_center(1) + obstacle_radius * cos(theta);
            y_circle = obstacle_center(2) + obstacle_radius * sin(theta);
            obstacle_handles(j) = fill(x_circle, y_circle, 'black', 'EdgeColor', 'none', 'FaceAlpha', 1.0, 'Tag', ['obstacle_', num2str(j)]);
        end
    end
    % Cleanup workspace
    existing_sensor = findobj('Tag', 'sensor');
    delete(existing_sensor);
    existing_path = findobj('Tag', 'projected_path');
    delete(existing_path);
    existing_state = findobj('Tag','state');
    delete(existing_state);

    % Initialize agent's state as a scatter plot
    marker_size = 50;
    state = scatter(x(start_index), y(start_index), marker_size, 'filled', 'Tag', 'state', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'black');

    % Initialize path plots
    path = plot(x(1:start_index), y(1:start_index), 'LineWidth', 2, 'Color', 'b', 'Tag', 'current_path');
    projected_path = plot(x(start_index:end), y(start_index:end), 'LineWidth', 2, 'Color', 'b', 'LineStyle', '--', 'Tag', 'projected_path');

    % Initialize the sensor plot
    sensor_plot = plot(nan, nan, 'k--', 'Tag', 'sensor', 'LineWidth', 2); % Dashed circle

    % Interpolation factor (adjust for smoother transitions)
    interp_steps = 10; % Number of interpolation points between each step

    % Iterate through the path up to the divergence point
    for i = start_index:divergence_index(plan_num)
        % Interpolate between the current state and the next state
        if i < divergence_index(plan_num)
            next_x = x(i + 1);
            next_y = y(i + 1);
            interp_x = linspace(x(i), next_x, interp_steps);
            interp_y = linspace(y(i), next_y, interp_steps);
        else
            interp_x = x(i); % No interpolation at the last step
            interp_y = y(i);
        end

        % Plot interpolated steps
        for k = 1:length(interp_x)
            % Update agent's position for interpolation
            set(state, 'XData', interp_x(k), 'YData', interp_y(k));
            set(sensor_plot, 'XData', interp_x(k) + sensing_radius * cos(theta), ...
                             'YData', interp_y(k) + sensing_radius * sin(theta));

            % Dynamically update the path to match the agent's position    
            % Full blue path
            set(path, 'XData', [x(1:i); interp_x(k)], 'YData', [y(1:i); interp_y(k)], 'Color', 'b');
            
            % Update obstacle colors based on proximity
            for j = 1:size(obstacles, 1)
                obstacle_center = obstacles(j, 1:2);
                obstacle_radius = obstacles(j, 3);
                distance_to_agent = norm([interp_x(k),interp_y(k)] - obstacle_center);
                % Determine obstacle color
                if distance_to_agent - obstacle_radius <= sensing_radius
                    if checkPathIntersection(x(i:end), y(i:end), obstacle_center, obstacle_radius+.05, sensing_radius)
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
            % Update the projected path (future trajectory)
            set(projected_path, 'XData', x(i+1:end), 'YData', y(i+1:end));
            % Ensure updates are rendered
            drawnow;
            frame = getframe(gcf);
            writeVideo(video_writer, frame);
        end
    end
end
