function propagateAgent(x, y, start_index, planning_index, divergence_index, obstacles, sensing_radius, plan_num)
    figure(1);
    hold on;

    % Declare persistent variables to store cumulative planning segments
    persistent planning_segments_x planning_segments_y;

    % Initialize persistent variables if empty
    if isempty(planning_segments_x)
        planning_segments_x = {};
        planning_segments_y = {};
    end

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
        obstacle_handles(j) = fill(x_circle, y_circle, 'r', 'EdgeColor', 'none', 'FaceAlpha', 1.0, 'Tag', ['obstacle_', num2str(j)]);
    end

    % Cleanup workspace
    existing_sensor = findobj('Tag', 'sensor');
    delete(existing_sensor);
    existing_path = findobj('Tag', 'projected_path');
    delete(existing_path);

    % Initialize agent's state as a scatter plot
    marker_size = 50;
    state = scatter(x(start_index), y(start_index), marker_size, 'filled', 'Tag', 'state', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'black');

    % Initialize path plots
    path = plot(x(1:start_index), y(1:start_index), 'LineWidth', 2, 'Color', 'b', 'Tag', 'current_path');
    planning_path = plot(nan, nan, 'LineWidth', 2, 'Color', 'b', 'Tag', 'planning_path');
    projected_path = plot(x(start_index:end), y(start_index:end), 'LineWidth', 2, 'Color', 'b', 'LineStyle', '--', 'Tag', 'projected_path');

    % Plot all cumulative planning segments from previous calls
    if plan_num>1
        for k = 1:plan_num-1
            plot(planning_segments_x{k}, planning_segments_y{k}, 'LineWidth', 2, 'Color', 'r', 'Tag', 'planning_path');
        end
    end
    % Initialize the sensor plot
    sensor_plot = plot(nan, nan, 'k--', 'Tag', 'sensor', 'LineWidth', 2); % Dashed circle

    % Iterate through the path up to the divergence point
    for i = start_index:divergence_index(plan_num)
        % Update the sensor circle
        sensor_x = x(i) + sensing_radius * cos(theta);
        sensor_y = y(i) + sensing_radius * sin(theta);

        % Update the agent's position and path
        set(state, 'XData', x(i), 'YData', y(i));
        set(sensor_plot, 'XData', sensor_x, 'YData', sensor_y);
        set(projected_path, 'XData', x(i:end), 'YData', y(i:end));

        % Update the trajectory dynamically
        if i >= planning_index(plan_num)
            set(planning_path, 'XData', x(planning_index(plan_num):i), 'YData', y(planning_index(plan_num):i), 'Color', 'r');
            set(path, 'XData', x(1:planning_index(plan_num)), 'YData', y(1:planning_index(plan_num)), 'Color', 'b');
        else
            set(path, 'XData', x(1:i), 'YData', y(1:i), 'Color', 'b');
        end

        % If in planning range, store the current planning segment
        if i == divergence_index(plan_num)
            % Store the planning segment for this path
            planning_segments_x{end + 1} = x(planning_index(plan_num):divergence_index(plan_num));
            planning_segments_y{end + 1} = y(planning_index(plan_num):divergence_index(plan_num));

            % Plot the current planning segment in red
            plot(planning_segments_x{end}, planning_segments_y{end}, 'LineWidth', 2, 'Color', 'r', 'Tag', 'planning_path');
        end

        % Update obstacle colors based on proximity
        for j = 1:size(obstacles, 1)
            obstacle_center = obstacles(j, 1:2);
            obstacle_radius = obstacles(j, 3);
            distance_to_agent = norm([x(i), y(i)] - obstacle_center);

            % Determine obstacle color
            if distance_to_agent - obstacle_radius <= sensing_radius
                if checkPathIntersection(x(planning_index(plan_num):end), y(planning_index(plan_num):end), obstacle_center, obstacle_radius, sensing_radius)
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

        % Pause for animation
        pause(0.2);
    end
end
