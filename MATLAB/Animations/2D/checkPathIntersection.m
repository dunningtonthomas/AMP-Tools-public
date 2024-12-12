function is_intersecting = checkPathIntersection(x_path, y_path, center, radius,sensing_radius)
    % Check if the path intersects a circular obstacle
    current_state = [x_path(1),y_path(1)];
    is_intersecting = false;
    for k = 2:length(x_path)
        % Define segment endpoints
        point1 = [x_path(k), y_path(k)];
        % Check if the point is within an obstacle
        if norm(point1-center)<=radius
            is_intersecting = true;
            return; % Exit early if intersection is found
        end

    end
end