function obstacles = plotObstacles(filename,x,y)
    % Read the obstacle data from the file
    data = load(filename);
    sensing_radius = 200;
    % Extract center points and radii
    x_centers = data(:, 1);
    y_centers = data(:, 2);
    radii = data(:, 3);
    not_intersection  = zeros(size(radii));
    obstacles = [x_centers, y_centers, radii];
    for i = 1:length(x)
        for j = 1:size(obstacles, 1)
                obstacle_center = obstacles(j, 1:2);
                obstacle_radius = obstacles(j, 3);
                if ~checkPathIntersection(x{i}, y{i}, obstacle_center, obstacle_radius, sensing_radius)
                    not_intersection(j) = 1;  
                end
        end
    end
    reduction = ones(size(radii))*.1.*not_intersection;
    % Combine data into a matrix
    obstacles = [x_centers, y_centers, radii-reduction];
end