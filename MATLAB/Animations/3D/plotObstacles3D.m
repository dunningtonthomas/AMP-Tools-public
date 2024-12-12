function obstacles = plotObstacles3D(filename,x,y)
    % Read the obstacle data from the file
    data = load(filename);
    sensing_radius = 100;
    % Extract center points, radii, and define heights for cylinders
    x_centers = data(:, 1);
    y_centers = data(:, 2);
    radii = data(:, 3);
    height = 5; % Define a fixed height for all cylinders
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

    % Plot obstacles as cylinders
    hold on;
    theta = linspace(0, 2*pi, 50); % For drawing circles
    for i = 1:length(radii)
        % Generate cylinder surface
        [X, Y, Z] = cylinder(radii(i), 50);
        Z = Z * height; % Scale the height of the cylinder
        surf(X + x_centers(i), Y + y_centers(i), Z, 'FaceColor', 'black', ...
            'EdgeColor', 'none', 'FaceAlpha', 0.5, 'Tag', ['obstacle_', num2str(i)]);
    end
end
