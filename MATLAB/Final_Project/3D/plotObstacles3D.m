function obstacles = plotObstacles3D(filename)
    % Read the obstacle data from the file
    data = load(filename);

    % Extract center points, radii, and define heights for cylinders
    x_centers = data(:, 1);
    y_centers = data(:, 2);
    radii = data(:, 3);
    height = 5; % Define a fixed height for all cylinders

    % Combine data into a matrix
    obstacles = [x_centers, y_centers, radii];

    % Plot obstacles as cylinders
    hold on;
    theta = linspace(0, 2*pi, 50); % For drawing circles
    for i = 1:length(radii)
        % Generate cylinder surface
        [X, Y, Z] = cylinder(radii(i), 50);
        Z = Z * height; % Scale the height of the cylinder
        surf(X + x_centers(i), Y + y_centers(i), Z, 'FaceColor', 'r', ...
            'EdgeColor', 'none', 'FaceAlpha', 0.5, 'Tag', ['obstacle_', num2str(i)]);
    end
end
