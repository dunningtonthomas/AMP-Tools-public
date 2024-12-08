function obstacles = plotObstacles(filename)
    % Read the obstacle data from the file
    data = load(filename);
    
    % Extract center points and radii
    x_centers = data(:, 1);
    y_centers = data(:, 2);
    radii = data(:, 3);
    
    % Combine data into a matrix
    obstacles = [x_centers, y_centers, radii];
    
    % Plot obstacles in the current figure
    hold on;
    theta = linspace(0, 2*pi, 100); % For drawing circles
    
    % Plot each obstacle as a red circle
    for i = 1:length(radii)
        x_circle = x_centers(i) + radii(i) * cos(theta);
        y_circle = y_centers(i) + radii(i) * sin(theta);
        fill(x_circle, y_circle, 'r', 'EdgeColor', 'none', 'FaceAlpha', 1, 'Tag', ['obstacle_', num2str(i)]);
    end
end