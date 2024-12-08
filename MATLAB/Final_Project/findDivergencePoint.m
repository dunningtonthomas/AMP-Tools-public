function [divergence_index] = findDivergencePoint(x_values,y_values)
divergence_index = zeros(length(x_values)-1,1);
for i =1:length(x_values)-1
    % Assume x1, y1, x2, y2 are the paths
    x1 = x_values{i};
    y1 = y_values{i};
    x2 = x_values{i+1};
    y2 = y_values{i+1};
    % Find the minimum length of the two paths
    min_length = min(length(x1), length(x2));
    % Compare up to the minimum length
    divergence_index(i) = find(x1(1:min_length) ~= x2(1:min_length) | y1(1:min_length) ~= y2(1:min_length), 1)-1;
end
divergence_index(end+1) = length(x_values{end});
end

