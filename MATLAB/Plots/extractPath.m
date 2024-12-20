function [x_values,y_values] = extractPath(filename)
    % Read the file into MATLAB
    data = fileread(filename);
    
    % Split the file into lines
    lines = strsplit(data, '\n');
    
    % Initialize cell arrays for x and y values
    x_values = {};
    y_values = {};
    
    % Loop through each line and process it
    for i = 1:length(lines)
        if isempty(lines{i})
            continue; % Skip empty lines
        end
        
        % Split the line into numbers
        values = sscanf(lines{i}, '%f');
        
        % Separate x and y values (odd indices for x, even indices for y)
        x_values{i} = values(1:2:end); % Odd indices
        y_values{i} = values(2:2:end); % Even indices
    end
end
