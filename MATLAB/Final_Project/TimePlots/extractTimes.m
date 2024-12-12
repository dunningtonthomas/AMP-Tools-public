function t_values = extractTimes(filename)
    % Read the file into MATLAB
    data = fileread(filename);
    
    % Split the file into lines
    lines = strsplit(data, '\n');
    
    % Initialize cell arrays for x and y values
    t_values = {};
    
    % Loop through each line and process it
    for i = 1:length(lines)
        if isempty(lines{i})
            continue; % Skip empty lines
        end
        
        % Split the line into numbers
        values = sscanf(lines{i}, '%f');
        
        % Separate x and y values (odd indices for x, even indices for y)
        t_values{i} = values;
    end
end