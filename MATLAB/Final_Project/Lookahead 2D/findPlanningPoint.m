function indices = findPlanningPoint(plan, x, y)
    % Initialize the indices array
    indices = zeros(length(plan),1);
    
    % Loop through each element in x and y
    for i = 1:length(plan)
        % Find the index where both x and y match matrix_x and matrix_y
        match = find(x{i} == plan(i,1) & y{i} == plan(i,2));
        
        if ~isempty(match)
            % If a match is found, store the index
            indices(i) = match(1); % Take the first match in case of duplicates
        else
            % If no match is found, store NaN (or -1, depending on preference)
            indices(i) = NaN; % No match found
        end
    end
end

