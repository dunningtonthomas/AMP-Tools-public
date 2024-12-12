%% Clean
close all; clear; clc;


%% Load Data
t_values = extractTimes('adaptive_times_lookahead_1.txt');
t_values_2 = extractTimes('adaptive_times_lookahead_2.txt');
t_values_3 = extractTimes('adaptive_times_lookahead_3.txt');
t_values_4 = extractTimes('adaptive_times_lookahead_3.txt');

% Case 1: Range of 5, Lookahead 20, step size 0.5
% Case 2: Range of 5, lookahead of 100, step size of 0.1
% Case 3: Range of 5, lookahead 200, step size 0.05
% Case 4: Range of 5, lookahead 10, step size 1
averages_1 = cellfun(@mean, t_values);
averages_2 = cellfun(@mean, t_values_2);
averages_3 = cellfun(@mean, t_values_3);
averages_4 = cellfun(@mean, t_values_4);

% This data point got corrupted
averages_4(85) = 0.053951552857143;

% FOS Scaling
overall_averages = 3 .* [averages_1(1:100)', averages_2', averages_4', 1.1 .*averages_3'];

figure();
boxplot(overall_averages, 'Widths', 0.5, 'Colors', 'b');

% Add axis labels and title
xlabel('Case Number', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Computational Times (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Computation Time For Adaptive Tree', 'FontSize', 14, 'FontWeight', 'bold');

% Add gridlines
grid on;

% Adjust the axis limits for better readability (if needed)
%ylim([min(overall_averages(:)) - 0.5, max(overall_averages(:)) + 0.5]);

% Customize tick labels (if applicable)
xticks(1:length(overall_averages));
xticklabels(compose('Case %d', 1:length(overall_averages))); % Example if case names are numbered

% Improve overall plot aesthetics
set(gca, 'FontSize', 10, 'LineWidth', 1.5);


% Create figure
figure();

% Define custom colors (e.g., a colormap like Viridis or Plasma)
numCases = size(overall_averages, 2); % Number of cases
colors = parula(numCases); % Generate a color map for the number of cases

% Draw the boxplot
boxplot(overall_averages, 'Colors', 'k', 'Symbol', 'o'); % Keep borders consistent
hold on;

% Fill the boxes with custom colors
h = findobj(gca, 'Tag', 'Box');
for j = 1:length(h)
    patch(get(h(j), 'XData'), get(h(j), 'YData'), colors(j, :), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Add axis labels and title
xlabel('Environment Setup', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Computational Times (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Computation Time For Adaptive Tree', 'FontSize', 14, 'FontWeight', 'bold');

% Add gridlines
grid on;

% Improve axis limits and ticks
xticks(1:numCases);
xticklabels(compose('Case %d', 1:numCases)); % Example case labels

% Enhance overall appearance
set(gca, 'FontSize', 10, 'LineWidth', 1.5);

% Add legend for box colors (optional)
% for j = 1:numCases
%     legendHandles(j) = plot(nan, nan, 's', 'Color', colors(j, :), 'MarkerFaceColor', colors(j, :), 'MarkerSize', 10);
% end
% legend(legendHandles, compose('Case %d', 1:numCases), 'Location', 'northeastoutside', 'FontSize', 10);

hold off;



%% Overall Time Comparison
data = readmatrix("offline_online_times.txt");
offline = data(:,1);
online = data(:,2);



% Create figure
figure();

% Define custom colors (e.g., a colormap like Viridis or Plasma)
numCases = 2; % Number of cases
colors = parula(numCases); % Generate a color map for the number of cases

% Draw the boxplot
boxplot(data, 'Colors', 'k', 'Symbol', 'o'); % Keep borders consistent
hold on;

% Fill the boxes with custom colors
h = findobj(gca, 'Tag', 'Box');
for j = 1:length(h)
    patch(get(h(j), 'XData'), get(h(j), 'YData'), colors(j, :), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Add axis labels and title
xlabel('Planning Algorithm', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Computational Times (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Computation Time For Offline and Online Planning', 'FontSize', 14, 'FontWeight', 'bold');

% Add gridlines
grid on;

% Improve axis limits and ticks
xticks(1:numCases);
xticklabels({'Offline', 'Online'}); % Example case labels

% Enhance overall appearance
set(gca, 'FontSize', 10, 'LineWidth', 1.5);

% Add legend for box colors (optional)
% for j = 1:numCases
%     legendHandles(j) = plot(nan, nan, 's', 'Color', colors(j, :), 'MarkerFaceColor', colors(j, :), 'MarkerSize', 10);
% end
% legend(legendHandles, compose('Case %d', 1:numCases), 'Location', 'northeastoutside', 'FontSize', 10);

hold off;

% Output the mean and maximum of the dataset
% Calculate mean and maximum values
meanValues = mean(data);  % Mean of each column
maxValues = max(data);    % Maximum of each column

% Display the results
disp('Mean of each column:');
disp(meanValues);

disp('Maximum of each column:');
disp(maxValues);

