% Sample data, replace this with your actual 'errors' array
n = 10;
errors = errors(samples:end-50,:,:);

% Create a figure for the plots
f10 = figure();
Legenda = {};
Legenda{1} = "Pseudoinverse";
Legenda{2} = "SVD";
Legenda{3} = "DLS";
Legenda{4} = "Optimization";
Legenda{5} = "TrainedWeights";

% Initialize array to store plot handles
plotHandles = gobjects(5, 1); % 5 is the number of different plots

for error_type = 1:6
    subplot(2, 3, error_type); % Create a subplot for each error type
    hold on; % To overlay the plots on the same subplot

    for i = 1:5
        h = plot(errors(:, i, error_type), 'DisplayName', Legenda{i});
        plotHandles(i) = h(1); % Store the first handle of each type
    end

    % Add labels for the current subplot
    xlabel('Data Point');
    ylabel('Error Value');
    title(['Joint  ' num2str(error_type)]);
end

% Create a single legend outside the loop
legend(plotHandles, Legenda, 'Location', 'BestOutside');

saveas(f10, 'C:\Users\lapoc\Desktop\link3errors.jpeg');
