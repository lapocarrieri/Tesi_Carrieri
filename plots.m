% Sample data, replace this with your actual 'errors' array
n = 10;
errors = errors(samples:end-50,:,:);

% Create a figure for the plots
figure;
Legenda={};
Legenda{1}="Pseudoinverse";
Legenda{2}="SVD";
Legenda{3}="DLS";
Legenda{4}="Optimization";
Legenda{5}="TrainedWeights";

for error_type = 1:6
    subplot(2, 3, error_type); % Create a subplot for each error type
    hold on; % To overlay the plots on the same subplot

    for i = 1:5
        plot(errors(:, i, error_type), 'DisplayName', [Legenda{i}]);
        hold on;
    end

    % Add labels and legend for the current subplot
    xlabel('Data Point');
    ylabel('Error Value');
    title(['Joint  ' num2str(error_type)]);
    legend('Location', 'Best'); % Place the legend within the current subplot
end