%num_particles: number of particles
%weights: weights associated to the particles
function new_indices = resample2(num_particles, weights)
    weightssorted=sort(weights)
    cumulative_sum = cumsum(weightssorted);
    rand_indexs = rand(1,num_particles);
    new_indices = zeros(1,num_particles);
    
    for i=1:num_particles
        new_indices(i) = find(cumulative_sum>=rand_indexs(i), 1);
    end
          % Sample data (replace with actual data)
    
    
% Create a probability distribution for the indices
% Example: uniform distribution (replace with actual probability distribution)
probability_distribution = weightssorted*100; % Uniform distribution




% Plotting
figure;

% Plotting the cumulative sum
yyaxis left;
plot(cumulative_sum, 'LineWidth', 2); % Plot cumulative sum
ylabel('Cumulative Sum');
hold on;
scatter(new_indices, cumulative_sum(new_indices), 'r'); % Plot points at new indices

% Drawing projections to x and y axis
for i = 1:length(new_indices)
    % Projections to the x-axis
    line([new_indices(i), new_indices(i)], [0, cumulative_sum(new_indices(i))], 'Color', 'green', 'LineStyle', '--');
    scatter(new_indices(i), 0, 'k', 'filled'); % Marking the point on x-axis

    % Projections to the y-axis
    line([0, new_indices(i)], [cumulative_sum(new_indices(i)), cumulative_sum(new_indices(i))], 'Color', 'blue', 'LineStyle', '--');
    scatter(0, cumulative_sum(new_indices(i)), 'k', 'filled'); % Marking the point on y-axis
end

% Plotting the probability distribution
yyaxis right;
plot(probability_distribution, 'LineWidth', 0.1, 'Color', 'red'); % Thin red line for probability distribution
ylabel('Probability Distribution');

xlabel('Index');
title('Cumulative Sum with Projections and Probability Distribution');
grid on;
hold off;


end