function new_indices = resample3(num_particles, weights)
    % Sort the weights in descending order and get the sorted indices
    [~, sorted_indices] = sort(weights, 'descend');
    
    % Select the indices corresponding to the num_particles best weights
    new_indices = sorted_indices(1:num_particles);
end