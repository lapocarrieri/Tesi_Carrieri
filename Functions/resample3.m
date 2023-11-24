function new_indices = resample3(num_particles, weights)
    
    % Sort the weights in descending order and get the indices
    [~, sorted_indices] = sort(weights, 'descend');
    
    % Select the top num_particles indices
    new_indices = sorted_indices(1:num_particles);

end