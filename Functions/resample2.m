%num_particles: number of particles
%weights: weights associated to the particles
function new_indeces = resample2(num_particles, weights)
    
    cumulative_sum = cumsum(weights);
    rand_indexs = rand(1,num_particles);
    new_indeces = zeros(1,num_particles);
    
    for i=1:num_particles
        new_indeces(i) = find(cumulative_sum>= (i), 1);
    end

end