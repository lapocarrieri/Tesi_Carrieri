%num_particles: number of particles
%weights: weights associated to the particles
resample(3,[0.1 0.3 0.4 0.1 0.2 0.5 1])
function new_indeces = resample(num_particles, weights)
    
    cumulative_sum = cumsum(weights);
    rand_indexs = rand(1,num_particles);
    new_indeces = zeros(1,num_particles);
    
    for i=1:num_particles
        new_indeces(i) = find(cumulative_sum>=rand_indexs(i), 1);
    end

end