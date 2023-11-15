%num_particles: number of particles
%weights: weights associated to the particles
function new_indeces = resample(num_particles, weights,num_previous_particles)
    particle_list = cell(num_previous_particles, 1);
    for index = 1:numel(weights)
        particle_list{index} = struct('index', index, 'weight', weights(index));
    end
    [~, sorted_indices] = sort(cellfun(@(x) x.weight, particle_list), 'ascend');
    sorted_particles = particle_list(sorted_indices);
    new_indeces = cellfun(@(x) x.index, sorted_particles(1:num_particles));
    

end