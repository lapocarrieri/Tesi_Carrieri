clc 
clear all
% Define the function f1
% Initialize particle filter variables
N = 100; % Number of particles
x = zeros(2, N); % State vector
w = ones(1, N) / N; % Importance weights
T = 10;
% Initialize state vector randomly
x(:, :) = randn(2, N);

% Loop over time steps
for t = 1:T

    for i = 1:N
        % Compute the cost using function f1
        cost = f1(x(:, i));
        
        % Update the importance weight using the exponential of the cost
        w(i) = exp(-cost/2);
    end
    
    % Normalize the importance weights
    w = w / sum(w);
    
    % Resample particles using the importance weights
   %[_, best_particle_id] = max(weights);
   [x, w] = resample(x, w, N);

end

function [new_samples, new_weights] = resample(samples, weights, dim_samples)

	indices = uniformSample(weights', dim_samples);

	new_samples = samples;
	new_weights = ones(1,dim_samples)/dim_samples;

	for i=1:dim_samples
		new_samples(:,i) = samples(:,indices(i));
    end

end
function sampled_indices = uniformSample(weights, num_desired_samples)
    sampled_indices= [];
    dim_weights = size(weights,1);
    
    %normalize the weights (if not normalized)
    normalizer = 1./sum(weights);
    %resize the indices
    step = 1./num_desired_samples;
    
    y0 = rand()*step; 	%sample between 0 and 1/num_desired_sample;ù
    yi = y0;		%value of the sample in the y space
    cumulative = 0;		%this ii our running cumulative distribution
    
        for weight_index=1:dim_weights
	    cumulative = cumulative + normalizer*weights(weight_index); %update cumulative
	    % fill with current_weight_index
	    % until the cumulative does not become larger than yi
	        while cumulative > yi
		        sampled_indices(end+1,1) = weight_index;
		        yi = yi + step;
            end
        end
end
function cost = f1(q)
    % Define your function here, e.g.

    cost = min(norm(residual-transpose(Jacobian(link))*Fext));
end