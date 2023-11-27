% Define the skew-symmetric matrix function
skew_symmetric = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

% Define the point P

bias = 1e-2; % Small positive bias value
% Define the system A and b

% Define the objective function
if size(Point_intersected,2)>1
    Point_intersected=Point_intersected';
end
objective = @(weights) norm(T*[pinv(-skew_symmetric(weights(1:3))) * weights(4:6);1] - [Point_intersected;1])^2+ bias*sum(weights(1:6));

% Initial guess for the weights
initial_weights = ones(6, 1);

% Set lower and upper bounds for the weights (optional)
lb = ones(6, 1) * 1e-2; % Small positive lower bound
ub = ones(6, 1)*0.95;

% Perform optimization to minimize the objective function
options = optimoptions('fmincon', 'Display', 'off');
[weights, fval] = fmincon(objective, initial_weights, [], [], [], [], lb, ub, [], options);

% Display the optimized weights

