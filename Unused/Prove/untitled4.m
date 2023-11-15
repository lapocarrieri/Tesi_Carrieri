% Define the skew-symmetric matrix function
skew_symmetric = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

% Define the point P


% Define the system A and b

% Define the objective function
objective = @(weights) norm(pinv(-skew_symmetric(weights(1:3))) * weights(4:6) - point)^2;

% Initial guess for the weights
initial_weights = ones(6, 1);

% Set lower and upper bounds for the weights (optional)
lb = zeros(6, 1);
ub = ones(6, 1);

% Perform optimization to minimize the objective function
options = optimoptions('fmincon', 'Display', 'iter');
[weights, fval] = fmincon(objective, initial_weights, [], [], [], [], lb, ub, [], options);

% Display the optimized weights

