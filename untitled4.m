% Define the skew-symmetric matrix function
skew_symmetric = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

% Define the point P
P = [1; 2; 3];

% Define the system A and b
A = [1 2 3 4 5 6; 7 8 9 10 11 12; 13 14 15 16 17 18; 19 20 21 22 23 24; 25 26 27 28 29 30; 31 32 33 34 35 36; 37 38 39 40 41 42];
b = [43; 44; 45; 46; 47; 48];

% Define the objective function
objective = @(weights) norm(skew_symmetric(A(:,1:3) * weights(1:3)) * (A(:,4:6) * weights(4:6)) - P)^2;

% Initial guess for the weights
initial_weights = ones(6, 1);

% Set lower and upper bounds for the weights (optional)
lb = zeros(6, 1);
ub = ones(6, 1);

% Perform optimization to minimize the objective function
options = optimoptions('fmincon', 'Display', 'iter');
[weights, fval] = fmincon(objective, initial_weights, [], [], [], [], lb, ub, [], options);

% Display the optimized weights
weights
