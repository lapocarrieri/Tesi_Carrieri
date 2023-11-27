function [F, m] = calculateWrenchWithFriction(triangles, P, J, tau)
    % Calculation of the normal to the triangle containing P
    [normal, found] = findNormal(triangles, P);
    if ~found
        error('Point P not found in any triangle');
    end

    % Friction coefficient for the friction cone
    mu = tan(deg2rad(30));

    % Objective function: minimize the error in wrench calculation
    objectiveFunction = @(x) norm(tau - J*x)^2;

    % Constraints for the friction cone
    constraints = @(x) deal([], [norm(x(1:2)) - mu*dot(x(1:3), normal); -dot(x(1:3), normal)]);

    % Options for the optimizer
    options = optimoptions('fmincon', 'Algorithm', 'sqp');

    % Solving the optimization problem
    x0 = pinv(J) * tau; % Starting point
    solution = fmincon(objectiveFunction, x0, [], [], [], [], [], [], constraints, options);

    % Extracting F and m from the solution
    F = solution(1:3);
    m = solution(4:end);
end